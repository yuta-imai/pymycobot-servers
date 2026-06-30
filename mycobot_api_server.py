"""
MyCobot REST API Server

REST API server for controlling MyCobot robot joints remotely.
Implements the OpenAPI specification defined in mycobot_api_spec.yaml.
"""

from fastapi import FastAPI, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from typing import List, Optional
from pathlib import Path
import uvicorn
import argparse
from datetime import datetime
import logging

from mycobot_joint_controller import (
    MyCobotJointController, RobotStallError, RobotTimeoutError, RobotMotionError,
)


# Pydantic models for request/response
class MoveJointRequest(BaseModel):
    angle: float = Field(..., ge=-180, le=180, description="Target angle in degrees")
    speed: int = Field(50, ge=1, le=100, description="Movement speed (1-100)")


class MoveAllJointsRequest(BaseModel):
    angles: List[float] = Field(..., min_items=6, max_items=6, description="6 joint angles in degrees")
    speed: int = Field(50, ge=1, le=100, description="Movement speed (1-100)")
    verify: bool = Field(
        False,
        description="When true, BLOCK and guarantee the move lands: send all joints, "
                    "then read back and re-send any straggler joint individually "
                    "(works around flaky multi-joint delivery). Returns once reached "
                    "or after retries. Default false = legacy non-blocking send.")


class JogJointRequest(BaseModel):
    direction: int = Field(..., description="1 for positive, -1 for negative direction")
    speed: int = Field(50, ge=1, le=100, description="Movement speed (1-100)")
    increment: float = Field(
        5.0, gt=0, le=90,
        description="Degrees to move per call (0 < increment <= 90). Defaults to 5."
    )

    class Config:
        schema_extra = {
            "example": {
                "direction": 1,
                "speed": 50,
                "increment": 5.0
            }
        }


class SpeedRequest(BaseModel):
    speed: int = Field(50, ge=1, le=100, description="Movement speed (1-100)")


class TopdownRequest(BaseModel):
    x: float = Field(..., description="Target X (mm, base frame / corrected-fk space)")
    y: float = Field(..., description="Target Y (mm)")
    z: float = Field(..., description="Target Z (mm); top-down approach")
    speed: int = Field(25, ge=1, le=100, description="Movement speed (1-100)")


class CalibrateWristRequest(BaseModel):
    """Run the accelerometer wrist calibration sweep (long-running, moves the arm).

    Drives a pose set, reads gravity from the BLE sensor at each pose, fits the
    corrected wrist model and hot-reloads it. Servos must be powered on first.
    """
    pose_set: str = Field(
        "default",
        description="'default' (~78 poses, accurate) or 'quick' (4 poses, smoke test)")
    speed: int = Field(20, ge=1, le=100, description="Sweep movement speed (1-100)")
    settle: float = Field(3.0, ge=0.5, le=10.0, description="Post-move settle seconds")
    step: int = Field(30, ge=10, le=90, description="Joint sweep step (deg) for 'default'")
    samples: int = Field(12, ge=4, le=60, description="Gravity samples per pose (~10Hz)")
    still_dps: float = Field(2.5, ge=0.5, le=10.0, description="Stillness gate (median gyro deg/s)")
    ble_address: Optional[str] = Field(
        "F7:50:70:EE:0D:DF", description="BLE accelerometer MAC (WT9011DCL)")
    ble_name: Optional[str] = Field(None, description="BLE name (alternative to address)")
    ble_mode: str = Field("witmotion", description="BLE payload mode")
    save_data: bool = Field(True, description="Also write the raw JSONL sweep next to the model")


class WaitRequest(BaseModel):
    timeout: float = Field(15.0, ge=0.1, le=60.0, description="Maximum time to wait in seconds")
    tolerance: float = Field(
        1.0, ge=0.0, le=10.0,
        description="Per-joint convergence tolerance in degrees (default 1.0)"
    )
    target: Optional[List[float]] = Field(
        None, min_items=6, max_items=6,
        description="Optional explicit 6-joint target. Defaults to the last commanded target."
    )
    raise_on_incomplete: bool = Field(
        False,
        description="When true, respond 409 (not 200) if the move stalls or times "
                    "out, instead of returning completed=false. Default false keeps "
                    "the legacy 200 contract."
    )


class GripperActionRequest(BaseModel):
    speed: int = Field(50, ge=1, le=100, description="Gripper movement speed (1-100)")
    gripper_type: Optional[int] = Field(
        None,
        ge=1,
        le=4,
        description="Gripper type: 1=adaptive, 2=5-finger dexterous, 3=parallel, 4=flexible"
    )


class GripperStateRequest(BaseModel):
    state: int = Field(..., description="0=open, 1=close, 10=release")
    speed: int = Field(50, ge=1, le=100, description="Gripper movement speed (1-100)")
    gripper_type: Optional[int] = Field(
        None,
        ge=1,
        le=4,
        description="Gripper type: 1=adaptive, 2=5-finger dexterous, 3=parallel, 4=flexible"
    )


class GripperValueRequest(BaseModel):
    value: int = Field(..., ge=0, le=100, description="Gripper opening value (0-100)")
    speed: int = Field(50, ge=1, le=100, description="Gripper movement speed (1-100)")
    gripper_type: Optional[int] = Field(
        None,
        ge=1,
        le=4,
        description="Gripper type (set value supports 1=adaptive, 3=parallel, 4=flexible)"
    )


# Response models
class HealthResponse(BaseModel):
    status: str
    robot_connected: bool
    api_version: str


class JointAngleResponse(BaseModel):
    joint_num: int
    angle: float
    timestamp: str


class AllJointAnglesResponse(BaseModel):
    angles: List[float]
    timestamp: str


class RobotStatusResponse(BaseModel):
    joint_angles: List[float]
    is_moving: bool
    timestamp: str
    error_code: Optional[int] = None
    error_message: Optional[str] = None
    joints_near_limit: List[dict] = []


class GripperStatusResponse(BaseModel):
    value: int
    is_moving: bool
    timestamp: str


class SuccessResponse(BaseModel):
    success: bool
    message: str
    timestamp: str


class WaitResponse(BaseModel):
    completed: bool
    elapsed_time: float
    reason: str
    max_error: Optional[float] = None


# Cartesian (coordinate-space) request/response models
class MoveCoordsRequest(BaseModel):
    coords: List[float] = Field(
        ..., min_items=6, max_items=6,
        description="Target pose [x, y, z, rx, ry, rz] in mm / degrees"
    )
    speed: int = Field(30, ge=1, le=100, description="Movement speed (1-100)")
    mode: int = Field(
        0, ge=0, le=1,
        description="0 = angular (point-to-point), 1 = linear (straight-line)"
    )
    validate: bool = Field(
        True,
        description="Run a no-move IK reachability check before sending; reject unreachable poses with 400"
    )


class CoordsRequest(BaseModel):
    coords: List[float] = Field(
        ..., min_items=6, max_items=6,
        description="Pose [x, y, z, rx, ry, rz] in mm / degrees"
    )


class WaitCoordsRequest(BaseModel):
    target: Optional[List[float]] = Field(
        None, min_items=6, max_items=6,
        description="Optional explicit target pose [x,y,z,rx,ry,rz]. Defaults to the last commanded coords target."
    )
    pos_tolerance: float = Field(
        3.0, ge=0.0, le=50.0, description="Position convergence tolerance in mm"
    )
    ori_tolerance: float = Field(
        3.0, ge=0.0, le=45.0, description="Orientation convergence tolerance in degrees"
    )
    timeout: float = Field(
        20.0, ge=0.1, le=60.0, description="Maximum time to wait in seconds"
    )


class CoordsResponse(BaseModel):
    coords: List[float]
    timestamp: str


class WaitCoordsResponse(BaseModel):
    completed: bool
    elapsed_time: float
    reason: str
    pos_error: Optional[float] = None
    ori_error: Optional[float] = None


class ReachableResponse(BaseModel):
    reachable: bool
    reason: str
    ik_angles: Optional[List[float]] = None
    timestamp: str


class ErrorResponse(BaseModel):
    error: str
    message: str
    timestamp: str


# Global controller instance
controller: Optional[MyCobotJointController] = None


# ---- wrist-calibration background job -------------------------------------
# The accel sweep is long-running (minutes, drives the arm), so it runs in a
# daemon thread and callers poll GET /robot/calibrate_wrist/status. Only one job
# at a time; the live controller (serial owner) drives the poses in-process.
import threading as _threading

_calib_lock = _threading.Lock()
_calib_state = {
    "status": "idle",          # idle | running | done | error
    "phase": None,             # collecting | fitting | done
    "progress": {"done": 0, "total": 0, "still": None},
    "params": None,
    "started_at": None,
    "finished_at": None,
    "result": None,
    "error": None,
}


def _run_wrist_calibration(params: dict) -> None:
    """Background worker: collect -> fit -> write -> hot-reload the wrist model."""
    import os as _os
    import sys as _sys
    _wc = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                        "scripts", "wrist_calib")
    if _wc not in _sys.path:
        _sys.path.insert(0, _wc)
    try:
        import service as _service  # scripts/wrist_calib/service.py

        def _progress(done, total, still):
            _calib_state["progress"] = {"done": done, "total": total, "still": still}
            _calib_state["phase"] = "fitting" if done >= total else "collecting"

        data_out = None
        if params.get("save_data"):
            data_out = _os.path.join(_wc, "calib_api_latest.jsonl")

        _calib_state["phase"] = "collecting"
        result = _service.run_calibration(
            controller,
            pose_set=params["pose_set"], step=params["step"], speed=params["speed"],
            settle=params["settle"], samples=params["samples"],
            still_dps=params["still_dps"], ble_address=params.get("ble_address"),
            ble_name=params.get("ble_name"), ble_mode=params["ble_mode"],
            data_out=data_out, progress_cb=_progress)
        _calib_state["phase"] = "done"
        _calib_state["result"] = result
        _calib_state["status"] = "done"
    except Exception as e:  # noqa: BLE001 — surface any failure to the poller
        _calib_state["status"] = "error"
        _calib_state["error"] = str(e)
    finally:
        _calib_state["finished_at"] = get_current_timestamp()


# FastAPI app initialization
app = FastAPI(
    title="MyCobot Joint Controller REST API",
    description="REST API for controlling MyCobot robot joints remotely",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# Add CORS middleware for external access
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Serve the 3D pose visualizer (static three.js app) same-origin at /ui.
# Self-contained: it fetches live state via the REST API above (HTTP GET).
_WEBUI_DIR = Path(__file__).resolve().parent / "webui"
if _WEBUI_DIR.is_dir():
    app.mount("/ui", StaticFiles(directory=str(_WEBUI_DIR), html=True), name="ui")
    logger.info(f"Pose visualizer UI mounted at /ui (from {_WEBUI_DIR})")
else:
    logger.warning(f"webui dir not found at {_WEBUI_DIR}; /ui not mounted")


def get_current_timestamp() -> str:
    """Get current timestamp in ISO format."""
    return datetime.utcnow().isoformat() + "Z"


def ensure_controller():
    """Ensure controller is initialized and connected."""
    global controller
    if controller is None:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Robot controller not initialized"
        )


def robot_error_detail(action: str, exc: Exception) -> str:
    """Build a 503 detail string, enriched with the firmware error code/message.

    Queries get_error_information() so a raw library error (e.g. a joint-limit
    or collision fault) is reported with meaning instead of being passed through
    opaquely. Only runs on the error path, so the extra serial round-trip is
    acceptable.
    """
    detail = f"{action}: {exc}"
    if controller is not None:
        try:
            code, message = controller.describe_error()
        except Exception:
            code, message = None, None
        if code:
            detail += f" [robot error {code}: {message}]"
    return detail


@app.on_event("startup")
async def startup_event():
    """Initialize robot controller on startup."""
    global controller
    try:
        controller = MyCobotJointController()
        logger.info("MyCobot controller initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize MyCobot controller: {e}")
        controller = None


@app.on_event("shutdown")
async def shutdown_event():
    """Clean up controller on shutdown."""
    global controller
    if controller:
        try:
            controller.close_connection()
            logger.info("MyCobot controller connection closed")
        except Exception as e:
            logger.error(f"Error closing controller connection: {e}")


# Health check endpoint
@app.get("/health", response_model=HealthResponse, tags=["system"])
async def health_check():
    """Check if the API server and robot connection are healthy."""
    robot_connected = controller is not None
    return HealthResponse(
        status="healthy" if robot_connected else "degraded",
        robot_connected=robot_connected,
        api_version="1.0.0"
    )


# Joint angle endpoints
@app.get("/joints/{joint_num}/angle", response_model=JointAngleResponse, tags=["joints"])
async def get_joint_angle(joint_num: int):
    """Get current angle of a specific joint."""
    ensure_controller()
    
    if not (1 <= joint_num <= 6):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Joint number must be between 1-6, got {joint_num}"
        )
    
    try:
        angle = controller.get_joint_angle(joint_num)
        return JointAngleResponse(
            joint_num=joint_num,
            angle=angle,
            timestamp=get_current_timestamp()
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to get joint angle: {str(e)}"
        )


@app.put("/joints/{joint_num}/angle", response_model=SuccessResponse, tags=["joints"])
async def move_joint(joint_num: int, request: MoveJointRequest):
    """Move a specific joint to target angle."""
    ensure_controller()
    
    if not (1 <= joint_num <= 6):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Joint number must be between 1-6, got {joint_num}"
        )
    
    try:
        controller.move_joint(joint_num, request.angle, request.speed)
        return SuccessResponse(
            success=True,
            message=f"Joint {joint_num} moving to {request.angle} degrees at speed {request.speed}",
            timestamp=get_current_timestamp()
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=robot_error_detail(f"Failed to move joint {joint_num}", e)
        )


@app.get("/joints/angles", response_model=AllJointAnglesResponse, tags=["joints"])
async def get_all_joint_angles():
    """Get current angles of all joints."""
    ensure_controller()
    
    try:
        angles = controller.get_all_joint_angles()
        return AllJointAnglesResponse(
            angles=angles,
            timestamp=get_current_timestamp()
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to get joint angles: {str(e)}"
        )


@app.put("/joints/angles", response_model=SuccessResponse, tags=["joints"])
async def move_all_joints(request: MoveAllJointsRequest):
    """Move all joints to specified angles simultaneously."""
    ensure_controller()
    
    try:
        if request.verify:
            res = controller.move_all_joints_safe(request.angles, request.speed)
            return SuccessResponse(
                success=bool(res.get("reached")),
                message=f"Verified move to {request.angles}: reached={res.get('reached')} "
                        f"rounds={res.get('rounds')} max_error={res.get('max_error')}",
                timestamp=get_current_timestamp()
            )
        controller.move_all_joints(request.angles, request.speed)
        return SuccessResponse(
            success=True,
            message=f"All joints moving to {request.angles} at speed {request.speed}",
            timestamp=get_current_timestamp()
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=robot_error_detail("Failed to move joints", e)
        )


@app.post("/joints/{joint_num}/jog", response_model=SuccessResponse, tags=["joints"])
async def jog_joint(joint_num: int, request: JogJointRequest):
    """Jog a joint in specified direction."""
    ensure_controller()
    
    if not (1 <= joint_num <= 6):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Joint number must be between 1-6, got {joint_num}"
        )
    
    if request.direction not in [-1, 1]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Direction must be 1 or -1"
        )
    
    try:
        target = controller.joint_jog(
            joint_num, request.direction, request.speed, request.increment
        )
        direction_str = "positive" if request.direction == 1 else "negative"
        return SuccessResponse(
            success=True,
            message=(
                f"Joint {joint_num} jogging {direction_str} by {request.increment}° "
                f"to {target:.2f}° at speed {request.speed}"
            ),
            timestamp=get_current_timestamp()
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=robot_error_detail(
                f"Failed to jog joint {joint_num} (direction={request.direction}, "
                f"increment={request.increment})", e
            )
        )


# Gripper control endpoints
@app.post("/gripper/open", response_model=SuccessResponse, tags=["gripper"])
async def open_gripper(request: Optional[GripperActionRequest] = None):
    """Open gripper."""
    ensure_controller()

    speed = request.speed if request else 50
    gripper_type = request.gripper_type if request else None

    try:
        controller.open_gripper(speed=speed, gripper_type=gripper_type)
        gripper_type_msg = f", gripper_type {gripper_type}" if gripper_type is not None else ""
        return SuccessResponse(
            success=True,
            message=f"Gripper opening at speed {speed}{gripper_type_msg}",
            timestamp=get_current_timestamp()
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to open gripper: {str(e)}"
        )


@app.post("/gripper/close", response_model=SuccessResponse, tags=["gripper"])
async def close_gripper(request: Optional[GripperActionRequest] = None):
    """Close gripper."""
    ensure_controller()

    speed = request.speed if request else 50
    gripper_type = request.gripper_type if request else None

    try:
        controller.close_gripper(speed=speed, gripper_type=gripper_type)
        gripper_type_msg = f", gripper_type {gripper_type}" if gripper_type is not None else ""
        return SuccessResponse(
            success=True,
            message=f"Gripper closing at speed {speed}{gripper_type_msg}",
            timestamp=get_current_timestamp()
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to close gripper: {str(e)}"
        )


@app.post("/gripper/release", response_model=SuccessResponse, tags=["gripper"])
async def release_gripper(request: Optional[GripperActionRequest] = None):
    """Release gripper (firmware-supported torque release mode)."""
    ensure_controller()

    speed = request.speed if request else 50
    gripper_type = request.gripper_type if request else None

    try:
        controller.release_gripper(speed=speed, gripper_type=gripper_type)
        gripper_type_msg = f", gripper_type {gripper_type}" if gripper_type is not None else ""
        return SuccessResponse(
            success=True,
            message=f"Gripper release command sent at speed {speed}{gripper_type_msg}",
            timestamp=get_current_timestamp()
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to release gripper: {str(e)}"
        )


@app.put("/gripper/state", response_model=SuccessResponse, tags=["gripper"])
async def set_gripper_state(request: GripperStateRequest):
    """Set gripper state directly (0=open, 1=close, 10=release)."""
    ensure_controller()

    try:
        controller.set_gripper_state(request.state, request.speed, request.gripper_type)
        gripper_type_msg = f", gripper_type {request.gripper_type}" if request.gripper_type is not None else ""
        return SuccessResponse(
            success=True,
            message=f"Gripper state set to {request.state} at speed {request.speed}{gripper_type_msg}",
            timestamp=get_current_timestamp()
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to set gripper state: {str(e)}"
        )


@app.put("/gripper/value", response_model=SuccessResponse, tags=["gripper"])
async def set_gripper_value(request: GripperValueRequest):
    """Set gripper opening value (0-100)."""
    ensure_controller()

    try:
        controller.set_gripper_value(request.value, request.speed, request.gripper_type)
        gripper_type_msg = f", gripper_type {request.gripper_type}" if request.gripper_type is not None else ""
        return SuccessResponse(
            success=True,
            message=f"Gripper value set to {request.value} at speed {request.speed}{gripper_type_msg}",
            timestamp=get_current_timestamp()
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to set gripper value: {str(e)}"
        )


@app.post("/gripper/calibrate", response_model=SuccessResponse, tags=["gripper"])
async def calibrate_gripper():
    """Calibrate gripper and set current position as reference."""
    ensure_controller()

    try:
        controller.calibrate_gripper()
        return SuccessResponse(
            success=True,
            message="Gripper calibration completed",
            timestamp=get_current_timestamp()
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to calibrate gripper: {str(e)}"
        )


# Robot control endpoints
@app.post("/robot/home", response_model=SuccessResponse, tags=["robot"])
async def home_position(request: Optional[SpeedRequest] = None):
    """Move all joints to home position (0 degrees)."""
    ensure_controller()
    
    speed = request.speed if request else 50
    
    try:
        controller.home_position(speed)
        return SuccessResponse(
            success=True,
            message=f"Moving to home position at speed {speed}",
            timestamp=get_current_timestamp()
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=robot_error_detail("Failed to move to home position", e)
        )


@app.post("/robot/stop", response_model=SuccessResponse, tags=["robot"])
async def stop_all_joints():
    """Emergency stop for all joint movements."""
    ensure_controller()
    
    try:
        controller.stop_all_joints()
        return SuccessResponse(
            success=True,
            message="All joints stopped",
            timestamp=get_current_timestamp()
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to stop joints: {str(e)}"
        )


@app.post("/robot/release", response_model=SuccessResponse, tags=["robot"])
async def release_all_servos():
    """Relax the WHOLE arm: cut torque on every joint servo so it can be moved by
    hand (freedrive). The arm goes limp and sags under gravity — support it
    before calling. Re-engage with POST /robot/power_on."""
    ensure_controller()
    try:
        controller.release_all_servos()
        return SuccessResponse(
            success=True,
            message="All servos released (arm is limp; support it). "
                    "Use /robot/power_on to re-engage.",
            timestamp=get_current_timestamp()
        )
    except NotImplementedError as e:
        raise HTTPException(status_code=status.HTTP_501_NOT_IMPLEMENTED, detail=str(e))
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to release servos: {str(e)}"
        )


@app.post("/robot/power_on", response_model=SuccessResponse, tags=["robot"])
async def power_on():
    """Re-engage (power on) all joint servos after /robot/release."""
    ensure_controller()
    try:
        controller.power_on()
        return SuccessResponse(
            success=True,
            message="All servos powered on",
            timestamp=get_current_timestamp()
        )
    except NotImplementedError as e:
        raise HTTPException(status_code=status.HTTP_501_NOT_IMPLEMENTED, detail=str(e))
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to power on servos: {str(e)}"
        )


@app.get("/robot/status", response_model=RobotStatusResponse, tags=["robot"])
async def get_robot_status(include_error: bool = False):
    """Get current robot status including joint angles and movement state.

    Set `include_error=true` to also report the firmware error code/message.
    It is off by default to keep this hot endpoint fast (it costs an extra
    serial round-trip).
    """
    ensure_controller()

    try:
        joint_angles = controller.get_all_joint_angles()
        # Self-computed movement detection (compares successive angle samples)
        # instead of the firmware is_moving() flag, which can stick at True.
        is_moving = controller.compute_is_moving(joint_angles)

        error_code, error_message = (None, None)
        if include_error:
            error_code, error_message = controller.describe_error()

        # Surface joints sitting at/near a hard limit (the "deadlock" state) so a
        # stuck arm is visible from status even while idle. Computed from the
        # angles already read, so it adds no serial round-trip.
        joints_near_limit = controller.detect_near_limits(angles=joint_angles)

        return RobotStatusResponse(
            joint_angles=joint_angles,
            is_moving=is_moving,
            timestamp=get_current_timestamp(),
            error_code=error_code,
            error_message=error_message,
            joints_near_limit=joints_near_limit,
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to get robot status: {str(e)}"
        )


@app.get("/gripper/status", response_model=GripperStatusResponse, tags=["gripper"])
async def get_gripper_status(gripper_type: Optional[int] = None):
    """Get current gripper opening value (0-100) and whether it is moving.

    `gripper_type` (1=adaptive, 3=parallel, 4=flexible) defaults to the firmware
    default when omitted.
    """
    ensure_controller()

    try:
        value = controller.get_gripper_value(gripper_type)
        is_moving = controller.is_gripper_moving()
        return GripperStatusResponse(
            value=value,
            is_moving=is_moving,
            timestamp=get_current_timestamp()
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except NotImplementedError as e:
        raise HTTPException(
            status_code=status.HTTP_501_NOT_IMPLEMENTED,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=robot_error_detail("Failed to get gripper status", e)
        )


@app.post("/robot/wait", response_model=WaitResponse, tags=["robot"])
async def wait_for_completion(request: Optional[WaitRequest] = None):
    """Wait for the robot to reach its target (converged / stalled / timeout).

    Uses tolerance-based convergence, stall detection, and a hard timeout.
    On stall or timeout the robot is stopped before returning.
    """
    ensure_controller()

    timeout = request.timeout if request else 15.0
    tolerance = request.tolerance if request else 1.0
    target = request.target if request else None
    raise_on_incomplete = request.raise_on_incomplete if request else False

    try:
        result = controller.wait_for_completion(
            target=target, tol=tolerance, timeout=timeout,
            raise_on_incomplete=raise_on_incomplete,
        )
        return WaitResponse(
            completed=result["completed"],
            elapsed_time=result["elapsed"],
            reason=result["reason"],
            max_error=result["max_error"],
        )
    except RobotMotionError as e:
        # opt-in strict mode: stall/timeout surfaces as 409 with the wait fields
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail={"reason": e.reason, "elapsed_time": e.elapsed,
                    "max_error": e.max_error, "message": str(e)},
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to wait for completion: {str(e)}"
        )


# Cartesian (coordinate-space) endpoints
@app.get("/robot/coords", response_model=CoordsResponse, tags=["robot"])
async def get_coords():
    """Get the current end-effector pose [x, y, z, rx, ry, rz] (mm / degrees)."""
    ensure_controller()

    try:
        coords = controller.get_coords()
        return CoordsResponse(coords=coords, timestamp=get_current_timestamp())
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=robot_error_detail("Failed to get coordinates", e),
        )


@app.put("/robot/coords", response_model=SuccessResponse, tags=["robot"])
async def move_coords(request: MoveCoordsRequest):
    """Move the end-effector to an absolute Cartesian pose (non-blocking).

    The pose is validated against the workspace box before sending; use
    POST /robot/coords/wait to wait for arrival.
    """
    ensure_controller()

    try:
        if request.validate:
            reachable, reason, _ = controller.check_pose_reachable(request.coords)
            if not reachable:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Target pose not reachable: {reason}",
                )
        controller.send_coords(request.coords, request.speed, request.mode)
        mode_str = "linear" if request.mode == 1 else "angular"
        return SuccessResponse(
            success=True,
            message=f"Moving to coords {request.coords} at speed {request.speed} ({mode_str})",
            timestamp=get_current_timestamp(),
        )
    except HTTPException:
        raise
    except ValueError as e:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(e))
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=robot_error_detail("Failed to move to coords", e),
        )


@app.post("/robot/topdown", response_model=SuccessResponse, tags=["robot"])
async def move_topdown(request: TopdownRequest):
    """Move the gripper straight-DOWN to (x,y,z) via the accel-calibrated top-down
    IK (solve_topdown_ik -> send_angles). 400 if unreachable (nothing is sent).
    Used by eye-to-hand calibration (command-observe) and picking."""
    ensure_controller()
    try:
        q = controller.move_topdown(request.x, request.y, request.z, request.speed)
        if q is None:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Top-down pose ({request.x:.1f},{request.y:.1f},"
                       f"{request.z:.1f}) not reachable",
            )
        return SuccessResponse(
            success=True,
            message=f"Top-down move to ({request.x:.1f},{request.y:.1f},"
                    f"{request.z:.1f}); joints={[round(a,1) for a in q]}",
            timestamp=get_current_timestamp(),
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=robot_error_detail("Failed top-down move", e),
        )


@app.post("/robot/calibrate_wrist", response_model=SuccessResponse, tags=["robot"])
async def calibrate_wrist(request: CalibrateWristRequest):
    """Start the accelerometer wrist calibration (LONG-RUNNING; moves the arm).

    Drives the pose set, reads the BLE accelerometer's gravity vector at each pose,
    fits the corrected wrist DH model and hot-reloads it into the live controller.
    Servos must be powered on (POST /robot/power_on) and the workspace clear.

    Returns immediately; poll GET /robot/calibrate_wrist/status for progress and
    the final orientation-error result. Only one calibration runs at a time."""
    ensure_controller()
    if request.pose_set not in ("default", "quick"):
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST,
                            detail="pose_set must be 'default' or 'quick'")
    with _calib_lock:
        if _calib_state["status"] == "running":
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="A wrist calibration is already running; poll "
                       "/robot/calibrate_wrist/status")
        params = request.dict()
        _calib_state.update({
            "status": "running", "phase": "starting",
            "progress": {"done": 0, "total": 0, "still": None},
            "params": params, "started_at": get_current_timestamp(),
            "finished_at": None, "result": None, "error": None,
        })
    _threading.Thread(target=_run_wrist_calibration, args=(params,),
                      daemon=True).start()
    return SuccessResponse(
        success=True,
        message=f"Wrist calibration started (pose_set={params['pose_set']}); "
                f"poll GET /robot/calibrate_wrist/status",
        timestamp=get_current_timestamp(),
    )


@app.get("/robot/calibrate_wrist/status", tags=["robot"])
async def calibrate_wrist_status():
    """Poll the wrist-calibration job (status/phase/progress/result/error)."""
    return {**_calib_state, "timestamp": get_current_timestamp()}


@app.post("/robot/coords/check", response_model=ReachableResponse, tags=["robot"])
async def check_coords_reachable(request: CoordsRequest):
    """Check, WITHOUT moving, whether a Cartesian pose is reachable (firmware IK)."""
    ensure_controller()

    try:
        reachable, reason, ik = controller.check_pose_reachable(request.coords)
        return ReachableResponse(
            reachable=reachable, reason=reason, ik_angles=ik,
            timestamp=get_current_timestamp(),
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=robot_error_detail("Failed to check reachability", e),
        )


@app.post("/robot/coords/wait", response_model=WaitCoordsResponse, tags=["robot"])
async def wait_for_coords(request: Optional[WaitCoordsRequest] = None):
    """Wait until the end-effector reaches its Cartesian target.

    Convergence is judged by position (mm) and orientation (deg) tolerance, with
    firmware IK-fault detection, stall detection, and a hard timeout. On
    fault/stall/timeout the robot is stopped before returning.
    """
    ensure_controller()

    target = request.target if request else None
    pos_tol = request.pos_tolerance if request else 3.0
    ori_tol = request.ori_tolerance if request else 3.0
    timeout = request.timeout if request else 20.0

    # Validate an explicit target up front so a bad pose returns 400 rather than
    # producing meaningless convergence math (and a 503) inside the wait loop.
    if target is not None:
        try:
            controller._validate_coords(target)
        except ValueError as e:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(e))

    try:
        result = controller.wait_for_coords_completion(
            target_coords=target, pos_tol=pos_tol, ori_tol=ori_tol, timeout=timeout
        )
        return WaitCoordsResponse(
            completed=result["completed"],
            elapsed_time=result["elapsed"],
            reason=result["reason"],
            pos_error=result["pos_error"],
            ori_error=result["ori_error"],
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to wait for coords completion: {str(e)}",
        )


def main():
    """Main function to run the API server."""
    parser = argparse.ArgumentParser(description='MyCobot REST API Server')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                       help='Host to bind server to (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8080,
                       help='Port to bind server to (default: 8080)')
    parser.add_argument('--robot-port', type=str, default='/dev/ttyACM0',
                       help='Robot serial port (default: /dev/ttyACM0)')
    parser.add_argument('--robot-baudrate', type=int, default=115200,
                       help='Robot serial baudrate (default: 115200)')
    parser.add_argument('--reload', action='store_true',
                       help='Enable auto-reload for development')
    
    args = parser.parse_args()
    
    # Store robot connection parameters globally (for startup event)
    global robot_port, robot_baudrate
    robot_port = args.robot_port
    robot_baudrate = args.robot_baudrate
    
    logger.info(f"Starting MyCobot REST API server on {args.host}:{args.port}")
    logger.info(f"Robot connection: {args.robot_port} @ {args.robot_baudrate} baud")
    
    uvicorn.run(
        "mycobot_api_server:app",
        host=args.host,
        port=args.port,
        reload=args.reload,
        log_level="info"
    )


if __name__ == "__main__":
    main()