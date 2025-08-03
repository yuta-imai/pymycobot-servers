"""
MyCobot REST API Server

REST API server for controlling MyCobot robot joints remotely.
Implements the OpenAPI specification defined in mycobot_api_spec.yaml.
"""

from fastapi import FastAPI, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Optional
import uvicorn
import argparse
from datetime import datetime
import logging
import time

from mycobot_joint_controller import MyCobotJointController


# Pydantic models for request/response
class MoveJointRequest(BaseModel):
    angle: float = Field(..., ge=-175, le=175, description="Target angle in degrees")
    speed: int = Field(50, ge=1, le=100, description="Movement speed (1-100)")


class MoveAllJointsRequest(BaseModel):
    angles: List[float] = Field(..., min_items=6, max_items=6, description="6 joint angles in degrees")
    speed: int = Field(50, ge=1, le=100, description="Movement speed (1-100)")


class JogJointRequest(BaseModel):
    direction: int = Field(..., description="1 for positive, -1 for negative direction")
    speed: int = Field(50, ge=1, le=100, description="Movement speed (1-100)")

    class Config:
        schema_extra = {
            "example": {
                "direction": 1,
                "speed": 50
            }
        }


class SpeedRequest(BaseModel):
    speed: int = Field(50, ge=1, le=100, description="Movement speed (1-100)")


class WaitRequest(BaseModel):
    timeout: float = Field(10.0, ge=0.1, le=60.0, description="Maximum time to wait in seconds")


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


class SuccessResponse(BaseModel):
    success: bool
    message: str
    timestamp: str


class WaitResponse(BaseModel):
    completed: bool
    elapsed_time: float


class ErrorResponse(BaseModel):
    error: str
    message: str
    timestamp: str


# Global controller instance
controller: Optional[MyCobotJointController] = None


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
            detail=f"Failed to move joint: {str(e)}"
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
            detail=f"Failed to move joints: {str(e)}"
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
        controller.joint_jog(joint_num, request.direction, request.speed)
        direction_str = "positive" if request.direction == 1 else "negative"
        return SuccessResponse(
            success=True,
            message=f"Joint {joint_num} jogging in {direction_str} direction at speed {request.speed}",
            timestamp=get_current_timestamp()
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to jog joint: {str(e)}"
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
            detail=f"Failed to move to home position: {str(e)}"
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


@app.get("/robot/status", response_model=RobotStatusResponse, tags=["robot"])
async def get_robot_status():
    """Get current robot status including joint angles and movement state."""
    ensure_controller()
    
    try:
        joint_angles = controller.get_all_joint_angles()
        is_moving = controller.mc.is_moving()
        
        return RobotStatusResponse(
            joint_angles=joint_angles,
            is_moving=is_moving,
            timestamp=get_current_timestamp()
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to get robot status: {str(e)}"
        )


@app.post("/robot/wait", response_model=WaitResponse, tags=["robot"])
async def wait_for_completion(request: Optional[WaitRequest] = None):
    """Wait for robot to complete current movement."""
    ensure_controller()
    
    timeout = request.timeout if request else 10.0
    
    try:
        start_time = time.time()
        completed = controller.wait_for_completion(timeout)
        elapsed_time = time.time() - start_time
        
        return WaitResponse(
            completed=completed,
            elapsed_time=elapsed_time
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Failed to wait for completion: {str(e)}"
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