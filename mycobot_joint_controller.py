"""
MyCobot Joint Controller Module

This module provides functions to control individual joints and gripper of a
MyCobot robot using the pymycobot library.
"""

from pymycobot import MyCobot
import time
from collections import deque
from typing import Union, List, Optional, Tuple


class MyCobotJointController:
    """Controller class for individual joint operations on MyCobot robot."""

    # Public gripper state values (API abstraction) -> firmware flag values.
    # Firmware only accepts 0/1/254; release is exposed as 10 but must be sent
    # to the firmware as 254 (torque off).
    _FIRMWARE_GRIPPER_FLAGS = {0: 0, 1: 1, 10: 254}

    # Fixed messages for robot error codes from get_error_information().
    # Ranges (joint-limit 1-6, collision 16-19) are handled in _error_message.
    _ERROR_CODE_MESSAGES = {
        0: "No error",
        32: "No inverse kinematics solution for the target pose",
        33: "Linear motion has no adjacent solution",
        34: "Linear motion has no adjacent solution",
    }


    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        """
        Initialize MyCobot connection.
        
        Args:
            port: Serial port for MyCobot connection
            baudrate: Baudrate for serial communication
        """
        self.mc = MyCobot(port, baudrate)
        time.sleep(0.5)  # Allow connection to stabilize

        # Prefer "refresh" mode so a new target or stop() takes effect
        # immediately instead of queuing behind earlier commands.
        self._configure_fresh_mode(1)

        # Joint limits (degrees) for MyCobot 280, per the official API docs
        # (MyCobot_280_en.md). Note joint 5 is asymmetric.
        self.joint_limits = {
            1: (-168, 168),   # Joint 1 (Base)
            2: (-135, 135),   # Joint 2 (Shoulder)
            3: (-150, 150),   # Joint 3 (Elbow)
            4: (-145, 145),   # Joint 4 (Wrist 1)
            5: (-155, 160),   # Joint 5 (Wrist 2)
            6: (-180, 180)    # Joint 6 (Wrist 3)
        }

        # Gripper limits and valid values
        self.gripper_value_limits = (0, 100)
        self.valid_gripper_states = {0, 1, 10}  # 0=open, 1=close, 10=release

        # Cartesian workspace limits for the MyCobot 280, from pymycobot
        # robot_info.py (key "MyCobot280"): [x, y, z] in mm, [rx, ry, rz] in deg.
        self.coords_min = [-350.0, -350.0, -70.0, -180.0, -180.0, -180.0]
        self.coords_max = [350.0, 350.0, 523.9, 180.0, 180.0, 180.0]

        # Last commanded target (6-element vector), used by wait_for_completion.
        # last-write-wins: every motion command overwrites it.
        self.last_target: Optional[List[float]] = None

        # Last commanded Cartesian target, used by wait_for_coords_completion.
        self.last_coords_target: Optional[List[float]] = None

        # Last sampled (timestamp, angles) for stateful is_moving computation.
        self._last_angle_sample: Optional[Tuple[float, List[float]]] = None
    
    def _validate_joint_number(self, joint_num: int) -> None:
        """Validate joint number is within valid range."""
        if joint_num not in range(1, 7):
            raise ValueError(f"Joint number must be between 1-6, got {joint_num}")
    
    def _validate_angle(self, joint_num: int, angle: float) -> None:
        """Validate angle is within joint limits."""
        self._validate_joint_number(joint_num)
        min_angle, max_angle = self.joint_limits[joint_num]
        if not (min_angle <= angle <= max_angle):
            raise ValueError(f"Angle {angle} out of range for joint {joint_num}. "
                           f"Valid range: {min_angle} to {max_angle} degrees")

    def _validate_speed(self, speed: int) -> None:
        """Validate speed is within valid range."""
        if not (1 <= speed <= 100):
            raise ValueError("Speed must be between 1-100")

    def _validate_gripper_value(self, value: int) -> None:
        """Validate gripper opening value is within valid range."""
        min_value, max_value = self.gripper_value_limits
        if not (min_value <= value <= max_value):
            raise ValueError(
                f"Gripper value must be between {min_value}-{max_value}, got {value}"
            )

    def _validate_gripper_type(self, gripper_type: Optional[int], allow_dexterous: bool = True) -> None:
        """Validate gripper type if provided."""
        if gripper_type is None:
            return

        valid_types = {1, 2, 3, 4} if allow_dexterous else {1, 3, 4}
        if gripper_type not in valid_types:
            raise ValueError(f"Invalid gripper_type {gripper_type}. Valid values: {sorted(valid_types)}")

    def _configure_fresh_mode(self, mode: int = 1) -> None:
        """Set motion mode (1=refresh/latest-first, 0=interpolation/queue).

        Guarded: older firmware/pymycobot versions may not expose
        set_fresh_mode, so a missing method or failure is ignored.
        """
        setter = getattr(self.mc, "set_fresh_mode", None)
        if setter is None:
            return
        try:
            setter(mode)
        except Exception:
            pass

    @classmethod
    def _error_message(cls, code: int) -> str:
        """Map a firmware error code to a human-readable message."""
        if code in cls._ERROR_CODE_MESSAGES:
            return cls._ERROR_CODE_MESSAGES[code]
        if 1 <= code <= 6:
            return f"Joint {code} limit exceeded"
        if 16 <= code <= 19:
            return f"Collision protection triggered (code {code})"
        return f"Robot error code {code}"

    def describe_error(self) -> Tuple[Optional[int], Optional[str]]:
        """Return (code, message) from get_error_information(), or (None, None).

        Returns (None, None) when the firmware reports no error, the call fails,
        or the library/firmware does not support error reporting.
        """
        getter = getattr(self.mc, "get_error_information", None)
        if getter is None:
            return None, None
        try:
            code = getter()
        except Exception:
            return None, None
        if code is None or code in (-1, 0):
            return None, None
        return code, self._error_message(code)

    def _force_stop(self, retries: int = 2) -> None:
        """Stop the robot and confirm it actually stopped.

        mc.stop() returns 1 (stopped), 0 (not stopped), or -1 (error). If the
        firmware reports not-stopped, re-issue a few times so the robot does not
        keep moving (observed during go_home in testing).
        """
        result = self.mc.stop()
        attempts = 0
        while result == 0 and attempts < retries:
            time.sleep(0.05)
            result = self.mc.stop()
            attempts += 1

    def _set_single_joint_target(self, joint_num: int, angle: float) -> None:
        """Record the last target as the current angles with one joint replaced.

        Reads the current full pose so that wait_for_completion can compare all
        six joints; only the moved joint changes, the rest stay where they are.
        """
        target = list(self.get_all_joint_angles())
        target[joint_num - 1] = angle
        self.last_target = target

    def move_joint(self, joint_num: int, angle: float, speed: int = 50) -> None:
        """
        Move a specific joint to target angle.

        Args:
            joint_num: Joint number (1-6)
            angle: Target angle in degrees
            speed: Movement speed (1-100)
        """
        self._validate_angle(joint_num, angle)
        self._validate_speed(speed)

        self._set_single_joint_target(joint_num, angle)
        self.mc.send_angle(joint_num, angle, speed)
    
    def get_joint_angle(self, joint_num: int) -> float:
        """
        Get current angle of a specific joint.
        
        Args:
            joint_num: Joint number (1-6)
            
        Returns:
            Current angle in degrees
        """
        self._validate_joint_number(joint_num)
        # NOTE: pymycobot's MyCobot has no get_angle() (singular); read the full
        # pose with get_angles() and index it.
        angles = self.get_all_joint_angles()
        return angles[joint_num - 1]
    
    def move_joint_1(self, angle: float, speed: int = 50) -> None:
        """Move Joint 1 (Base) to specified angle."""
        self.move_joint(1, angle, speed)
    
    def move_joint_2(self, angle: float, speed: int = 50) -> None:
        """Move Joint 2 (Shoulder) to specified angle."""
        self.move_joint(2, angle, speed)
    
    def move_joint_3(self, angle: float, speed: int = 50) -> None:
        """Move Joint 3 (Elbow) to specified angle."""
        self.move_joint(3, angle, speed)
    
    def move_joint_4(self, angle: float, speed: int = 50) -> None:
        """Move Joint 4 (Wrist 1) to specified angle."""
        self.move_joint(4, angle, speed)
    
    def move_joint_5(self, angle: float, speed: int = 50) -> None:
        """Move Joint 5 (Wrist 2) to specified angle."""
        self.move_joint(5, angle, speed)
    
    def move_joint_6(self, angle: float, speed: int = 50) -> None:
        """Move Joint 6 (Wrist 3) to specified angle."""
        self.move_joint(6, angle, speed)

    def set_gripper_state(self, flag: int, speed: int = 50, gripper_type: Optional[int] = None) -> None:
        """
        Set gripper state.

        Args:
            flag: 0=open, 1=close, 10=release
            speed: Movement speed (1-100)
            gripper_type: 1=adaptive, 2=5-finger dexterous, 3=parallel, 4=flexible
        """
        if flag not in self.valid_gripper_states:
            raise ValueError("Gripper state must be 0 (open), 1 (close), or 10 (release)")

        self._validate_speed(speed)
        self._validate_gripper_type(gripper_type, allow_dexterous=True)

        # Translate the public state to the firmware flag (release 10 -> 254).
        firmware_flag = self._FIRMWARE_GRIPPER_FLAGS[flag]

        if gripper_type is None:
            self.mc.set_gripper_state(firmware_flag, speed)
        else:
            self.mc.set_gripper_state(firmware_flag, speed, gripper_type)

    def open_gripper(self, speed: int = 50, gripper_type: Optional[int] = None) -> None:
        """Open gripper."""
        self.set_gripper_state(0, speed, gripper_type)

    def close_gripper(self, speed: int = 50, gripper_type: Optional[int] = None) -> None:
        """Close gripper."""
        self.set_gripper_state(1, speed, gripper_type)

    def release_gripper(self, speed: int = 50, gripper_type: Optional[int] = None) -> None:
        """Release gripper (torque off behavior supported by firmware)."""
        self.set_gripper_state(10, speed, gripper_type)

    def set_gripper_value(self, value: int, speed: int = 50, gripper_type: Optional[int] = None) -> None:
        """
        Set gripper opening value.

        Args:
            value: Opening value (0-100)
            speed: Movement speed (1-100)
            gripper_type: 1=adaptive, 3=parallel, 4=flexible
        """
        self._validate_gripper_value(value)
        self._validate_speed(speed)
        self._validate_gripper_type(gripper_type, allow_dexterous=False)

        if gripper_type is None:
            self.mc.set_gripper_value(value, speed)
        else:
            self.mc.set_gripper_value(value, speed, gripper_type)

    def calibrate_gripper(self) -> None:
        """Calibrate gripper and set current position as reference."""
        self.mc.set_gripper_calibration()

    def get_gripper_value(self, gripper_type: Optional[int] = None) -> int:
        """
        Read the current gripper opening value (0-100).

        Args:
            gripper_type: 1=adaptive, 3=parallel, 4=flexible (firmware default 1)
        """
        self._validate_gripper_type(gripper_type, allow_dexterous=False)
        getter = getattr(self.mc, "get_gripper_value", None)
        if getter is None:
            raise NotImplementedError(
                "get_gripper_value is not available in this pymycobot/firmware version"
            )
        if gripper_type is None:
            return getter()
        return getter(gripper_type)

    def is_gripper_moving(self) -> bool:
        """Return True if the gripper is currently moving.

        Firmware returns 1 (moving), 0 (stationary), or -1 (error); -1/None are
        treated as not moving.
        """
        getter = getattr(self.mc, "is_gripper_moving", None)
        if getter is None:
            raise NotImplementedError(
                "is_gripper_moving is not available in this pymycobot/firmware version"
            )
        result = getter()
        if result == -1 or result is None:
            return False
        return bool(result)
    
    def get_all_joint_angles(self) -> List[float]:
        """Get current angles of all joints.

        Retries a few times: pymycobot's get_angles() can intermittently return
        -1/None or raise on a transient serial read. After retries are exhausted
        it raises RuntimeError (rather than fabricating an all-zeros home pose)
        so the REST layer surfaces a 503 and motion/IK logic never acts on fake
        angles. Callers that can tolerate a miss (poll loops) should catch it.
        """
        for _ in range(3):
            try:
                angles = self.mc.get_angles()
            except Exception:
                angles = None
            if isinstance(angles, (list, tuple)) and len(angles) == 6:
                return list(angles)
            time.sleep(0.05)
        raise RuntimeError("Failed to read joint angles from robot")
    
    def move_all_joints(self, angles: List[float], speed: int = 50) -> None:
        """
        Move all joints to specified angles simultaneously.
        
        Args:
            angles: List of 6 angles in degrees [J1, J2, J3, J4, J5, J6]
            speed: Movement speed (1-100)
        """
        if len(angles) != 6:
            raise ValueError("Must provide exactly 6 angles")

        self._validate_speed(speed)

        for i, angle in enumerate(angles, 1):
            self._validate_angle(i, angle)

        self.last_target = list(angles)
        self.mc.send_angles(angles, speed)

    def home_position(self, speed: int = 50) -> None:
        """Move all joints to home position (0 degrees)."""
        home_angles = [0, 0, 0, 0, 0, 0]
        self.move_all_joints(home_angles, speed)

    def joint_jog(self, joint_num: int, direction: int, speed: int = 50,
                  increment: float = 5.0) -> float:
        """
        Jog a joint by a fixed increment, non-blocking.

        Reads the current angle and sends an absolute incremental move via
        send_angle (like move_joint), so the call returns immediately instead
        of blocking on the firmware's continuous-jog API.

        Args:
            joint_num: Joint number (1-6)
            direction: 1 for positive direction, -1 for negative
            speed: Movement speed (1-100)
            increment: Degrees to move per call (0 < increment <= 90)

        Returns:
            The clamped absolute target angle the joint was sent to.
        """
        self._validate_joint_number(joint_num)
        if direction not in [-1, 1]:
            raise ValueError("Direction must be 1 or -1")
        self._validate_speed(speed)
        if not (0 < increment <= 90):
            raise ValueError(f"Increment must be in (0, 90], got {increment}")

        # Read the full pose once: gives the current angle and the basis for
        # last_target (other joints stay put).
        angles = list(self.get_all_joint_angles())
        current = angles[joint_num - 1]
        min_angle, max_angle = self.joint_limits[joint_num]
        # Clamp the target into the joint's safe range (report 4(b)).
        target = max(min_angle, min(max_angle, current + direction * increment))

        angles[joint_num - 1] = target
        self.last_target = angles
        self.mc.send_angle(joint_num, target, speed)
        return target
    
    def stop_joint(self, joint_num: int) -> None:
        """Stop movement of a specific joint."""
        self._validate_joint_number(joint_num)
        self._force_stop()

    def stop_all_joints(self) -> None:
        """Stop movement of all joints."""
        self._force_stop()
    
    def compute_is_moving(self, current_angles: Optional[List[float]] = None,
                          eps: float = 0.5) -> bool:
        """Decide whether the robot is moving by comparing successive samples.

        Stateful and latency-free: compares the current angles against the
        previously stored sample (updated each call). Avoids the firmware
        is_moving() flag, which can stay stuck at True after a move finishes.

        Args:
            current_angles: Current joint angles; fetched if not supplied.
            eps: Per-joint angle change (degrees) above which we call it moving.

        Returns:
            True if any joint moved more than eps since the previous sample.
        """
        if current_angles is None:
            current_angles = self.get_all_joint_angles()

        now = time.monotonic()
        prev = self._last_angle_sample
        self._last_angle_sample = (now, list(current_angles))

        if prev is None:
            # No baseline yet; can't tell, assume not moving.
            return False

        _, prev_angles = prev
        if len(prev_angles) != len(current_angles):
            return False
        max_delta = max(abs(c - p) for c, p in zip(current_angles, prev_angles))
        return max_delta > eps

    def wait_for_completion(self, target: Optional[List[float]] = None,
                            tol: float = 1.0, timeout: float = 15.0,
                            poll: float = 0.1, stall_window: float = 2.5,
                            stall_min_progress: float = 0.3) -> dict:
        """
        Wait for the robot to reach the target, with stall and timeout guards.

        Resolves three terminal conditions:
          1. converged  - all joints within `tol` of target -> success, no stop
          2. stalled     - net motion in the recent window below threshold while
                           not converged -> stop the robot, failure
          3. timeout     - exceeded `timeout` while not converged -> stop, failure

        Args:
            target: Target angles; falls back to self.last_target. If neither is
                available, degrades to legacy is_moving() polling.
            tol: Per-joint tolerance in degrees for convergence.
            timeout: Hard upper bound in seconds.
            poll: Polling interval in seconds.
            stall_window: Window (seconds) over which net progress is measured.
            stall_min_progress: Minimum net progress (degrees) within the window
                before a non-converged pose is declared stalled.

        Returns:
            dict with keys: completed (bool), reason (str), elapsed (float),
            max_error (float | None).
        """
        if target is None:
            target = self.last_target

        start = time.monotonic()

        # Fallback: no known target -> legacy is_moving() polling.
        if target is None:
            while time.monotonic() - start < timeout:
                if not self.mc.is_moving():
                    return {"completed": True, "reason": "idle",
                            "elapsed": time.monotonic() - start, "max_error": None}
                time.sleep(poll)
            self._force_stop()
            return {"completed": False, "reason": "timeout",
                    "elapsed": time.monotonic() - start, "max_error": None}

        history = deque()  # (timestamp, angles)
        while True:
            now = time.monotonic()
            try:
                cur = self.get_all_joint_angles()
            except RuntimeError:
                cur = None
            if cur is None or len(cur) != len(target):
                # Transient read failure; retry until timeout.
                if now - start > timeout:
                    self._force_stop()
                    return {"completed": False, "reason": "timeout",
                            "elapsed": now - start, "max_error": None}
                time.sleep(poll)
                continue

            err = max(abs(c - t) for c, t in zip(cur, target))

            # 1) converged -> success (already stopped, no stop needed)
            if err <= tol:
                return {"completed": True, "reason": "converged",
                        "elapsed": now - start, "max_error": err}

            # 2) stall detection (net progress within the recent window)
            history.append((now, list(cur)))
            while history and now - history[0][0] > stall_window:
                history.popleft()
            if now - start > stall_window and len(history) >= 2:
                progress = max(abs(c - h) for c, h in zip(cur, history[0][1]))
                if progress < stall_min_progress:
                    self._force_stop()
                    return {"completed": False, "reason": "stalled",
                            "elapsed": now - start, "max_error": err}

            # 3) timeout -> cancel
            if now - start > timeout:
                self._force_stop()
                return {"completed": False, "reason": "timeout",
                        "elapsed": now - start, "max_error": err}

            time.sleep(poll)

    # ------------------------------------------------------------------
    # Cartesian (coordinate-space) control
    # ------------------------------------------------------------------

    # Labels for the 6-element coords vector [x, y, z, rx, ry, rz].
    _COORD_LABELS = ("x", "y", "z", "rx", "ry", "rz")

    @staticmethod
    def _angle_delta(a: float, b: float) -> float:
        """Smallest absolute difference between two angles in degrees (wraps ±180)."""
        return abs((a - b + 180.0) % 360.0 - 180.0)

    @staticmethod
    def _wrap_angle(a: float) -> float:
        """Normalize an angle into [-180, 180)."""
        return (a + 180.0) % 360.0 - 180.0

    def _read_coords(self, retries: int = 3) -> Optional[List[float]]:
        """Read the current Cartesian pose, or None on failure.

        pymycobot 4.0.0 get_coords() returns [x, y, z, rx, ry, rz] (mm / deg),
        or the int -1 (read retries exhausted) / None (frame parse failure).
        Any of those, a wrong length, or an exception is reported as None so the
        caller can retry instead of acting on a bad reading. Note -1 is truthy,
        so an isinstance/len check (not `if not coords`) is required.
        """
        for _ in range(max(1, retries)):
            try:
                coords = self.mc.get_coords()
            except Exception:
                coords = None
            if isinstance(coords, (list, tuple)) and len(coords) == 6:
                return list(coords)
            time.sleep(0.05)
        return None

    def get_coords(self, retries: int = 3) -> List[float]:
        """Return the current Cartesian pose [x, y, z, rx, ry, rz] (mm / deg).

        Raises RuntimeError if the pose cannot be read, so the REST layer
        surfaces a 503 instead of a fabricated pose.
        """
        coords = self._read_coords(retries=retries)
        if coords is None:
            raise RuntimeError("Failed to read coordinates from robot")
        return coords

    def _validate_coords(self, coords: List[float]) -> None:
        """Validate a 6-element Cartesian target against the 280 workspace box."""
        if len(coords) != 6:
            raise ValueError(
                f"Must provide exactly 6 coords [x,y,z,rx,ry,rz], got {len(coords)}"
            )
        for value, lo, hi, label in zip(coords, self.coords_min,
                                        self.coords_max, self._COORD_LABELS):
            if not (lo <= value <= hi):
                raise ValueError(
                    f"Coord {label}={value} out of workspace range [{lo}, {hi}]"
                )

    def send_coords(self, coords: List[float], speed: int = 30, mode: int = 0) -> None:
        """Move the end-effector to an absolute Cartesian pose (non-blocking).

        Args:
            coords: [x, y, z, rx, ry, rz] in mm / degrees.
            speed: 1-100.
            mode: 0 = angular (point-to-point, default), 1 = linear (straight line).

        send_coords does NOT verify reachability; an in-range but unreachable
        pose silently does not move (firmware error 32). Use check_pose_reachable()
        before and wait_for_coords_completion() after.
        """
        coords = list(coords)
        # Wrap the orientation triple into [-180, 180) so a yaw computed slightly
        # past the boundary (e.g. 180.5) is accepted instead of rejected.
        if len(coords) == 6:
            coords[3:] = [self._wrap_angle(a) for a in coords[3:]]
        self._validate_coords(coords)
        self._validate_speed(speed)
        if mode not in (0, 1):
            raise ValueError("mode must be 0 (angular) or 1 (linear)")
        self.last_coords_target = list(coords)
        # pymycobot 4.0.0: always pass mode explicitly (omitting it sends no
        # mode byte, which is undefined firmware behaviour).
        self.mc.send_coords(list(coords), speed, mode)

    def solve_ik(self, target_coords: List[float],
                 current_angles: Optional[List[float]] = None,
                 retries: int = 3) -> Optional[List[float]]:
        """Return a 6-joint IK solution (degrees) for target_coords, or None.

        Wraps pymycobot solve_inv_kinematics (firmware IK), which returns a
        6-element angle list on success, or the int -1 / None when there is no
        solution OR on a transient read failure; we retry and report None for
        all non-solution outcomes.
        """
        if len(target_coords) != 6:
            raise ValueError("target_coords must have 6 elements")
        solver = getattr(self.mc, "solve_inv_kinematics", None)
        if solver is None:
            return None
        for _ in range(max(1, retries)):
            try:
                seed = (self.get_all_joint_angles() if current_angles is None
                        else list(current_angles))
                result = solver(list(target_coords), seed)
            except Exception:
                result = None
            if isinstance(result, (list, tuple)) and len(result) == 6:
                return list(result)
            time.sleep(0.05)
        return None

    def check_pose_reachable(self, coords: List[float]
                             ) -> Tuple[bool, str, Optional[List[float]]]:
        """Best-effort, no-move reachability check for a Cartesian pose.

        Returns (reachable, reason, ik_angles):
          - (False, "out_of_range",  None)     outside the workspace box,
          - (False, "joint_limit",   [angles]) IK solved but a joint exceeds its limit,
          - (True,  "ok",            [angles]) IK solved and within joint limits,
          - (True,  "ok_unverified", None)     in-box but firmware IK returned no
                                               solution, so reachability cannot be
                                               confirmed before moving.

        Note: this unit's firmware solve_inv_kinematics returns -1 for EVERY
        pose (IK unsupported), so in practice every in-box pose is reported
        "ok_unverified" and the authoritative reachability verdict comes at run
        time from the error-32 (no IK solution) check in
        wait_for_coords_completion. The IK branch is kept so the check upgrades
        automatically if a firmware with working IK is ever installed.
        """
        coords = list(coords)
        if len(coords) == 6:
            coords[3:] = [self._wrap_angle(a) for a in coords[3:]]
        try:
            self._validate_coords(coords)
        except ValueError:
            return False, "out_of_range", None
        ik = self.solve_ik(coords)
        if ik is None:
            return True, "ok_unverified", None
        for i, angle in enumerate(ik, 1):
            lo, hi = self.joint_limits[i]
            if not (lo <= angle <= hi):
                return False, "joint_limit", ik
        return True, "ok", ik

    def wait_for_coords_completion(self, target_coords: Optional[List[float]] = None,
                                   pos_tol: float = 3.0, ori_tol: float = 3.0,
                                   timeout: float = 20.0, poll: float = 0.15,
                                   stall_window: float = 2.5,
                                   stall_min_progress: float = 1.0,
                                   stall_min_ori_progress: float = 0.5) -> dict:
        """Wait until the end-effector reaches target_coords (mm / deg).

        Mirrors wait_for_completion but in Cartesian space. Terminal conditions:
          1. converged      - position within pos_tol (mm) AND orientation within
                              ori_tol (deg) -> success, no stop
          2. ik_no_solution - firmware error 32/33/34 observed -> stop, failure
          3. stalled        - net progress (position mm AND orientation deg)
                              below threshold while not converged -> stop, failure
          4. timeout        - exceeded timeout while not converged -> stop, failure
          5. no_target      - no target known and none was ever commanded

        Returns dict: completed, reason, elapsed, pos_error, ori_error.

        Caveat: orientation error is a per-axis Euler comparison, not a true
        SO(3) metric; near gimbal singularities (rx ≈ ±90, which the 280's home
        pose sits at) it can overstate the error and delay convergence. Adequate
        for v1 top-down grasps; revisit with quaternions if needed.
        """
        if target_coords is None:
            target_coords = self.last_coords_target

        start = time.monotonic()

        # No known target: we cannot judge Cartesian convergence, so report it
        # honestly rather than polling the unreliable is_moving() flag (which can
        # stick at True and waste the whole timeout before a spurious stop).
        if target_coords is None:
            return {"completed": False, "reason": "no_target",
                    "elapsed": time.monotonic() - start,
                    "pos_error": None, "ori_error": None}

        history = deque()  # (timestamp, [x, y, z])
        last_err_check = 0.0
        while True:
            now = time.monotonic()
            cur = self._read_coords(retries=2)
            if cur is None:
                # Transient read failure; retry until timeout.
                if now - start > timeout:
                    self._force_stop()
                    return {"completed": False, "reason": "timeout",
                            "elapsed": now - start,
                            "pos_error": None, "ori_error": None}
                time.sleep(poll)
                continue

            pos_error = max(abs(c - t) for c, t in zip(cur[:3], target_coords[:3]))
            ori_error = max(self._angle_delta(c, t)
                            for c, t in zip(cur[3:], target_coords[3:]))

            # 1) converged -> success (already stopped, no stop needed)
            if pos_error <= pos_tol and ori_error <= ori_tol:
                return {"completed": True, "reason": "converged",
                        "elapsed": now - start,
                        "pos_error": pos_error, "ori_error": ori_error}

            # 2) firmware fault -> stop early (throttled poll). 32/33/34 are
            #    IK / linear-motion no-solution; 1-6 (joint limit) and 16-19
            #    (collision) are surfaced as a generic robot_fault so the caller
            #    is warned to inspect the arm instead of seeing a bare timeout.
            if now - last_err_check > 1.0:
                last_err_check = now
                code, _ = self.describe_error()
                if code in (32, 33, 34):
                    self._force_stop()
                    return {"completed": False, "reason": "ik_no_solution",
                            "elapsed": now - start,
                            "pos_error": pos_error, "ori_error": ori_error}
                if code:
                    self._force_stop()
                    return {"completed": False, "reason": "robot_fault",
                            "elapsed": now - start,
                            "pos_error": pos_error, "ori_error": ori_error}

            # 3) stall detection: net progress over the recent window in BOTH
            #    position (mm) and orientation (deg). Both must be below their
            #    thresholds to count as stalled, so a pure-reorientation move
            #    (XYZ ~constant) is not falsely flagged.
            history.append((now, list(cur)))
            while history and now - history[0][0] > stall_window:
                history.popleft()
            if now - start > stall_window and len(history) >= 2:
                ref = history[0][1]
                pos_progress = max(abs(c - h) for c, h in zip(cur[:3], ref[:3]))
                ori_progress = max(self._angle_delta(c, h)
                                   for c, h in zip(cur[3:], ref[3:]))
                if pos_progress < stall_min_progress and ori_progress < stall_min_ori_progress:
                    self._force_stop()
                    return {"completed": False, "reason": "stalled",
                            "elapsed": now - start,
                            "pos_error": pos_error, "ori_error": ori_error}

            # 4) timeout -> cancel
            if now - start > timeout:
                self._force_stop()
                return {"completed": False, "reason": "timeout",
                        "elapsed": now - start,
                        "pos_error": pos_error, "ori_error": ori_error}

            time.sleep(poll)

    def close_connection(self) -> None:
        """Close the serial connection to MyCobot."""
        self.mc.close()


# Convenience functions for direct usage without class instantiation
def create_controller(port: str = "/dev/ttyACM0", baudrate: int = 115200) -> MyCobotJointController:
    """Create and return a MyCobotJointController instance."""
    return MyCobotJointController(port, baudrate)


# Example usage
if __name__ == "__main__":
    # Initialize controller
    controller = create_controller()
    
    try:
        # Move to home position
        print("Moving to home position...")
        controller.home_position()
        controller.wait_for_completion()
        
        # Move individual joints
        print("Moving Joint 1 to 45 degrees...")
        controller.move_joint_1(45)
        controller.wait_for_completion()
        
        print("Moving Joint 2 to -30 degrees...")
        controller.move_joint_2(-30)
        controller.wait_for_completion()
        
        # Get current joint angles
        current_angles = controller.get_all_joint_angles()
        print(f"Current joint angles: {current_angles}")
        
        # Move all joints simultaneously
        target_angles = [30, -45, 60, 0, -30, 90]
        print(f"Moving all joints to: {target_angles}")
        controller.move_all_joints(target_angles)
        controller.wait_for_completion()

        # Gripper operations
        print("Opening gripper...")
        controller.open_gripper()
        time.sleep(1.0)

        print("Closing gripper...")
        controller.close_gripper()
        time.sleep(1.0)
        
    finally:
        # Return to home and close connection
        controller.home_position()
        controller.wait_for_completion()
        controller.close_connection()
        print("Connection closed.")
