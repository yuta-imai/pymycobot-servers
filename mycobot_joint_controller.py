"""
MyCobot Joint Controller Module

This module provides functions to control individual joints of a MyCobot robot
using the pymycobot library.
"""

from pymycobot import MyCobot
import time
from typing import Union, List, Optional


class MyCobotJointController:
    """Controller class for individual joint operations on MyCobot robot."""
    
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        """
        Initialize MyCobot connection.
        
        Args:
            port: Serial port for MyCobot connection
            baudrate: Baudrate for serial communication
        """
        self.mc = MyCobot(port, baudrate)
        time.sleep(0.5)  # Allow connection to stabilize
        
        # Joint limits (degrees) for MyCobot 280
        self.joint_limits = {
            1: (-165, 165),   # Joint 1 (Base)
            2: (-165, 165),   # Joint 2 (Shoulder)
            3: (-165, 165),   # Joint 3 (Elbow)
            4: (-165, 165),   # Joint 4 (Wrist 1)
            5: (-165, 165),   # Joint 5 (Wrist 2)
            6: (-175, 175)    # Joint 6 (Wrist 3)
        }
    
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
    
    def move_joint(self, joint_num: int, angle: float, speed: int = 50) -> None:
        """
        Move a specific joint to target angle.
        
        Args:
            joint_num: Joint number (1-6)
            angle: Target angle in degrees
            speed: Movement speed (1-100)
        """
        self._validate_angle(joint_num, angle)
        if not (1 <= speed <= 100):
            raise ValueError("Speed must be between 1-100")
        
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
        return self.mc.get_angle(joint_num)
    
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
    
    def get_all_joint_angles(self) -> List[float]:
        """Get current angles of all joints."""
        return self.mc.get_angles()
    
    def move_all_joints(self, angles: List[float], speed: int = 50) -> None:
        """
        Move all joints to specified angles simultaneously.
        
        Args:
            angles: List of 6 angles in degrees [J1, J2, J3, J4, J5, J6]
            speed: Movement speed (1-100)
        """
        if len(angles) != 6:
            raise ValueError("Must provide exactly 6 angles")
        
        for i, angle in enumerate(angles, 1):
            self._validate_angle(i, angle)
        
        self.mc.send_angles(angles, speed)
    
    def home_position(self, speed: int = 50) -> None:
        """Move all joints to home position (0 degrees)."""
        home_angles = [0, 0, 0, 0, 0, 0]
        self.move_all_joints(home_angles, speed)
    
    def joint_jog(self, joint_num: int, direction: int, speed: int = 50) -> None:
        """
        Jog a joint in specified direction.
        
        Args:
            joint_num: Joint number (1-6)
            direction: 1 for positive direction, -1 for negative
            speed: Movement speed (1-100)
        """
        self._validate_joint_number(joint_num)
        if direction not in [-1, 1]:
            raise ValueError("Direction must be 1 or -1")
        
        self.mc.jog_angle(joint_num, direction, speed)
    
    def stop_joint(self, joint_num: int) -> None:
        """Stop movement of a specific joint."""
        self._validate_joint_number(joint_num)
        self.mc.stop()
    
    def stop_all_joints(self) -> None:
        """Stop movement of all joints."""
        self.mc.stop()
    
    def wait_for_completion(self, timeout: float = 10.0) -> bool:
        """
        Wait for robot to complete current movement.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if movement completed, False if timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self.mc.is_moving():
                return True
            time.sleep(0.1)
        return False
    
    def close_connection(self) -> None:
        """Close the serial connection to MyCobot."""
        self.mc.close()


# Convenience functions for direct usage without class instantiation
def create_controller(port: str = "/dev/ttyUSB0", baudrate: int = 115200) -> MyCobotJointController:
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
        
    finally:
        # Return to home and close connection
        controller.home_position()
        controller.wait_for_completion()
        controller.close_connection()
        print("Connection closed.")