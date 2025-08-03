"""
MyCobot MCP Server

Model Context Protocol (MCP) server for MyCobot robot control.
Provides tools for external systems to control the robot through REST API.
"""

import asyncio
import json
import logging
import sys
from typing import Any, Dict, List, Optional, Union
import argparse
import aiohttp

from mcp.server import Server
from mcp.server.models import InitializationOptions
from mcp.server.stdio import stdio_server
from mcp.types import (
    CallToolRequest,
    CallToolResult,
    GetPromptRequest,
    GetPromptResult,
    ListPromptsRequest,
    ListPromptsResult,
    ListResourcesRequest,
    ListResourcesResult,
    ListToolsRequest,
    ListToolsResult,
    Prompt,
    PromptMessage,
    Resource,
    Tool,
    TextContent,
    ImageContent,
    EmbeddedResource
)


# Global API client session and base URL
api_session: Optional[aiohttp.ClientSession] = None
api_base_url: str = "http://localhost:8080"

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def ensure_api_session():
    """Ensure API session is initialized."""
    global api_session
    if api_session is None:
        raise RuntimeError("API session not initialized")


async def get_joint_angle(joint_num: int) -> Dict[str, Any]:
    """Get current angle of a specific joint."""
    ensure_api_session()
    
    if not (1 <= joint_num <= 6):
        raise ValueError(f"Joint number must be between 1-6, got {joint_num}")
    
    async with api_session.get(f"{api_base_url}/joints/{joint_num}/angle") as response:
        if response.status != 200:
            error_text = await response.text()
            raise RuntimeError(f"API request failed: {response.status} - {error_text}")
        
        data = await response.json()
        return {
            "joint_num": joint_num,
            "angle": data["angle"],
            "unit": "degrees"
        }


async def move_joint(joint_num: int, angle: float, speed: int = 50) -> Dict[str, Any]:
    """Move a specific joint to target angle."""
    ensure_api_session()
    
    if not (1 <= joint_num <= 6):
        raise ValueError(f"Joint number must be between 1-6, got {joint_num}")
    
    payload = {"angle": angle, "speed": speed}
    async with api_session.put(f"{api_base_url}/joints/{joint_num}/angle", json=payload) as response:
        if response.status != 200:
            error_text = await response.text()
            raise RuntimeError(f"API request failed: {response.status} - {error_text}")
        
        return {
            "success": True,
            "message": f"Joint {joint_num} moving to {angle} degrees at speed {speed}",
            "joint_num": joint_num,
            "target_angle": angle,
            "speed": speed
        }


async def get_all_joint_angles() -> Dict[str, Any]:
    """Get current angles of all joints."""
    ensure_api_session()
    
    async with api_session.get(f"{api_base_url}/joints/angles") as response:
        if response.status != 200:
            error_text = await response.text()
            raise RuntimeError(f"API request failed: {response.status} - {error_text}")
        
        data = await response.json()
        angles = data["angles"]
        return {
            "joint_angles": angles,
            "joints": [
                {"joint": 1, "angle": angles[0], "name": "Base"},
                {"joint": 2, "angle": angles[1], "name": "Shoulder"},
                {"joint": 3, "angle": angles[2], "name": "Elbow"},
                {"joint": 4, "angle": angles[3], "name": "Wrist 1"},
                {"joint": 5, "angle": angles[4], "name": "Wrist 2"},
                {"joint": 6, "angle": angles[5], "name": "Wrist 3"}
            ],
            "unit": "degrees"
        }


async def move_all_joints(angles: List[float], speed: int = 50) -> Dict[str, Any]:
    """Move all joints to specified angles simultaneously."""
    ensure_api_session()
    
    if len(angles) != 6:
        raise ValueError("Must provide exactly 6 angles")
    
    payload = {"angles": angles, "speed": speed}
    async with api_session.put(f"{api_base_url}/joints/angles", json=payload) as response:
        if response.status != 200:
            error_text = await response.text()
            raise RuntimeError(f"API request failed: {response.status} - {error_text}")
        
        return {
            "success": True,
            "message": f"All joints moving to {angles} at speed {speed}",
            "target_angles": angles,
            "speed": speed
        }


async def home_position(speed: int = 50) -> Dict[str, Any]:
    """Move all joints to home position (0 degrees)."""
    ensure_api_session()
    
    payload = {"speed": speed}
    async with api_session.post(f"{api_base_url}/robot/home", json=payload) as response:
        if response.status != 200:
            error_text = await response.text()
            raise RuntimeError(f"API request failed: {response.status} - {error_text}")
        
        return {
            "success": True,
            "message": f"Moving to home position at speed {speed}",
            "target_angles": [0, 0, 0, 0, 0, 0],
            "speed": speed
        }


async def stop_robot() -> Dict[str, Any]:
    """Emergency stop for all joint movements."""
    ensure_api_session()
    
    async with api_session.post(f"{api_base_url}/robot/stop") as response:
        if response.status != 200:
            error_text = await response.text()
            raise RuntimeError(f"API request failed: {response.status} - {error_text}")
        
        return {
            "success": True,
            "message": "All joints stopped"
        }


async def jog_joint(joint_num: int, direction: int, speed: int = 50) -> Dict[str, Any]:
    """Jog a joint in specified direction."""
    ensure_api_session()
    
    if not (1 <= joint_num <= 6):
        raise ValueError(f"Joint number must be between 1-6, got {joint_num}")
    
    if direction not in [-1, 1]:
        raise ValueError("Direction must be 1 or -1")
    
    payload = {"direction": direction, "speed": speed}
    async with api_session.post(f"{api_base_url}/joints/{joint_num}/jog", json=payload) as response:
        if response.status != 200:
            error_text = await response.text()
            raise RuntimeError(f"API request failed: {response.status} - {error_text}")
        
        direction_str = "positive" if direction == 1 else "negative"
        return {
            "success": True,
            "message": f"Joint {joint_num} jogging in {direction_str} direction at speed {speed}",
            "joint_num": joint_num,
            "direction": direction,
            "speed": speed
        }


async def wait_for_completion(timeout: float = 10.0) -> Dict[str, Any]:
    """Wait for robot to complete current movement."""
    ensure_api_session()
    
    payload = {"timeout": timeout}
    async with api_session.post(f"{api_base_url}/robot/wait", json=payload) as response:
        if response.status != 200:
            error_text = await response.text()
            raise RuntimeError(f"API request failed: {response.status} - {error_text}")
        
        data = await response.json()
        return {
            "completed": data["completed"],
            "elapsed_time": data["elapsed_time"],
            "timeout": timeout
        }


async def get_robot_status() -> Dict[str, Any]:
    """Get current robot status including joint angles and movement state."""
    ensure_api_session()
    
    async with api_session.get(f"{api_base_url}/robot/status") as response:
        if response.status != 200:
            error_text = await response.text()
            raise RuntimeError(f"API request failed: {response.status} - {error_text}")
        
        data = await response.json()
        joint_angles = data["joint_angles"]
        is_moving = data["is_moving"]
        
        return {
            "joint_angles": joint_angles,
            "is_moving": is_moving,
            "joints": [
                {"joint": 1, "angle": joint_angles[0], "name": "Base"},
                {"joint": 2, "angle": joint_angles[1], "name": "Shoulder"},
                {"joint": 3, "angle": joint_angles[2], "name": "Elbow"},
                {"joint": 4, "angle": joint_angles[3], "name": "Wrist 1"},
                {"joint": 5, "angle": joint_angles[4], "name": "Wrist 2"},
                {"joint": 6, "angle": joint_angles[5], "name": "Wrist 3"}
            ],
            "unit": "degrees"
        }


# Create MCP server
server = Server("mycobot-controller")


@server.list_tools()
async def handle_list_tools() -> List[Tool]:
    """List available tools for MyCobot control."""
    return [
        Tool(
            name="get_joint_angle",
            description="Get current angle of a specific joint (1-6)",
            inputSchema={
                "type": "object",
                "properties": {
                    "joint_num": {
                        "type": "integer",
                        "minimum": 1,
                        "maximum": 6,
                        "description": "Joint number (1-6)"
                    }
                },
                "required": ["joint_num"]
            }
        ),
        Tool(
            name="move_joint",
            description="Move a specific joint to target angle",
            inputSchema={
                "type": "object",
                "properties": {
                    "joint_num": {
                        "type": "integer",
                        "minimum": 1,
                        "maximum": 6,
                        "description": "Joint number (1-6)"
                    },
                    "angle": {
                        "type": "number",
                        "minimum": -175,
                        "maximum": 175,
                        "description": "Target angle in degrees"
                    },
                    "speed": {
                        "type": "integer",
                        "minimum": 1,
                        "maximum": 100,
                        "default": 50,
                        "description": "Movement speed (1-100)"
                    }
                },
                "required": ["joint_num", "angle"]
            }
        ),
        Tool(
            name="get_all_joint_angles",
            description="Get current angles of all joints",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        Tool(
            name="move_all_joints",
            description="Move all joints to specified angles simultaneously",
            inputSchema={
                "type": "object",
                "properties": {
                    "angles": {
                        "type": "array",
                        "items": {
                            "type": "number",
                            "minimum": -175,
                            "maximum": 175
                        },
                        "minItems": 6,
                        "maxItems": 6,
                        "description": "6 joint angles in degrees [J1, J2, J3, J4, J5, J6]"
                    },
                    "speed": {
                        "type": "integer",
                        "minimum": 1,
                        "maximum": 100,
                        "default": 50,
                        "description": "Movement speed (1-100)"
                    }
                },
                "required": ["angles"]
            }
        ),
        Tool(
            name="home_position",
            description="Move all joints to home position (0 degrees)",
            inputSchema={
                "type": "object",
                "properties": {
                    "speed": {
                        "type": "integer",
                        "minimum": 1,
                        "maximum": 100,
                        "default": 50,
                        "description": "Movement speed (1-100)"
                    }
                },
                "required": []
            }
        ),
        Tool(
            name="stop_robot",
            description="Emergency stop for all joint movements",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        Tool(
            name="jog_joint",
            description="Jog a joint in specified direction",
            inputSchema={
                "type": "object",
                "properties": {
                    "joint_num": {
                        "type": "integer",
                        "minimum": 1,
                        "maximum": 6,
                        "description": "Joint number (1-6)"
                    },
                    "direction": {
                        "type": "integer",
                        "enum": [-1, 1],
                        "description": "1 for positive direction, -1 for negative"
                    },
                    "speed": {
                        "type": "integer",
                        "minimum": 1,
                        "maximum": 100,
                        "default": 50,
                        "description": "Movement speed (1-100)"
                    }
                },
                "required": ["joint_num", "direction"]
            }
        ),
        Tool(
            name="wait_for_completion",
            description="Wait for robot to complete current movement",
            inputSchema={
                "type": "object",
                "properties": {
                    "timeout": {
                        "type": "number",
                        "minimum": 0.1,
                        "maximum": 60.0,
                        "default": 10.0,
                        "description": "Maximum time to wait in seconds"
                    }
                },
                "required": []
            }
        ),
        Tool(
            name="get_robot_status",
            description="Get current robot status including joint angles and movement state",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        )
    ]


@server.call_tool()
async def handle_call_tool(name: str, arguments: Dict[str, Any]) -> CallToolResult:
    """Handle tool calls."""
    try:
        if name == "get_joint_angle":
            result = await get_joint_angle(arguments["joint_num"])
        elif name == "move_joint":
            result = await move_joint(
                arguments["joint_num"],
                arguments["angle"],
                arguments.get("speed", 50)
            )
        elif name == "get_all_joint_angles":
            result = await get_all_joint_angles()
        elif name == "move_all_joints":
            result = await move_all_joints(
                arguments["angles"],
                arguments.get("speed", 50)
            )
        elif name == "home_position":
            result = await home_position(arguments.get("speed", 50))
        elif name == "stop_robot":
            result = await stop_robot()
        elif name == "jog_joint":
            result = await jog_joint(
                arguments["joint_num"],
                arguments["direction"],
                arguments.get("speed", 50)
            )
        elif name == "wait_for_completion":
            result = await wait_for_completion(arguments.get("timeout", 10.0))
        elif name == "get_robot_status":
            result = await get_robot_status()
        else:
            raise ValueError(f"Unknown tool: {name}")
        
        return CallToolResult(content=[TextContent(type="text", text=json.dumps(result, indent=2))])
    
    except Exception as e:
        error_result = {
            "error": str(e),
            "tool": name,
            "arguments": arguments
        }
        return CallToolResult(content=[TextContent(type="text", text=json.dumps(error_result, indent=2))])


@server.list_prompts()
async def handle_list_prompts() -> List[Prompt]:
    """List available prompts."""
    return [
        Prompt(
            name="robot_status",
            description="Get comprehensive robot status information",
            arguments=[]
        ),
        Prompt(
            name="joint_info",
            description="Get information about robot joints and their limits",
            arguments=[]
        ),
        Prompt(
            name="basic_movements",
            description="Examples of basic robot movements",
            arguments=[]
        )
    ]


@server.get_prompt()
async def handle_get_prompt(name: str, arguments: Dict[str, str]) -> GetPromptResult:
    """Handle prompt requests."""
    if name == "robot_status":
        try:
            status = await get_robot_status()
            prompt_text = f"""
MyCobot Robot Status:

Current Joint Angles:
- Joint 1 (Base): {status['joints'][0]['angle']:.1f}°
- Joint 2 (Shoulder): {status['joints'][1]['angle']:.1f}°
- Joint 3 (Elbow): {status['joints'][2]['angle']:.1f}°
- Joint 4 (Wrist 1): {status['joints'][3]['angle']:.1f}°
- Joint 5 (Wrist 2): {status['joints'][4]['angle']:.1f}°
- Joint 6 (Wrist 3): {status['joints'][5]['angle']:.1f}°

Robot Movement State: {"Moving" if status['is_moving'] else "Stationary"}
"""
        except Exception as e:
            prompt_text = f"Error getting robot status: {str(e)}"
        
        return GetPromptResult(
            description="Current robot status",
            messages=[
                PromptMessage(
                    role="user",
                    content=TextContent(type="text", text=prompt_text)
                )
            ]
        )
    
    elif name == "joint_info":
        prompt_text = """
MyCobot 280 Joint Information:

Joint Limits (degrees):
- Joint 1 (Base): -165° to +165°
- Joint 2 (Shoulder): -165° to +165°
- Joint 3 (Elbow): -165° to +165°
- Joint 4 (Wrist 1): -165° to +165°
- Joint 5 (Wrist 2): -165° to +165°
- Joint 6 (Wrist 3): -175° to +175°

Movement Speed Range: 1-100 (1=slowest, 100=fastest)
Home Position: All joints at 0°
"""
        return GetPromptResult(
            description="Robot joint information and limits",
            messages=[
                PromptMessage(
                    role="user",
                    content=TextContent(type="text", text=prompt_text)
                )
            ]
        )
    
    elif name == "basic_movements":
        prompt_text = """
MyCobot Basic Movement Examples:

1. Move to Home Position:
   - Use: home_position tool with speed parameter

2. Move Single Joint:
   - Use: move_joint tool with joint_num (1-6), angle (-175 to 175), speed (1-100)

3. Move All Joints:
   - Use: move_all_joints tool with array of 6 angles and speed

4. Get Current Position:
   - Use: get_all_joint_angles or get_joint_angle tools

5. Jog Joint:
   - Use: jog_joint tool with joint_num, direction (1 or -1), speed

6. Emergency Stop:
   - Use: stop_robot tool

7. Wait for Movement:
   - Use: wait_for_completion tool with timeout

Always check robot status before and after movements for safety.
"""
        return GetPromptResult(
            description="Basic robot movement examples and usage",
            messages=[
                PromptMessage(
                    role="user",
                    content=TextContent(type="text", text=prompt_text)
                )
            ]
        )
    
    else:
        raise ValueError(f"Unknown prompt: {name}")


async def main():
    """Main function to run the MCP server."""
    parser = argparse.ArgumentParser(description='MyCobot MCP Server')
    parser.add_argument('--api-host', type=str, default='localhost',
                       help='API server host (default: localhost)')
    parser.add_argument('--api-port', type=int, default=8080,
                       help='API server port (default: 8080)')
    
    args = parser.parse_args()
    
    # Initialize API session
    global api_session, api_base_url
    api_base_url = f"http://{args.api_host}:{args.api_port}"
    
    try:
        api_session = aiohttp.ClientSession()
        
        # Test API connection
        async with api_session.get(f"{api_base_url}/health") as response:
            if response.status == 200:
                health_data = await response.json()
                logger.info(f"Connected to MyCobot API server at {api_base_url}")
                logger.info(f"Robot connected: {health_data.get('robot_connected', False)}")
            else:
                logger.warning(f"API server responded with status {response.status}")
                
    except Exception as e:
        logger.error(f"Failed to connect to API server at {api_base_url}: {e}")
        logger.info("Make sure the MyCobot API server is running")
        sys.exit(1)
    
    try:
        # Run MCP server
        async with stdio_server() as (read_stream, write_stream):
            await server.run(
                read_stream,
                write_stream,
                InitializationOptions(
                    server_name="mycobot-controller",
                    server_version="1.0.0",
                    capabilities=server.get_capabilities(
                        notification_options=None,
                        experimental_capabilities={}
                    )
                )
            )
    except KeyboardInterrupt:
        logger.info("Server interrupted by user")
    except Exception as e:
        logger.error(f"Server error: {e}")
    finally:
        # Clean up API session
        if api_session:
            try:
                await api_session.close()
                logger.info("API session closed")
            except Exception as e:
                logger.error(f"Error closing API session: {e}")


if __name__ == "__main__":
    asyncio.run(main())