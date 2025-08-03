"""
MyCobot MCP Library

Shared library for MyCobot MCP Server implementations.
Provides common functionality for both STDIO and HTTP transports.
"""

import asyncio
import json
import logging
from typing import Any, Dict, List, Optional, Union
import aiohttp

from mcp.server import Server
from mcp.server.models import InitializationOptions
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
from mcp.server.lowlevel.server import NotificationOptions


# Setup logging
logger = logging.getLogger(__name__)


class MyCobotMCPServer:
    """MyCobot MCP Server implementation."""
    
    def __init__(self, api_host: str = "localhost", api_port: int = 8080):
        """Initialize MCP server with API configuration."""
        self.api_host = api_host
        self.api_port = api_port
        self.api_base_url = f"http://{api_host}:{api_port}"
        self.api_session: Optional[aiohttp.ClientSession] = None
        self.server = Server("mycobot-controller")
        
        # Setup handlers
        self._setup_handlers()
        
        # Robot data for search/fetch
        self._robot_data = {}
        self._initialize_robot_data()
    
    def _initialize_robot_data(self):
        """Initialize robot data for search/fetch functionality."""
        self._robot_data = {
            "status": {
                "id": "robot_status",
                "type": "status",
                "description": "Current robot status including joint angles and movement state"
            },
            "joint_1": {
                "id": "joint_1",
                "type": "joint",
                "name": "Base Joint",
                "description": "Base rotation joint (Joint 1)",
                "limits": {"min": -165, "max": 165}
            },
            "joint_2": {
                "id": "joint_2", 
                "type": "joint",
                "name": "Shoulder Joint",
                "description": "Shoulder joint (Joint 2)",
                "limits": {"min": -165, "max": 165}
            },
            "joint_3": {
                "id": "joint_3",
                "type": "joint", 
                "name": "Elbow Joint",
                "description": "Elbow joint (Joint 3)",
                "limits": {"min": -165, "max": 165}
            },
            "joint_4": {
                "id": "joint_4",
                "type": "joint",
                "name": "Wrist 1 Joint", 
                "description": "First wrist rotation joint (Joint 4)",
                "limits": {"min": -165, "max": 165}
            },
            "joint_5": {
                "id": "joint_5",
                "type": "joint",
                "name": "Wrist 2 Joint",
                "description": "Second wrist rotation joint (Joint 5)", 
                "limits": {"min": -165, "max": 165}
            },
            "joint_6": {
                "id": "joint_6",
                "type": "joint",
                "name": "Wrist 3 Joint",
                "description": "Third wrist rotation joint (Joint 6)",
                "limits": {"min": -175, "max": 175}
            },
            "movements": {
                "id": "movements",
                "type": "capability",
                "description": "Available robot movement capabilities and control methods"
            },
            "home_position": {
                "id": "home_position",
                "type": "position",
                "description": "Robot home position (all joints at 0 degrees)"
            }
        }
    
    async def initialize_api_session(self) -> bool:
        """Initialize API session and test connection."""
        try:
            self.api_session = aiohttp.ClientSession()
            
            # Test API connection
            async with self.api_session.get(f"{self.api_base_url}/health") as response:
                if response.status == 200:
                    health_data = await response.json()
                    logger.info(f"Connected to MyCobot API server at {self.api_base_url}")
                    logger.info(f"Robot connected: {health_data.get('robot_connected', False)}")
                    return True
                else:
                    logger.warning(f"API server responded with status {response.status}")
                    return False
                    
        except Exception as e:
            logger.error(f"Failed to connect to API server at {self.api_base_url}: {e}")
            return False
    
    async def cleanup(self):
        """Clean up resources."""
        if self.api_session:
            try:
                await self.api_session.close()
                logger.info("API session closed")
            except Exception as e:
                logger.error(f"Error closing API session: {e}")
    
    def ensure_api_session(self):
        """Ensure API session is initialized."""
        if self.api_session is None:
            raise RuntimeError("API session not initialized")
    
    def _setup_handlers(self):
        """Setup MCP server handlers."""
        
        @self.server.list_tools()
        async def handle_list_tools() -> List[Tool]:
            """List available tools for MyCobot control."""
            tools = [
                # ChatGPT Required tools
                Tool(
                    name="search",
                    description="Search through MyCobot robot resources, capabilities, and status information. Use this to find information about joints, movements, positions, or robot capabilities. Query can be about joint names, robot status, movement types, or any robot-related topics.",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "query": {
                                "type": "string",
                                "description": "Search query for robot resources (e.g., 'joint 1', 'status', 'movements', 'home position')"
                            }
                        },
                        "required": ["query"]
                    }
                ),
                Tool(
                    name="fetch",
                    description="Fetch detailed information about a specific robot resource by ID. Use the IDs returned by the search tool to get comprehensive details about joints, status, or capabilities.",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "id": {
                                "type": "string",
                                "description": "Resource ID to fetch (e.g., 'robot_status', 'joint_1', 'movements')"
                            }
                        },
                        "required": ["id"]
                    }
                ),
                
                # Robot control tools
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
            return tools
        
        @self.server.call_tool()
        async def handle_call_tool(name: str, arguments: Dict[str, Any]) -> CallToolResult:
            """Handle tool calls."""
            try:
                if name == "search":
                    result = await self._search_resources(arguments["query"])
                elif name == "fetch":
                    result = await self._fetch_resource(arguments["id"])
                elif name == "get_joint_angle":
                    result = await self.get_joint_angle(arguments["joint_num"])
                elif name == "move_joint":
                    result = await self.move_joint(
                        arguments["joint_num"],
                        arguments["angle"],
                        arguments.get("speed", 50)
                    )
                elif name == "get_all_joint_angles":
                    result = await self.get_all_joint_angles()
                elif name == "move_all_joints":
                    result = await self.move_all_joints(
                        arguments["angles"],
                        arguments.get("speed", 50)
                    )
                elif name == "home_position":
                    result = await self.home_position(arguments.get("speed", 50))
                elif name == "stop_robot":
                    result = await self.stop_robot()
                elif name == "jog_joint":
                    result = await self.jog_joint(
                        arguments["joint_num"],
                        arguments["direction"],
                        arguments.get("speed", 50)
                    )
                elif name == "wait_for_completion":
                    result = await self.wait_for_completion(arguments.get("timeout", 10.0))
                elif name == "get_robot_status":
                    result = await self.get_robot_status()
                else:
                    unknown_tool_content = TextContent(
                        type="text",
                        text=f"Unknown tool: {name}",
                        annotations=None
                    )
                    return CallToolResult(
                        content=[unknown_tool_content],
                        isError=True,
                        meta=None
                    )
                
                text_content = TextContent(
                    type="text",
                    text=json.dumps(result, indent=2),
                    annotations=None
                )
                return CallToolResult(
                    content=[text_content],
                    isError=False,
                    meta=None
                )
            
            except Exception as e:
                error_result = {
                    "error": str(e),
                    "tool": name,
                    "arguments": arguments
                }
                error_text_content = TextContent(
                    type="text", 
                    text=json.dumps(error_result, indent=2),
                    annotations=None
                )
                return CallToolResult(
                    content=[error_text_content],
                    isError=True,
                    meta=None
                )
        
        @self.server.list_prompts()
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
        
        @self.server.get_prompt()
        async def handle_get_prompt(name: str, arguments: Dict[str, str]) -> GetPromptResult:
            """Handle prompt requests."""
            if name == "robot_status":
                try:
                    status = await self.get_robot_status()
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
                            content=TextContent(
                                type="text", 
                                text=prompt_text,
                                annotations=None
                            )
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
                            content=TextContent(
                                type="text", 
                                text=prompt_text,
                                annotations=None
                            )
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
                            content=TextContent(
                                type="text", 
                                text=prompt_text,
                                annotations=None
                            )
                        )
                    ]
                )
            
            else:
                raise ValueError(f"Unknown prompt: {name}")
    
    # Search and Fetch functionality for ChatGPT compatibility
    async def _search_resources(self, query: str) -> Dict[str, Any]:
        """Search through robot resources and return matching IDs."""
        query_lower = query.lower()
        matching_ids = []
        
        for resource_id, resource_data in self._robot_data.items():
            # Search in ID, name, description, and type
            searchable_text = " ".join([
                resource_id,
                resource_data.get("name", ""),
                resource_data.get("description", ""),
                resource_data.get("type", "")
            ]).lower()
            
            if query_lower in searchable_text:
                matching_ids.append(resource_id)
        
        return {
            "query": query,
            "matching_ids": matching_ids,
            "total_matches": len(matching_ids),
            "available_resources": list(self._robot_data.keys())
        }
    
    async def _fetch_resource(self, resource_id: str) -> Dict[str, Any]:
        """Fetch detailed information about a specific resource."""
        if resource_id not in self._robot_data:
            raise ValueError(f"Resource not found: {resource_id}")
        
        resource_data = self._robot_data[resource_id].copy()
        
        # Add real-time data for certain resources
        if resource_id == "robot_status":
            try:
                current_status = await self.get_robot_status()
                resource_data.update({
                    "current_data": current_status,
                    "timestamp": "real-time"
                })
            except Exception as e:
                resource_data["error"] = f"Could not fetch real-time status: {str(e)}"
        
        elif resource_id.startswith("joint_"):
            try:
                joint_num = int(resource_id.split("_")[1])
                current_angle = await self.get_joint_angle(joint_num)
                resource_data.update({
                    "current_angle": current_angle["angle"],
                    "timestamp": "real-time"
                })
            except Exception as e:
                resource_data["error"] = f"Could not fetch real-time angle: {str(e)}"
        
        elif resource_id == "movements":
            resource_data.update({
                "available_tools": [
                    "move_joint", "move_all_joints", "home_position", 
                    "jog_joint", "stop_robot", "wait_for_completion"
                ],
                "capabilities": {
                    "individual_joint_control": True,
                    "simultaneous_movement": True,
                    "position_feedback": True,
                    "emergency_stop": True,
                    "speed_control": True
                }
            })
        
        return resource_data
    
    # Robot API functions
    async def get_joint_angle(self, joint_num: int) -> Dict[str, Any]:
        """Get current angle of a specific joint."""
        self.ensure_api_session()
        
        if not (1 <= joint_num <= 6):
            raise ValueError(f"Joint number must be between 1-6, got {joint_num}")
        
        async with self.api_session.get(f"{self.api_base_url}/joints/angles") as response:
            if response.status != 200:
                error_text = await response.text()
                raise RuntimeError(f"API request failed: {response.status} - {error_text}")
            
            data = await response.json()
            angles = data["angles"]
            
            return {
                "joint_num": joint_num,
                "angle": angles[joint_num - 1],
                "unit": "degrees"
            }
    
    async def move_joint(self, joint_num: int, angle: float, speed: int = 50) -> Dict[str, Any]:
        """Move a specific joint to target angle."""
        self.ensure_api_session()
        
        if not (1 <= joint_num <= 6):
            raise ValueError(f"Joint number must be between 1-6, got {joint_num}")
        
        payload = {"angle": angle, "speed": speed}
        async with self.api_session.put(f"{self.api_base_url}/joints/{joint_num}/angle", json=payload) as response:
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
    
    async def get_all_joint_angles(self) -> Dict[str, Any]:
        """Get current angles of all joints."""
        self.ensure_api_session()
        
        async with self.api_session.get(f"{self.api_base_url}/joints/angles") as response:
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
    
    async def move_all_joints(self, angles: List[float], speed: int = 50) -> Dict[str, Any]:
        """Move all joints to specified angles simultaneously."""
        self.ensure_api_session()
        
        if len(angles) != 6:
            raise ValueError("Must provide exactly 6 angles")
        
        payload = {"angles": angles, "speed": speed}
        async with self.api_session.put(f"{self.api_base_url}/joints/angles", json=payload) as response:
            if response.status != 200:
                error_text = await response.text()
                raise RuntimeError(f"API request failed: {response.status} - {error_text}")
            
            return {
                "success": True,
                "message": f"All joints moving to {angles} at speed {speed}",
                "target_angles": angles,
                "speed": speed
            }
    
    async def home_position(self, speed: int = 50) -> Dict[str, Any]:
        """Move all joints to home position (0 degrees)."""
        self.ensure_api_session()
        
        payload = {"speed": speed}
        async with self.api_session.post(f"{self.api_base_url}/robot/home", json=payload) as response:
            if response.status != 200:
                error_text = await response.text()
                raise RuntimeError(f"API request failed: {response.status} - {error_text}")
            
            return {
                "success": True,
                "message": f"Moving to home position at speed {speed}",
                "target_angles": [0, 0, 0, 0, 0, 0],
                "speed": speed
            }
    
    async def stop_robot(self) -> Dict[str, Any]:
        """Emergency stop for all joint movements."""
        self.ensure_api_session()
        
        async with self.api_session.post(f"{self.api_base_url}/robot/stop") as response:
            if response.status != 200:
                error_text = await response.text()
                raise RuntimeError(f"API request failed: {response.status} - {error_text}")
            
            return {
                "success": True,
                "message": "All joints stopped"
            }
    
    async def jog_joint(self, joint_num: int, direction: int, speed: int = 50) -> Dict[str, Any]:
        """Jog a joint in specified direction."""
        self.ensure_api_session()
        
        if not (1 <= joint_num <= 6):
            raise ValueError(f"Joint number must be between 1-6, got {joint_num}")
        
        if direction not in [-1, 1]:
            raise ValueError("Direction must be 1 or -1")
        
        payload = {"direction": direction, "speed": speed}
        async with self.api_session.post(f"{self.api_base_url}/joints/{joint_num}/jog", json=payload) as response:
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
    
    async def wait_for_completion(self, timeout: float = 10.0) -> Dict[str, Any]:
        """Wait for robot to complete current movement."""
        self.ensure_api_session()
        
        payload = {"timeout": timeout}
        async with self.api_session.post(f"{self.api_base_url}/robot/wait", json=payload) as response:
            if response.status != 200:
                error_text = await response.text()
                raise RuntimeError(f"API request failed: {response.status} - {error_text}")
            
            data = await response.json()
            return {
                "completed": data["completed"],
                "elapsed_time": data["elapsed_time"],
                "timeout": timeout
            }
    
    async def get_robot_status(self) -> Dict[str, Any]:
        """Get current robot status including joint angles and movement state."""
        self.ensure_api_session()
        
        async with self.api_session.get(f"{self.api_base_url}/robot/status") as response:
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