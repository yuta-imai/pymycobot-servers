# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a robotics project focused on MyCobot arm control and camera streaming using Python. The project includes comprehensive joint control functionality for the MyCobot robot and RTSP streaming capabilities for Raspberry Pi camera modules.

## Project Structure

```
/home/factory/arms/
├── package.json                    # Node.js dependencies (Claude Code)
├── package-lock.json              # NPM lockfile
├── requirements.txt                # Python dependencies
├── mycobot_joint_controller.py    # Main MyCobot joint control module
├── rtsp_camera_server.py          # RTSP camera streaming server
├── mycobot_api_spec.yaml          # OpenAPI specification for REST API
├── mycobot_api_server.py          # REST API server (FastAPI)
├── mycobot_mcp_server.py          # MCP server (backward compatible, enhanced)
├── mycobot_mcp_lib.py             # Shared MCP server library
├── mycobot_mcp_stdio.py           # STDIO-specific MCP server
├── mycobot_mcp_http.py            # HTTP-specific MCP server (Remote MCP)
└── node_modules/                  # Node.js dependencies
```

## Development Commands

### Python Development
- Install dependencies: `pip install -r requirements.txt`
- Run the joint controller example: `python mycobot_joint_controller.py`
- Run the RTSP camera server: `python rtsp_camera_server.py`
  - With custom frame rate: `python rtsp_camera_server.py --fps 15`
  - With custom resolution: `python rtsp_camera_server.py --width 1280 --height 720 --fps 10`
  - For external access (default): `python rtsp_camera_server.py --bind-address 0.0.0.0`
  - For local access only: `python rtsp_camera_server.py --bind-address 127.0.0.1`
  - View all options: `python rtsp_camera_server.py --help`
- Run the REST API server: `python mycobot_api_server.py`
  - With custom host/port: `python mycobot_api_server.py --host 0.0.0.0 --port 8080`
  - API docs available at: `http://localhost:8080/docs`
  - View all options: `python mycobot_api_server.py --help`
- Run the MCP server (backward compatible): `python mycobot_mcp_server.py`
  - With STDIO transport (default): `python mycobot_mcp_server.py --transport stdio`
  - With HTTP transport: `python mycobot_mcp_server.py --transport http --http-host 0.0.0.0 --http-port 8081`
  - With custom API server: `python mycobot_mcp_server.py --api-host 192.168.1.100 --api-port 8080`
  - View all options: `python mycobot_mcp_server.py --help`
- Run the STDIO MCP server: `python mycobot_mcp_stdio.py`
  - With custom API server: `python mycobot_mcp_stdio.py --api-host 192.168.1.100 --api-port 8080`
  - View all options: `python mycobot_mcp_stdio.py --help`
- Run the HTTP MCP server (Remote MCP for ChatGPT): `python mycobot_mcp_http.py`
  - With external access: `python mycobot_mcp_http.py --host 0.0.0.0 --port 8081`
  - With custom API server: `python mycobot_mcp_http.py --api-host 192.168.1.100 --api-port 8080`
  - For ChatGPT integration: `python mycobot_mcp_http.py --host 0.0.0.0 --port 8081`
  - View all options: `python mycobot_mcp_http.py --help`
- Import modules:
  - `from mycobot_joint_controller import MyCobotJointController`
  - `from rtsp_camera_server import RTSPCameraServer`
  - `from mycobot_mcp_lib import MyCobotMCPServer`

### System Dependencies
- **FFmpeg**: Required for RTSP streaming: `sudo apt install ffmpeg`
- **GStreamer** (optional): Alternative streaming backend: `sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good`

## Architecture

### MyCobot Joint Controller (`mycobot_joint_controller.py`)
- **MyCobotJointController**: Main controller class for robot joint operations
- **Individual Joint Functions**: `move_joint_1()` through `move_joint_6()` for specific joint control
- **Utility Functions**: Joint validation, angle limits, movement completion detection
- **Safety Features**: Angle validation, joint limits enforcement, emergency stop functionality

### RTSP Camera Server (`rtsp_camera_server.py`)
- **RTSPCameraServer**: Main streaming server class using FFmpeg backend
- **SimpleRTSPServer**: Alternative implementation using GStreamer
- **Camera Control**: Captures from `/dev/video0` with configurable resolution/FPS
- **Stream Management**: Start/stop streaming with proper resource cleanup
- **External Access**: Server binds to `0.0.0.0` by default for external network access

### REST API Server (`mycobot_api_server.py`)
- **FastAPI-based**: REST API server for remote robot control
- **OpenAPI Compliance**: Follows specification in `mycobot_api_spec.yaml`
- **External Access**: Serves on `0.0.0.0:8080` by default for network access
- **Interactive Docs**: Swagger UI available at `/docs` endpoint
- **CORS Enabled**: Supports cross-origin requests for web applications

### MCP Server (`mycobot_mcp_server.py`)
- **Model Context Protocol**: MCP server for external system integration
- **Backward Compatibility**: Enhanced version maintaining full compatibility
- **ChatGPT Remote MCP Support**: Includes required `search` and `fetch` tools
- **Dual Transport Support**: STDIO (pipe/stdin) and HTTP (SSE) transports
- **Tool Interface**: Provides structured tools for robot control
- **Prompt Support**: Built-in prompts for robot status and usage examples
- **Async Operations**: Asynchronous tool execution for responsiveness
- **HTTP Endpoints**: `/sse` for MCP communication, `/health` for status checks

### MCP Library (`mycobot_mcp_lib.py`)
- **Shared Library**: Common MCP server functionality for all implementations
- **MyCobotMCPServer Class**: Unified server implementation with full robot control
- **ChatGPT Tools**: Required `search` and `fetch` tools for Remote MCP compatibility
- **Robot Data Management**: Searchable robot resource database
- **API Integration**: Manages aiohttp sessions and REST API communication
- **Tool Handlers**: Complete set of robot control tools and prompts

### STDIO MCP Server (`mycobot_mcp_stdio.py`)
- **Dedicated STDIO**: Lightweight server focused on pipe-based communication
- **Traditional MCP**: Optimized for local MCP clients and development tools
- **Full Compatibility**: Uses shared library for complete robot functionality

### HTTP MCP Server (`mycobot_mcp_http.py`)
- **Remote MCP Ready**: Designed specifically for ChatGPT and web-based integration
- **External Access**: Configured for public endpoint access (0.0.0.0 binding)
- **Enhanced CORS**: Full cross-origin support for browser-based clients
- **Discovery Endpoints**: `/mcp` and `/.well-known/mcp` for server specification
- **Health Monitoring**: Comprehensive health checks with robot status integration

### Key Features

#### MyCobot Control
- Individual joint control with angle validation
- Simultaneous multi-joint movements
- Joint jogging and incremental movements
- Real-time angle reading
- Movement completion detection
- Safety limits and error handling

#### Camera Streaming
- RTSP streaming from Raspberry Pi camera module
- Configurable resolution, FPS, and bitrate
- H.264 encoding with low-latency tuning
- Dual backend support (FFmpeg/GStreamer)
- Stream URL: `rtsp://IP_ADDRESS:8554/camera`
- External network access (binds to 0.0.0.0 by default)
- Automatic resource management and cleanup

#### Remote Control APIs
- **REST API**: HTTP-based control with OpenAPI specification
  - GET/PUT endpoints for joint control
  - JSON request/response format
  - Interactive documentation at `/docs`
  - Health checks and status monitoring
- **MCP Protocol**: Integration with external AI systems
  - **STDIO Transport**: Traditional pipe-based communication for local applications
  - **HTTP Transport**: Server-Sent Events (SSE) for web-based integration
  - Structured tool definitions for robot control
  - Built-in prompts for guidance and examples
  - Asynchronous operation support
  - CORS-enabled HTTP server for browser compatibility

## Memories
- to memorize