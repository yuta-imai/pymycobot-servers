# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a robotics project focused on MyCobot arm control and camera streaming using Python. The project includes comprehensive joint control functionality for the MyCobot robot and RTSP streaming capabilities for Raspberry Pi camera modules.

## Project Structure

```
/home/factory/arms/
├── requirements.txt                # Python dependencies
├── mycobot_joint_controller.py    # Main MyCobot joint control module
├── rtsp_camera_server.py          # RTSP camera streaming server
├── mycobot_api_spec.yaml          # OpenAPI specification for REST API
├── mycobot_api_server.py          # REST API server (FastAPI)
├── mycobot_server_ctl.py          # Atomic start/stop wrapper for the API server
├── mycobot_server_ctl.sh          # Shell entry point (activates venv/pyenv, then runs the wrapper)
└── mcp-server/                    # MCP server for Claude Desktop (Node/TypeScript, separate runtime)
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
- Managed start/stop (recommended): `python mycobot_server_ctl.py {start|stop|restart|status|run}`
  - `start` is health-gated (waits for `/health`); `stop` tears down the whole process group gracefully so the serial port is always released
  - `run` runs in the foreground; Ctrl-C stops the server cleanly
  - Forwards the same `--host/--port/--robot-port/--robot-baudrate/--reload` options to the server
- Shell entry point: `./mycobot_server_ctl.sh {start|stop|restart|status|run|env}`
  - Activates the Python environment before running: `$PYTHON_BIN` → `./.venv` (or `$VENV_DIR`) → pyenv (respects `.python-version`) → system `python3`
  - `env` prints the resolved interpreter; `exec`s the Python wrapper so signals reach `run` directly
- Import modules:
  - `from mycobot_joint_controller import MyCobotJointController`
  - `from rtsp_camera_server import RTSPCameraServer`

### MCP Server (Node/TypeScript, in `mcp-server/`)
- Separate runtime with **no code dependency** on the Python code; talks to the REST API over HTTP.
- Install/build: `cd mcp-server && npm install && npm run build`
- Regenerate API types after editing `mycobot_api_spec.yaml`: `npm run generate`
- Inspect tools locally: `npm run inspect` (opens the MCP Inspector)
- Distributed via npm; runs through `npx @yuta-imai/mycobot-mcp-server`

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

### Server Control Wrapper (`mycobot_server_ctl.py`)
- **Atomic supervisor**: Manages `mycobot_api_server.py` (which owns the robot controller/serial connection) as a single unit; stdlib-only, no extra dependencies
- **Health-gated start**: `start` only reports success once `/health` responds, otherwise the child is torn down — no half-started state
- **Guaranteed shutdown**: Spawns the server in its own session/process group (`start_new_session`) and stops it with `SIGTERM` → graceful window → `SIGKILL`, so the serial port is released and uvicorn workers are never orphaned (including on Ctrl-C during startup)
- **PID-file lifecycle**: `start/stop/restart/status` with stale-PID detection; LSB-style exit codes (`status` returns 3 when stopped)
- **Shell entry point** (`mycobot_server_ctl.sh`): resolves/activates the Python environment (explicit `PYTHON_BIN` → project venv → pyenv with `.python-version` → system `python3`), then `exec`s the Python wrapper

### MCP Server (`mcp-server/`)
- **Separate Node/TypeScript runtime**: Published to npm, run via `npx`; intended mainly for Claude Desktop (stdio transport)
- **No code coupling**: Controls the robot only through the REST API over HTTP; the sole contract is `mycobot_api_spec.yaml`, consumed at build time via `openapi-typescript`
- **Curated tools**: `get_robot_status`, `get_joint_angles`, `move_joint`, `move_all_joints`, `jog_joint`, `control_gripper`, `go_home`, `stop_robot`, `wait_for_movement`
- **Input validation**: zod schemas enforce joint/speed/gripper limits before any request is sent
- **Layout**: `src/api-client.ts` (typed HTTP wrapper), `src/tools.ts` (tool definitions), `src/server.ts` + `src/index.ts` (server + stdio entry), `src/generated/api-types.ts` (generated, committed)

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

## Memories
- Assumption: This project now uses MyCobot with a two-finger (parallel) gripper installed and connected.
- Before running Python gripper commands, ensure the gripper hardware is mounted and recognized by the robot controller.
- For two-finger gripper control in `pymycobot`, use gripper APIs with `gripper_type=3` where applicable (parallel gripper), based on Elephant Robotics docs: https://docs.elephantrobotics.com/docs/mycobot-280-JN-en/3-FunctionsAndApplications/6.developmentGuide/python/6_gripper.html
- to memorize