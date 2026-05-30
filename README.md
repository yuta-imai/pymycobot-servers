# MyCobot Robot Control System

A comprehensive robotics control system for MyCobot arms with camera streaming and remote API access.

## Features

- **Joint Control**: Precise individual and simultaneous joint movements
- **Gripper Control**: Open/close/release operations and value-based gripper positioning
- **Camera Streaming**: RTSP video streaming from Raspberry Pi camera
- **REST API**: HTTP-based remote control with OpenAPI specification
- **MCP Server**: Control the arm from Claude Desktop and other MCP clients (see [`mcp-server/`](mcp-server/))
- **Safety Features**: Angle validation, joint limits, and emergency stops

## Installation

### System Dependencies

```bash
# Install FFmpeg for RTSP streaming
sudo apt install ffmpeg

# Optional: Install GStreamer as alternative streaming backend
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good
```

### Python Dependencies

```bash
# Install Python dependencies
pip install -r requirements.txt
```

## Quick Start

### 1. Basic Joint Control

```bash
# Run the basic joint controller example
python mycobot_joint_controller.py
```

### 2. Camera Streaming

```bash
# Start RTSP camera server (default settings)
python rtsp_camera_server.py

# Custom settings
python rtsp_camera_server.py --fps 15 --width 1280 --height 720

# View stream at: rtsp://YOUR_PI_IP:8554/camera
```

### 3. REST API Server

#### Start the API Server

```bash
# Basic startup (binds to all interfaces on port 8080)
python mycobot_api_server.py

# Custom host and port
python mycobot_api_server.py --host 0.0.0.0 --port 8080

# Custom robot connection
python mycobot_api_server.py --robot-port /dev/ttyACM0 --robot-baudrate 115200

# Development mode with auto-reload
python mycobot_api_server.py --reload
```

#### API Documentation

Once the server is running, access the interactive API documentation:

- **Swagger UI**: http://localhost:8080/docs
- **ReDoc**: http://localhost:8080/redoc
- **OpenAPI Spec**: http://localhost:8080/openapi.json

#### API Examples

```bash
# Health check
curl http://localhost:8080/health

# Get all joint angles
curl http://localhost:8080/joints/angles

# Move joint 1 to 45 degrees
curl -X PUT "http://localhost:8080/joints/1/angle" \
     -H "Content-Type: application/json" \
     -d '{"angle": 45, "speed": 50}'

# Move all joints simultaneously
curl -X PUT "http://localhost:8080/joints/angles" \
     -H "Content-Type: application/json" \
     -d '{"angles": [0, 45, -30, 90, -15, 60], "speed": 50}'

# Open gripper
curl -X POST "http://localhost:8080/gripper/open" \
     -H "Content-Type: application/json" \
     -d '{"speed": 50}'

# Set gripper opening value (0-100)
curl -X PUT "http://localhost:8080/gripper/value" \
     -H "Content-Type: application/json" \
     -d '{"value": 30, "speed": 40}'

# Close gripper
curl -X POST "http://localhost:8080/gripper/close" \
     -H "Content-Type: application/json" \
     -d '{"speed": 50}'

# Move to home position
curl -X POST "http://localhost:8080/robot/home" \
     -H "Content-Type: application/json" \
     -d '{"speed": 50}'

# Emergency stop
curl -X POST "http://localhost:8080/robot/stop"

# Get robot status
curl http://localhost:8080/robot/status
```

### 4. MCP Server (Claude Desktop)

A standalone MCP server lets Claude Desktop (and other MCP clients) control the arm.
It is a separate Node/TypeScript runtime that talks to the REST API above over HTTP —
there is no code dependency on the Python code. It is published to npm and runs via
`npx`, so no local build is required to use it:

```json
{
  "mcpServers": {
    "mycobot": {
      "command": "npx",
      "args": ["-y", "@yuta-imai/mycobot-mcp-server"],
      "env": { "MYCOBOT_API_BASE_URL": "http://localhost:8080" }
    }
  }
}
```

See [`mcp-server/`](mcp-server/) for the full tool list, configuration, and development
instructions.

## API Reference

### REST API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/health` | System health check |
| GET | `/joints/{joint_num}/angle` | Get specific joint angle |
| PUT | `/joints/{joint_num}/angle` | Move specific joint |
| GET | `/joints/angles` | Get all joint angles |
| PUT | `/joints/angles` | Move all joints |
| POST | `/joints/{joint_num}/jog` | Jog joint in direction |
| POST | `/gripper/open` | Open gripper |
| POST | `/gripper/close` | Close gripper |
| POST | `/gripper/release` | Release gripper |
| PUT | `/gripper/state` | Set gripper state (0/1/10) |
| PUT | `/gripper/value` | Set gripper opening value (0-100) |
| POST | `/gripper/calibrate` | Calibrate gripper |
| POST | `/robot/home` | Move to home position |
| POST | `/robot/stop` | Emergency stop |
| GET | `/robot/status` | Get robot status |
| POST | `/robot/wait` | Wait for movement completion |

### Joint Limits

| Joint | Range | Description |
|-------|-------|-------------|
| 1 | -165° to +165° | Base rotation |
| 2 | -165° to +165° | Shoulder |
| 3 | -165° to +165° | Elbow |
| 4 | -165° to +165° | Wrist 1 |
| 5 | -165° to +165° | Wrist 2 |
| 6 | -175° to +175° | Wrist 3 |

### Speed Settings

- **Range**: 1-100 (1=slowest, 100=fastest)
- **Default**: 50
- **Recommended**: 20-80 for smooth movements

### Gripper Parameters

- **State values** (`/gripper/state`):
  - `0`: open
  - `1`: close
  - `10`: release
- **Opening value range** (`/gripper/value`): `0-100`
- **Gripper type** (optional):
  - `1`: adaptive gripper
  - `2`: 5-finger dexterous hand (state commands only)
  - `3`: parallel gripper
  - `4`: flexible gripper

## Configuration

### Command Line Options

#### API Server
```bash
python mycobot_api_server.py --help
```

Options:
- `--host`: Host to bind server (default: 0.0.0.0)
- `--port`: Port to bind server (default: 8080)
- `--robot-port`: Robot serial port (default: /dev/ttyACM0)
- `--robot-baudrate`: Robot baudrate (default: 115200)
- `--reload`: Enable auto-reload for development

#### Camera Server
```bash
python rtsp_camera_server.py --help
```

Options:
- `--fps`: Frame rate (default: 30)
- `--device`: Camera device (default: /dev/video0)
- `--port`: RTSP port (default: 8554)
- `--width`: Video width (default: 640)
- `--height`: Video height (default: 480)
- `--bind-address`: Bind address (default: 0.0.0.0)

## Safety Guidelines

1. **Always check robot status** before sending movement commands
2. **Use appropriate speeds** (20-80 recommended for smooth operation)
3. **Respect joint limits** to prevent mechanical damage
4. **Emergency stop available** via `/robot/stop` endpoint
5. **Wait for completion** using `/robot/wait` before next movement

## Troubleshooting

### Connection Issues

```bash
# Check if robot is connected
ls /dev/ttyUSB*

# Test basic connection
python -c "from mycobot_joint_controller import MyCobotJointController; c = MyCobotJointController(); print('Connected:', c.get_all_joint_angles())"
```

### API Server Issues

```bash
# Check if port is available
netstat -tulpn | grep :8080

# Check server logs
python mycobot_api_server.py --reload
```

### Camera Streaming Issues

```bash
# Check camera device
ls /dev/video*

# Test camera access
python -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK:', cap.isOpened())"
```

## Development

### Project Structure

```
/home/factory/arms/
├── mycobot_joint_controller.py    # Core joint control module
├── rtsp_camera_server.py          # RTSP streaming server
├── mycobot_api_server.py          # REST API server
├── mycobot_api_spec.yaml          # OpenAPI specification
├── mcp-server/                    # MCP server for Claude Desktop (Node/TypeScript)
├── requirements.txt               # Python dependencies
└── README.md                      # This file
```

### Contributing

1. Follow the existing code style and patterns
2. Test all API endpoints before submitting changes
3. Update documentation for new features
4. Ensure safety features remain intact

## License

This project is designed for educational and research purposes. Please follow your local robotics safety regulations.

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review the API documentation at `/docs`
3. Examine the OpenAPI specification in `mycobot_api_spec.yaml`