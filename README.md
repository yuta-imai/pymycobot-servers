# MyCobot Robot Control System

A comprehensive robotics control system for MyCobot arms with camera streaming and remote API access.

## Features

- **Joint Control**: Precise individual and simultaneous joint movements
- **Camera Streaming**: RTSP video streaming from Raspberry Pi camera
- **REST API**: HTTP-based remote control with OpenAPI specification
- **MCP Server**: Model Context Protocol integration for AI systems
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
python mycobot_api_server.py --robot-port /dev/ttyUSB0 --robot-baudrate 115200

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

# Move to home position
curl -X POST "http://localhost:8080/robot/home" \
     -H "Content-Type: application/json" \
     -d '{"speed": 50}'

# Emergency stop
curl -X POST "http://localhost:8080/robot/stop"

# Get robot status
curl http://localhost:8080/robot/status
```

### 4. MCP Server

The MCP (Model Context Protocol) server provides integration with external AI systems by exposing MyCobot control as structured tools and prompts.

```bash
# Start MCP server (connects to REST API server)
python mycobot_mcp_server.py

# Custom API server connection
python mycobot_mcp_server.py --api-host 192.168.1.100 --api-port 8080
```

#### MCP Server Features

- **9 Control Tools**: Joint movement, position reading, jogging, and emergency stop
- **3 Built-in Prompts**: Robot status, joint information, and movement examples
- **REST API Integration**: Communicates with MyCobot through the REST API server
- **Error Handling**: Comprehensive error reporting and validation
- **Safety Features**: Input validation and movement limits

#### Available MCP Tools

| Tool | Description |
|------|-------------|
| `get_joint_angle` | Get current angle of specific joint (1-6) |
| `move_joint` | Move specific joint to target angle |
| `get_all_joint_angles` | Get current angles of all joints |
| `move_all_joints` | Move all joints simultaneously |
| `home_position` | Move all joints to home position (0°) |
| `stop_robot` | Emergency stop for all movements |
| `jog_joint` | Jog joint in specified direction |
| `wait_for_completion` | Wait for movement to complete |
| `get_robot_status` | Get comprehensive robot status |

#### Available MCP Prompts

| Prompt | Description |
|--------|-------------|
| `robot_status` | Get comprehensive robot status information |
| `joint_info` | Get information about robot joints and limits |
| `basic_movements` | Examples of basic robot movements |

#### Claude Desktop Integration

To use the MCP server with Claude Desktop, add this configuration to your Claude Desktop settings:

```json
{
  "mcpServers": {
    "mycobot-controller": {
      "command": "docker",
      "args": [
        "run", "--rm", "-i",
        "--network", "host",
        "your-dockerhub-username/mycobot-mcp-server:latest",
        "--api-host", "192.168.1.100",
        "--api-port", "8080"
      ]
    }
  }
}
```

For localhost API server (default):

```json
{
  "mcpServers": {
    "mycobot-controller": {
      "command": "docker",
      "args": [
        "run", "--rm", "-i",
        "--network", "host",
        "your-dockerhub-username/mycobot-mcp-server:latest"
      ]
    }
  }
}
```

For local development without Docker:

```json
{
  "mcpServers": {
    "mycobot-controller": {
      "command": "python",
      "args": ["/path/to/your/project/mycobot_mcp_server.py"],
      "env": {
        "PYTHONPATH": "/path/to/your/project"
      }
    }
  }
}
```

#### Docker Setup

Create a `Dockerfile` for the MCP server:

```dockerfile
FROM python:3.9-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt

COPY mycobot_mcp_server.py .
CMD ["python", "mycobot_mcp_server.py"]
```

Build and push to Docker Hub:

```bash
# Build the image
docker build -t your-dockerhub-username/mycobot-mcp-server:latest .

# Push to Docker Hub
docker push your-dockerhub-username/mycobot-mcp-server:latest
```

**Prerequisites for MCP Server:**
1. MyCobot REST API server must be running (`python mycobot_api_server.py`)
2. MyCobot robot must be connected and accessible
3. Network connectivity between MCP server and API server

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

## Configuration

### Command Line Options

#### API Server
```bash
python mycobot_api_server.py --help
```

Options:
- `--host`: Host to bind server (default: 0.0.0.0)
- `--port`: Port to bind server (default: 8080)
- `--robot-port`: Robot serial port (default: /dev/ttyUSB0)
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
├── mycobot_mcp_server.py          # MCP server
├── mycobot_api_spec.yaml          # OpenAPI specification
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