# MyCobot MCP Server

A [Model Context Protocol](https://modelcontextprotocol.io) server for controlling a
MyCobot arm from MCP clients such as **Claude Desktop**.

It is a standalone Node/TypeScript runtime with **no code dependency** on the robot's
Python code: every operation goes through the existing REST API
(`mycobot_api_server.py`) over HTTP. The TypeScript types are generated from the
project's OpenAPI spec (`../mycobot_api_spec.yaml`), so the spec is the single source
of truth for the contract.

## Prerequisites

1. The MyCobot REST API server must be running and reachable:
   ```bash
   python mycobot_api_server.py            # defaults to http://0.0.0.0:8080
   ```
2. The robot must be connected for movement commands to succeed.

## Use with Claude Desktop

Add this to `claude_desktop_config.json`
(macOS: `~/Library/Application Support/Claude/claude_desktop_config.json`):

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

Restart Claude Desktop. You can then ask things like *"What's the robot's status?"* or
*"Move joint 2 to 45 degrees."*

## Configuration

The REST API base URL is resolved in this order (default `http://localhost:8080`):

1. `--api-base-url <url>` CLI argument
2. `MYCOBOT_API_BASE_URL` environment variable

## Tools

| Tool | Description |
|------|-------------|
| `get_robot_status` | All joint angles, movement state, and a health check |
| `get_joint_angles` | Current angle of all six joints |
| `move_joint` | Move one joint to an absolute angle |
| `move_all_joints` | Move all six joints simultaneously |
| `jog_joint` | Incrementally jog a joint in a direction |
| `control_gripper` | `open` / `close` / `release` / `set_value` / `calibrate` |
| `go_home` | Move all joints to 0° |
| `stop_robot` | Emergency stop |
| `wait_for_movement` | Block until movement completes or times out |

Inputs are validated with [zod](https://zod.dev) (joint numbers 1–6, speed 1–100,
gripper value 0–100, etc.) before any request is sent.

## Development

```bash
npm install
npm run generate     # regenerate src/generated/api-types.ts from ../mycobot_api_spec.yaml
npm run typecheck
npm run build        # bundle to dist/index.js (executable, with shebang)
npm run inspect      # build, then open the MCP Inspector against the server
```

When the REST API changes, update `../mycobot_api_spec.yaml` and run `npm run generate`
(also run automatically on `prepublishOnly`).

## Publishing

```bash
npm publish --access public
```

`prepublishOnly` regenerates types and rebuilds, and only `dist/` is published.
