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

## Tools

| Tool | Description |
|------|-------------|
| `get_robot_status` | All joint angles, movement state, firmware error code, and a health check |
| `get_joint_angles` | Current angle of all six joints |
| `move_joint` | Move one joint to an absolute angle |
| `move_all_joints` | Move all six joints simultaneously |
| `jog_joint` | Jog a joint by a fixed increment (non-blocking) |
| `control_gripper` | `open` / `close` / `release` / `set_value` / `calibrate` |
| `get_gripper_status` | Current gripper opening value and moving state |
| `go_home` | Move all joints to 0° |
| `stop_robot` | Emergency stop |
| `wait_for_movement` | Wait for the target (converged / stalled / timeout, auto-stop) |

Inputs are validated with [zod](https://zod.dev) (joint numbers 1–6, speed 1–100,
gripper value 0–100, etc.) before any request is sent.

## Waza: motions taught by hand

A **waza** (技) is a motion a person taught the arm by posing it in freedrive,
stored together with a `setsumei` — a plain-language description of *when* the
motion should be used. The server turns each waza into an MCP tool whose
**description is that sentence**, so the model selects a motion by reading what a
human wrote about it, not by matching a function name.

This makes the description the programmable surface: change the sentence, and the
robot's behaviour changes, with no code edit and no restart.

Motions are taught through conversation (release the servos, pose the arm by
hand, save), so no extra tooling is needed. Waza are off unless you point the
server at a file:

```json
{
  "mcpServers": {
    "mycobot": {
      "command": "npx",
      "args": ["-y", "@yuta-imai/mycobot-mcp-server"],
      "env": {
        "MYCOBOT_API_BASE_URL": "http://192.168.0.136:8080",
        "MYCOBOT_WAZA_FILE": "~/mycobot/waza.json"
      }
    }
  }
}
```

### Tools

| Tool | Description |
|------|-------------|
| `list_waza` | Every taught waza with its `setsumei` |
| `do_waza` | Perform a waza by name |
| `save_waza` | Store the arm's current pose under a name + description, then re-engage servos |
| `add_waza_pose` | Append another pose to an existing waza |
| `update_waza_setsumei` | Rewrite the description, leaving the motion untouched |
| `forget_waza` | Delete a waza |
| `waza_<hash>` | One dynamically registered tool per waza (see below) |

### File format

`MYCOBOT_WAZA_FILE` holds either `{ "version": 1, "waza": [...] }` or a bare
array. See [`../waza.example.json`](../waza.example.json).

```jsonc
{
  "name": "てをふる",                       // display name, any script
  "setsumei": "バイバイするときにつかう",  // what the model reads
  "poses": [                              // or a single "angles": [...]
    { "angles": [0, -30, 10, 0, 60, -30] },
    { "angles": [0, -30, 10, 0, 60,  30], "hold_ms": 300 }
  ],
  "repeat": 3,                            // 1-5
  "speed": 50                             // default for poses that omit it
}
```

Per-joint limits are enforced at load time. An entry that fails validation is
skipped and reported (via `list_waza` and stderr) rather than taking the rest of
the file down with it — and while any entry is invalid, the write tools refuse to
save, so a rewrite can never silently drop it.

### Dynamic tools and hot reload

By default each waza is registered as its own tool named `waza_<hash of name>`,
with the `setsumei` as its description. The file is watched, so teaching, editing
a description by hand, or saving from the teaching script all emit
`notifications/tools/list_changed` and update the client's tool list without a
restart.

Set `MYCOBOT_WAZA_DYNAMIC_TOOLS=false` to expose only `list_waza` / `do_waza`
instead. That variant makes the model's reasoning more visible (it must fetch the
list and choose), at the cost of an extra round trip.

### Teaching by hand

Teaching needs no separate tooling — it happens in conversation:

1. Ask the person to **support the arm**, and wait for them to confirm.
2. `release_all_servos` — the arm goes limp; they pose it by hand.
3. `save_waza` — records the current joint angles under a name and a
   description, then **re-engages the servos automatically**.

Servos are powered back on in a `finally`, so the arm is never left limp because
a save failed or a follow-up call was forgotten. Pass `keep_relaxed: true` to
stay in freedrive when recording several poses back to back
(`add_waza_pose`), and omit it on the last one.

`update_waza_setsumei` rewrites a description without touching the motion, which
is the cheapest way to test how wording affects tool selection.

**Safety:** the arm sags the instant the servos release. Warn and confirm
*before* calling `release_all_servos`, not after. Playback clamps speed to a
minimum of 20 because J2 stalls under its own weight below that; a stall or
timeout aborts the remaining poses and reports which pose failed.

## Configuration reference

| Variable | Default | Meaning |
|----------|---------|---------|
| `MYCOBOT_API_BASE_URL` | `http://localhost:8080` | REST API base URL (also `--api-base-url`) |
| `MYCOBOT_API_TIMEOUT_MS` | `30000` | Per-request timeout |
| `MYCOBOT_WAZA_FILE` | *(unset — waza off)* | Path to the waza file (also `--waza-file`); `~` is expanded |
| `MYCOBOT_WAZA_DYNAMIC_TOOLS` | `true` | Register one tool per waza |
| `MYCOBOT_WAZA_SPEED` | `40` | Default playback speed |
| `MYCOBOT_WAZA_POSE_TIMEOUT` | `15` | Seconds to wait for each pose to converge |
| `MYCOBOT_GRIPPER_TYPE` | *(server default)* | `3` for the parallel gripper |

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
