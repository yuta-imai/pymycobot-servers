import type { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import type { CallToolResult } from "@modelcontextprotocol/sdk/types.js";
import { z } from "zod";
import { ApiError, MyCobotClient } from "./api-client.js";

const speed = z
  .number()
  .int()
  .min(1)
  .max(100)
  .describe("Movement speed, 1 (slowest) to 100 (fastest). Defaults to 50.")
  .optional();

const gripperType = z
  .number()
  .int()
  .min(1)
  .max(4)
  .describe(
    "Gripper type: 1=adaptive, 2=5-finger dexterous, 3=parallel, 4=flexible. Omit to use the server default.",
  )
  .optional();

const angle = z
  .number()
  .min(-175)
  .max(175)
  .describe(
    "Target angle in degrees. Per-joint safe limits: joints 1-5 ±165°, joint 6 ±175°. Out-of-range values are rejected by the robot.",
  );

/** Wrap a tool body so REST/network failures become a clean MCP error result. */
async function run(fn: () => Promise<unknown>): Promise<CallToolResult> {
  try {
    const data = await fn();
    return {
      content: [{ type: "text", text: JSON.stringify(data, null, 2) }],
    };
  } catch (err) {
    const message =
      err instanceof ApiError
        ? err.message
        : err instanceof Error
          ? `Could not reach the MyCobot REST API: ${err.message}`
          : String(err);
    return {
      content: [{ type: "text", text: message }],
      isError: true,
    };
  }
}

export function registerTools(server: McpServer, client: MyCobotClient): void {
  server.registerTool(
    "get_robot_status",
    {
      title: "Get robot status",
      description:
        "Get comprehensive robot status: all joint angles, whether the robot is currently moving, and a server health check. Call this before issuing movement commands.",
      inputSchema: {},
    },
    () =>
      run(async () => ({
        status: await client.getStatus(),
        health: await client.getHealth(),
      })),
  );

  server.registerTool(
    "get_joint_angles",
    {
      title: "Get joint angles",
      description: "Get the current angle (in degrees) of all six joints.",
      inputSchema: {},
    },
    () => run(() => client.getAllJointAngles()),
  );

  server.registerTool(
    "move_joint",
    {
      title: "Move a single joint",
      description:
        "Move one joint to an absolute target angle. Other joints stay put.",
      inputSchema: {
        joint: z
          .number()
          .int()
          .min(1)
          .max(6)
          .describe("Joint number, 1 (base) to 6 (wrist)."),
        angle,
        speed,
      },
    },
    ({ joint, angle, speed }) =>
      run(() => client.moveJoint(joint, angle, speed)),
  );

  server.registerTool(
    "move_all_joints",
    {
      title: "Move all joints",
      description:
        "Move all six joints simultaneously to the given absolute target angles (one entry per joint, ordered joint 1 to 6).",
      inputSchema: {
        angles: z
          .array(angle)
          .length(6)
          .describe("Exactly six target angles in degrees, for joints 1-6."),
        speed,
      },
    },
    ({ angles, speed }) => run(() => client.moveAllJoints(angles, speed)),
  );

  server.registerTool(
    "jog_joint",
    {
      title: "Jog a joint",
      description:
        "Incrementally jog a single joint in the positive or negative direction. Useful for small manual adjustments.",
      inputSchema: {
        joint: z
          .number()
          .int()
          .min(1)
          .max(6)
          .describe("Joint number, 1 (base) to 6 (wrist)."),
        direction: z
          .union([z.literal(1), z.literal(-1)])
          .describe("1 for the positive direction, -1 for the negative."),
        speed,
      },
    },
    ({ joint, direction, speed }) =>
      run(() => client.jogJoint(joint, direction, speed)),
  );

  server.registerTool(
    "control_gripper",
    {
      title: "Control the gripper",
      description:
        "Operate the gripper. Actions: 'open', 'close', 'release' (torque off), 'set_value' (move to a 0-100 opening, requires `value`), or 'calibrate'.",
      inputSchema: {
        action: z
          .enum(["open", "close", "release", "set_value", "calibrate"])
          .describe("Which gripper operation to perform."),
        value: z
          .number()
          .int()
          .min(0)
          .max(100)
          .describe("Opening value 0 (closed) to 100 (open). Required for 'set_value'.")
          .optional(),
        speed,
        gripper_type: gripperType,
      },
    },
    ({ action, value, speed, gripper_type }) =>
      run(() => {
        switch (action) {
          case "open":
            return client.gripperOpen(speed, gripper_type);
          case "close":
            return client.gripperClose(speed, gripper_type);
          case "release":
            return client.gripperRelease(speed, gripper_type);
          case "calibrate":
            return client.gripperCalibrate();
          case "set_value":
            if (value === undefined) {
              throw new ApiError("`value` (0-100) is required when action is 'set_value'.");
            }
            return client.gripperSetValue(value, speed, gripper_type);
        }
      }),
  );

  server.registerTool(
    "go_home",
    {
      title: "Move to home position",
      description: "Move all joints to the home position (all angles at 0°).",
      inputSchema: { speed },
    },
    ({ speed }) => run(() => client.goHome(speed)),
  );

  server.registerTool(
    "stop_robot",
    {
      title: "Emergency stop",
      description: "Immediately stop all robot movement. Use this as an emergency stop.",
      inputSchema: {},
    },
    () => run(() => client.stop()),
  );

  server.registerTool(
    "wait_for_movement",
    {
      title: "Wait for movement to complete",
      description:
        "Block until the current movement finishes or the timeout elapses. Returns whether it completed and the elapsed time.",
      inputSchema: {
        timeout: z
          .number()
          .min(0.1)
          .max(60)
          .describe("Maximum seconds to wait (0.1-60). Defaults to 10.")
          .optional(),
      },
    },
    ({ timeout }) => run(() => client.waitForMovement(timeout)),
  );
}
