import createClient, { type Client } from "openapi-fetch";
import type { paths } from "./generated/api-types.js";

/**
 * Error raised when the REST API returns a non-2xx response or is unreachable.
 * Tool handlers convert this into a user-facing MCP error result.
 */
export class ApiError extends Error {
  constructor(
    message: string,
    public readonly status?: number,
    public readonly detail?: unknown,
  ) {
    super(message);
    this.name = "ApiError";
  }
}

type FetchResult<T> = {
  data?: T;
  error?: unknown;
  response: Response;
};

/**
 * Thin, typed wrapper over the MyCobot REST API.
 *
 * This is the ONLY coupling point to the robot backend, and it is a pure HTTP
 * contract — types come from `mycobot_api_spec.yaml` via openapi-typescript, not
 * from any shared code. Nothing here imports the Python implementation.
 */
export class MyCobotClient {
  private readonly client: Client<paths>;

  constructor(private readonly baseUrl: string) {
    this.client = createClient<paths>({ baseUrl });
  }

  private unwrap<T>(result: FetchResult<T>): T {
    if (result.error !== undefined || !result.response.ok) {
      const detail =
        (result.error as { detail?: unknown } | undefined)?.detail ??
        result.error;
      const detailText =
        detail === undefined || detail === null
          ? ""
          : `: ${typeof detail === "string" ? detail : JSON.stringify(detail)}`;
      throw new ApiError(
        `REST API ${result.response.status} ${result.response.statusText}${detailText}`,
        result.response.status,
        result.error,
      );
    }
    return result.data as T;
  }

  async getHealth() {
    return this.unwrap(await this.client.GET("/health"));
  }

  async getStatus() {
    return this.unwrap(await this.client.GET("/robot/status"));
  }

  async getAllJointAngles() {
    return this.unwrap(await this.client.GET("/joints/angles"));
  }

  async moveJoint(joint: number, angle: number, speed?: number) {
    return this.unwrap(
      await this.client.PUT("/joints/{joint_num}/angle", {
        params: { path: { joint_num: joint } },
        body: { angle, ...(speed !== undefined ? { speed } : {}) },
      }),
    );
  }

  async moveAllJoints(angles: number[], speed?: number) {
    return this.unwrap(
      await this.client.PUT("/joints/angles", {
        body: { angles, ...(speed !== undefined ? { speed } : {}) },
      }),
    );
  }

  async jogJoint(joint: number, direction: 1 | -1, speed?: number) {
    return this.unwrap(
      await this.client.POST("/joints/{joint_num}/jog", {
        params: { path: { joint_num: joint } },
        body: { direction, ...(speed !== undefined ? { speed } : {}) },
      }),
    );
  }

  async gripperOpen(speed?: number, gripperType?: number) {
    return this.unwrap(
      await this.client.POST("/gripper/open", {
        body: this.gripperActionBody(speed, gripperType),
      }),
    );
  }

  async gripperClose(speed?: number, gripperType?: number) {
    return this.unwrap(
      await this.client.POST("/gripper/close", {
        body: this.gripperActionBody(speed, gripperType),
      }),
    );
  }

  async gripperRelease(speed?: number, gripperType?: number) {
    return this.unwrap(
      await this.client.POST("/gripper/release", {
        body: this.gripperActionBody(speed, gripperType),
      }),
    );
  }

  async gripperSetValue(value: number, speed?: number, gripperType?: number) {
    return this.unwrap(
      await this.client.PUT("/gripper/value", {
        body: {
          value,
          ...(speed !== undefined ? { speed } : {}),
          ...(gripperType !== undefined ? { gripper_type: gripperType } : {}),
        },
      }),
    );
  }

  async gripperCalibrate() {
    return this.unwrap(await this.client.POST("/gripper/calibrate"));
  }

  async goHome(speed?: number) {
    return this.unwrap(
      await this.client.POST("/robot/home", {
        body: speed !== undefined ? { speed } : {},
      }),
    );
  }

  async stop() {
    return this.unwrap(await this.client.POST("/robot/stop"));
  }

  async waitForMovement(timeout?: number) {
    return this.unwrap(
      await this.client.POST("/robot/wait", {
        body: timeout !== undefined ? { timeout } : {},
      }),
    );
  }

  private gripperActionBody(speed?: number, gripperType?: number) {
    return {
      ...(speed !== undefined ? { speed } : {}),
      ...(gripperType !== undefined ? { gripper_type: gripperType } : {}),
    };
  }
}
