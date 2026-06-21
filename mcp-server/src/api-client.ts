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
/** Default per-request timeout (ms). Bounds how long any tool can hang if the
 *  robot/serial stops responding, instead of waiting minutes for a TCP/MCP
 *  timeout. Override with the `timeoutMs` constructor arg. */
export const DEFAULT_REQUEST_TIMEOUT_MS = 30_000;

export class MyCobotClient {
  private readonly client: Client<paths>;

  constructor(
    private readonly baseUrl: string,
    private readonly timeoutMs: number = DEFAULT_REQUEST_TIMEOUT_MS,
  ) {
    this.client = createClient<paths>({
      baseUrl,
      fetch: (request: Request) => this.fetchWithTimeout(request),
    });
  }

  /** Wrap fetch with an abort-based timeout so no request hangs indefinitely. */
  private async fetchWithTimeout(request: Request): Promise<Response> {
    try {
      return await fetch(request, { signal: AbortSignal.timeout(this.timeoutMs) });
    } catch (err) {
      if (err instanceof Error && err.name === "TimeoutError") {
        throw new ApiError(
          `Request timed out after ${this.timeoutMs}ms — the robot may be busy or unresponsive (try get_robot_status, then stop_robot).`,
        );
      }
      throw err;
    }
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

  async getStatus(includeError?: boolean) {
    return this.unwrap(
      await this.client.GET("/robot/status", {
        params: {
          query: includeError !== undefined ? { include_error: includeError } : {},
        },
      }),
    );
  }

  async getGripperStatus(gripperType?: number) {
    return this.unwrap(
      await this.client.GET("/gripper/status", {
        params: {
          query: gripperType !== undefined ? { gripper_type: gripperType } : {},
        },
      }),
    );
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

  async jogJoint(
    joint: number,
    direction: 1 | -1,
    speed?: number,
    increment?: number,
  ) {
    return this.unwrap(
      await this.client.POST("/joints/{joint_num}/jog", {
        params: { path: { joint_num: joint } },
        body: {
          direction,
          ...(speed !== undefined ? { speed } : {}),
          ...(increment !== undefined ? { increment } : {}),
        },
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

  async releaseAllServos() {
    return this.unwrap(await this.client.POST("/robot/release"));
  }

  async moveTopdown(x: number, y: number, z: number, speed?: number) {
    return this.unwrap(
      await this.client.POST("/robot/topdown", {
        body: { x, y, z, ...(speed !== undefined ? { speed } : {}) },
      }),
    );
  }

  async powerOn() {
    return this.unwrap(await this.client.POST("/robot/power_on"));
  }

  async waitForMovement(timeout?: number, tolerance?: number) {
    return this.unwrap(
      await this.client.POST("/robot/wait", {
        body: {
          ...(timeout !== undefined ? { timeout } : {}),
          ...(tolerance !== undefined ? { tolerance } : {}),
        },
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
