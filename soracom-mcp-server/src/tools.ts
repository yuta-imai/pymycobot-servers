import type { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import type { CallToolResult } from "@modelcontextprotocol/sdk/types.js";
import { z } from "zod";
import { ApiError, SoracomClient } from "./api-client.js";

const deviceIdSchema = z
  .string()
  .min(1)
  .describe(
    "操作対象のソラカメ対応カメラのデバイス ID。省略時は環境変数 SORACOM_DEVICE_ID を使用します。",
  )
  .optional();

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
          ? `Could not reach SORACOM API: ${err.message}`
          : String(err);
    return {
      content: [{ type: "text", text: message }],
      isError: true,
    };
  }
}

export function registerTools(
  server: McpServer,
  client: SoracomClient,
  defaultDeviceId?: string,
): void {
  const resolveDeviceId = (deviceId?: string): string => {
    const resolved = deviceId ?? defaultDeviceId;
    if (!resolved) {
      throw new ApiError(
        "デバイス ID が指定されていません。ツール引数 deviceId を指定するか、環境変数 SORACOM_DEVICE_ID を設定してください。",
      );
    }
    return resolved;
  };

  server.registerTool(
    "get_live_view_unlimited",
    {
      title: "ライブ視聴 URL 取得",
      description:
        "ソラカメ対応カメラのライブ動画を再生する URL (MPEG-DASH、約60秒間有効) を取得します。",
      inputSchema: {
        deviceId: deviceIdSchema,
      },
    },
    ({ deviceId }) =>
      run(() => client.getLiveView({ deviceId: resolveDeviceId(deviceId) })),
  );

  server.registerTool(
    "get_live_still_image",
    {
      title: "ライブ静止画取得",
      description:
        "ソラカメ対応カメラのライブ静止画 (JPEG) を撮影し、Base64 付き JSON で返します。撮影準備のため約15秒かかります。",
      inputSchema: {
        deviceId: deviceIdSchema,
      },
    },
    ({ deviceId }) =>
      run(() =>
        client.getLiveStillImage({ deviceId: resolveDeviceId(deviceId) }),
      ),
  );
}
