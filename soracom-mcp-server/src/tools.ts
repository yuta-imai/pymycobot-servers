import type { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import type { CallToolResult } from "@modelcontextprotocol/sdk/types.js";
import { z } from "zod";
import { ApiError, SoracomClient } from "./api-client.js";

const subscriptionId = z
  .string()
  .min(1)
  .describe("ライブ視聴対象の subscriptionId (SIM ID など) を指定します。");

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

export function registerTools(server: McpServer, client: SoracomClient): void {
  server.registerTool(
    "get_live_view_unlimited",
    {
      title: "ライブ視聴見放題 API 呼び出し",
      description:
        "ライブ視聴見放題 API を呼び出してライブ視聴 URL などを取得します。",
      inputSchema: {
        subscriptionId,
        expiresInSeconds: z
          .number()
          .int()
          .min(1)
          .max(86400)
          .describe("URL の有効期限秒。API が対応する場合のみ使用してください。")
          .optional(),
      },
    },
    ({ subscriptionId, expiresInSeconds }) =>
      run(() => client.getLiveView({ subscriptionId, expiresInSeconds })),
  );

  server.registerTool(
    "get_live_still_image",
    {
      title: "ライブ静止画取得 API 呼び出し",
      description:
        "ライブ静止画取得 API を呼び出し、画像を Base64 付き JSON で返します。",
      inputSchema: {
        subscriptionId,
        width: z
          .number()
          .int()
          .min(1)
          .max(7680)
          .describe("取得したい静止画の横幅ピクセル。")
          .optional(),
        height: z
          .number()
          .int()
          .min(1)
          .max(4320)
          .describe("取得したい静止画の縦幅ピクセル。")
          .optional(),
      },
    },
    ({ subscriptionId, width, height }) =>
      run(() => client.getLiveStillImage({ subscriptionId, width, height })),
  );
}
