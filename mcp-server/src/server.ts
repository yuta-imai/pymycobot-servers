import { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import { MyCobotClient } from "./api-client.js";
import { registerTools } from "./tools.js";
import { registerWazaTools, type WazaOptions } from "./waza-tools.js";
import { WazaStore } from "./waza.js";

export interface ServerOptions {
  /** Base URL of the MyCobot REST API, e.g. http://localhost:8080 */
  apiBaseUrl: string;
  /** Per-request timeout in ms; omit to use the client default. */
  timeoutMs?: number;
  /**
   * Path to the waza (taught-motion) JSON file. When set, the waza tools are
   * registered and the file is watched for external edits.
   */
  wazaFile?: string;
  /** Waza playback/exposure settings. Ignored when `wazaFile` is unset. */
  waza?: Partial<WazaOptions>;
}

const WAZA_DEFAULTS: WazaOptions = {
  defaultSpeed: 40,
  gripperType: undefined,
  dynamicTools: true,
  poseTimeout: 15,
};

/**
 * Build a fully-configured MCP server wired to the MyCobot REST API.
 *
 * Async because the waza file is read before the server is connected, so the
 * very first `tools/list` already includes everything the robot has been
 * taught — no empty-then-populated flicker in the client.
 */
export async function createServer({
  apiBaseUrl,
  timeoutMs,
  wazaFile,
  waza,
}: ServerOptions): Promise<McpServer> {
  const server = new McpServer({
    name: "mycobot-mcp-server",
    version: "0.2.0",
  });

  const client = new MyCobotClient(apiBaseUrl, timeoutMs);
  registerTools(server, client);

  if (wazaFile) {
    const store = new WazaStore(wazaFile);
    const snapshot = await store.load();
    registerWazaTools(server, client, store, { ...WAZA_DEFAULTS, ...waza });
    store.startWatching();

    console.error(
      `[waza] ${wazaFile} から ${snapshot.waza.length} 個の技を読み込みました` +
        (snapshot.missing ? " (ファイルはまだありません)" : ""),
    );
    for (const issue of snapshot.issues) {
      console.error(`[waza] 読み飛ばし: ${issue.where}: ${issue.message}`);
    }
  }

  return server;
}
