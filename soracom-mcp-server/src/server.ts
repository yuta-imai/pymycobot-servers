import { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import { SoracomClient } from "./api-client.js";
import { registerTools } from "./tools.js";

export interface ServerOptions {
  baseUrl: string;
  authKeyId: string;
  authKey: string;
  tokenTimeoutSeconds?: number;
  defaultDeviceId?: string;
}

export function createServer(options: ServerOptions): McpServer {
  const server = new McpServer({
    name: "soracom-mcp-server",
    version: "0.1.0",
  });

  const client = new SoracomClient({
    baseUrl: options.baseUrl,
    authKeyId: options.authKeyId,
    authKey: options.authKey,
    tokenTimeoutSeconds: options.tokenTimeoutSeconds,
  });
  registerTools(server, client, options.defaultDeviceId);

  return server;
}
