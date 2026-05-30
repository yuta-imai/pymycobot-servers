import { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import { SoracomClient } from "./api-client.js";
import { registerTools } from "./tools.js";

export interface ServerOptions {
  apiBaseUrl: string;
  apiKey?: string;
  apiToken?: string;
  bearerToken?: string;
  liveViewPathTemplate: string;
  stillImagePathTemplate: string;
}

export function createServer(options: ServerOptions): McpServer {
  const server = new McpServer({
    name: "soracom-mcp-server",
    version: "0.1.0",
  });

  const client = new SoracomClient({
    baseUrl: options.apiBaseUrl,
    apiKey: options.apiKey,
    apiToken: options.apiToken,
    bearerToken: options.bearerToken,
    liveViewPathTemplate: options.liveViewPathTemplate,
    stillImagePathTemplate: options.stillImagePathTemplate,
  });
  registerTools(server, client);

  return server;
}
