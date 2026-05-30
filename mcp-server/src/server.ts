import { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import { MyCobotClient } from "./api-client.js";
import { registerTools } from "./tools.js";

export interface ServerOptions {
  /** Base URL of the MyCobot REST API, e.g. http://localhost:8080 */
  apiBaseUrl: string;
}

/** Build a fully-configured MCP server wired to the MyCobot REST API. */
export function createServer({ apiBaseUrl }: ServerOptions): McpServer {
  const server = new McpServer({
    name: "mycobot-mcp-server",
    version: "0.1.0",
  });

  const client = new MyCobotClient(apiBaseUrl);
  registerTools(server, client);

  return server;
}
