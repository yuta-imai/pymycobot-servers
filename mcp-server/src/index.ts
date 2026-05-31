import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import { createServer } from "./server.js";

const DEFAULT_API_BASE_URL = "http://localhost:8080";

/** Resolve the REST API base URL from `--api-base-url` or `MYCOBOT_API_BASE_URL`. */
function resolveApiBaseUrl(argv: string[]): string {
  const flagIndex = argv.indexOf("--api-base-url");
  if (flagIndex !== -1 && argv[flagIndex + 1]) {
    return argv[flagIndex + 1];
  }
  const inline = argv.find((a) => a.startsWith("--api-base-url="));
  if (inline) {
    return inline.slice("--api-base-url=".length);
  }
  return process.env.MYCOBOT_API_BASE_URL ?? DEFAULT_API_BASE_URL;
}

/** Optional per-request timeout (ms) from `MYCOBOT_API_TIMEOUT_MS`. */
function resolveTimeoutMs(): number | undefined {
  const raw = process.env.MYCOBOT_API_TIMEOUT_MS;
  if (!raw) return undefined;
  const value = Number(raw);
  return Number.isFinite(value) && value > 0 ? value : undefined;
}

async function main(): Promise<void> {
  const apiBaseUrl = resolveApiBaseUrl(process.argv.slice(2));
  const server = createServer({ apiBaseUrl, timeoutMs: resolveTimeoutMs() });

  const transport = new StdioServerTransport();
  await server.connect(transport);

  // stdout is reserved for the MCP protocol; log to stderr.
  console.error(`mycobot-mcp-server ready (REST API: ${apiBaseUrl})`);
}

main().catch((err) => {
  console.error("Fatal error starting mycobot-mcp-server:", err);
  process.exit(1);
});
