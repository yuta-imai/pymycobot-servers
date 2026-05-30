import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import { createServer } from "./server.js";

const DEFAULT_API_BASE_URL = "https://api.soracom.io";
const DEFAULT_LIVE_VIEW_PATH = "/v1/livestream/subscriptions/{subscriptionId}/view";
const DEFAULT_STILL_IMAGE_PATH = "/v1/livestream/subscriptions/{subscriptionId}/image";

function findArg(argv: string[], key: string): string | undefined {
  const flagIndex = argv.indexOf(key);
  if (flagIndex !== -1 && argv[flagIndex + 1]) {
    return argv[flagIndex + 1];
  }

  const inline = argv.find((a) => a.startsWith(`${key}=`));
  if (inline) {
    return inline.slice(`${key}=`.length);
  }

  return undefined;
}

async function main(): Promise<void> {
  const args = process.argv.slice(2);

  const apiBaseUrl =
    findArg(args, "--api-base-url") ??
    process.env.SORACOM_API_BASE_URL ??
    DEFAULT_API_BASE_URL;

  const liveViewPathTemplate =
    findArg(args, "--live-view-path") ??
    process.env.SORACOM_LIVE_VIEW_PATH ??
    DEFAULT_LIVE_VIEW_PATH;

  const stillImagePathTemplate =
    findArg(args, "--still-image-path") ??
    process.env.SORACOM_STILL_IMAGE_PATH ??
    DEFAULT_STILL_IMAGE_PATH;

  const server = createServer({
    apiBaseUrl,
    apiKey: process.env.SORACOM_API_KEY,
    apiToken: process.env.SORACOM_TOKEN,
    bearerToken: process.env.SORACOM_BEARER_TOKEN,
    liveViewPathTemplate,
    stillImagePathTemplate,
  });

  const transport = new StdioServerTransport();
  await server.connect(transport);

  console.error(`soracom-mcp-server ready (SORACOM API: ${apiBaseUrl})`);
}

main().catch((err) => {
  console.error("Fatal error starting soracom-mcp-server:", err);
  process.exit(1);
});
