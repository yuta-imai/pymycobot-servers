import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import { createServer } from "./server.js";

const COVERAGE_BASE_URLS: Record<string, string> = {
  jp: "https://api.soracom.io",
  g: "https://g.api.soracom.io",
};
const DEFAULT_COVERAGE = "jp";
const DEFAULT_TOKEN_TIMEOUT_SECONDS = 86400;

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

  const authKeyId =
    findArg(args, "--auth-key-id") ?? process.env.SORACOM_AUTH_KEY_ID;
  const authKey = findArg(args, "--auth-key") ?? process.env.SORACOM_AUTH_KEY;

  if (!authKeyId || !authKey) {
    console.error(
      "Missing SORACOM credentials. Set SORACOM_AUTH_KEY_ID and SORACOM_AUTH_KEY " +
        "(or pass --auth-key-id / --auth-key).",
    );
    process.exit(1);
  }

  const coverage = (
    findArg(args, "--coverage") ??
    process.env.SORACOM_COVERAGE ??
    DEFAULT_COVERAGE
  ).toLowerCase();
  const baseUrl = COVERAGE_BASE_URLS[coverage];
  if (!baseUrl) {
    console.error(
      `Unknown coverage "${coverage}". Use "jp" or "g".`,
    );
    process.exit(1);
  }

  const tokenTimeoutRaw =
    findArg(args, "--token-timeout") ??
    process.env.SORACOM_TOKEN_TIMEOUT_SECONDS;
  const tokenTimeoutSeconds = tokenTimeoutRaw
    ? Number(tokenTimeoutRaw)
    : DEFAULT_TOKEN_TIMEOUT_SECONDS;
  if (!Number.isFinite(tokenTimeoutSeconds) || tokenTimeoutSeconds <= 0) {
    console.error(`Invalid token timeout: "${tokenTimeoutRaw}".`);
    process.exit(1);
  }

  const defaultDeviceId =
    findArg(args, "--device-id") ?? process.env.SORACOM_DEVICE_ID;

  const server = createServer({
    baseUrl,
    authKeyId,
    authKey,
    tokenTimeoutSeconds,
    defaultDeviceId,
  });

  const transport = new StdioServerTransport();
  await server.connect(transport);

  console.error(
    `soracom-mcp-server ready (coverage: ${coverage}, SORACOM API: ${baseUrl})`,
  );
}

main().catch((err) => {
  console.error("Fatal error starting soracom-mcp-server:", err);
  process.exit(1);
});
