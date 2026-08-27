import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import { homedir } from "node:os";
import { isAbsolute, resolve } from "node:path";
import { createServer } from "./server.js";

const DEFAULT_API_BASE_URL = "http://localhost:8080";

/** Read `--flag value` or `--flag=value` from argv. */
function readFlag(argv: string[], flag: string): string | undefined {
  const index = argv.indexOf(flag);
  if (index !== -1 && argv[index + 1]) return argv[index + 1];
  const inline = argv.find((a) => a.startsWith(`${flag}=`));
  return inline?.slice(flag.length + 1);
}

/** Resolve the REST API base URL from `--api-base-url` or `MYCOBOT_API_BASE_URL`. */
function resolveApiBaseUrl(argv: string[]): string {
  return (
    readFlag(argv, "--api-base-url") ??
    process.env.MYCOBOT_API_BASE_URL ??
    DEFAULT_API_BASE_URL
  );
}

/** Optional per-request timeout (ms) from `MYCOBOT_API_TIMEOUT_MS`. */
function resolveTimeoutMs(): number | undefined {
  const raw = process.env.MYCOBOT_API_TIMEOUT_MS;
  if (!raw) return undefined;
  const value = Number(raw);
  return Number.isFinite(value) && value > 0 ? value : undefined;
}

/** Expand a leading `~` so config files can use a home-relative path. */
function expandHome(p: string): string {
  return p.startsWith("~/") ? resolve(homedir(), p.slice(2)) : p;
}

/**
 * Path to the taught-motion file. Unset means the waza feature stays off, so
 * existing deployments are unaffected by this addition.
 */
function resolveWazaFile(argv: string[]): string | undefined {
  const raw = readFlag(argv, "--waza-file") ?? process.env.MYCOBOT_WAZA_FILE;
  if (!raw) return undefined;
  const expanded = expandHome(raw);
  return isAbsolute(expanded) ? expanded : resolve(process.cwd(), expanded);
}

function envNumber(name: string, fallback: number): number {
  const value = Number(process.env[name]);
  return Number.isFinite(value) && value > 0 ? value : fallback;
}

function envBool(name: string, fallback: boolean): boolean {
  const raw = process.env[name]?.trim().toLowerCase();
  if (raw === undefined || raw === "") return fallback;
  return !["0", "false", "no", "off"].includes(raw);
}

async function main(): Promise<void> {
  const argv = process.argv.slice(2);
  const apiBaseUrl = resolveApiBaseUrl(argv);
  const wazaFile = resolveWazaFile(argv);

  const server = await createServer({
    apiBaseUrl,
    timeoutMs: resolveTimeoutMs(),
    wazaFile,
    waza: {
      defaultSpeed: envNumber("MYCOBOT_WAZA_SPEED", 40),
      poseTimeout: envNumber("MYCOBOT_WAZA_POSE_TIMEOUT", 15),
      dynamicTools: envBool("MYCOBOT_WAZA_DYNAMIC_TOOLS", true),
      gripperType: process.env.MYCOBOT_GRIPPER_TYPE
        ? Number(process.env.MYCOBOT_GRIPPER_TYPE)
        : undefined,
    },
  });

  const transport = new StdioServerTransport();
  await server.connect(transport);

  // stdout is reserved for the MCP protocol; log to stderr.
  console.error(
    `mycobot-mcp-server ready (REST API: ${apiBaseUrl}` +
      (wazaFile ? `, waza: ${wazaFile}` : ", waza: off") +
      ")",
  );
}

main().catch((err) => {
  console.error("Fatal error starting mycobot-mcp-server:", err);
  process.exit(1);
});
