import { createHash } from "node:crypto";
import { existsSync, watch, type FSWatcher } from "node:fs";
import { mkdir, readFile, rename, writeFile } from "node:fs/promises";
import { dirname, join } from "node:path";
import { z } from "zod";

/**
 * "Waza" (技) = a motion taught by a human, stored as joint angles plus a
 * plain-language description of *when* to use it.
 *
 * The description (`setsumei`) is the whole point: it is what an LLM reads to
 * decide whether this motion fits the user's request. Editing it changes the
 * robot's behaviour without touching any code — which is the intended lesson.
 */

const ANGLE_LIMITS: ReadonlyArray<readonly [number, number]> = [
  [-168, 168], // J1 base
  [-135, 135], // J2 shoulder
  [-150, 150], // J3 elbow
  [-145, 145], // J4 wrist 1
  [-155, 160], // J5 wrist 2
  [-180, 180], // J6 wrist 3
];

/** Playback caps. Keep a taught motion bounded so a typo cannot run forever. */
export const MAX_POSES = 20;
export const MAX_REPEAT = 5;
export const MAX_HOLD_MS = 5_000;

const anglesSchema = z
  .array(z.number())
  .length(6)
  .superRefine((angles, ctx) => {
    angles.forEach((a, i) => {
      const [lo, hi] = ANGLE_LIMITS[i]!;
      if (a < lo || a > hi) {
        ctx.addIssue({
          code: z.ZodIssueCode.custom,
          message: `J${i + 1} の角度 ${a}° が可動域 (${lo}°〜${hi}°) の外です`,
        });
      }
    });
  });

const poseSchema = z.object({
  angles: anglesSchema,
  speed: z.number().int().min(1).max(100).optional(),
  gripper: z.number().int().min(0).max(100).optional(),
  hold_ms: z.number().int().min(0).max(MAX_HOLD_MS).optional(),
});

/** One entry as it may appear in waza.json. `angles` and `poses` are alternatives. */
const rawWazaSchema = z
  .object({
    id: z
      .string()
      .regex(/^[a-zA-Z0-9_-]{1,48}$/)
      .optional(),
    name: z.string().min(1).max(48),
    setsumei: z.string().max(600).default(""),
    angles: anglesSchema.optional(),
    poses: z.array(poseSchema).min(1).max(MAX_POSES).optional(),
    speed: z.number().int().min(1).max(100).optional(),
    gripper: z.number().int().min(0).max(100).optional(),
    repeat: z.number().int().min(1).max(MAX_REPEAT).optional(),
  })
  .refine((w) => w.angles !== undefined || w.poses !== undefined, {
    message: "`angles` か `poses` のどちらかが必要です",
  });

/** Accept either `{ "waza": [...] }` or a bare `[...]`. */
const fileSchema = z.union([
  z.object({ version: z.number().optional(), waza: z.array(rawWazaSchema) }),
  z.array(rawWazaSchema),
]);

export type Pose = z.infer<typeof poseSchema>;

export interface Waza {
  /** Stable ASCII handle. Used as the dynamic MCP tool name. */
  id: string;
  /** Human-facing name, typically Japanese. */
  name: string;
  /** The child's own explanation of when this motion should be used. */
  setsumei: string;
  poses: Pose[];
  repeat: number;
}

/** Raised for waza-file problems that are the user's to fix, not the robot's. */
export class WazaError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "WazaError";
  }
}

/** Problems found while loading, surfaced to the user instead of thrown away. */
export interface WazaLoadIssue {
  where: string;
  message: string;
}

export interface WazaSnapshot {
  waza: Waza[];
  issues: WazaLoadIssue[];
  /** True when the file does not exist yet (not an error — nothing taught yet). */
  missing: boolean;
}

/** Deterministic id so hand-written entries keep the same tool name across reloads. */
function deriveId(name: string): string {
  const hash = createHash("sha1").update(name).digest("hex").slice(0, 8);
  return `waza_${hash}`;
}

function normalize(raw: z.infer<typeof rawWazaSchema>): Waza {
  const poses: Pose[] =
    raw.poses ??
    [
      {
        angles: raw.angles!,
        ...(raw.speed !== undefined ? { speed: raw.speed } : {}),
        ...(raw.gripper !== undefined ? { gripper: raw.gripper } : {}),
      },
    ];

  // Entry-level speed/gripper act as defaults for poses that omit them.
  const filled = poses.map((p) => ({
    ...p,
    speed: p.speed ?? raw.speed,
    gripper: p.gripper ?? raw.gripper,
  }));

  return {
    id: raw.id ?? deriveId(raw.name),
    name: raw.name,
    setsumei: raw.setsumei,
    poses: filled,
    repeat: raw.repeat ?? 1,
  };
}

/**
 * Reads and writes the waza file, and reloads it whenever it changes on disk.
 *
 * The file is deliberately the single source of truth: the teaching script
 * (`scripts/teach_waza.py`) and a child with a text editor both write to it,
 * and the server picks up edits without a restart.
 */
export class WazaStore {
  private snapshot: WazaSnapshot = { waza: [], issues: [], missing: true };
  private watcher?: FSWatcher;
  private dirWatcher?: FSWatcher;
  private debounce?: NodeJS.Timeout;
  private listeners = new Set<(s: WazaSnapshot) => void>();

  constructor(public readonly filePath: string) {}

  get current(): WazaSnapshot {
    return this.snapshot;
  }

  find(idOrName: string): Waza | undefined {
    const needle = idOrName.trim();
    return this.snapshot.waza.find(
      (w) => w.id === needle || w.name === needle || w.name.trim() === needle,
    );
  }

  /** Register a callback fired after every successful (re)load. */
  onChange(fn: (s: WazaSnapshot) => void): void {
    this.listeners.add(fn);
  }

  async load(): Promise<WazaSnapshot> {
    let text: string;
    try {
      text = await readFile(this.filePath, "utf8");
    } catch (err) {
      const missing = (err as NodeJS.ErrnoException).code === "ENOENT";
      this.snapshot = {
        waza: [],
        missing,
        issues: missing
          ? []
          : [{ where: this.filePath, message: `読み込みに失敗: ${String(err)}` }],
      };
      return this.snapshot;
    }

    if (text.trim() === "") {
      this.snapshot = { waza: [], issues: [], missing: false };
      return this.snapshot;
    }

    let json: unknown;
    try {
      json = JSON.parse(text);
    } catch (err) {
      // Keep the previously loaded waza so a half-saved file does not wipe the
      // robot's repertoire mid-session.
      this.snapshot = {
        ...this.snapshot,
        missing: false,
        issues: [
          {
            where: this.filePath,
            message: `JSON として読めません (カンマや括弧を確認してください): ${(err as Error).message}`,
          },
        ],
      };
      return this.snapshot;
    }

    const parsed = fileSchema.safeParse(json);
    if (!parsed.success) {
      // Fall back to per-entry parsing so one bad waza does not disable the rest.
      const entries = Array.isArray(json)
        ? json
        : ((json as { waza?: unknown[] }).waza ?? []);
      const waza: Waza[] = [];
      const issues: WazaLoadIssue[] = [];
      entries.forEach((entry, i) => {
        const one = rawWazaSchema.safeParse(entry);
        const label =
          (entry as { name?: string } | null)?.name ?? `${i + 1} 番目の技`;
        if (one.success) {
          waza.push(normalize(one.data));
        } else {
          issues.push({
            where: label,
            message: one.error.issues.map((e) => e.message).join(" / "),
          });
        }
      });
      this.snapshot = { waza: dedupe(waza, issues), issues, missing: false };
      this.emit();
      return this.snapshot;
    }

    const entries = Array.isArray(parsed.data) ? parsed.data : parsed.data.waza;
    const issues: WazaLoadIssue[] = [];
    const waza = dedupe(entries.map(normalize), issues);
    this.snapshot = { waza, issues, missing: false };
    this.emit();
    return this.snapshot;
  }

  /** Insert or replace a waza by name, then persist. Returns the stored entry. */
  async save(input: {
    name: string;
    setsumei: string;
    poses: Pose[];
    repeat?: number;
  }): Promise<Waza> {
    await this.load();
    const entry: Waza = {
      id: this.find(input.name)?.id ?? deriveId(input.name),
      name: input.name,
      setsumei: input.setsumei,
      poses: input.poses,
      repeat: input.repeat ?? 1,
    };
    const next = this.snapshot.waza.filter((w) => w.name !== input.name);
    next.push(entry);
    await this.write(next);
    return entry;
  }

  /** Remove a waza by id or name. Returns the removed entry, if any. */
  async remove(idOrName: string): Promise<Waza | undefined> {
    await this.load();
    const target = this.find(idOrName);
    if (!target) return undefined;
    await this.write(this.snapshot.waza.filter((w) => w.id !== target.id));
    return target;
  }

  private async write(waza: Waza[]): Promise<void> {
    // Rewriting the file would drop entries that failed to parse, so refuse
    // until they are fixed rather than quietly deleting someone's work.
    if (this.snapshot.issues.length > 0) {
      const detail = this.snapshot.issues
        .map((i) => `・${i.where}: ${i.message}`)
        .join("\n");
      throw new WazaError(
        `${this.filePath} に直せていないところがあるので、保存できません。\n${detail}\n` +
          "先にファイルを直してから、もういちど試してください。",
      );
    }

    await mkdir(dirname(this.filePath), { recursive: true });
    const body = JSON.stringify({ version: 1, waza }, null, 2) + "\n";
    // Write-then-rename so a reader never sees a truncated file.
    const tmp = `${this.filePath}.tmp-${process.pid}`;
    await writeFile(tmp, body, "utf8");
    await rename(tmp, this.filePath);
    this.snapshot = { waza, issues: [], missing: false };
    this.emit();
  }

  /**
   * Watch the file for external edits. Watches the parent directory too, since
   * editors that save via rename break a watch bound to the file inode.
   */
  startWatching(): void {
    const dir = dirname(this.filePath);
    const reload = () => {
      clearTimeout(this.debounce);
      this.debounce = setTimeout(() => {
        void this.load();
      }, 150);
      this.debounce.unref?.();
    };

    try {
      if (existsSync(this.filePath)) {
        this.watcher = watch(this.filePath, reload);
        this.watcher.unref?.();
      }
      if (existsSync(dir)) {
        this.dirWatcher = watch(dir, (_e, filename) => {
          if (!filename || join(dir, filename.toString()) === this.filePath) {
            // Re-bind the file watcher if the file was just created/replaced.
            this.watcher?.close();
            this.watcher = existsSync(this.filePath)
              ? watch(this.filePath, reload)
              : undefined;
            this.watcher?.unref?.();
            reload();
          }
        });
        this.dirWatcher.unref?.();
      }
    } catch (err) {
      console.error(`[waza] ファイル監視を開始できませんでした: ${String(err)}`);
    }
  }

  stopWatching(): void {
    clearTimeout(this.debounce);
    this.watcher?.close();
    this.dirWatcher?.close();
  }

  private emit(): void {
    for (const fn of this.listeners) {
      try {
        fn(this.snapshot);
      } catch (err) {
        console.error(`[waza] リロード通知でエラー: ${String(err)}`);
      }
    }
  }
}

/** Drop duplicate ids (same name taught twice), keeping the last definition. */
function dedupe(waza: Waza[], issues: WazaLoadIssue[]): Waza[] {
  const byId = new Map<string, Waza>();
  for (const w of waza) {
    if (byId.has(w.id)) {
      issues.push({
        where: w.name,
        message: "同じ名前の技が複数あります。あとの方だけを使います。",
      });
    }
    byId.set(w.id, w);
  }
  return [...byId.values()];
}
