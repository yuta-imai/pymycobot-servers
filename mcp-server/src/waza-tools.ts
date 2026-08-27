import type { McpServer, RegisteredTool } from "@modelcontextprotocol/sdk/server/mcp.js";
import type { CallToolResult } from "@modelcontextprotocol/sdk/types.js";
import { z } from "zod";
import { ApiError, MyCobotClient } from "./api-client.js";
import {
  MAX_POSES,
  MAX_REPEAT,
  MAX_HOLD_MS,
  WazaError,
  WazaStore,
  type Pose,
  type Waza,
} from "./waza.js";

export interface WazaOptions {
  /** Default playback speed. Below ~20 J2 stalls under its own weight. */
  defaultSpeed: number;
  /** Gripper type passed to the REST API (3 = parallel gripper). */
  gripperType?: number;
  /**
   * When true, every taught waza is exposed as its own MCP tool whose
   * description is the child's own sentence. This is the point of the exercise:
   * the description is what the model selects on.
   */
  dynamicTools: boolean;
  /** Per-pose convergence timeout in seconds. */
  poseTimeout: number;
}

const MIN_SAFE_SPEED = 20;

const keepRelaxed = z
  .boolean()
  .describe(
    "true にすると、記録したあとも腕の力を抜いたままにします。続けて別のポーズを記録するときだけ true。" +
      "省略すると記録後にサーボを入れなおして、腕が形を保つようにします。",
  )
  .optional();

function ok(data: unknown): CallToolResult {
  return { content: [{ type: "text", text: JSON.stringify(data, null, 2) }] };
}

function fail(message: string): CallToolResult {
  return { content: [{ type: "text", text: message }], isError: true };
}

async function run(fn: () => Promise<CallToolResult>): Promise<CallToolResult> {
  try {
    return await fn();
  } catch (err) {
    if (err instanceof WazaError) return fail(err.message);
    if (err instanceof ApiError) return fail(err.message);
    if (err instanceof Error) {
      return fail(`ロボットのREST APIに届きませんでした: ${err.message}`);
    }
    return fail(String(err));
  }
}

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

interface PlaybackStep {
  pose: number;
  round: number;
  angles: number[];
  speed: number;
  reason: string;
  elapsed_time: number;
  max_error?: number | null;
  gripper?: number;
}

/**
 * Replay a taught motion pose by pose, waiting for convergence between poses.
 *
 * Playback stops at the first pose that stalls or times out rather than piling
 * further commands onto an arm that is already stuck — and reports which pose
 * failed, which is usually a torque problem on J2 at low speed.
 */
async function playWaza(
  client: MyCobotClient,
  waza: Waza,
  opts: WazaOptions,
  speedOverride?: number,
): Promise<{ ok: boolean; steps: PlaybackStep[]; message: string }> {
  const steps: PlaybackStep[] = [];

  for (let round = 1; round <= waza.repeat; round++) {
    for (const [i, pose] of waza.poses.entries()) {
      const speed = Math.max(
        MIN_SAFE_SPEED,
        speedOverride ?? pose.speed ?? opts.defaultSpeed,
      );

      await client.moveAllJoints(pose.angles, speed);
      const wait = await client.waitForMovement(opts.poseTimeout, 2.0);

      const step: PlaybackStep = {
        pose: i + 1,
        round,
        angles: pose.angles,
        speed,
        reason: wait.reason,
        elapsed_time: wait.elapsed_time,
        max_error: wait.max_error,
      };

      if (wait.reason === "stalled" || wait.reason === "timeout") {
        steps.push(step);
        return {
          ok: false,
          steps,
          message:
            `「${waza.name}」の ${i + 1} 番目のポーズで腕が止まりました (${wait.reason})。` +
            `速度 ${speed} では関節のトルクが足りないか、ポーズに無理があります。` +
            `速度を上げる (speed: 50) か、そのポーズを教えなおしてみてください。`,
        };
      }

      if (pose.gripper !== undefined) {
        await client.gripperSetValue(pose.gripper, speed, opts.gripperType);
        step.gripper = pose.gripper;
      }

      steps.push(step);
      if (pose.hold_ms) await sleep(Math.min(pose.hold_ms, MAX_HOLD_MS));
    }
  }

  return {
    ok: true,
    steps,
    message: `「${waza.name}」をやりました。`,
  };
}

/**
 * Read the arm's current pose, then re-engage the servos.
 *
 * Teaching happens with the servos released, so the arm is limp and held up by
 * a person at the moment of capture. Powering back on is in a `finally` because
 * leaving the arm limp is a physical hazard that must not depend on whether the
 * save succeeded, or on the model remembering a follow-up call.
 */
async function captureCurrentPose(
  client: MyCobotClient,
  keepRelaxed: boolean,
): Promise<number[]> {
  try {
    return (await client.getAllJointAngles()).angles;
  } finally {
    if (!keepRelaxed) {
      try {
        await client.powerOn();
      } catch (err) {
        console.error(`[waza] サーボの再投入に失敗しました: ${String(err)}`);
      }
    }
  }
}

/** Tell the human whether they can let go of the arm. */
function armState(anglesWereGiven: boolean, keepRelaxed: boolean): string {
  if (anglesWereGiven) return "腕には触っていません。";
  return keepRelaxed
    ? "腕は力が抜けたままです。支えつづけてください。"
    : "サーボを入れなおしました。手を離して大丈夫です。";
}

/** What the model sees when it asks what the robot knows how to do. */
function describe(w: Waza) {
  return {
    name: w.name,
    setsumei: w.setsumei || "(せつめいが まだ かかれていません)",
    poses: w.poses.length,
    repeat: w.repeat,
  };
}

export function registerWazaTools(
  server: McpServer,
  client: MyCobotClient,
  store: WazaStore,
  opts: WazaOptions,
): void {
  // ---------------------------------------------------------------- listing

  server.registerTool(
    "list_waza",
    {
      title: "教わった技の一覧",
      description:
        "このロボットが人間から教わった技（わざ）の一覧を返します。各技には、それを教えた人が書いた `setsumei`（どんなときに使う動きなのかの説明）が付いています。" +
        "ユーザーから何か動きを頼まれたら、まずこれを呼んで、`setsumei` を読んで一番合う技を選び、do_waza で実行してください。" +
        "どの説明にも当てはまらない場合は、勝手に近そうな技を選ばず「その動きはまだ教わっていない」と伝えてください。",
      inputSchema: {},
    },
    async () =>
      run(async () => {
        const snap = await store.load();
        return ok({
          file: store.filePath,
          count: snap.waza.length,
          waza: snap.waza.map(describe),
          ...(snap.missing
            ? { note: "まだ技のファイルがありません。save_waza で最初の技を教えてください。" }
            : {}),
          ...(snap.issues.length ? { problems: snap.issues } : {}),
        });
      }),
  );

  // -------------------------------------------------------------- execution

  server.registerTool(
    "do_waza",
    {
      title: "技をやる",
      description:
        "教わった技を名前で指定して実行します。名前は list_waza が返したものと完全に一致させてください。" +
        "推測で名前を作らないこと。ロボットが知らない名前を渡すと、知っている技の一覧が返ります。",
      inputSchema: {
        name: z.string().describe("list_waza が返した技の名前。"),
        speed: z
          .number()
          .int()
          .min(MIN_SAFE_SPEED)
          .max(100)
          .describe(
            `動く速さ (${MIN_SAFE_SPEED}-100)。省略すると技に保存された速さを使います。`,
          )
          .optional(),
      },
    },
    async ({ name, speed }) =>
      run(async () => {
        await store.load();
        const waza = store.find(name);
        if (!waza) {
          return fail(
            `「${name}」という技は教わっていません。知っているのは: ` +
              (store.current.waza.map((w) => w.name).join("、") || "(まだ何もありません)"),
          );
        }
        const result = await playWaza(client, waza, opts, speed);
        const payload = {
          waza: waza.name,
          setsumei: waza.setsumei,
          message: result.message,
          steps: result.steps,
        };
        return result.ok ? ok(payload) : fail(JSON.stringify(payload, null, 2));
      }),
  );

  // --------------------------------------------------------------- teaching

  server.registerTool(
    "save_waza",
    {
      title: "今の形を技として覚える",
      description:
        "ロボットに新しい技を覚えさせます。`angles` を省略すると、いまの腕の形（手で動かした形でもOK）をそのまま記録します。" +
        "`setsumei` には「どんなときに使う動きなのか」を書いてもらってください。この文章を読んでAIが技を選ぶので、" +
        "形の説明（りょうてをあげる）よりも、使う場面（うれしいとき、あいさつするとき）が書いてある方がうまく選べます。" +
        "同じ名前で保存すると上書きされます。\n" +
        "手で形を作って教えるときの手順: " +
        "(1) 先に「腕を手で支えてください」と伝えて、支えたことを確認する " +
        "(2) release_all_servos で力を抜く " +
        "(3) 形ができたと本人が言ってから save_waza を呼ぶ。" +
        "記録のあとは自動でサーボを入れなおすので、power_on を別に呼ぶ必要はありません。",
      inputSchema: {
        name: z.string().min(1).max(48).describe("技の名前。日本語でよい。"),
        setsumei: z
          .string()
          .max(600)
          .describe("どんなときに使う動きなのかの説明。ユーザー本人の言葉をそのまま使うこと。"),
        angles: z
          .array(z.number())
          .length(6)
          .describe("6つの関節角度(度)。省略すると今の腕の形を使う。")
          .optional(),
        gripper: z
          .number()
          .int()
          .min(0)
          .max(100)
          .describe("ハンドの開き具合 0(閉)〜100(開)。省略すると動かさない。")
          .optional(),
        repeat: z
          .number()
          .int()
          .min(1)
          .max(MAX_REPEAT)
          .describe(`ポーズを繰り返す回数 (1-${MAX_REPEAT})。手を振るなど往復する動きに使う。`)
          .optional(),
        keep_relaxed: keepRelaxed,
      },
    },
    async ({ name, setsumei, angles, gripper, repeat, keep_relaxed }) =>
      run(async () => {
        const resolved =
          angles ?? (await captureCurrentPose(client, keep_relaxed ?? false));
        const pose: Pose = {
          angles: resolved,
          ...(gripper !== undefined ? { gripper } : {}),
        };
        const saved = await store.save({
          name,
          setsumei,
          poses: [pose],
          repeat,
        });
        return ok({
          message: `「${saved.name}」をおぼえました。`,
          saved: describe(saved),
          angles: resolved,
          arm: armState(angles !== undefined, keep_relaxed ?? false),
          file: store.filePath,
        });
      }),
  );

  server.registerTool(
    "add_waza_pose",
    {
      title: "技にポーズを追加する",
      description:
        "すでにある技のうしろにポーズを1つ足します。`angles` を省略するといまの腕の形を使います。" +
        "手を振る・おじぎするなど、いくつかの形を順番に通る動きを教えるときに使います。",
      inputSchema: {
        name: z.string().describe("追加したい技の名前。"),
        angles: z
          .array(z.number())
          .length(6)
          .describe("6つの関節角度(度)。省略すると今の腕の形を使う。")
          .optional(),
        gripper: z.number().int().min(0).max(100).describe("ハンドの開き具合 0-100。").optional(),
        hold_ms: z
          .number()
          .int()
          .min(0)
          .max(MAX_HOLD_MS)
          .describe(`このポーズで止まる時間(ミリ秒, 0-${MAX_HOLD_MS})。`)
          .optional(),
        keep_relaxed: keepRelaxed,
      },
    },
    async ({ name, angles, gripper, hold_ms, keep_relaxed }) =>
      run(async () => {
        await store.load();
        const waza = store.find(name);
        if (!waza) return fail(`「${name}」という技はまだありません。先に save_waza で作ってください。`);
        if (waza.poses.length >= MAX_POSES) {
          return fail(`「${waza.name}」のポーズは上限の ${MAX_POSES} 個に達しています。`);
        }
        const resolved =
          angles ?? (await captureCurrentPose(client, keep_relaxed ?? false));
        const pose: Pose = {
          angles: resolved,
          ...(gripper !== undefined ? { gripper } : {}),
          ...(hold_ms !== undefined ? { hold_ms } : {}),
        };
        const saved = await store.save({
          name: waza.name,
          setsumei: waza.setsumei,
          poses: [...waza.poses, pose],
          repeat: waza.repeat,
        });
        return ok({
          message: `「${saved.name}」に ${saved.poses.length} 番目のポーズを足しました。`,
          saved: describe(saved),
          angles: resolved,
          arm: armState(angles !== undefined, keep_relaxed ?? false),
        });
      }),
  );

  server.registerTool(
    "update_waza_setsumei",
    {
      title: "技の説明を書きかえる",
      description:
        "技の動き（角度）はそのままに、`setsumei` だけを書きかえます。" +
        "説明の書き方でAIの選び方がどう変わるかを試すための道具です。動きは変わりません。",
      inputSchema: {
        name: z.string().describe("書きかえたい技の名前。"),
        setsumei: z.string().max(600).describe("新しい説明。ユーザー本人の言葉をそのまま使うこと。"),
      },
    },
    async ({ name, setsumei }) =>
      run(async () => {
        await store.load();
        const waza = store.find(name);
        if (!waza) return fail(`「${name}」という技はまだありません。`);
        const before = waza.setsumei;
        const saved = await store.save({
          name: waza.name,
          setsumei,
          poses: waza.poses,
          repeat: waza.repeat,
        });
        return ok({
          message: `「${saved.name}」の説明を書きかえました。動きは変わっていません。`,
          before,
          after: saved.setsumei,
        });
      }),
  );

  server.registerTool(
    "forget_waza",
    {
      title: "技を忘れる",
      description:
        "教わった技を1つ消します。取り消せないので、実行する前に必ず本人にどれを消すか確認してください。",
      inputSchema: {
        name: z.string().describe("消したい技の名前。"),
      },
    },
    async ({ name }) =>
      run(async () => {
        const removed = await store.remove(name);
        if (!removed) return fail(`「${name}」という技はありません。`);
        return ok({
          message: `「${removed.name}」を忘れました。`,
          remaining: store.current.waza.map((w) => w.name),
        });
      }),
  );

  // --------------------------------------------------------- dynamic tools

  if (!opts.dynamicTools) return;

  /**
   * Mirror the waza file into real MCP tools, one per waza, so the sentence a
   * child wrote *is* the tool description the model selects on. The SDK emits a
   * tools/list_changed notification on register/remove, so edits to the file
   * show up in the client without a restart.
   */
  const registered = new Map<string, { tool: RegisteredTool; signature: string }>();

  const sync = (waza: Waza[]) => {
    const seen = new Set<string>();

    for (const w of waza) {
      seen.add(w.id);
      const description =
        (w.setsumei || `「${w.name}」という動き。説明はまだ書かれていません。`) +
        `\n(人がロボットに教えた技「${w.name}」。この説明に合う場面でだけ使ってください。)`;
      const signature = `${w.name}\u0000${description}`;
      const existing = registered.get(w.id);

      if (existing) {
        if (existing.signature !== signature) {
          existing.tool.update({ title: `技: ${w.name}`, description });
          registered.set(w.id, { tool: existing.tool, signature });
        }
        continue;
      }

      const tool = server.registerTool(
        w.id,
        { title: `技: ${w.name}`, description, inputSchema: {} },
        async () =>
          run(async () => {
            await store.load();
            const fresh = store.find(w.id) ?? w;
            const result = await playWaza(client, fresh, opts);
            const payload = {
              waza: fresh.name,
              setsumei: fresh.setsumei,
              message: result.message,
              steps: result.steps,
            };
            return result.ok ? ok(payload) : fail(JSON.stringify(payload, null, 2));
          }),
      );
      registered.set(w.id, { tool, signature });
    }

    for (const [id, entry] of registered) {
      if (!seen.has(id)) {
        entry.tool.remove();
        registered.delete(id);
      }
    }
  };

  store.onChange((snap) => sync(snap.waza));
  sync(store.current.waza);
}
