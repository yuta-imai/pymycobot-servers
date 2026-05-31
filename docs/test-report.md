# mycobot MCP サーバー 改善ポイントまとめ

実機（MyCobot, REST API `http://192.168.0.136:8080` 経由）に対して全ツールを一通り検証した結果と、改善提案をまとめたもの。

- 検証日: 2026-05-31
- 構成: Claude Desktop (Windows) → `wsl` → node (nodenv shims) → MCP サーバー → REST API → MyCobot 実機
- サーバー: `mycobot-mcp-server` v0.1.0 / REST API v1.0.0

---

## 1. 検証結果サマリ

| 機能 | 結果 | 備考 |
|------|------|------|
| `get_robot_status` | ✅ | 安定。即応答 |
| `get_joint_angles` | ✅ | 正常 |
| `move_joint` | ✅ | J1→30° 正確に到達 |
| `move_all_joints` | ✅ | 6関節同時、目標±0.3°で到達 |
| `go_home` | ✅ | 0°復帰OK |
| `stop_robot` | ✅ | 即停止 |
| `wait_for_movement` | △ | 動作するが完了判定が不正確 |
| `jog_joint` | ❌ | 約4分応答なし・ハング・実際に動かない |
| gripper: open / close / calibrate / set_value | ✅ | 4種正常 |
| gripper: release | ❌ | flag 値バグ（503） |

コア動作（status / 単関節・全関節移動 / 主要グリッパー操作）は安定。修正対象は明確なバグ2件＋判定ロジック・安全性の改善。

---

## 2. 明確なバグ

### 2.1 `jog_joint` がハングする（最優先）

- 呼び出すと応答が返らず、約4分でクライアント側タイムアウト。
- 呼び出し後も対象関節の角度は変化なし（J1 が 19.77° のまま）→ jog は実行されていない。
- 直後の `get_robot_status` は即応答 → **サーバー全体は生きており、jog_joint ハンドラ内でブロッキングが発生**している。
- 原因の見当: pymycobot の連続ジョグ系 API を呼んだまま、停止/完了待ちでブロックしている可能性が高い。
- 対策: 非ブロッキング化（増分移動して即リターン、または内部タイムアウト）。後述の `wait_for_completion` ロジックに乗せ替えるのが理想。

### 2.2 `release`（グリッパー）の flag 値バグ

エラー内容:

```
REST API 503 Service Unavailable: Failed to release gripper:
The data supported by parameter flag is 0 or 1 or 254, but the received value is 10
```

- release ハンドラが pymycobot のグリッパー解放（トルクオフ）に渡す flag に `10` を渡している。
- 仕様上 flag は `0 / 1 / 254` のみ。トルクオフは通常 `254`。
- open=0 / close=1 は正しく動作しているので、**release だけ定数マッピングを取りこぼしている**典型パターン。
- 対策: release 時の flag を `254`（実機仕様に合わせる）へ修正。

---

## 3. 完了判定ロジックの改善（`is_moving` / `wait_for_movement`）

### 3.1 現状の問題

- 目標到達後も `is_moving: true` が残る。
- `wait_for_movement` が、小さな移動でも `completed: false`（タイムアウト）を返す。
- go_home・move_all_joints・ホーム復帰のすべてで再現。
- 現状はファーム由来フラグや単純タイマーに依存していると推測。
- 加えて、タイムアウトで wait が返っても **実機は動き続ける**（go_home が止まらないのを実測）。

### 3.2 設計方針：終了条件を3つに分ける

| # | 条件 | 判定 | アクション |
|---|------|------|-----------|
| 1 | 収束（成功） | 全関節が目標角に対し許容誤差内 | 完了を返す（stop 不要） |
| 2 | ストール（失敗） | 未収束のまま一定時間ほぼ動かない | stop してから返す |
| 3 | タイムアウト（失敗） | 規定時間を超過 | stop してから返す |

- **1（許容誤差）** … サーボ精度起因の「終わらないコマンド」を防止。
- **3（タイムアウト＋キャンセル）** … 上限で必ず終わらせ、かつ実機を止める。
- **2（ストール検知）** … 物理的に届かない場合（例: 関節が引っかかる）にハングしないため。1 と 3 だけでは拾いきれないケースを補完。

### 3.3 パラメータの根拠（実測ベース）

到達誤差の実測:

- `move_all_joints → 20°`: 実測 19.68〜20.3°（誤差 ±0.3°程度）
- `go_home → 0°`: 実測 -0.17〜0.35°

→ **許容誤差 `tol = 1.0°`** 程度で、定常誤差・ディザリングを飲み込みつつ、明らかな未到達は弾ける。

### 3.4 ストールとスローの区別

低速（speed 5）の正当な移動と、引っかかりによるストールは瞬間速度では区別しづらい。
**「直近 T 秒の正味移動量」**で判定すると自然に分離できる。

- 直近の窓（例 2.5 秒）で目標方向に一定量（例 0.3°）以上進んでいれば「遅いが前進中」→ 継続
- 窓内の正味移動が閾値未満かつ未収束 → ストール

（J4 のケースは「数分で約1°」だったため、この基準で明確にストール判定される）

### 3.5 実装イメージ

ポーリングは HTTP 往復のない REST / Python 層（pymycobot に近い側）に置く。
MCP(node) 側から毎回 get_angles を叩くと往復コストが乗るため、
`POST /wait_for_completion`（tolerance, timeout を受ける）として Python 側に閉じ込めるのがよい。

```python
def wait_for_completion(target, *, tol=1.0, timeout=15.0,
                        poll=0.1, stall_window=2.5, stall_min_progress=0.3):
    start = time.monotonic()
    history = deque()  # (t, angles)
    while True:
        now = time.monotonic()
        cur = mc.get_angles()  # 取得失敗時の None ガードも要る
        err = max(abs(c - t) for c, t in zip(cur, target))

        # 1) 収束 → 成功
        if err <= tol:
            return {"completed": True, "reason": "converged",
                    "elapsed": now - start, "max_error": err}

        # 2) ストール検知（窓内の正味移動で判定）
        history.append((now, cur))
        while history and now - history[0][0] > stall_window:
            history.popleft()
        if now - start > stall_window and len(history) >= 2:
            progress = max(abs(c - h) for c, h
                           in zip(cur, history[0][1]))
            if progress < stall_min_progress:
                mc.stop()
                return {"completed": False, "reason": "stalled",
                        "elapsed": now - start, "max_error": err}

        # 3) タイムアウト → キャンセル
        if now - start > timeout:
            mc.stop()
            return {"completed": False, "reason": "timeout",
                    "elapsed": now - start, "max_error": err}

        time.sleep(poll)
```

### 3.6 注意点

- **target をどう知るか**: 現状の wait は引数なし。直前コマンドの目標をサーバー側で保持するか、wait に target を渡す。単腕なら「最後の指令目標を保持（last-write-wins）」で十分。
- **stall 閾値はディザリングの上に置く**: 静止時ノイズは ±0.5°未満だったので、窓 2.5 秒で `stall_min_progress=0.3` 程度。実機で要微調整。
- **収束時は stop しない**: 既に止まっているので不要。stop するのは 2 / 3 のときだけ。
- **`is_moving` を自前計算に置き換える案**: ファーム由来フラグが張り付くため、「前回ポーリングから eps 以上動いたか」で自前判定すれば status の信頼性も上がる。
- **動的タイムアウト**: 大きな移動＋低速は正当に時間がかかるため、固定値より「移動量と速度から見積もった値＋上限」にするとより堅い（初期は固定でも可）。
- **返り値を理由付きに**: `{completed, reason, elapsed, max_error}` とすると、クライアント側で「収束／ストール／時間切れ」を区別して扱える。

---

## 4. 安全ガード

- 速度5での go_home 中、J4 がホーム（0°）とは逆方向に動いて **-161.98°**（限界 -165° の手前）まで張り付いた。リセットで解消したため主因は実機側だが、サーバー側でも保険を入れたい。
  - (a) 関節が指令と逆方向／不動のときに検知して中断（= 3.2 のストール検知で兼ねられる）。
  - (b) 安全限界手前でのクランプ（指令値・動作中ともに）。

---

## 5. エラーハンドリングと入力検証

- release の 503 のように、下位ライブラリの生エラーがそのまま透過している。
  - (a) 引数を送る前にバリデーション（flag・値域チェック）。
  - (b) 503 等を意味のあるメッセージに整形して返す。
- → MCP クライアント側での原因特定が速くなる。

---

## 6. 良かった点（維持したい）

- node 解決（nodenv shims のフルパス指定）が確実。
- `MYCOBOT_API_BASE_URL` を起動コマンドにインライン指定して、WSL 内プロセスへ確実に渡している。
- 起動ログを **stderr** に出している（stdout を汚さず、MCP の stdio 規約に適合）。
- コア動作（status / get_joint_angles / move_joint / move_all_joints / go_home / stop / 主要グリッパー操作）は安定。

---

## 7. 対応の優先度

1. `jog_joint` のハング解消（非ブロッキング化）
2. `release` の flag 値を `254` に修正
3. `wait_for_completion` 導入（許容誤差＋ストール＋タイムアウト＋自動 stop）
4. `is_moving` の自前判定化／安全限界クランプ
5. 入力バリデーションとエラーメッセージ整形
