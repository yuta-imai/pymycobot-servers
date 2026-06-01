# 視覚ガイド ピック&プレース — `locate` / `pick_at` 詳細設計

前提（確定済み）:
- **平面拘束**: 対象は既知の支持平面（テーブル, 基準座標系で z = z₀）上にある
- **カメラ・アーム固定**: 外部パラメータ `T_base_cam` は不変（較正は一回きり）
- **v1 はオープンループ**: 接近中の再撮影なし。tight な閉ループは持たない

共有契約（再掲・両サーバー共通）:
- 正準フレーム = **ロボット基準座標系**（右手系, pymycobot coords 規約）
- 単位 = 並進 **mm** / 回転 **度**
- 姿勢 = `[x,y,z]`(mm) ＋ `[rx,ry,rz]`(deg)
- 全アクションは `{ ok, reason, ...data }` を返す（typed failure）
- vision は **基準座標系・mm に変換済み**の座標を返す。arm は座標と把持指令だけ受ける。

`locate` の出力（`base_xyz_mm`, `yaw_deg`）が、そのまま `pick_at` の入力になる。

---

# 実装ステータス・重要な設計変更（2026-06-01 実機検証済み）

この節は実機（MyCobot 280, `192.168.0.136:8080`, pymycobot 4.0.0）で確定した事実と、
それに伴う設計変更を記録する。元の設計（Part 1/2）の前提を一部上書きする。

## ハードウェアで判明した制約

- **firmware の IK は壊れている**: `solve_inv_kinematics` は**全姿勢で `-1`**（解なし）を返す。
  電源ON・home・可達な姿勢でも同じ。firmware IK は使用不能。
- **firmware の引数付き FK も壊れている**: `angles_to_coords` は**引数を無視**して現在姿勢を返す。
- **firmware の Cartesian 移動 (`send_coords`) は危険**: 壊れた IK でゴミ関節角を生成し、
  実機で暴走（J3 がハードリミットに突入、電源OFF＋手動復帰が必要だった）。**呼んではいけない。**
- 一方で **`get_coords`（実姿勢読み取り）と `send_angles`（関節移動）は正確**。
- **把持検証は不可**: `get_gripper_value`/`is_gripper_moving` がこの firmware に無い（501）。
- **`go_home`（firmware）はハングする**（約4分）。`send_angles([0]*6)` を使う。

## 採用した設計（Part 2 を上書き）

- **Cartesian 制御 = 自前 IK（ikpy + URDF）→ `send_angles`**。
  - `urdf/mycobot_280_m5.urdf` を ikpy で読み、6関節を名前ベースの active mask で駆動。
  - **安全ゲート**: `solve_ik` は IK 解を ikpy FK で往復検算し、目標姿勢と
    **2mm / 2deg** を超えてズレる解は棄却（`None`）。これにより不正解が `send_angles` に到達しない。
  - ikpy FK は実機 `get_coords` と **2.5〜10mm** で一致（`scripts/verify_fk.py`）、
    自前IK→`send_angles` の着地誤差は **≤5mm / ≤1.6deg**（`scripts/verify_ik.py --move`）で検証済み。
- **REST 契約（A0/A0''実装済み）**:
  - `GET /robot/coords` … 実姿勢 `[x,y,z,rx,ry,rz]`
  - `PUT /robot/coords` … 自前IK→`send_angles`（`mode` 引数は受理するが無視。常に関節補間）。
    workspace box 外 / IK解なし / 関節リミット超過は **400 で拒否し、動かさない**。
  - `POST /robot/coords/check` … 動かさず到達判定（`ok` / `ik_failed` / `out_of_range` /
    `joint_limit` / `ok_unverified`）。**ikpy 採用後は authoritative**。
  - `POST /robot/coords/wait` … 位置(mm)＋姿勢(deg)収束 / stall / timeout / auto-stop。
  - `GET /robot/status` に **`joints_near_limit`**（ハードリミット±3°内の関節）を追加 → デッドロック可視化。
- **`pick_at` の VALIDATE**: 設計書2.4 の「IK 事前チェック」は **firmware IK ではなく自前 ikpy IK**
  （`check_pose_reachable`）で実施。pre_grasp / grasp / retreat の3姿勢すべてを動かす前に検証。
- **把持検証 (VERIFY_GRASP)**: gripper フィードバック不可のため、v1 は掴んで持ち上げ
  `grasped:"unknown"` / `ok_unverified` を返す（設計書2.7 のとおり）。

## 共有契約への補足

- 元の契約「pymycobot coords 規約」は維持。座標系・単位・Euler 規約は ikpy FK ⇔ 実機 `get_coords`
  で一致を確認済み（姿勢角の規約も `--move` の `ori_err ≤1.6deg` で裏取り）。
- home 付近のグリッパー姿勢は実測で `rpy ≈ [-90, 0, -90]`（deg）。`top_down_rpy(yaw)` の起点。

## A1 確定: 真下把持は J5≈0 で素直に可能（実機・ikpy FK 検証 2026-06-01）

重要: 当初「J5=-90 で真下、しかし保持できない」と誤解していたが、ikpy FK で tool-Z（グリッパー
接近軸）を計算した結果、**J5 の意味が逆**だったと判明（`scripts/probe_j5_direction.py`）。

- **前傾 reach 姿勢 `[0,-40,-40,0,J5,0]` の grippper 向き（downness: +1=真下, -1=真上）**:
  - **J5 = 0  → downness +0.985（ほぼ完全真下）** ← 真下はここ
  - J5 = ±30 → +0.853（まだ十分 down）
  - J5 = ±60 → +0.492（横向きに近い）
  - J5 = ±90 → 0.000（完全に水平＝横向き）
- つまり **J5 を ±90 へ振るほどグリッパーは真下から「横向き」へ倒れる**。以前 J5=-90 で
  サーボが力尽きて振動したのは、真下ではなく**腕を水平に伸ばす最も不利な姿勢**だったため。
  真下把持（J5≈0）ではこの問題は起きない。
- **`probe_j5_depth.py` の「-83まで保持」も真下ではなく横向きへ倒すテストだった**（解釈を訂正）。
- **ヨー反転（アームを真上から180°回す）は無効**: `get_coords` は基準座標系で返るため運動学不変、
  ヨー回転は重力トルクも変えない（実機で同一 coords を確認）。これは依然有効な結論。
- **設計への反映**:
  - 完全な鉛直真下把持は**可能**。実装は `solve_topdown_ik(x, y, z, yaw)`（controller）。
  - **真下は「tool-Z 軸だけ拘束」で出す**: ikpy `orientation_mode="Z"`（target = [0,0,-1]）。
    full rotation matrix を与えると ry≈±90 のジンバル退化で破綻する（手書き行列で ori_err 90°
    を確認済み）ため、approach 軸のみ拘束し、手首の残り自由度はアームに委ねる。
  - **yaw は J6 へ重畳**: tool-Z 鉛直のとき dJ6 と tool-X の azimuth は **傾き -1**（実測:
    dJ6 +30 → yaw -30）。よって `J6 = base_J6 + (natural_yaw - yaw_deg)`。平行グリッパーは
    対称なので J6 がリミット外なら 180° flip で代替。
  - **検証結果（`scripts/verify_topdown.py --no-connect`）**: テーブル全域×yaw{0,45,90,-45}の
    全16ケースで **downness +1.000 / J5≈0 / yaw_err 0.0**。実機 `--move` 着地確認は次段。

---

# Part 1. `vision.locate`

自然言語クエリから、対象の **基準座標系での 3D 位置と yaw** を返す。読み取り専用・冪等（ただしシーンが変われば結果は変わる）。

## 1.1 シグネチャ

```
locate(
  query: str,                 # 例 "red block", "the plate"
  max_candidates: int = 5,
  assume_height_mm: float = 0,# 視差補正用（後述 1.4）。既定 0 = 支持平面で交差
  return_image: bool = false
) → {
  found: bool,
  reason: ok | not_found | low_confidence | ambiguous
        | plane_intersect_failed | not_calibrated,
  plane_z_mm: float,          # 実際に交差させた平面 z（z₀ + assume_height）
  candidates: [ {
    base_xyz_mm: [x, y, z],   # 光線 ∩ 平面 の交点（1.3）
    yaw_deg: float | null,    # 基準座標系での主軸角（1.5）。不定なら null
    yaw_confidence: float,    # 0-1
    pixel_uv: [u, v],         # 採用したマスク重心ピクセル
    bbox_xywh: [x, y, w, h],
    mask_area_px: int,
    confidence: float,        # 検出スコア
    label: str
  } ],                        # confidence 降順
  image_id?: str              # snapshot との突き合わせ用
}
```

補助ツール:
```
locate_all(query) → 上記 candidates をそのまま
snapshot() → 画像（人/LLM が「見る」用。座標計算には使わせない）
calibration_status() → { intrinsics_ok, extrinsics_ok, plane_ok, calibrated_at }
```

## 1.2 検出パイプライン

1. オープン語彙検出（GroundingDINO / OWL-ViT 等）で query → bbox 候補
2. セグメンテーション（SAM 等）で各候補のマスク
3. 採用ピクセル = **マスク重心**（`pixel_uv`）
4. しきい値処理:
   - 最良 confidence < `conf_min` → `low_confidence`
   - 検出ゼロ → `not_found`
   - 同一クエリに高スコア候補が複数 → `found:true` かつ `ambiguous`（解決はオーケストレータ/LLM に委ねる）

> **原則**: VLM に直接ピクセル/座標を吐かせない。bbox/マスクを取り、**重心と主軸はコードで計算**する。

## 1.3 ピクセル → 3D（光線 ∩ 平面）

中核の変換。手順を厳密に:

```
1. 歪み除去: (u,v) を K, dist で undistort → 正規化座標
2. カメラ系の光線:
     方向 d_cam = normalize( K⁻¹ · [u, v, 1]ᵀ )
     原点   o_cam = 0
3. 基準系へ変換（T_base_cam = (R, t)）:
     o_base = t                      # 基準系でのカメラ位置
     d_base = R · d_cam
4. 平面（水平テーブル）: n·X = d,  n = [0,0,1], d = plane_z_mm
5. λ を解く:
     denom = n · d_base
     |denom| < ε  → reason = plane_intersect_failed   # 光線が平面と平行
     λ = (d − n·o_base) / denom
     λ ≤ 0        → plane_intersect_failed             # 平面がカメラ背後
6. 交点: X_base = o_base + λ · d_base   → base_xyz_mm
```

サニティチェック: `base_xyz_mm` が設定済みテーブル bbox の外なら、背景誤検出として弾く（任意で `confidence` を下げる or 候補から除外）。

> **責務境界**: 到達可能性（reach）の最終判定は **arm 側**。vision は座標を返すだけ。粗いテーブル bbox フィルタはランキング目的で持ってもよいが、authoritative ではない。

## 1.4 高さによる視差オフセット（重要な注意）

斜めから見るカメラでは、**「見かけの重心ピクセルを通る光線」を支持平面 z₀ で交差させた点は、高さのある物体の真の footprint 重心とは一致しない**。物体が高いほど、カメラが斜めなほど、系統的にズレる（光線が物体の奥側で平面に当たる）。

対策（v1）:
- 平たい物体に限定すれば無視できる
- 較正をなるべく真上寄りにする
- 高さが分かるなら `assume_height_mm` を渡し、**z₀ ではなく z₀ + height の平面で交差**させて上面中心の XY を取る（オフセットを縮小）
- これは Z 把持高さの未解決問題（Part 2 で扱う）と同根

## 1.5 yaw 推定（画像角 → 基準系 yaw）

把持の手首回転に使う。**画像内の角度をそのまま yaw にしてはいけない**（斜め投影のため）。正しい手順:

```
1. マスクから主軸を取る（PCA もしくは minAreaRect の長辺方向）
2. 主軸の両端ピクセル p1, p2 を取る
3. p1, p2 を それぞれ 1.3 の光線∩平面で 3D 化 → P1_base, P2_base
4. yaw_deg = atan2( P2.y − P1.y, P2.x − P1.x )   # 基準系で計算
```

注意:
- 長軸は **180° 周期**（mod 180）。平行グリッパーは対称なので通常は問題なし
- ほぼ正方形/円形 = 主軸が不定 → `yaw_deg = null`, `yaw_confidence` を低く返す（pick_at 側は yaw 無指定として扱う）

## 1.6 失敗モードまとめ

| reason | 意味 | オーケストレータの対応例 |
|--------|------|------------------------|
| `ok` | 単一の確実な候補 | そのまま pick_at へ |
| `not_found` | しきい値超え検出なし | 撮り直し / クエリ変更 / 中断 |
| `low_confidence` | 最良候補が conf_min 未満 | 中断 / 人に確認 |
| `ambiguous` | 高スコア候補が複数 | LLM が選ぶ / 人に確認 |
| `plane_intersect_failed` | 光線が平面と平行 or 背後 | 較正/設置を疑う |
| `not_calibrated` | 内部/外部/平面のいずれか未較正 | 較正実行 |

## 1.7 較正依存

`locate` は **内部パラメータ + 外部パラメータ + 平面 z₀** を要する。いずれか欠落で `not_calibrated`。較正は vision サーバーだけがロードする設定成果物（YAML/JSON）。`calibration_status()` で事前確認可能。

---

# Part 2. `arm.pick_at`

座標と yaw を受け、**上空接近 → 降下 → 把持 → 検証 → 退避**を実行する状態機械。失敗時は必ず**安全な既知状態**で終える（不意の落下をさせない）。

## 2.1 シグネチャ

```
pick_at(
  xyz_mm: [x, y, z],          # locate の base_xyz_mm（通常 z = 支持平面）
  yaw_deg: float | null = null,
  object_height_mm: float | null = null,
  grasp_width: int | null = null,   # 0-100 開度。null = 全開で接近
  speed: int = 30
) → { ok: bool, reason: <enum 2.5>, grasped: bool | "unknown",
      final_pose: { base_xyz_mm, rpy_deg } }
```

frame/単位は共有契約に一致（locate の出力をそのまま渡せる）。

## 2.2 派生姿勢の計算

```
把持姿勢 rpy   = top_down_rpy(yaw_deg)   # グリッパー鉛直下向き + yaw を rz に
                                          # ※ MyCobot のオイラー規約に合わせ要調整
grasp_z       = z + grasp_z_offset_mm                     （object_height 未指定時）
              = z₀ + object_height_mm − grip_penetration  （指定時, 上面把持）
pre_grasp     = (x, y, grasp_z + approach_clearance_mm, rpy)
retreat       = (x, y, grasp_z + retreat_clearance_mm,  rpy)
```

既定パラメータ（要実機調整）:
| param | 既定 | 意味 |
|-------|------|------|
| `approach_clearance_mm` | 60 | 上空待機の余裕 |
| `retreat_clearance_mm` | 80 | 退避高さ |
| `grasp_z_offset_mm` | 8 | 平たい物体の指のかかり代 |
| `grip_penetration` | 5 | 上面からの食い込み |
| `descend_speed` | 15 | 降下は低速 |
| `travel_speed` | speed | 水平移動 |
| `empty_close_value` | 8 | 閉値がこれ未満なら空振り（把持失敗） |

## 2.3 状態機械

```
            ┌──────────┐
            │ VALIDATE │  入力/可動域/IK実行可否/既把持チェック
            └────┬─────┘
                 │ ok                    fail → 即終了（未動作のまま）
                 ▼
          ┌─────────────┐
          │ PREP_GRIPPER│  grasp_width or 全開
          └──────┬──────┘
                 │ ok                    fail → gripper_error（未動作）
                 ▼
          ┌─────────────┐
          │ GO_PREGRASP │  move_to_pose(pre_grasp), wait収束
          └──────┬──────┘
                 │ converged             stalled/timeout/unreach
                 ▼                          → ABORT(retreat,open)
          ┌─────────────┐
          │   DESCEND   │  move_to_pose(grasp), 低速, wait
          └──────┬──────┘
                 │ converged             早期stall=接触(任意) / timeout
                 ▼                          → 2.6 の判断
          ┌─────────────┐
          │    GRASP    │  close gripper(値), 整定待ち
          └──────┬──────┘
                 ▼
          ┌──────────────┐
          │ VERIFY_GRASP │  gripper開度 vs empty_close_value
          └──────┬───────┘
            grasped │ empty → grasp_failed → ABORT(open, retreat)
                 ▼
          ┌─────────────┐
          │   RETREAT   │  move_to_pose(retreat)（把持物を持ち上げ）
          └──────┬──────┘
                 ▼
            DONE(ok, grasped)
```

## 2.4 各状態の詳細

- **VALIDATE**（無動作）: 引数妥当性 / `pre_grasp`・`grasp`・`retreat` の **IK 実行可否**（send_coords 解の存在）/ 可動域・リミット / **既に把持物がないか**（gripper 開度確認）。失敗は動かす前に返す。
- **PREP_GRIPPER**: `grasp_width`（null なら全開）に開く。
- **GO_PREGRASP**: 対象 XY/yaw の上空へ。`wait_for_movement` の `converged` を要求。
- **DESCEND**: 把持高さへ**低速降下**。位置移動（MyCobot は力フィードバックが弱い）。
- **GRASP**: 閉じる。整定待ち。
- **VERIFY_GRASP**: 閉値が `empty_close_value` 以上で残れば把持成功（指が物に当たって止まった）。完全に閉じきった（≈0）なら空振り。
- **RETREAT**: 退避高さへ持ち上げて終了。

## 2.5 reason 列挙（typed failure）

| reason | grasped | 意味 |
|--------|---------|------|
| `ok` | true | 把持検証まで成功 |
| `ok_unverified` | "unknown" | 動作成功・把持検証不可（2.7） |
| `invalid_args` | false | 引数不正 |
| `already_holding` | false | 既に何か把持中（VALIDATE で拒否） |
| `unreachable` | false | いずれかの姿勢が可動域外 |
| `ik_failed` | false | send_coords が解を返さない |
| `blocked_descend` | false | 把持高さに到達できず（stall/timeout） |
| `grasp_failed` | false | 閉じきり＝空振り |
| `gripper_error` | false | グリッパー指令失敗 |
| `aborted_estop` | 状況依存 | 外部 stop |
| `move_timeout` / `move_stalled` | 状況依存 | 移動系失敗 |

arm の `wait_for_movement` の `reason`（converged/stalled/timeout）を、上記へ写像する。

## 2.6 失敗時の不変条件（安全設計）

- **降下中で止めない**: いかなる中断も、まず上方退避して対象から離す。
- **把持前の失敗** → グリッパーを開いて退避。
- **把持後に後段が失敗** → **掴んだまま安全高さへ退避**し、`grasped:true` を付けて返す（**勝手に落とさない**）。置くか持ち帰るかはオーケストレータが判断。
- **再入不可**: arm は単一資源。同時呼び出し禁止。

## 2.7 正直な制約（この設計でも残る）

- **把持検証はグリッパーのフィードバックに依存** → `get_gripper_status`（現状 `get_gripper_value` バグ）を直す必要がある。直るまで `pick_at` は `ok_unverified` / `grasped:"unknown"` を返す（誠実に）。
- **力フィードバックが弱い** → 降下は位置ベース。`grasp_z` が誤ると「テーブルに突っ込む」か「空を掴む」。v1 は (a) `object_height_mm` 既知、(b) 平たい物を平面で掴む、で割り切る。
- **接触ヒューリスティック（任意・実験的）**: 把持高さより少し下を指令し、**先に作った stall 検知で早期停止した位置を接触とみなす**手もある。ただし MyCobot はストレス/エラーになり得るので過信しない。
- **狭い可動域**: テーブル上でも到達不能/姿勢が苦しい点が多い。VALIDATE の IK 事前チェックで早期に弾くのが効く。

---

# 付録. locate → pick_at 連携（v1・LLM オーケストレーション）

```
1. vision.calibration_status()                      # 較正OK確認
2. r = vision.locate("red block")
     reason != ok（low_confidence/ambiguous/not_found）→ 中断 or 人に確認
3. p = arm.pick_at(r.candidates[0].base_xyz_mm,
                   yaw_deg=r.candidates[0].yaw_deg)
     p.grasped != true → 中断（落下物の放置を防ぐ）
4. d = vision.locate("plate")
5. arm.place_at(d.candidates[0].base_xyz_mm, yaw_deg=d.candidates[0].yaw_deg)
6. arm.home()
```

- 幾何は vision、把持モーションは arm の内側。LLM は離散ステップの指揮のみ（(4) の原則を満たす）。
- 原子性（pick 成功・place 失敗時のロールバック）やリトライが欲しくなったら、v2 で **素の Python オーケストレータ**（vision/arm の REST を直叩き）へ。MCP 越し再帰呼び出しはしない。

---

## 次に詰めるべき項目（TODO）

**完了（実機検証済み）**
- ~~Cartesian 制御の土台~~ → A0/A0'' で自前 ikpy IK→`send_angles` を実装・検証済み（上の実装ステータス節）
- ~~`get_gripper_status` 修復~~ → firmware に該当 API が無いと判明。修復不能のため `ok_unverified` で割り切り（設計変更）

**進行中 / 残り**
- **A1（次）**: `top_down_rpy(yaw)` を確定。home 実測 `rpy ≈ [-90,0,-90]` を起点に、グリッパー鉛直下向きの
  rx,ry,rz と yaw→軸の写像を実機で詰める（ikpy 経由で `check`/`coords` を使い、動かす前に検証）。
- **A2**: `pick_at` 状態機械（VALIDATE は `check_pose_reachable`=自前IK、移動は `PUT /robot/coords`、
  把持検証は skip→`ok_unverified`）。
- `place_at` の状態機械（pick_at とほぼ対称: 接近→降下→開→退避＋「本当に離した」検証）
- 較正手順書（内部パラメータ → ArUco ハンドアイ → 平面 z₀ 確定 → クリックしたピクセルへ動かして実測誤差）
- パラメータ実機チューニング（clearance / offset / empty_close_value / 各 speed）

> 注: `get_gripper_status` 修復・firmware IK 依存は **不可と確定**したので TODO から除外（上の実装ステータス節参照）。
