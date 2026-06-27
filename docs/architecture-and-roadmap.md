# 視覚ピック&プレース — アーキテクチャ方針 & ロードマップ

このドキュメントは**確定した前提**と**進む方向**を固定し、以後の開発はこれに沿って進める。
（ビジョン数理・校正の詳細は [`eye-to-hand-soracam.md`](eye-to-hand-soracam.md) を参照。）

ステータス: **前提合意済み（2026-06-21）**。Phase 1 から本実装に入る。

---

## 0. ゴール

MyCobot 280 で、固定 SORACAM（eye-to-hand）と A4 ChArUco を使い、**白い発泡スチロール
ブロック（約2–3cm角）**を見つけて掴んで置く（pick & place）。最終的な操作 UI は
**Claude.ai のチャット**。

---

## 1. 前提（確定 / 要確認）

確定:
- **UI = Claude.ai チャット**（Claude Code ではない）。→ 実行時に都度コードを書けない。
  **必要な操作はすべて MCP ツールとして提供**しておく必要がある。【最重要の制約】
- **Pi = 薄い実行役**（REST API ＋ MCP）。**知能（ビジョン・校正・計画）は母艦側**。
- カメラは**単眼 SORACAM 1台**（eye-to-hand 固定）。2台化はしない（ボトルネックはロボット側で、
  視覚側ではないため費用対効果が悪い）。
- 作業面 = **A4 ChArUco（10×7, DICT_4X4_50, 1マス28mm実測）**。校正ターゲット兼・作業平面兼・
  座標系。対象はこのボード上に置く。
- **pixel↔board ホモグラフィは ~0.5mm で確立済み**（単発撮影）。
- **board→base が壁**：`corrected_fk` の絶対位置が非射影的に歪む（既知の未校正ギャップ）。
  静的校正（ホモグラフィ/RBF, FK/カメラいずれ）は **~15–20mm が限界**＝25mm把持には粗い。
- グリッパに**力覚なし**（get_gripper_value 501）→ 把持成否は**視覚で判定**（home 後に物体が
  盤上から消えたか）。
- **真下把持で盤面に届く領域が狭い**（280 のリーチ限界）→ 対象配置はその領域内。
- 認証情報（Soracom）は**一箇所に集約**（重複コピーを作らない）。

確定（2026-06-21 ユーザー確認済み）:
- **place 先 = 固定の置き場所（ベース座標で1点指定）**。pick→その座標へ運んで release。
- **作業領域 = 当面は到達領域内に対象を置く（現状維持）**。物理再配置による領域拡大はしない
  （将来課題として保留）。
- **認証 = Pick MCP（母艦）の env に集約**。`.soracom.env` は廃止（config の env ローダも今後撤去）。
  Pick MCP が SORACAM フレームを Soracom HTTP で自前取得する。

未確定（小・後決め可）:
- 1サイクルの期待スループット（当面はサーボ収束で数秒/個を許容と仮定）。

---

## 2. アーキテクチャ（目標形）

```
 Claude.ai チャット (UI)
        │  MCP ツール呼び出しのみ
        ▼
 ┌─────────────────────────────────────────────┐
 │ Pick MCP サーバ (Python, 母艦)               │  ← 知能・オーケストレーション
 │  - フレーム取得(SORACAM) + cv2 ビジョン       │
 │  - 校正計算 / locate / visual servo / pick    │
 │  - 高レベルツール: calibrate / locate / pick  │
 └───────────┬───────────────────┬──────────────┘
             │ REST (urllib)      │ Soracom API (frames)
             ▼                    ▼
   Pi REST API (薄い実行役)    SORACAM (クラウド)
   /robot/topdown,release,...
             │ serial
             ▼
        MyCobot 280
```

- **Pi**：既存 `mycobot_api_server.py`（robot primitives）。知能を持たせない。
- **母艦の Pick MCP（新規・Python）**：cv2 が要るので Python。フレーム取得＋ビジョン＋
  ロボット REST 呼び出しを束ね、**高レベルツール**を Claude.ai に提供。
- 既存の **soracom-mcp / robot-mcp(TS)** は低レベル用途として残置（汎用）。本ワークフローは
  Pick MCP に集約（Claude.ai は高レベル動詞だけ叩けば良い）。
- **認証は Pick MCP の env に集約**（.soracom.env は使わない）。

---

## 3. 採用する pick 方式（現方針）

静的 board→base 校正が ~15–20mm で頭打ちなので、**閉ループ（visual servoing）**に寄せる：

1. **gripper にマーカー**（DICT_5X5_100 id0, ~30mm, 上面・先端軸中心）→ カメラ＋ホモグラフィで
   **グリッパ先端の board 位置を自動 sub-mm 計測**（手読みノイズ・FK歪みを回避）。
2. **locate**：空ボード差分（盤マスク常時）→ 対象の board 位置。
3. **visual servo**：grip-marker board 位置 → 対象 board 位置へ、誤差ぶん指令補正を反復
   （board→base は粗くて良い＝初期指令と補正方向にのみ使用）。収束で **FK 歪みを吸収**。
4. **descend → close → lift → 把持判定（視覚：home 後に対象が盤上から消えたか）**。
5. **place** → 固定の置き場所（ベース座標）へ移動 → release → home。

補助（粗い board→base のブートストラップ）：relax+FK タッチ or camera-observe。サーボの初期値用。

---

## 4. MCP ツール面（目標仕様 / まだ実装しない）

高レベル（Pick MCP）:
- `health()` — 構成・校正状態（homography/board→base/gripper-marker の有無と品質）。
- `calibrate_homography()` — ボード撮影 → pixel↔board H 保存。要: アーム退避。
- `calibrate_board_to_base()` — camera-observe（gripperマーカー自動）で粗い board→base。
- `locate_object()` — 盤上の対象を検出 → board/base 座標（＋注釈画像）。
- `pick_object()` — locate → visual servo → grasp → lift → verify。成否を返す。
- `place_object(target)` — 置き場へ移動 → release。
- `home()` / `relax()` / `power_on()` — 退避・弛緩・通電。

低レベルは既存 robot-mcp / soracom-mcp に存在（move_topdown, release, power_on, gripper, still）。

設計原則: **各ツールは Claude.ai から1コールで完結**（人手は物理操作のみ）。画像は要約＋必要時のみ
注釈画像パスを返す（base64 をチャットに流さない）。

---

## 5. 現状の資産（Phase 0 = 概ね完了）

`scripts/vision/`（母艦 Python ライブラリ。MCP ツールはこれを薄く包む）:
- `geometry.py`（apply_homography/umeyama/kabsch/plane, 純numpy・自己テスト）,
  `kinematics.py`（standalone corrected_fk）, `charuco_board.py`, `config.py`,
  `frame_source.py`（soracom HTTP / mcp-still / rtsp）, `homography_calibrate.py`,
  `locate.py`（board-mask 常時 + grasp_succeeded）, `calibrate_board_to_base.py`(relax+FK,REST),
  `calibrate_camera_observe.py`(command-observe,REST), `board_to_pdf.py`,
  `placement_calculator.py`。
- Pi API/MCP に追加済み: `POST /robot/release` `/robot/power_on` `/robot/topdown`（+MCP tools）。

---

## 6. ロードマップ

- **Phase 0（済）**: 単眼+ホモグラフィ確立、locate、API/MCP primitives、校正ツール群、知見の確定。
- **Phase 1（次・実機）**: グリッパマーカー検出 + **visual-servo pick を1個成功**させる
  （`scripts/vision/` の関数として）。把持判定（視覚）込み。
- **Phase 2**: Phase 1 で固まった流れを **Pick MCP（Python, FastMCP）にツール化**。Claude.ai から
  `calibrate_*` / `locate_object` / `pick_object` / `place_object` を呼べる状態に。認証を MCP env に集約。
- **Phase 3**: 堅牢化（複数個、置き、リカバリ、再校正トリガ、ドリフト検知）、ドキュメント整備。

各 Phase は **library 関数を先に実機検証 → 検証済みを MCP で薄く公開**、の順（ロジックと公開を分離）。

---

## 7. 設計上の決定事項（記録）

- 単眼+平面ホモグラフィ（D435/ステレオ不採用）。
- board→base は静的校正に依存せず **visual servoing 主体**（FK絶対位置の歪みを閉ループで吸収）。
- Pi 薄実行役 / 母艦知能。ロジックは Python ライブラリ → MCP は薄いラッパ。
- 認証は一箇所。チャットへ base64 画像を流さない（要約＋注釈画像パス）。
