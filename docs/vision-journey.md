# 視覚ピック&プレース — 試行錯誤ログ（経緯と学び）

このドキュメントは `feat-vision` フェーズで**実際に試したこと・行き止まり・そこから得た
判断**を時系列で記録する。設計の確定形は [`architecture-and-roadmap.md`](architecture-and-roadmap.md)、
ビジョン数理は [`eye-to-hand-soracam.md`](eye-to-hand-soracam.md)、統合詳細は
[`integration-plan.md`](integration-plan.md) を参照。ここは「なぜ今の形に落ち着いたか」を残す
ための**敗北の記録**。

ステータス: 2026-06-22 時点。ブランチ `feat-vision`。実機は `192.168.0.136:8080`。

---

## 0. ゴール（不変）

MyCobot 280 + 固定 SORACAM（eye-to-hand）+ A4 ChArUco で、白い発泡スチロール（約2–3cm角）
を**見つけて掴んで置く**。操作 UI は Claude.ai チャット → **必要な操作はすべて MCP ツール**
として用意する必要がある（実行時にコードは書けない）。Pi は薄い実行役、知能は母艦側。

---

## 1. 確立したこと（足場 — ここは堅い）

- **深度カメラ（D435）を捨てて単眼 SORACAM + 既知平面に振った**。対象がボード平面上にある
  以上、画素→平面のホモグラフィで XY が出る。配置解析上、視覚誤差はアーム実行誤差（~3.5mm）
  より小さく、視覚は律速にならない → 深度は不要と判断。
- **pixel→board ホモグラフィは完成**。1枚のボード撮影から画素↔ボードmmを当てる方式。
  LOO 残差 ~0.5mm（カメラ再配置後の上向きトップダウンでは **54/54 コーナー・LOO 0.73mm**、
  過去最良）。レンズ歪みはボード領域内の内挿として吸収。**ここは疑う必要がない**。
- **ボードの実体確定**: 10×7 ChArUco, `DICT_4X4_50`, A4, 1マス **28mm 実測**（定規確認）。
  当初の想定 dict から実物に合わせて作り直した（`board_to_pdf.py` で原寸 PDF 生成）。
- **アーキテクチャ確定**: Pi = REST/MCP の薄い実行役、母艦 = 指令+計算。
  この phase で API/MCP に `release_all_servos` / `power_on` / `POST /robot/topdown` を追加。
  母艦側に standalone な `kinematics.py`（pure-numpy corrected_fk, シリアル不要）を分離。
- 認証情報は **soracam MCP の中だけ**に置く方針に統一（`.soracom.env` を廃し、vision は
  MCP 経由でフレーム取得 → `locate.py --mcp-json`）。

---

## 2. 壁 — `board→base` が較正しきれない（このフェーズ最大の難所）

pixel→board が 0.5mm で出るのに、その先で詰まった。**根本原因はロボット側**:

- このユニットの `corrected_fk` は**姿勢（orientation）は良いが、絶対位置が非射影的に歪む**
  （長く既知の未解決ギャップ）。FK の get_coords も信用できない（[[mycobot-hardware-quirks]] /
  [[topdown-ik-wrong-wrist-model]] / [[firmware-ik-broken-use-ikpy]]）。
- そのため board→base は（FK タッチ点でも、camera-observe 経由でも）**非射影写像**になり:
  - ホモグラフィを当てると 9 点中 ~4 点が必ず RANSAC outlier、非フィット点で 20–39mm 誤差。
  - RBF leave-one-out でも ~15mm。
  - **静的な少点較正は手法（homography / RBF）・入力（FK / camera）に関わらず ~15–20mm が床**。
    25mm の発泡スチロールには粗すぎる。
- 追い打ち: 280 の**トップダウンで盤面に届く範囲が狭い**（hover はできても接地は限定的 →
  較正点が ~4–7 点しか取れない）。人間のグリッド目視も ±5–10mm 上乗せ。

**学び**: ビジョン精度ではなくロボットの絶対位置精度が律速。静的較正でこの歪みを表に
押し込もうとするアプローチは全部この床に当たる。

---

## 3. 試した行き止まり（dead ends）

時系列で、何を試し・なぜ捨てたか。git 履歴と対応。

| 試したこと | 結果 / なぜ捨てた |
|---|---|
| **board→base を剛体変換（rigid）で当てる** | 非射影歪みを表現できず却下。`homography(xy)+plane(z)` に変更（`d864663`） |
| **FK タッチ点で board→base 較正**（`calibrate_board_to_base.py`, relax モード） | 接地できる点が少なく、cf-space の位置歪みをそのまま拾う → ~15–20mm 床 |
| **camera-observe で board→base 較正**（`solve_topdown_ik` 経由で着地を読む） | 入力自体が cf-space なので同じく非射影 → 同じ床。`--base-targets` で到達可能 base 姿勢を直接指令する版まで作った（`cc6ec80`）が床は超えない |
| **solve_topdown_ik のシード多様化**（near-surface 到達狙い） | 効果なし。元シードに revert（`0a61cdb` → `c5f3147`）。1解 >120s の問題は `2905f01` で nfev/シード整理 |
| **2台目カメラの追加** | **却下**。ボトルネックはロボット側で視覚側ではない → 費用対効果が悪い |
| **グリッパーマーカー = `DICT_5X5_100` id0 ~小サイズ** | 細かすぎ、glare/blur/curl で検出が死んだ → `DICT_4X4_50` id49 ~45mm に変更（board と同 dict なので 1 パスで board+gripper 両検出） |
| **マーカーを光沢紙・曲がったフラップに貼る** | 鏡面反射でビットが飛び検出不可 → **マット印刷+剛体カードに平貼り必須**という運用知見 |

---

## 4. 解けた小問題（IK ストール）

near-base のトップダウン点は IK 分岐が2つあり、Pi の `solve_topdown_ik` は最初の解＝
**肘折れ分岐（J2≈-135 リミット）**を返す。これが J3=141 を指令しアームが 46 で止まる（ストール）。
もう一方の分岐（J2≈-16, 緩い）は到達できる。**修正 = 関節リミット余裕が最大の分岐を選ぶ**。
母艦の `topdown_ik.py`（`solve_reachable`）に実装済み、PUT /joints/angles で指令。
**TODO: 同じ選択ロジックを Pi の `solve_topdown_ik` にも移植**して `/robot/topdown` が
ストール解を返さないようにする。

---

## 5. カメラ再配置（2026-06-21）

カメラを**高く・ほぼ真下向き**に再設置 → 両較正が無効化。
- homography は綺麗なボードフレームから**やり直し済み**（54/54, LOO 0.73mm — 過去最良）。
- board→base は無効（グリッパーが盤外に着地）→ **camera-observe のやり直しが必要**
  （マーカー自動化ステップの途中）。
- 運用: ボード撮影前にアームを盤外へ退避（PUT angles `[90,-60,0,0,0,0]`）。

---

## 6. 現在の決定 — グリッパー ArUco マーカー + visual servoing

静的較正で歪みを潰すのを諦め、**閉ループに切り替える**。

- マーカー `DICT_4X4_50` id49 ~45mm をグリッパー**上面・接近軸中央**に平貼り
  （上面なのはカメラがトップダウンだから / 中央なら J6 に依らずマーカー中心≒爪先のボード位置）。
  `marker_to_pdf.py --dict DICT_4X4_50 --id 49 --mm 45`、`gripper_marker.py` が glare-robust に検出。
- グリッパーのボード位置をホモグラフィで**自動取得（サブmm・人間ノイズ無し）**。
- その上で: 密な自動 camera-observe + RBF、**かつ/または visual serviong**
  （カメラ実測でグリッパーマーカーを対象へ寄せる → **FK 歪みに対して頑健。これが本命の修正**）。
  marker-to-tip オフセットは一度だけ較正。

**深掘り代替（未着手）**: アームの絶対 DH / リンク長そのものを較正する（歪みを根から潰す）。

---

## 7. 検討中の次の一手 — eye-in-hand（アーム先端カメラ）

「先端カメラで Local Planning、固定 SORACAM で Global Planning」という分割案を検討中。

- **効く理由**: 壁の正体は「base 座標が FK 歪みで信用できない」こと。visual servoing、特に
  eye-in-hand は**精度ループから FK を排除**でき、接近するほど対象が画像内で大きくなる＝
  必要な瞬間に解像度が上がる → 把持精度の上限が FK ではなくカメラ解像度になる。
- **分割**: Global（固定 SORACAM, 15–20mm でも可）で pre-grasp 姿勢へ粗く移動 → Local
  （eye-in-hand IBVS）で画像中央寄せ＋降下。**精度要求を FK 精度から切り離せる**のが本質。
- **安物アームゆえの実コスト（成否を分ける3点）**:
  1. 手首への重量増 → 重力トルクで J5 の張り/spring-back を悪化。極小カメラ+軽量ケーブル必須。
  2. マウント剛性（たわみ＝誤差）。
  3. 把持直前の盲点（対象が画角外）→「中央寄せ+高さ推定まで→既知の短距離 open-loop 降下」設計。
- hand-eye 較正は歪んだ get_coords ではなく **SORACAM を truth** にして作るのが筋。
- **進め方の私見**: まず現行マーカー方式（ハード追加ゼロ）で把持公差に届くか測る → Z・最終
  センタリングが足りなければ eye-in-hand を追加。長期的には両方持つのが正解。

---

## 8. 未解決 TODO

- [ ] camera-observe の board→base やり直し（カメラ再配置後・マーカー自動化で）
- [ ] Pi の `solve_topdown_ik` に「リミット余裕最大の分岐選択」を移植（`/robot/topdown` のストール解消）
- [ ] visual servoing ループの実装と把持公差の実測
- [ ] （必要なら）eye-in-hand の最小構成 PoC（極小カメラ・剛体マウント・盲点降下）
- [ ] （深掘り）アーム絶対 DH/リンク長較正の是非判断

---

## アクセス情報

- 母艦（この dev box）: cv2 + soracam MCP（still grab ~15s, 1920×1080, ファイル保存 → decode / `--mcp-json`）。**実機は無し**。
- 実機: `ssh -i ~/.ssh/id_mycobot factory@192.168.0.136`、`~/arms`、cv2 4.12。
  API サーバーは `./mycobot_server_ctl.sh restart`（`/dev/ttyACM0` を保持 → 他の robot Python を同時実行しない）。
</content>
</invoke>
