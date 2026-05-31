# Soracom Cloud Camera Services ライブ視聴見放題 API リファレンス

**Limited Preview のご案内**

SORACOM, INC.
2024-04-01

> **CONFIDENTIAL**

---

## 目次

1. [はじめに](#1-はじめに)
   - [1-1. 注意事項](#1-1-注意事項)
2. [Soracom Cloud Camera Services ライブ視聴見放題とは](#2-soracom-cloud-camera-services-ライブ視聴見放題とは)
   - [2-1. ライブ視聴見放題 API とは](#2-1-ライブ視聴見放題-api-とは)
   - [2-2. ライブ静止画取得 API とは](#2-2-ライブ静止画取得-api-とは)
   - [2-3. ATOM Cam Swing 首振り API とは](#2-3-atom-cam-swing-首振り-api-とは)
3. [Soracom Cloud Camera Services ライブ視聴見放題 API リファレンス](#3-soracom-cloud-camera-services-ライブ視聴見放題-api-リファレンス)
   - [3-1. ライブ視聴見放題 API](#3-1-ライブ視聴見放題-api)
   - [3-2. ライブ静止画取得 API](#3-2-ライブ静止画取得-api)
   - [3-3. ATOM Cam Swing 首振り API](#3-3-atom-cam-swing-首振り-api)

---

## 1. はじめに

このたびは SORACOM Limited Preview プログラムをご利用いただきありがとうございます。この資料では、以下の新機能の概要と利用方法を説明します。フィードバックをお寄せいただけますと幸いです。

- Soracom Cloud Camera Services ライブ視聴見放題 に関する以下の API
  - ライブ視聴見放題 API
  - ライブ静止画取得 API
  - ATOM Cam Swing 首振り API

### 1-1. 注意事項

- Limited Preview プログラムは、貴社の秘密保持義務を前提としてリリース前の機能を検証目的でご利用いただくプログラムです (提供する機能またはサービスを以下では「本機能」といいます)。この資料に記載される情報およびサンプルとして提供されるプログラムやスクリプトは機密情報です。取り扱いには十分ご注意ください。
- Limited Preview 期間中は、予告なく機能を停止したり、仕様が変更されることがあります。本番運用環境でのご利用は想定しておりません。
- Limited Preview に関するご不明点やご質問は担当者へご連絡ください。SORACOM サポートでは対応できません。
- 本機能は現状有姿でのご提供となり、当社はその動作、品質等につき保証いたしません。
- この新機能は正式にリリースされない可能性があります。また、その場合の理由は開示されません。
- Limited Preview プログラムにおいて貴社からいただいたフィードバックについては当社で自由に利用することができるものとし、貴社はそれらにつき権利主張を行わないものとします。
- SAM ユーザーで実行する場合は、SAM ユーザーに、API を呼び出すために必要な権限 (例: `SoraCam:*`) に加えて、`OAuth2:authorize` を実行する権限を追加してください。権限設定について詳しくは、[SAM ユーザーの権限を設定する](https://users.soracom.io/ja-jp/docs/sam/set-permissions/) を参照してください。
- 貴社は本機能につき、リバースエンジニアリング、逆コンパイル、逆アセンブルその他の解析を行ってはならないものとします。

---

## 2. Soracom Cloud Camera Services ライブ視聴見放題とは

Soracom Cloud Camera Services ライブ視聴見放題 とは、ATOM アプリと同様にブラウザでライブ映像を見るための機能です。ライブ映像見放題を有効化したソラカメ対応カメラの映像は、エクスポート可能時間を消費することなく、制限なく視聴できます。また、Soracom Cloud Camera Services (通称: ソラカメ) の映像を AI で解析するような目的でも利用できます。

Soracom Cloud Camera Services ライブ視聴見放題 Limited Preview に申し込んだオペレーター (SORACOM アカウント) では、以下の API を利用できます。

- ライブ視聴見放題 API
- ライブ静止画取得 API
- ATOM Cam Swing 首振り API

> ⚠️ **Soracom Cloud Camera Services ライブ視聴見放題 は Limited Preview です**
>
> - お使いのオペレーターで Soracom Cloud Camera Services ライブ視聴見放題 を利用するには、[ユーザーコンソール](https://console.soracom.io/) にルートユーザーでログインし、**[メニュー] → [ソラコムクラウドカメラサービス] → [ライブ視聴見放題 Limited Preview]** の順にクリックして、画面の指示に従って申請してください。
> - オペレーターで Soracom Cloud Camera Services ライブ視聴見放題 の利用を開始しても、すぐには利用料金は発生しません。ソラカメ対応カメラごとにライブ視聴見放題を有効化して、ライブ映像を視聴すると利用料金が発生します。

> ℹ️ ライブ視聴見放題については、[ライブ視聴見放題 Limited Preview](https://users.soracom.io/ja-jp/docs/soracom-cloud-camera-services/unlimited-live-streaming/) を参照してください。

### 2-1. ライブ視聴見放題 API とは

ライブ視聴見放題 API は、ソラカメ対応カメラのライブ視聴見放題の利用を設定したり、利用状況を確認したりする機能です。ライブ動画を再生する URL を取得することもできます。

> ℹ️ **動画のエクスポート可能時間を消費しません**
>
> ライブ視聴見放題 API では、動画のエクスポート可能時間が消費されません。

### 2-2. ライブ静止画取得 API とは

ライブ静止画取得 API は、ソラカメ対応カメラで JPEG 形式の画像を撮影して、ダウンロードする機能です。

撮影した JPEG 形式の画像は、任意のストレージに保存したり、その他のシステムで解析したりできます。ライブ視聴見放題と比べて通信量を削減できます。

> ⚠️ 静止画撮影の準備に数秒かかるため、ライブ静止画取得 API を呼び出した直後の映像は確認できません。

> ℹ️ **動画のエクスポート可能時間を消費しません**
>
> ライブ静止画取得 API では、動画のエクスポート可能時間が消費されません。

### 2-3. ATOM Cam Swing 首振り API とは

ATOM Cam Swing 首振り API は、ATOM Cam Swing のカメラの向きを設定したり、リセットしたりする機能です。

---

## 3. Soracom Cloud Camera Services ライブ視聴見放題 API リファレンス

> ℹ️ SORACOM API を呼び出す際は、API キーと API トークンが必要です。詳しくは、[API キーと API トークンの取り扱いについて](https://users.soracom.io/ja-jp/tools/api/key-and-token/) を参照してください。

### 3-1. ライブ視聴見放題 API

#### ソラカメ対応カメラのライブ視聴見放題の設定を取得する

`SoraCam:isSoraCamDeviceAtomCamUnlimitedLiveStreamEnabled`

ソラカメ対応カメラでライブ視聴見放題が有効化されているかどうかを取得します。

> ⚠️ この API を SAM ユーザーの API キーと API トークンで実行する場合は、SAM ユーザーに、この API を呼び出すために必要な権限 (例: `SoraCam:*`) に加えて、`OAuth2:authorize` を実行する権限を追加してください。権限設定について詳しくは、[SAM ユーザーの権限を設定する](https://users.soracom.io/ja-jp/docs/sam/set-permissions/) を参照してください。

**リクエスト**

```
GET /sora_cam/devices/{device_id}/atom_cam/live_stream/unlimited_enabled
```

パスパラメータ

- `device_id` (string) **\* 必須**: 対象のソラカメ対応カメラのデバイス ID。

**レスポンス**

`200 OK.`

```json
{
  "enabled": true
}
```

- `enabled` (boolean): ライブ視聴見放題が有効化されているかどうか。
  - `true`: 有効
  - `false`: 無効。無効でも、ライブ視聴見放題の試用可能時間が残っている場合はライブ映像を視聴できます。

`404` 指定したソラカメ対応カメラが見つかりません。

---

#### ソラカメ対応カメラのライブ視聴見放題を有効化する

`SoraCam:enableSoraCamDeviceAtomCamUnlimitedLiveStream`

ソラカメ対応カメラのライブ視聴見放題を有効化します。

> ⚠️
> - この API を SAM ユーザーの API キーと API トークンで実行する場合は、SAM ユーザーに、この API を呼び出すために必要な権限 (例: `SoraCam:*`) に加えて、`OAuth2:authorize` を実行する権限を追加してください。権限設定について詳しくは、[SAM ユーザーの権限を設定する](https://users.soracom.io/ja-jp/docs/sam/set-permissions/) を参照してください。
> - ソラカメ対応カメラごとにライブ視聴見放題を有効化しても、すぐには利用料金は発生しません。ライブ映像を視聴すると利用料金が発生します。

**リクエスト**

```
POST /sora_cam/devices/{device_id}/atom_cam/live_stream/enable_unlimited
```

パスパラメータ

- `device_id` (string) **\* 必須**: 対象のソラカメ対応カメラのデバイス ID。

**レスポンス**

`200 OK.`

`404` 指定したソラカメ対応カメラが見つかりません。

---

#### ソラカメ対応カメラのライブ視聴見放題を無効化する

`SoraCam:disableSoraCamDeviceAtomCamUnlimitedLiveStream`

ソラカメ対応カメラのライブ視聴見放題を無効化します。

> ⚠️ この API を SAM ユーザーの API キーと API トークンで実行する場合は、SAM ユーザーに、この API を呼び出すために必要な権限 (例: `SoraCam:*`) に加えて、`OAuth2:authorize` を実行する権限を追加してください。権限設定について詳しくは、[SAM ユーザーの権限を設定する](https://users.soracom.io/ja-jp/docs/sam/set-permissions/) を参照してください。

**リクエスト**

```
POST /sora_cam/devices/{device_id}/atom_cam/live_stream/disable_unlimited
```

パスパラメータ

- `device_id` (string) **\* 必須**: 対象のソラカメ対応カメラのデバイス ID。

**レスポンス**

`200 OK.`

`404` 指定したソラカメ対応カメラが見つかりません。

---

#### ソラカメ対応カメラのライブ視聴見放題の試用可能時間を取得する

`SoraCam:getSoraCamDeviceAtomCamUnlimitedLiveStreamTrialUsage`

ソラカメ対応カメラのライブ視聴見放題の試用可能時間の今月の使用状況を取得します。

> ⚠️ この API を SAM ユーザーの API キーと API トークンで実行する場合は、SAM ユーザーに、この API を呼び出すために必要な権限 (例: `SoraCam:*`) に加えて、`OAuth2:authorize` を実行する権限を追加してください。権限設定について詳しくは、[SAM ユーザーの権限を設定する](https://users.soracom.io/ja-jp/docs/sam/set-permissions/) を参照してください。

**リクエスト**

```
GET /sora_cam/devices/{device_id}/atom_cam/live_stream/trial_usage
```

パスパラメータ

- `device_id` (string) **\* 必須**: 対象のソラカメ対応カメラのデバイス ID。

**レスポンス**

`200 OK.`

```json
{
  "consumedSeconds": 150,
  "totalTrialSeconds": 300
}
```

- `consumedSeconds` (integer): ライブ視聴見放題の試用可能時間のうち、今月の消費時間 (秒単位)。
- `totalTrialSeconds` (integer): ライブ視聴見放題の試用可能時間 (秒単位)。

`404` 指定したソラカメ対応カメラが見つかりません。

---

#### ライブ動画を再生する URL を取得する

`SoraCam:startSoraCamDeviceAtomCamLiveStream`

ライブ動画を再生する URL を取得します。

> ⚠️
> - クラウド常時録画ライセンスまたはクラウドモーション検知 無制限 録画ライセンスを割り当てたソラカメ対応カメラの場合は、1 台ごとに月あたり約 5 分間の試用可能時間が用意されています。そのため、ライブ視聴見放題を有効化していない (無効化した) ソラカメ対応カメラでも、試用可能時間内はライブ映像を見られます。試用可能時間を使い切ったソラカメ対応カメラでライブ映像を見るには、ライブ視聴見放題を有効化してください。
> - この API を SAM ユーザーの API キーと API トークンで実行する場合は、SAM ユーザーに、この API を呼び出すために必要な権限 (例: `SoraCam:*`) に加えて、`OAuth2:authorize` を実行する権限を追加してください。権限設定について詳しくは、[SAM ユーザーの権限を設定する](https://users.soracom.io/ja-jp/docs/sam/set-permissions/) を参照してください。

**リクエスト**

```
POST /sora_cam/devices/{device_id}/atom_cam/live_stream/start
```

パスパラメータ

- `device_id` (string) **\* 必須**: 対象のソラカメ対応カメラのデバイス ID。

**レスポンス**

`200 OK.`

```json
{
  "url": "https://xxxxxxxxxxxxxxx.p.soracam.soracom.io/devices/7CDDDEADBEEF/live_stream/live.mpd?token=foo"
}
```

- `url` (string): ライブ映像を再生するための URL (MPEG-DASH)。URL の有効期間は約 60 秒間です。
  - 有効期間が過ぎると、この URL では再生を開始できません。URL を再発行してください。
  - URL が有効な間に再生を開始すれば、有効期間後もライブストリーミング再生を継続できます。

`404` 指定したソラカメ対応カメラが見つかりません。

> ℹ️ 取得した URL を [VLC media player](https://www.videolan.org/vlc/) で再生したり、[streamlink](https://streamlink.github.io/) と [ffmpeg](https://ffmpeg.org/) を利用して保存したりできます。なお、VLC media player や streamlink、ffmpeg の操作はサポート対象外です。
>
> **VLC media player を利用する例:**
>
> **[メディア] → [ネットワークストリームを開く]** の順にクリックして、**[ネットワーク URL を入力してください]** に、API のレスポンスで返された URL を指定してください。
> - VLC media player を使用する場合は、約 60 秒前まで遡れます。
>
> **streamlink と ffmpeg を利用する例:**
>
> ```bash
> $ filename=$(date +"out_%Y%m%d%H%M")
> $ streamlink --stream-timeout 15 "$url" best -o "$filename.ts"
> $ ffmpeg -i "$filename.ts" -c copy "$filename.mp4"
> ```
>
> `$url` には、API のレスポンスで返された URL を指定してください。

---

### 3-2. ライブ静止画取得 API

ライブ静止画を取得できます。

> ℹ️ SORACOM API を呼び出す際は、API キーと API トークンが必要です。詳しくは、[API キーと API トークンの取り扱いについて](https://users.soracom.io/ja-jp/tools/api/key-and-token/) を参照してください。

#### ライブ静止画を撮影するための URL を取得する

`SoraCam:startSoraCamDeviceAtomCamStillPicture`

ソラカメ対応カメラを指定して、ライブ静止画を**撮影するための URL** を取得します。

> ⚠️
> - クラウド常時録画ライセンスまたはクラウドモーション検知 無制限 録画ライセンスを割り当てたソラカメ対応カメラの場合は、1 台ごとに月あたり約 5 分間の試用可能時間が用意されています。そのため、ライブ視聴見放題を有効化していない (無効化した) ソラカメ対応カメラでも、試用可能時間内はライブ静止画を撮影できます。試用可能時間を使い切ったソラカメ対応カメラでライブ静止画を撮影するには、ライブ視聴見放題を有効化してください。
> - `SoraCam:startSoraCamDeviceAtomCamStillPicture` API を呼び出した時点では、ライブ静止画は撮影されません。ライブ静止画を撮影する手順については [ライブ静止画を撮影する](#ライブ静止画を撮影する) を参照してください。
> - この API を SAM ユーザーの API キーと API トークンで実行する場合は、SAM ユーザーに、この API を呼び出すために必要な権限 (例: `SoraCam:*`) に加えて、`OAuth2:authorize` を実行する権限を追加してください。権限設定について詳しくは、[SAM ユーザーの権限を設定する](https://users.soracom.io/ja-jp/docs/sam/set-permissions/) を参照してください。

**リクエスト**

```
GET /sora_cam/devices/{device_id}/atom_cam/still_picture
```

パスパラメータ

- `device_id` (string) **\* 必須**: 対象のソラカメ対応カメラのデバイス ID。

**レスポンス**

`200 OK.`

```json
{
  "url": "https://atom-live-stream-abcd0123.soracam.soracom.io/devices/7CDDDEADBEEF/still_picture/image.jpg?token=xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
}
```

- `url` (string): ライブ静止画を**撮影するための URL**。URL の有効期間は、約 5 分間です。
  - 有効期間が過ぎると、この URL では静止画を撮影できません。URL を再発行してください。
  - URL が有効な間に、1 枚だけライブ静止画を撮影できます。

`404` 指定したソラカメ対応カメラが見つかりません。

#### ライブ静止画を撮影する

`SoraCam:startSoraCamDeviceAtomCamStillPicture` API で取得した URL にアクセスすると、ソラカメ対応カメラによってライブ静止画が撮影され、約 15 秒後にライブ静止画がダウンロードされます。

- URL の有効期間は、約 5 分間です。URL が有効な間に、1 枚だけライブ静止画を撮影できます。

> ⚠️
> - 取得した URL には、2 回アクセスしないでください。
> - ライブ静止画を撮影する URL には、ユーザー認証がありません。そのため、権限を設定して SAM ユーザーによる撮影を禁止する、といった設定はできません。

> ℹ️ 取得した URL にアクセスすると、以下の情報が `_` で連結された文字列がファイル名として、レスポンスヘッダーの `Content-Disposition` に設定されます。`curl -O -J $url` のような方法でダウンロードすると、このファイル名でダウンロードできます。
>
> - 撮影日時 (UTC) (例: 静止画の場合は `2023-08-01-00-00-00-000Z`)
> - オペレーター ID (例: `OP0123456789`)
> - デバイス ID (例: `7CDDDEADBEEF`)
> - ソラカメ対応カメラの名前 (例: `CameraName`)
> - 拡張子 (静止画の場合は `.jpg`)
>
> 例: `2024-03-01-00-00-00-000Z_OP0123456789_7CDDDEADBEEF_CameraName.jpg`

**サンプルリクエスト**

```bash
$response=$(curl -X GET \
  'https://api.soracom.io/v1/sora_cam/devices/7CDDDEADBEEF/atom_cam/still_picture' \
  -H "X-Soracom-API-Key: ${X_SORACOM_API_KEY}" \
  -H "X-Soracom-Token: ${X_SORACOM_TOKEN}")
echo $response
url=$(echo "$response" | jq -r .url)

curl -O -J $url
```

---

### 3-3. ATOM Cam Swing 首振り API

ATOM Cam Swing のカメラの向きを設定したり、リセットしたりできます。

> ℹ️ SORACOM API を呼び出す際は、API キーと API トークンが必要です。詳しくは、[API キーと API トークンの取り扱いについて](https://users.soracom.io/ja-jp/tools/api/key-and-token/) を参照してください。

#### カメラの向きを取得する

`SoraCam:getSoraCamDeviceAtomCamSwingPosition`

ATOM Cam Swing のカメラの向きを取得します。

> ⚠️ この API を SAM ユーザーの API キーと API トークンで実行する場合は、SAM ユーザーに、この API を呼び出すために必要な権限 (例: `SoraCam:*`) に加えて、`OAuth2:authorize` を実行する権限を追加してください。権限設定について詳しくは、[SAM ユーザーの権限を設定する](https://users.soracom.io/ja-jp/docs/sam/set-permissions/) を参照してください。

**リクエスト**

```
GET /sora_cam/devices/{device_id}/atom_cam/swing/position
```

パスパラメータ

- `device_id` (string) **\* 必須**: 対象の ATOM Cam Swing のデバイス ID。

**レスポンス**

`200 OK.`

```json
{
  "horizontal": 45,
  "vertical": -30
}
```

- `horizontal` (integer): 水平方向の向き (`-177` ～ `177`)。
- `vertical` (integer): 垂直方向の向き (`-90` ～ `90`)。

`404` 指定したソラカメ対応カメラが見つかりません。

---

#### カメラの向きを設定する

`SoraCam:setSoraCamDeviceAtomCamSwingPosition`

ATOM Cam Swing のカメラの向きを設定します。

> ⚠️
> - この API を SAM ユーザーの API キーと API トークンで実行する場合は、SAM ユーザーに、この API を呼び出すために必要な権限 (例: `SoraCam:*`) に加えて、`OAuth2:authorize` を実行する権限を追加してください。権限設定について詳しくは、[SAM ユーザーの権限を設定する](https://users.soracom.io/ja-jp/docs/sam/set-permissions/) を参照してください。
> - カメラの向きによっては、誤差が生じることがあります。また、調整を繰り返すと、カメラの実際の向きとカメラの内部状態に差異が発生することがあります。その場合は、[カメラの向きをリセット (`SoraCam:resetSoraCamDeviceAtomCamSwingPosition`)](#カメラの向きをリセットする) してください。
> - [画像を 180° 回転させる設定](https://users.soracom.io/ja-jp/tools/api/reference/#/SoraCam/getSoraCamDeviceAtomCamSettingsRotation) が `180`: ON (180° 回転) の場合は、`0`: OFF (回転なし) と比べると、カメラが動く向きが逆に感じられます。これは、実際にカメラを 180° 回転させて設置した場合を想定した動作です。画像を 180° 回転させる設定は、実際のカメラの設置状況と一致させてください。

**リクエスト**

```
POST /sora_cam/devices/{device_id}/atom_cam/swing/position
```

パスパラメータ

- `device_id` (string) **\* 必須**: 対象の ATOM Cam Swing のデバイス ID。

リクエストボディ

```json
{
  "horizontal": 45,
  "vertical": -30
}
```

- `horizontal` (integer): 水平方向の向き (`-177` ～ `177`)。
- `vertical` (integer): 垂直方向の向き (`-90` ～ `90`)。

**レスポンス**

`200 OK.`

`400` 不正なリクエストです。

`404` 指定したソラカメ対応カメラが見つかりません。

---

#### カメラの向きをリセットする

`SoraCam:resetSoraCamDeviceAtomCamSwingPosition`

ATOM Cam Swing のカメラの向きを初期化します。

> ⚠️ この API を SAM ユーザーの API キーと API トークンで実行する場合は、SAM ユーザーに、この API を呼び出すために必要な権限 (例: `SoraCam:*`) に加えて、`OAuth2:authorize` を実行する権限を追加してください。権限設定について詳しくは、[SAM ユーザーの権限を設定する](https://users.soracom.io/ja-jp/docs/sam/set-permissions/) を参照してください。

**リクエスト**

```
POST /sora_cam/devices/{device_id}/atom_cam/swing/position/reset
```

パスパラメータ

- `device_id` (string) **\* 必須**: 対象の ATOM Cam Swing のデバイス ID。

**レスポンス**

`200 OK.`

`400` 不正なリクエストです。

`404` 指定したソラカメ対応カメラが見つかりません。
