# SORACOM MCP Server

Soracom Cloud Camera Services (ソラカメ / SoraCam) の **ライブ視聴 URL 取得 API** と
**ライブ静止画取得 API** を MCP ツールとして呼び出す Node.js / TypeScript 製サーバーです。

起動時に SORACOM の認証情報 (AuthKeyId / AuthKey) を受け取り、初回 API 呼び出し時に
[Auth API](https://users.soracom.io/ja-jp/tools/api/reference/#/Auth/auth) で
API キー / トークンを取得して、以降の呼び出しに自動付与します。

## Tools

- `get_live_view_unlimited`:
  ライブ動画を再生する URL (MPEG-DASH、約60秒間有効) を取得します。
  - 内部: `POST /v1/sora_cam/devices/{deviceId}/atom_cam/live_stream/start`
- `get_live_still_image`:
  ライブ静止画 (JPEG) を撮影し、Base64 付き JSON で返します。
  - 内部 2 段階:
    1. `GET /v1/sora_cam/devices/{deviceId}/atom_cam/still_picture` で撮影用 URL を取得
    2. その URL に **認証ヘッダー無しで** アクセスして撮影・ダウンロード (約15秒)

いずれのツールも `deviceId` 引数は任意で、省略時は環境変数 `SORACOM_DEVICE_ID` を使います。

## 設定

| 環境変数 | CLI | 必須 | 説明 |
|---|---|---|---|
| `SORACOM_AUTH_KEY_ID` | `--auth-key-id` | ✔ | Auth API 用の AuthKeyId |
| `SORACOM_AUTH_KEY` | `--auth-key` | ✔ | Auth API 用の AuthKey (秘密鍵) |
| `SORACOM_COVERAGE` | `--coverage` | – | `jp` (既定) または `g`。base URL を決定 |
| `SORACOM_DEVICE_ID` | `--device-id` | – | 操作対象デバイス ID の既定値 (ツール引数で上書き可) |
| `SORACOM_TOKEN_TIMEOUT_SECONDS` | `--token-timeout` | – | Auth トークン有効秒 (既定 `86400`) |

coverage と base URL の対応:

- `jp` → `https://api.soracom.io`
- `g` → `https://g.api.soracom.io`

> ℹ️ AuthKeyId / AuthKey は SORACOM ユーザーコンソールで発行する認証キーです。
> SAM ユーザーで実行する場合は `SoraCam:*` に加えて `OAuth2:authorize` の権限が必要です。

## 開発

```bash
cd soracom-mcp-server
npm install
npm run typecheck
npm run build
```
