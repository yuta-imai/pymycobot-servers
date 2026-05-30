# SORACOM MCP Server

ライブ視聴見放題 API とライブ静止画取得 API を MCP ツールとして呼び出すための
Node.js / TypeScript 製サーバーです。

## Tools

- `get_live_view_unlimited`:
  ライブ視聴見放題 API を呼び出して URL などを取得
- `get_live_still_image`:
  ライブ静止画取得 API を呼び出して画像(Base64)を取得

## 設定

環境変数:

- `SORACOM_API_BASE_URL` (default: `https://api.soracom.io`)
- `SORACOM_API_KEY`
- `SORACOM_TOKEN`
- `SORACOM_BEARER_TOKEN` (optional)
- `SORACOM_LIVE_VIEW_PATH` (default: `/v1/liveStreaming/subscribers/{subscriptionId}/viewUnlimited`)
- `SORACOM_STILL_IMAGE_PATH` (default: `/v1/liveStreaming/subscribers/{subscriptionId}/stillImage`)
- `SORACOM_LIVE_VIEW_EXPIRES_QUERY` (default: `expiresInSeconds`)
- `SORACOM_STILL_IMAGE_WIDTH_QUERY` (default: `width`)
- `SORACOM_STILL_IMAGE_HEIGHT_QUERY` (default: `height`)

CLI オプション:

- `--api-base-url`
- `--live-view-path`
- `--still-image-path`
- `--live-view-expires-query`
- `--still-image-width-query`
- `--still-image-height-query`

## 開発

```bash
cd soracom-mcp-server
npm install
npm run typecheck
npm run build
```
