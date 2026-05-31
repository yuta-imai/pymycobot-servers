export interface SoracomClientOptions {
  baseUrl: string;
  authKeyId: string;
  authKey: string;
  tokenTimeoutSeconds?: number;
}

export class ApiError extends Error {
  constructor(
    message: string,
    public readonly status?: number,
    public readonly detail?: unknown,
  ) {
    super(message);
    this.name = "ApiError";
  }
}

interface AuthResult {
  apiKey: string;
  token: string;
  operatorId?: string;
}

interface RequestOptions {
  body?: unknown;
  /** Whether to attach the SORACOM API key/token headers. Defaults to true. */
  auth?: boolean;
}

export class SoracomClient {
  private auth?: AuthResult;

  constructor(private readonly options: SoracomClientOptions) {}

  /**
   * ライブ動画を再生する URL (MPEG-DASH) を取得します。
   * URL の有効期間は約 60 秒です。
   */
  async getLiveView(params: { deviceId: string }) {
    const response = await this.request(
      "POST",
      `/v1/sora_cam/devices/${encodeURIComponent(params.deviceId)}/atom_cam/live_stream/start`,
    );
    return response.json();
  }

  /**
   * ライブ静止画 (JPEG) を 2 段階フローで撮影し、Base64 で返します。
   * 1. 撮影用 URL を取得 (認証あり)。
   * 2. その URL にアクセスして撮影・ダウンロード (認証なし・約 15 秒)。
   */
  async getLiveStillImage(params: { deviceId: string }) {
    const urlResponse = await this.request(
      "GET",
      `/v1/sora_cam/devices/${encodeURIComponent(params.deviceId)}/atom_cam/still_picture`,
    );
    const { url } = (await urlResponse.json()) as { url?: string };
    if (!url) {
      throw new ApiError("ライブ静止画の撮影用 URL を取得できませんでした。");
    }

    // 撮影用 URL は署名付きでユーザー認証を持たないため、認証ヘッダーは付与しない。
    const imageResponse = await fetch(url, {
      method: "GET",
      headers: { Accept: "*/*" },
    });
    if (!imageResponse.ok) {
      throw await this.toApiError(imageResponse);
    }

    const contentType =
      imageResponse.headers.get("content-type") ?? "image/jpeg";
    const fileName = this.parseContentDispositionFileName(
      imageResponse.headers.get("content-disposition"),
    );
    const bytes = new Uint8Array(await imageResponse.arrayBuffer());

    return {
      contentType,
      contentLength: bytes.byteLength,
      fileName,
      imageBase64: Buffer.from(bytes).toString("base64"),
    };
  }

  private async authenticate(): Promise<AuthResult> {
    const url = new URL("/v1/auth", this.options.baseUrl);
    const response = await fetch(url, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        Accept: "application/json",
      },
      body: JSON.stringify({
        authKeyId: this.options.authKeyId,
        authKey: this.options.authKey,
        tokenTimeoutSeconds: this.options.tokenTimeoutSeconds ?? 86400,
      }),
    });

    if (!response.ok) {
      throw await this.toApiError(response, "SORACOM Auth API");
    }

    const data = (await response.json()) as {
      apiKey?: string;
      token?: string;
      operatorId?: string;
    };
    if (!data.apiKey || !data.token) {
      throw new ApiError("SORACOM Auth API のレスポンスに apiKey/token が含まれていません。");
    }

    this.auth = {
      apiKey: data.apiKey,
      token: data.token,
      operatorId: data.operatorId,
    };
    return this.auth;
  }

  private async request(
    method: "GET" | "POST",
    path: string,
    options: RequestOptions = {},
  ): Promise<Response> {
    const { body, auth = true } = options;
    const send = async (): Promise<Response> => {
      const url = new URL(path, this.options.baseUrl);
      const headers: Record<string, string> = { Accept: "application/json" };

      if (auth) {
        const credentials = this.auth ?? (await this.authenticate());
        headers["X-Soracom-API-Key"] = credentials.apiKey;
        headers["X-Soracom-Token"] = credentials.token;
      }
      if (body !== undefined) {
        headers["Content-Type"] = "application/json";
      }

      return fetch(url, {
        method,
        headers,
        body: body !== undefined ? JSON.stringify(body) : undefined,
      });
    };

    let response = await send();

    // トークン失効に備え、401/403 のときは 1 度だけ再認証してリトライ。
    if (auth && (response.status === 401 || response.status === 403)) {
      this.auth = undefined;
      response = await send();
    }

    if (!response.ok) {
      throw await this.toApiError(response);
    }

    return response;
  }

  private async toApiError(
    response: Response,
    label = "SORACOM API",
  ): Promise<ApiError> {
    let detail: unknown;
    try {
      detail = await response.json();
    } catch {
      try {
        detail = await response.text();
      } catch {
        detail = undefined;
      }
    }
    const detailText = detail
      ? `: ${typeof detail === "string" ? detail : JSON.stringify(detail)}`
      : "";
    return new ApiError(
      `${label} ${response.status} ${response.statusText}${detailText}`,
      response.status,
      detail,
    );
  }

  private parseContentDispositionFileName(
    header: string | null,
  ): string | undefined {
    if (!header) {
      return undefined;
    }
    const utf8Match = header.match(/filename\*=(?:UTF-8'')?([^;]+)/i);
    if (utf8Match) {
      try {
        return decodeURIComponent(utf8Match[1].trim().replace(/^"|"$/g, ""));
      } catch {
        return utf8Match[1].trim().replace(/^"|"$/g, "");
      }
    }
    const match = header.match(/filename="?([^";]+)"?/i);
    return match ? match[1].trim() : undefined;
  }
}
