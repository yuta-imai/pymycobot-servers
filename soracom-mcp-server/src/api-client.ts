export interface SoracomClientOptions {
  baseUrl: string;
  apiKey?: string;
  apiToken?: string;
  bearerToken?: string;
  liveViewPathTemplate: string;
  stillImagePathTemplate: string;
  liveViewExpiresQueryName: string;
  stillImageWidthQueryName: string;
  stillImageHeightQueryName: string;
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

interface QueryValue {
  value: string | number | boolean;
}

export class SoracomClient {
  constructor(private readonly options: SoracomClientOptions) {}

  async getLiveView(params: {
    subscriptionId: string;
    expiresInSeconds?: number;
  }) {
    const query: Record<string, QueryValue> = {};
    if (params.expiresInSeconds !== undefined) {
      query[this.options.liveViewExpiresQueryName] = { value: params.expiresInSeconds };
    }

    const response = await this.request(
      "GET",
      this.interpolatePath(this.options.liveViewPathTemplate, {
        subscriptionId: params.subscriptionId,
      }),
      query,
    );

    const contentType = response.headers.get("content-type") ?? "";
    if (contentType.includes("application/json")) {
      return response.json();
    }

    return {
      url: (await response.text()).trim(),
      contentType,
    };
  }

  async getLiveStillImage(params: {
    subscriptionId: string;
    width?: number;
    height?: number;
  }) {
    const query: Record<string, QueryValue> = {};
    if (params.width !== undefined) {
      query[this.options.stillImageWidthQueryName] = { value: params.width };
    }
    if (params.height !== undefined) {
      query[this.options.stillImageHeightQueryName] = { value: params.height };
    }

    const response = await this.request(
      "GET",
      this.interpolatePath(this.options.stillImagePathTemplate, {
        subscriptionId: params.subscriptionId,
      }),
      query,
    );

    const contentType = response.headers.get("content-type") ?? "application/octet-stream";
    if (contentType.includes("application/json")) {
      return response.json();
    }

    const bytes = new Uint8Array(await response.arrayBuffer());
    return {
      contentType,
      contentLength: bytes.byteLength,
      imageBase64: Buffer.from(bytes).toString("base64"),
    };
  }

  private async request(
    method: "GET",
    path: string,
    query: Record<string, QueryValue>,
  ): Promise<Response> {
    const url = new URL(path, this.options.baseUrl);
    Object.entries(query).forEach(([key, q]) => {
      url.searchParams.set(key, String(q.value));
    });

    const headers: Record<string, string> = {
      Accept: "*/*",
    };

    if (this.options.apiKey && this.options.apiToken) {
      headers["X-Soracom-API-Key"] = this.options.apiKey;
      headers["X-Soracom-Token"] = this.options.apiToken;
    }
    if (this.options.bearerToken) {
      headers.Authorization = "Bearer " + this.options.bearerToken;
    }

    const response = await fetch(url, {
      method,
      headers,
    });

    if (!response.ok) {
      let detail: unknown;
      try {
        detail = await response.json();
      } catch {
        detail = await response.text();
      }
      const detailText = detail ? `: ${typeof detail === "string" ? detail : JSON.stringify(detail)}` : "";
      throw new ApiError(
        `SORACOM API ${response.status} ${response.statusText}${detailText}`,
        response.status,
        detail,
      );
    }

    return response;
  }

  private interpolatePath(template: string, values: Record<string, string>): string {
    return template.replace(/\{([a-zA-Z0-9_]+)\}/g, (_, key: string) => {
      const value = values[key];
      if (value === undefined) {
        throw new ApiError(`Path template variable {${key}} was not provided.`);
      }
      return encodeURIComponent(value);
    });
  }
}
