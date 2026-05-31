import { defineConfig } from "tsup";

export default defineConfig({
  entry: ["src/index.ts"],
  format: ["esm"],
  target: "node18",
  clean: true,
  // Bundle deps so `npx` works without an install step at the consumer side.
  noExternal: [/.*/],
  banner: {
    js: "#!/usr/bin/env node",
  },
});
