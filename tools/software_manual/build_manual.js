#!/usr/bin/env node
"use strict";

/*
 * 软件说明手册 HTML 生成与实时预览工具。
 *
 * 默认执行一次构建：
 *   node tools/software_manual/build_manual.js
 *
 * 启动监听、HTTP 服务并自动打开浏览器：
 *   node tools/software_manual/build_manual.js --watch --open
 *
 * Markdown 始终是唯一内容真值；HTML 是由本工具生成的浏览器阅读版本，
 * 不允许直接在生成后的 HTML 中维护业务内容。
 */

const crypto = require("crypto");
const fs = require("fs");
const http = require("http");
const path = require("path");
const { spawn } = require("child_process");
const { marked, Renderer } = require("./vendor/marked.umd.cjs");

const PROJECT_ROOT = path.resolve(__dirname, "..", "..");
const DOCS_DIR = path.join(PROJECT_ROOT, "docs");
const SOURCE_PATH = path.join(DOCS_DIR, "软件说明手册.md");
const OUTPUT_PATH = path.join(DOCS_DIR, "软件说明手册.html");
const TEMPLATE_PATH = path.join(__dirname, "manual_template.html");
const DEFAULT_PORT = 8765;
const WATCH_INTERVAL_MS = 500;
const REBUILD_DEBOUNCE_MS = 180;

/*
 * 函数功能：对准备写入 HTML 属性或文本节点的内容进行转义。
 * 输入参数：value 为任意可转成字符串的内容。
 * 返回参数：不会破坏 HTML 结构的安全字符串。
 */
function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

/*
 * 函数功能：从 marked 标题 token 中提取不带 Markdown 格式的目录文字。
 * 输入参数：tokens 为 marked 解析后的行内 token 数组。
 * 返回参数：用于标题 ID、目录和搜索索引的纯文本。
 */
function inlineTokensToText(tokens) {
  if (!Array.isArray(tokens)) {
    return "";
  }

  return tokens
    .map((token) => {
      if (Array.isArray(token.tokens)) {
        return inlineTokensToText(token.tokens);
      }

      if (typeof token.text === "string") {
        return token.text;
      }

      if (typeof token.raw === "string") {
        return token.raw;
      }

      return "";
    })
    .join("");
}

/*
 * 函数功能：按 GitHub/MPE 常见规则把中文标题转换为稳定锚点。
 * 输入参数：headingText 为标题纯文本。
 * 返回参数：可用于 id 和 #hash 的基础锚点。
 */
function makeBaseSlug(headingText) {
  const normalized = String(headingText)
    .normalize("NFKC")
    .trim()
    .toLowerCase()
    .replace(/[\u0000-\u001f\u007f]/gu, "")
    .replace(/[^\p{Letter}\p{Number}\s_-]/gu, "")
    .replace(/\s+/gu, "-")
    .replace(/-+/gu, "-")
    .replace(/^-|-$/gu, "");

  return normalized || "section";
}

/*
 * 函数功能：为重复标题生成不会冲突的稳定 ID。
 * 输入参数：无，函数内部维护本次构建的标题计数。
 * 返回参数：接收标题文字并返回唯一 ID 的函数。
 */
function createSlugger() {
  const counts = new Map();

  return (headingText) => {
    const base = makeBaseSlug(headingText);
    const count = counts.get(base) || 0;
    counts.set(base, count + 1);
    return count === 0 ? base : `${base}-${count}`;
  };
}

/*
 * 函数功能：建立适用于长篇工程手册的 Markdown 渲染器。
 * 输入参数：无。
 * 返回参数：配置了标题锚点、代码块和 Mermaid 容器的 marked Renderer。
 */
function createRenderer() {
  const renderer = new Renderer();
  const slug = createSlugger();

  renderer.heading = function renderHeading(token) {
    const plainText = inlineTokensToText(token.tokens).trim();
    const id = slug(plainText);
    const inlineHtml = this.parser.parseInline(token.tokens);
    const depth = Math.min(Math.max(Number(token.depth) || 2, 1), 6);

    return [
      `<h${depth} id="${escapeHtml(id)}" data-toc-title="${escapeHtml(plainText)}">`,
      `<a class="heading-anchor" href="#${escapeHtml(id)}" aria-label="复制本节链接">#</a>`,
      inlineHtml,
      `</h${depth}>`,
    ].join("");
  };

  renderer.code = function renderCode(token) {
    const language = String(token.lang || "")
      .trim()
      .split(/\s+/u)[0]
      .toLowerCase();
    const codeText = String(token.text || "");

    if (language === "mermaid") {
      return [
        '<figure class="diagram-card" data-diagram-state="pending">',
        '<figcaption><span class="diagram-dot"></span>结构/流程图</figcaption>',
        `<div class="mermaid">${escapeHtml(codeText)}</div>`,
        "</figure>",
      ].join("");
    }

    const label = language || "TEXT";
    return [
      `<div class="code-card" data-language="${escapeHtml(label)}">`,
      '<div class="code-toolbar">',
      `<span>${escapeHtml(label.toUpperCase())}</span>`,
      '<button class="copy-code" type="button" aria-label="复制代码">复制</button>',
      "</div>",
      `<pre><code class="language-${escapeHtml(label)}">${escapeHtml(codeText)}</code></pre>`,
      "</div>",
    ].join("");
  };

  return renderer;
}

/*
 * 函数功能：把软件说明手册 Markdown 转换成带导航阅读界面的完整 HTML。
 * 输入参数：无，函数从固定的 docs/软件说明手册.md 读取内容。
 * 返回参数：包含输出路径、哈希和生成时间的构建结果。
 */
function buildManual() {
  if (!fs.existsSync(SOURCE_PATH)) {
    throw new Error(`找不到 Markdown 源文件：${SOURCE_PATH}`);
  }

  if (!fs.existsSync(TEMPLATE_PATH)) {
    throw new Error(`找不到 HTML 模板：${TEMPLATE_PATH}`);
  }

  const markdown = fs.readFileSync(SOURCE_PATH, "utf8").replace(/^\uFEFF/u, "");
  const sourceStat = fs.statSync(SOURCE_PATH);
  const sourceHash = crypto.createHash("sha256").update(markdown, "utf8").digest("hex");
  const generatedAt = new Date();
  const renderer = createRenderer();

  const articleHtml = marked.parse(markdown, {
    renderer,
    gfm: true,
    breaks: false,
    pedantic: false,
  });

  const template = fs.readFileSync(TEMPLATE_PATH, "utf8");
  const replacements = new Map([
    ["__MANUAL_CONTENT__", articleHtml],
    ["__SOURCE_SHA256__", escapeHtml(sourceHash)],
    ["__SOURCE_SHA_SHORT__", escapeHtml(sourceHash.slice(0, 12).toUpperCase())],
    ["__SOURCE_UPDATED_AT__", escapeHtml(sourceStat.mtime.toLocaleString("zh-CN", { hour12: false }))],
    ["__GENERATED_AT__", escapeHtml(generatedAt.toLocaleString("zh-CN", { hour12: false }))],
  ]);

  let output = template;
  for (const [placeholder, value] of replacements) {
    output = output.replaceAll(placeholder, () => value);
  }

  /*
   * 直接写固定输出文件，让浏览器在普通双击模式也始终有一份完整静态快照。
   * 监听模式会在 Markdown 保存后重复执行本函数。
   */
  fs.writeFileSync(OUTPUT_PATH, output, "utf8");

  return {
    outputPath: OUTPUT_PATH,
    html: output,
    sourceHash,
    generatedAt,
    bytes: Buffer.byteLength(output, "utf8"),
  };
}

/*
 * 函数功能：把文件扩展名映射为本地预览服务器的 Content-Type。
 * 输入参数：filePath 为准备响应的文件路径。
 * 返回参数：HTTP Content-Type 字符串。
 */
function getContentType(filePath) {
  const extension = path.extname(filePath).toLowerCase();
  const types = {
    ".html": "text/html; charset=utf-8",
    ".md": "text/markdown; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".gif": "image/gif",
    ".svg": "image/svg+xml",
    ".webp": "image/webp",
  };

  return types[extension] || "application/octet-stream";
}

/*
 * 函数功能：打开系统默认浏览器进入实时预览地址。
 * 输入参数：url 为 localhost 手册地址。
 * 返回参数：无。
 */
function openBrowser(url) {
  const child = spawn("cmd.exe", ["/d", "/c", "start", "", url], {
    detached: true,
    stdio: "ignore",
    windowsHide: true,
  });
  child.unref();
}

/*
 * 函数功能：向所有浏览器 EventSource 连接发送手册更新事件。
 * 输入参数：clients 为响应对象集合，payload 为事件数据。
 * 返回参数：无。
 */
function broadcastUpdate(clients, payload) {
  const body = `event: manual-updated\ndata: ${JSON.stringify(payload)}\n\n`;
  for (const response of clients) {
    try {
      response.write(body);
    } catch {
      clients.delete(response);
    }
  }
}

/*
 * 函数功能：启动只服务 docs 目录的本地 HTTP 预览服务器。
 * 输入参数：preferredPort 为首选端口，openWhenReady 表示监听成功后是否打开浏览器，
 *           getManualHtml 用于取得内存中的最新手册 HTML。
 * 返回参数：Promise，成功后返回服务器、端口和实时连接集合。
 */
function startPreviewServer(preferredPort, openWhenReady, getManualHtml) {
  const clients = new Set();

  return new Promise((resolve, reject) => {
    const tryListen = (port, remainingAttempts) => {
      const server = http.createServer((request, response) => {
        const requestUrl = new URL(request.url || "/", `http://127.0.0.1:${port}`);
        const pathname = decodeURIComponent(requestUrl.pathname);

        if (pathname === "/__manual_events") {
          response.writeHead(200, {
            "Content-Type": "text/event-stream; charset=utf-8",
            "Cache-Control": "no-cache",
            Connection: "keep-alive",
            "X-Accel-Buffering": "no",
          });
          response.write("event: ready\ndata: connected\n\n");
          clients.add(response);
          request.on("close", () => clients.delete(response));
          return;
        }

        if (pathname === "/") {
          response.writeHead(302, {
            Location: `/${encodeURIComponent(path.basename(OUTPUT_PATH))}`,
            "Cache-Control": "no-store",
          });
          response.end();
          return;
        }

        /*
         * 加密软件可能允许 Node.js 生成HTML，却不允许浏览器直接解密磁盘上的 .html 文件。
         * 因此主页面始终从生成器内存返回，浏览器不再直接读取可能被加密的磁盘文件。
         */
        if (pathname === `/${path.basename(OUTPUT_PATH)}`) {
          response.writeHead(200, {
            "Content-Type": "text/html; charset=utf-8",
            "Cache-Control": "no-store, max-age=0",
            "X-EH-Manual-Source": "memory",
          });
          response.end(getManualHtml(), "utf8");
          return;
        }

        /*
         * 预览服务只允许访问 docs 目录，防止浏览器通过 ../ 读取工程其它文件。
         */
        const relativePath = pathname.replace(/^\/+/u, "");
        const requestedPath = path.resolve(DOCS_DIR, relativePath);
        const docsPrefix = `${path.resolve(DOCS_DIR)}${path.sep}`;
        if ((requestedPath !== path.resolve(DOCS_DIR)) && !requestedPath.startsWith(docsPrefix)) {
          response.writeHead(403, { "Content-Type": "text/plain; charset=utf-8" });
          response.end("禁止访问 docs 目录以外的文件。");
          return;
        }

        if (!fs.existsSync(requestedPath) || !fs.statSync(requestedPath).isFile()) {
          response.writeHead(404, { "Content-Type": "text/plain; charset=utf-8" });
          response.end("文件不存在。");
          return;
        }

        response.writeHead(200, {
          "Content-Type": getContentType(requestedPath),
          "Cache-Control": "no-store, max-age=0",
        });
        fs.createReadStream(requestedPath).pipe(response);
      });

      server.once("error", (error) => {
        if ((error.code === "EADDRINUSE") && (remainingAttempts > 0)) {
          tryListen(port + 1, remainingAttempts - 1);
          return;
        }
        reject(error);
      });

      server.listen(port, "127.0.0.1", () => {
        const url = `http://127.0.0.1:${port}/${encodeURIComponent(path.basename(OUTPUT_PATH))}`;
        console.log(`[预览] ${url}`);
        console.log("[监听] 保存 docs/软件说明手册.md 后，HTML 会自动重建并刷新浏览器。");
        console.log("[退出] 在本窗口按 Ctrl+C。");
        if (openWhenReady) {
          openBrowser(url);
        }
        resolve({ server, port, clients, url });
      });
    };

    tryListen(preferredPort, 10);
  });
}

/*
 * 函数功能：解析命令行端口参数。
 * 输入参数：args 为 process.argv 的业务参数。
 * 返回参数：合法端口，未指定时返回默认端口。
 */
function parsePort(args) {
  const portIndex = args.indexOf("--port");
  if ((portIndex < 0) || (portIndex + 1 >= args.length)) {
    return DEFAULT_PORT;
  }

  const port = Number(args[portIndex + 1]);
  if (!Number.isInteger(port) || (port < 1024) || (port > 65535)) {
    throw new Error("--port 必须是 1024～65535 之间的整数。");
  }

  return port;
}

/*
 * 函数功能：执行一次构建，或进入实时监听模式。
 * 输入参数：无，直接读取 process.argv。
 * 返回参数：Promise<void>。
 */
async function main() {
  const args = process.argv.slice(2);
  const watchMode = args.includes("--watch");
  const openWhenReady = args.includes("--open");
  const port = parsePort(args);
  const initial = buildManual();
  let previewHtml = initial.html;

  console.log(
    `[生成] ${path.relative(PROJECT_ROOT, initial.outputPath)} ` +
      `(${initial.bytes.toLocaleString("zh-CN")} 字节, SHA ${initial.sourceHash.slice(0, 12).toUpperCase()})`,
  );

  if (!watchMode) {
    if (openWhenReady) {
      openBrowser(`file:///${OUTPUT_PATH.replaceAll("\\", "/")}`);
    }
    return;
  }

  const preview = await startPreviewServer(port, openWhenReady, () => previewHtml);
  let rebuildTimer = null;

  fs.watchFile(SOURCE_PATH, { interval: WATCH_INTERVAL_MS }, (current, previous) => {
    if ((current.mtimeMs === previous.mtimeMs) && (current.size === previous.size)) {
      return;
    }

    if (rebuildTimer !== null) {
      clearTimeout(rebuildTimer);
    }

    rebuildTimer = setTimeout(() => {
      rebuildTimer = null;
      try {
        const result = buildManual();
        previewHtml = result.html;
        console.log(
          `[更新] ${new Date().toLocaleTimeString("zh-CN", { hour12: false })} ` +
            `SHA ${result.sourceHash.slice(0, 12).toUpperCase()}`,
        );
        broadcastUpdate(preview.clients, {
          sha: result.sourceHash.slice(0, 12).toUpperCase(),
          generatedAt: result.generatedAt.toISOString(),
        });
      } catch (error) {
        console.error(`[失败] ${error.stack || error.message}`);
      }
    }, REBUILD_DEBOUNCE_MS);
  });

  const shutdown = () => {
    fs.unwatchFile(SOURCE_PATH);
    for (const response of preview.clients) {
      response.end();
    }
    preview.server.close(() => process.exit(0));
  };

  process.on("SIGINT", shutdown);
  process.on("SIGTERM", shutdown);
}

main().catch((error) => {
  console.error(`[失败] ${error.stack || error.message}`);
  process.exitCode = 1;
});
