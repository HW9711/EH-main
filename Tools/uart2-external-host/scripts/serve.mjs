import { createReadStream, existsSync, statSync } from "node:fs";
import { createServer } from "node:http";
import { extname, join, normalize, resolve, sep } from "node:path";

// rootDir 固定为工具目录，避免请求路径跳出静态网页资源边界。
const rootDir = resolve(import.meta.dirname, "..");
// port 允许现场临时改端口；默认 4173 便于和常见前端端口区分。
const port = Number(process.env.PORT || 4173);

// 常用静态资源类型表，保证浏览器按 ES module 和 CSS 正确解析。
const mimeTypes = new Map([
  [".html", "text/html; charset=utf-8"],
  [".css", "text/css; charset=utf-8"],
  [".js", "text/javascript; charset=utf-8"],
  [".mjs", "text/javascript; charset=utf-8"],
  [".json", "application/json; charset=utf-8"],
  [".svg", "image/svg+xml; charset=utf-8"]
]);

function resolveRequestPath(urlPath) {
  // URL 解码失败时直接返回空，避免把异常路径交给文件系统。
  let decodedPath;
  try {
    decodedPath = decodeURIComponent(urlPath.split("?")[0]);
  } catch {
    return null;
  }

  // 根路径映射到主页面，方便现场只打开 localhost:4173。
  const relativePath = decodedPath === "/" ? "index.html" : decodedPath.replace(/^\/+/, "");
  // normalize 消除 ./ 和 ../，后面继续做根目录前缀检查。
  const filePath = resolve(rootDir, normalize(relativePath));
  // 根目录前缀检查阻止 ../ 访问工程其它文件。
  if (filePath !== rootDir && !filePath.startsWith(rootDir + sep)) {
    return null;
  }
  return filePath;
}

const server = createServer((req, res) => {
  // 只支持 GET/HEAD，串口调试网页不需要写服务器状态。
  if (req.method !== "GET" && req.method !== "HEAD") {
    res.writeHead(405, { Allow: "GET, HEAD" });
    res.end("Method Not Allowed");
    return;
  }

  const filePath = resolveRequestPath(req.url || "/");
  // 路径非法或文件不存在时返回 404，避免误导浏览器缓存旧资源。
  if (!filePath || !existsSync(filePath) || !statSync(filePath).isFile()) {
    res.writeHead(404, { "Content-Type": "text/plain; charset=utf-8" });
    res.end("Not Found");
    return;
  }

  // 根据扩展名设置 Content-Type，未知类型按二进制流处理。
  const contentType = mimeTypes.get(extname(filePath).toLowerCase()) || "application/octet-stream";
  res.writeHead(200, {
    "Content-Type": contentType,
    "Cache-Control": "no-store"
  });

  // HEAD 只返回响应头，用于浏览器或测试探测服务状态。
  if (req.method === "HEAD") {
    res.end();
    return;
  }

  createReadStream(filePath).pipe(res);
});

server.listen(port, "127.0.0.1", () => {
  console.log(`ExternalComm host: http://localhost:${port}`);
});
