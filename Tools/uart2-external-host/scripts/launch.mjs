import { spawn } from "node:child_process";
import { existsSync, openSync } from "node:fs";
import { get } from "node:http";
import { tmpdir } from "node:os";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const scriptDir = dirname(fileURLToPath(import.meta.url));
const rootDir = resolve(scriptDir, "..");
const port = Number(process.env.PORT || 4173);
const url = `http://localhost:${port}`;

function requestHome() {
  // 用页面关键字确认 4173 端口确实是本上位机，而不是其它本地服务。
  return new Promise((resolveCheck) => {
    let settled = false;
    const finish = (value) => {
      if (!settled) {
        settled = true;
        resolveCheck(value);
      }
    };
    const req = get(`http://127.0.0.1:${port}/`, (res) => {
      let body = "";
      res.setEncoding("utf8");
      res.on("data", (chunk) => {
        body += chunk;
      });
      res.on("end", () => {
        // 品牌标题可能调整，启动探测使用稳定的页面结构和品牌名判断当前端口是否为本上位机。
        finish(res.statusCode === 200 && body.includes('id="workspace"') && body.includes("贵州梓锐科技"));
      });
      res.on("error", () => finish(false));
    });
    req.on("timeout", () => {
      req.destroy();
      finish(false);
    });
    req.setTimeout(900);
    req.on("error", () => finish(false));
  });
}

function startServer() {
  // Node 24 对尚未 open 完成的 WriteStream 作为 stdio 更严格，使用同步文件句柄保证双击启动稳定。
  const out = openSync(join(tmpdir(), "uart2-external-host.out.log"), "a");
  const err = openSync(join(tmpdir(), "uart2-external-host.err.log"), "a");
  const child = spawn(process.execPath, [join(rootDir, "scripts", "serve.mjs")], {
    cwd: rootDir,
    detached: true,
    stdio: ["ignore", out, err],
    windowsHide: true
  });
  child.unref();
}

async function waitForServer() {
  for (let index = 0; index < 20; index += 1) {
    if (await requestHome()) {
      return true;
    }
    await new Promise((resolveWait) => setTimeout(resolveWait, 250));
  }
  return false;
}

function browserCandidates() {
  const candidates = [];
  const programFilesX86 = process.env["ProgramFiles(x86)"];
  const programFiles = process.env.ProgramFiles;
  if (programFilesX86) {
    candidates.push(join(programFilesX86, "Microsoft", "Edge", "Application", "msedge.exe"));
  }
  if (programFiles) {
    candidates.push(join(programFiles, "Microsoft", "Edge", "Application", "msedge.exe"));
    candidates.push(join(programFiles, "Google", "Chrome", "Application", "chrome.exe"));
  }
  return candidates;
}

function openBrowser() {
  // Edge/Chrome 对 Web Serial 支持最好；找不到时退回系统默认浏览器。
  for (const candidate of browserCandidates()) {
    if (!existsSync(candidate)) {
      continue;
    }
    try {
      const child = spawn(candidate, [url], {
        detached: true,
        stdio: "ignore",
        windowsHide: true
      });
      child.unref();
      return;
    } catch {
      // 继续尝试下一个浏览器候选项。
    }
  }
  const child = spawn("cmd", ["/c", "start", "", url], {
    detached: true,
    stdio: "ignore",
    windowsHide: true
  });
  child.unref();
}

if (!(await requestHome())) {
  startServer();
  await waitForServer();
}

openBrowser();
