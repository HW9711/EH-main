@echo off
chcp 65001 >nul
setlocal
title EH 软件说明手册自动更新

rem 以本批处理所在的 docs 目录为基准返回工程根目录，避免依赖当前工作目录。
set "PROJECT_ROOT=%~dp0.."
cd /d "%PROJECT_ROOT%"

rem 自动更新工具依赖 Node.js；缺失时给出明确提示，不静默退出。
where node >nul 2>nul
if errorlevel 1 (
    echo 未找到 Node.js，无法生成并打开软件说明手册 HTML。
    echo 请安装 Node.js 后重新双击本文件。
    pause
    exit /b 1
)

echo 正在生成软件说明手册 HTML，并启动自动更新服务……
echo 当前电脑有文件加密软件，请使用浏览器自动打开的 localhost 地址阅读。
echo 请勿直接双击 docs\软件说明手册.html，否则浏览器可能显示加密乱码。
echo 本窗口保持打开时，保存 docs\软件说明手册.md 后浏览器会自动刷新。
echo 需要停止自动更新时，请在本窗口按 Ctrl+C。
echo.

node "tools\software_manual\build_manual.js" --watch --open

if errorlevel 1 (
    echo.
    echo 软件说明手册启动失败，请查看上方错误信息。
    pause
)

endlocal
