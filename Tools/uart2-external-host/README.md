# 外部通信调试上位机

这是一个静态网页工具，用来调试 F413 ExternalComm 外部通信协议。页面通过 Web Serial 连接串口，支持命令下发、心跳可视化、日志查看导出、手动协议解析和 JSON 配置导入。

## 启动

最省事的方式：直接双击工具目录里的 `启动上位机.vbs`。它会自动启动本地服务，并打开 `http://localhost:4173`。

如果需要手动启动，也可以运行：

```powershell
cd D:\EH_main\soft\F413EXOsSSCH_RTOSV1.5-EH_mainReconstructV1\F413EXOsSSCH_RTOSV1.5-EH_mainReconstruct\Tools\uart2-external-host
npm run serve
```

然后用 Chrome 或 Edge 打开：

```text
http://localhost:4173
```

Web Serial 需要浏览器安全上下文，建议固定通过 `localhost:4173` 打开，不要直接双击 HTML 文件。

## 日志查看

通信日志默认使用“滚动模式”，最新 TX/RX 帧固定显示在顶部，方便持续观察应答和心跳。

如果需要回看历史帧，切换到“手动模式”；新日志会继续进入缓存，但页面不会抢走当前滚动位置，并会显示待查看的新日志数量。点击“显示新日志”可跳回最新位置。

## 默认串口

- 波特率：`115200`
- 数据位：`8`
- 停止位：`1`
- 校验：`none`
- 流控：`none`

## 配置导入

配置文件是 JSON。可覆盖授权码、默认速度/频率/泵速、安全确认和 EEPROM 模板。

```json
{
  "authCode": "11 22 33 44 55 66 77 88",
  "presets": {
    "speed": 3000,
    "freq": 20,
    "pumpA": 100,
    "pumpB": 100
  },
  "eepromTemplates": [
    {
      "name": "业务页模板",
      "areaCode": "01",
      "bytes": "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
    }
  ]
}
```

EEPROM 模板导入后只会填入页面表单，不会自动写入设备；仍需手动点击“写业务页”或“写导航页”。
