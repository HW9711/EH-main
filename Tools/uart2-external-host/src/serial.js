export class SerialConnection extends EventTarget {
  constructor() {
    super();
    // port 保存浏览器授权后的串口对象，断开时必须释放给系统。
    this.port = null;
    // reader 是可取消的读取锁，断开流程要先 cancel 再 release。
    this.reader = null;
    // writer 是串口发送锁，所有下行帧通过它写出。
    this.writer = null;
    // keepReading 控制异步读取循环，避免断开后继续派发旧数据。
    this.keepReading = false;
  }

  get supported() {
    // Web Serial 只在 Chrome/Edge 的安全上下文中存在，UI 用它决定是否禁用连接按钮。
    return typeof navigator !== "undefined" && "serial" in navigator;
  }

  get connected() {
    // writer 存在表示端口已经 open 且可发送。
    return Boolean(this.port && this.writer);
  }

  async connect(options) {
    if (!this.supported) {
      throw new Error("当前浏览器不支持 Web Serial，请使用 Chrome 或 Edge，并通过 localhost 打开页面");
    }

    // requestPort 会弹出系统 COM 口选择框，用户确认后才可访问硬件。
    this.port = await navigator.serial.requestPort();
    await this.port.open({
      baudRate: options.baudRate,
      dataBits: options.dataBits,
      stopBits: options.stopBits,
      parity: options.parity,
      flowControl: options.flowControl
    });

    // 读取器和写入器分别占用 readable/writable 锁，后续断开时要成对释放。
    this.reader = this.port.readable.getReader();
    this.writer = this.port.writable.getWriter();
    this.keepReading = true;
    this.dispatchEvent(new CustomEvent("status", { detail: { connected: true } }));
    void this.readLoop();
  }

  async disconnect() {
    // 先停止循环，避免 cancel 后 readLoop 继续把空包当数据。
    this.keepReading = false;
    if (this.reader) {
      await this.reader.cancel().catch(() => {});
      this.reader.releaseLock();
      this.reader = null;
    }
    if (this.writer) {
      this.writer.releaseLock();
      this.writer = null;
    }
    if (this.port) {
      await this.port.close().catch(() => {});
      this.port = null;
    }
    this.dispatchEvent(new CustomEvent("status", { detail: { connected: false } }));
  }

  async send(bytes) {
    if (!this.writer) {
      throw new Error("串口未连接，无法发送");
    }
    // Uint8Array 明确按二进制写入，避免 TextEncoder 改变 HEX 帧内容。
    const payload = bytes instanceof Uint8Array ? bytes : Uint8Array.from(bytes);
    await this.writer.write(payload);
    this.dispatchEvent(new CustomEvent("tx", { detail: { bytes: payload, timestamp: Date.now() } }));
  }

  async readLoop() {
    try {
      while (this.keepReading && this.reader) {
        const { value, done } = await this.reader.read();
        if (done || !this.keepReading) {
          break;
        }
        if (value && value.length > 0) {
          // RX 事件只传原始字节；拆包和协议解释交给 protocol.js，保持串口层单一职责。
          this.dispatchEvent(new CustomEvent("rx", { detail: { bytes: value, timestamp: Date.now() } }));
        }
      }
    } catch (error) {
      this.dispatchEvent(new CustomEvent("error", { detail: { error } }));
    } finally {
      if (this.keepReading) {
        await this.disconnect();
      }
    }
  }
}
