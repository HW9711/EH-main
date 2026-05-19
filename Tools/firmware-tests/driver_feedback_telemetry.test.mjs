import { readFileSync } from "node:fs";
import { join } from "node:path";

const repoRoot = process.cwd();

function readSource(relativePath) {
  return readFileSync(join(repoRoot, relativePath), "utf8");
}

function assert(condition, message) {
  if (!condition) {
    throw new Error(message);
  }
}

const pubinterfaceHeader = readSource("User/Application/include/Pubinterface.h");
const motorUartSource = readSource("User/Application/MotorUartData/motoruartdata.c");
const externalCommSource = readSource("User/Application/ExternalComm/external_comm_task.c");
const driveSource = readSource("User/Application/Beep/sscDrive.c");

assert(
  pubinterfaceHeader.includes("driver_current_x100"),
  "WorkMessage_t 应新增 driver_current_x100，单独保存驱动板反馈的实时电流，不能复用 current_work。"
);

assert(
  pubinterfaceHeader.includes("driver_speed_feedback"),
  "WorkMessage_t 应新增 driver_speed_feedback，单独保存驱动板反馈的实际转速，避免覆盖控制目标速度。"
);

assert(
  /driver_current_x100\s*=\s*\(uint16_t\)\(\(\(uint16_t\)dat1\[8\]\s*<<\s*8U\)\s*\|\s*dat1\[9\]\)/.test(motorUartSource),
  "motoruartdata.c 应从驱动板 0xAA 回包 byte8~9 解码实时电流 x100。"
);

assert(
  /driver_speed_feedback\s*=\s*\(uint16_t\)\(\(\(uint16_t\)dat1\[4\]\s*<<\s*8U\)\s*\|\s*dat1\[5\]\)/.test(motorUartSource),
  "motoruartdata.c 应从驱动板 0xAA 回包 byte4~5 解码反馈转速。"
);

assert(
  /ExternalComm_HeartbeatAppendBE16\(heartbeat_info,\s*&heartbeat_len,\s*WorkMessage\.driver_current_x100\)/.test(externalCommSource),
  "external_comm_task.c 心跳中的工作电流应上传驱动反馈电流，而不是保护电流阈值。"
);

assert(
  /msg\.pro_current_h\s*=\s*WorkMessage\.current_work\s*\/\s*256/.test(driveSource) &&
    /msg\.pro_current_l\s*=\s*WorkMessage\.current_work\s*%\s*256/.test(driveSource),
  "sscDrive.c 仍应使用 WorkMessage.current_work 下发保护电流阈值，不能被实时反馈电流替代。"
);

console.log("driver_feedback_telemetry.test.mjs passed");
