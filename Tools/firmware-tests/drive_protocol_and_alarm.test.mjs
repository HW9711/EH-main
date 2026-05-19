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

const driveSource = readSource("User/Application/Beep/sscDrive.c");
const motorUartSource = readSource("User/Application/MotorUartData/motoruartdata.c");

assert(
  driveSource.includes("MotorDrive_BuildCommandFrequency"),
  "sscDrive.c 应通过 MotorDrive_BuildCommandFrequency() 生成驱动板命令频率，避免主控侧额外乘 2。"
);

assert(
  !driveSource.includes("msg.frequency=WorkMessage.freq_work*2") &&
    !driveSource.includes("msg.frequency = WorkMessage.freq_work * 2"),
  "sscDrive.c 不应把 WorkMessage.freq_work 再乘 2；参考驱动接收端会自行乘 2。"
);

assert(
  driveSource.includes("WorkMessage.tool_type") &&
    !driveSource.includes("WorkMessage.hand_model!=PX_YIP_ONLINES") &&
    !driveSource.includes("WorkMessage.hand_model!=PX_YIM_ONLINES"),
  "sscDrive.c 应使用 WorkMessage.tool_type 判断有刷/无刷刀具，不能用手柄型号字段判断刀具类型。"
);

assert(
  !driveSource.includes("msg.pro_current_l=0x64") &&
    !driveSource.includes("msg.pro_current_l = 0x64"),
  "sscDrive.c 不应在 MotorStart() 中把保护电流低字节强制改成 0x64，否则会覆盖 EEPROM/上位机设置的过流阈值。"
);

assert(
  motorUartSource.includes("MotorUart_MapDriverErrorToAlarm"),
  "motoruartdata.c 应显式解码驱动板 dat1[7] 错误码，再映射为主控报警码。"
);

assert(
  !motorUartSource.includes("MotorUart_SetAlarm(8U);") &&
    !motorUartSource.includes("MotorUart_SetAlarm(9U);"),
  "motoruartdata.c 不应把任意驱动板非零错误码固定映射成 A=8/B=9。"
);

assert(
  motorUartSource.includes("MotorUart_ClearDriverAlarmIfOwned"),
  "motoruartdata.c 应在驱动板错误码恢复为 0 时，清除本模块拥有的驱动报警。"
);

console.log("drive_protocol_and_alarm.test.mjs passed");
