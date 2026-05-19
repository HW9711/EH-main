import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import test from "node:test";

const repoRoot = path.resolve(import.meta.dirname, "..", "..");

function readSource(...parts) {
  return readFileSync(path.join(repoRoot, ...parts), "utf8");
}

test("APP task scheduler must create one static FreeRTOS thread per soft task", () => {
  const appTaskHeader = readSource("Inc", "app_task.h");
  const appTaskSource = readSource("Src", "app_task.c");
  const kernelScheduler = readSource("User", "Kernel", "Scheduler", "kernel_scheduler.c");

  assert.match(appTaskHeader, /APP_TASK_THREAD_STACK_DEPTH/);
  assert.match(appTaskHeader, /StaticTask_t\s+threadTcb/);
  assert.match(appTaskHeader, /StackType_t\s+threadStack\s*\[\s*APP_TASK_THREAD_STACK_DEPTH\s*\]/);
  assert.match(appTaskHeader, /TaskHandle_t\s+threadHandle/);

  assert.match(appTaskSource, /xSemaphoreCreateMutexStatic/);
  assert.match(appTaskSource, /AppTaskRuntimeGate/);
  assert.match(appTaskSource, /xTaskCreateStatic/);
  assert.match(appTaskSource, /AppTaskWorker/);
  assert.doesNotMatch(appTaskSource, /xTaskCreate\s*\(\s*AppTaskScheduler/);
  assert.doesNotMatch(appTaskSource, /static\s+TaskHandle_t\s+app_task_scheduler_handle/);

  assert.match(kernelScheduler, /app_task_create_named/);
  assert.match(kernelScheduler, /app_task_start/);
  assert.match(kernelScheduler, /app_task_stop/);
});
