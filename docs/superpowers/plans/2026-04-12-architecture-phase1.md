# STM32 Layered Architecture Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land phase 1 of the four-layer architecture without changing business behavior.

**Architecture:** Add stable wrapper entry points for hardware, kernel, and application bootstrap; keep old business modules running behind wrappers. Reorganize the EIDE project into four top-level layers so later migrations can move modules incrementally without breaking CubeMX regeneration.

**Tech Stack:** STM32F413, HAL, FreeRTOS, EIDE, AC5

---

### Task 1: Add stable layer entry points

**Files:**
- Create: `User/Hardware/include/hw_bootstrap.h`
- Create: `User/Hardware/include/bsp_board.h`
- Create: `User/Hardware/Bsp/hw_bootstrap.c`
- Create: `User/Kernel/include/kernel_entry.h`
- Create: `User/Kernel/include/kernel_scheduler.h`
- Create: `User/Kernel/Scheduler/kernel_entry.c`
- Create: `User/Kernel/Scheduler/kernel_scheduler.c`
- Create: `User/App/include/app_bootstrap.h`
- Create: `User/App/Bootstrap/app_bootstrap.c`

- [ ] Define stable wrapper APIs for hardware post-init, kernel scheduler start, and app bootstrap.
- [ ] Keep wrappers behavior-preserving by delegating to existing `Userparser_Init()` and `AppTaskScheduler_Init()`.
- [ ] Use new wrappers as the only new public entry points in this phase.

### Task 2: Bridge CubeMX generated files into stable entry points

**Files:**
- Modify: `Src/main.c`
- Modify: `Src/freertos.c`
- Modify: `Src/usart.c`

- [ ] Replace direct app bootstrap call in `main.c` with `Hardware_PostInit()` plus `App_Bootstrap_Init()`.
- [ ] Replace direct scheduler call in `freertos.c` with `Kernel_Scheduler_Start()`.
- [ ] Move custom board include in `usart.c` into a USER CODE block and route it through `bsp_board.h`.

### Task 3: Rebuild EIDE project view around four layers

**Files:**
- Modify: `EIDE/.eide/eide.yml`

- [ ] Change virtual folders to `Hardware`, `Driver`, `Kernel`, `Application`.
- [ ] Add new include roots for `User/Hardware/include`, `User/Driver/include`, `User/Kernel/include`, and `User/App/include`.
- [ ] Keep old include roots during migration to avoid breaking existing modules.

### Task 4: Verify no immediate integration regressions

**Files:**
- Review: `git diff --stat`
- Review: `rg -n '#include \"app_task.h\"|#include \"board.h\"' Src User`

- [ ] Check that high-risk business files such as `handlescan.c` stay untouched.
- [ ] Check that generated files now depend on stable wrappers instead of old direct includes where phase 1 allows.
- [ ] Record next migration target as board/BSP extraction and app module include cleanup.
