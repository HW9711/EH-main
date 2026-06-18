# REASONIX.md -- F413EXOsSSCH RTOS V1.5

## Stack
- C (STM32F413VGT6, Cortex-M4), Python 3 for static checks
- FreeRTOS v10.3.1 (CMSIS_V2) + STM32F4xx HAL (CubeMX)
- Keil MDK-ARM v5 (ARM Compiler 5) via EIDE VSCode extension

## Layout
- Src/, Inc/ -- CubeMX-generated HAL init. Edit only between USER CODE BEGIN/END markers.
- Drivers/ -- CMSIS + STM32F4xx HAL (vendor, read-only)
- Middlewares/ -- FreeRTOS kernel
- User/Application/ -- business logic (Handle, Motor, Pump, FootPedal, DriveCtrl, ExternalComm, etc.)
- User/board/board.h -- single-source hardware pinout/macro config
- User/Peripheral/ -- low-level drivers (ADC, EEPROM, flash, delay, LCD, UART BSP)
- User/UI/ -- screen UI logic
- MDK-ARM/ -- Keil uVision project + build outputs (.o/.d in MainCtrlF413MXOs/ subdir)
- EIDE/ -- EIDE build config (eide.yml, builder.params, .clang-format)
- tools/ -- Python static checks: check_handle_*.py, check_pump_*.py, check_ssc_merge_policy.py
- docs/ -- architecture docs; MainCtrlF413MXOs.ioc -- CubeMX project file

## Commands
- Build: EIDE UI (VSCode) or Keil uVision -- no CLI build
- Static checks: python tools/check_handle_channel_logic.py
- Format: clang-format at EIDE/.clang-format (Microsoft-based, 4-space indent, no tabs)

## Conventions
- CubeMX USER CODE markers in Src//Inc/ demarcate custom-code zones; .ioc re-gen overwrites outside them
- Chinese comments required in business code (User/Application/, User/UI/, User/Peripheral/, User/Hardware/)
- All pinouts/macros in User/board/board.h; modules reference those, not raw GPIOs
- snake_case files, CamelCase types, UPPER_CASE macros; task handles typed kernel_task_t
- Chinese commit messages, no Conventional Commits

## Watch out for
- Two builder.params: build/ and EIDE/build/ -- EIDE uses the latter. Sync both + eide.yml + .uvprojx when adding .c files.
- Preserve file encoding when editing -- no batch rewrites via shell pipes
- MDK-ARM/MainCtrlF413MXOs/ is build output -- don't edit .o/.d files
