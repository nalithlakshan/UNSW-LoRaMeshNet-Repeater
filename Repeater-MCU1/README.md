# Repeater-MCU1

Simple end-to-end workflow from `main()`.

## Startup path

1. Reset enters startup code in `startup_stm32wle5ccux.s`.
2. Startup calls `main()` in `Core/Src/main.c`.
3. `main()` runs hardware init:
   - `HAL_Init()`
   - `SystemClock_Config()`
   - `MX_GPIO_Init()`, `MX_DMA_Init()`, `MX_USART1_UART_Init()`, `MX_RTC_Init()`
   - `MX_SubGHz_Phy_Init()`

## SubGHz init path

`MX_SubGHz_Phy_Init()` in `SubGHz_Phy/App/app_subghz_phy.c` does:

- `SystemApp_Init()` (from `Core/Src/sys_app.c`)
  - Initializes timer, trace, and low-power manager.
- `SubghzApp_Init()` (from `SubGHz_Phy/App/subghz_phy_app.c`)
  - Registers radio callbacks (`OnTxDone`, `OnRxDone`, etc.).
  - Calls `Radio.Init()`.
  - Sets RF frequency and LoRa modem parameters.
  - Registers a sequencer TX task (`CFG_SEQ_Task_LoRaTx`).
  - Starts a periodic timer (`UTIL_TIMER_Create` + `UTIL_TIMER_Start`) every 2000 ms.

## Runtime loop

Inside `while(1)` in `Core/Src/main.c`:

- `MX_SubGHz_Phy_Process()` runs continuously.
- It calls `UTIL_SEQ_Run(UTIL_SEQ_DEFAULT)` to execute scheduled tasks/events.

## Periodic transmit + sleep behavior

Timer callback `TxTimerCb()` in `SubGHz_Phy/App/subghz_phy_app.c`:

- Does **not** send directly from timer context.
- Sets a sequencer task flag (`UTIL_SEQ_SetTask`) for `CFG_SEQ_Task_LoRaTx`.

Sequencer task `TxTask()` in `SubGHz_Phy/App/subghz_phy_app.c`:

- Toggles LED1.
- Increments TX counter.
- Sends payload with `Radio.Send(TxBuf, PAYLOAD_LEN)`.

Low-power mode is enabled in `Core/Inc/sys_conf.h` (`LOW_POWER_DISABLE = 0`), so between events the app can enter low-power state from the sequencer idle path.

Radio callbacks log TX/RX status using `APP_LOG`.

## Key config files

- `Repeater-MCU1.ioc`: CubeMX peripheral/radio configuration.
- `SubGHz_Phy/App/subghz_phy_app.h`: RF frequency/modem settings.
- `Core/Inc/sys_conf.h`: debug + low-power configuration.
- `STM32WLE5CCUX_FLASH.ld`: linker memory layout.

## One-line flow summary

`Reset_Handler -> main() -> MX_SubGHz_Phy_Init() -> SystemApp_Init() + SubghzApp_Init() -> timer-driven Radio.Send() -> MX_SubGHz_Phy_Process()/UTIL_SEQ_Run()`.
