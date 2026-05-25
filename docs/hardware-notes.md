# FLARE — Hardware Notes

---

## STM32H723ZGT6 (FK723M1 board)

- Cortex-M7 @ 192MHz SYSCLK (HSI PLL: PLLM=4, PLLN=12, PLLP=1)
- 1MB Flash, 564KB RAM, 144-pin LQFP
- **Clone ST-Link V2 (V2J37S7):** GUI connection in CubeProgrammer fails — use
  CLI only with `freq=100 reset=HWrst` flags. ITM SWO non-functional.

### Pin Assignments

| Function      | Pin            | Notes                                        |
|---------------|----------------|----------------------------------------------|
| SPI1 SCK      | PA5            | BMI323 + SD card (shared bus)                |
| SPI1 MISO     | PA6            | BMI323 + SD card (shared bus)                |
| SPI1 MOSI     | PA7            | BMI323 + SD card (shared bus)                |
| IMU CS        | PB0            | Active-low, manual GPIO                      |
| IMU INT1      | PB1            | Not yet used                                 |
| SD CS         | PC0            | Planned — GPIO output, to be added in CubeMX |
| I2C1 SCL      | PB6            | QMC5883L compass (M100-5883, deferred)        |
| I2C1 SDA      | PB7            | QMC5883L compass (M100-5883, deferred)        |
| USART1 TX     | PA9            | CP2102 RX (debug)                            |
| USART1 RX     | PA10           | CP2102 TX (debug)                            |
| USART2 TX     | PA2            | ESP32 quad-side UART RX (unused in Phase 4)  |
| USART2 RX     | PA3            | ESP32 quad-side UART TX                      |
| USART3 TX     | PB10           | GPS M100-5883 RX (config, optional)          |
| USART3 RX     | PB11           | GPS M100-5883 TX                             |
| User LED      | PG7 (LED_USER) |                                              |
| TIM4 CH1      | PD12           | DSHOT M1 (Front-Left)                        |
| TIM4 CH2      | PD13           | DSHOT M2 (Front-Right)                       |
| TIM4 CH3      | PD14           | DSHOT M3 (Rear-Right)                        |
| TIM4 CH4      | PD15           | DSHOT M4 (Rear-Left)                         |

> **Not broken out (do not use):** PA13, PA14 (SWD), PC14, PC15 (oscillator),
> PH0, PH1 (oscillator).

### CubeMX SPI1 Configuration (confirmed working)

```c
hspi1.Init.Mode                    = SPI_MODE_MASTER;
hspi1.Init.Direction               = SPI_DIRECTION_2LINES;
hspi1.Init.DataSize                = SPI_DATASIZE_8BIT;
hspi1.Init.CLKPolarity             = SPI_POLARITY_LOW;
hspi1.Init.CLKPhase                = SPI_PHASE_1EDGE;
hspi1.Init.NSS                     = SPI_NSS_SOFT;
hspi1.Init.BaudRatePrescaler       = SPI_BAUDRATEPRESCALER_16;  /* 6 MHz */
hspi1.Init.FirstBit                = SPI_FIRSTBIT_MSB;
hspi1.Init.NSSPMode                = SPI_NSS_PULSE_DISABLE;
hspi1.Init.FifoThreshold           = SPI_FIFO_THRESHOLD_01DATA;
hspi1.Init.MasterKeepIOState       = SPI_MASTER_KEEP_IO_STATE_ENABLE;
hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
```

### CubeMX I2C1 Configuration (confirmed working)

- Speed Mode: Fast Mode
- Clock Speed: 400000 Hz (400 kHz)
- Pins: PB6 = SCL (AF4), PB7 = SDA (AF4)

### CubeMX USART1 Configuration (debug)

- 115200 baud, 8N1, TX/RX mode
- PA9 = TX (AF7), PA10 = RX (AF7)
- No interrupt — polling TX via `HAL_UART_Transmit(..., HAL_MAX_DELAY)`

### CubeMX USART2 Configuration (ESP32 RC link)

- 115200 baud, 8N1, TX/RX mode
- PA2 = TX (AF7), PA3 = RX (AF7)
- **NVIC interrupt enabled**, priority **8, 0**
- Direct register RX: `SET_BIT(USART2->CR1, USART_CR1_RXNEIE_RXFNEIE)` armed
  from `RC_Init()` — bypasses `HAL_UART_Receive_IT` which hangs on STM32H7
- `USART2_IRQHandler` in `stm32h7xx_it.c` reads `USART2->RDR` directly and
  calls `RC_UART_RxCpltCallback()`

### CubeMX USART3 Configuration (GPS)

- **115200 baud**, 8N1, TX/RX mode
- PB10 = TX (AF7), PB11 = RX (AF7)
- DMA: DMA1_Stream1, USART3_RX request, Peripheral→Memory, byte width,
  **Circular mode** — continuous ring buffer receive
- NVIC interrupt enabled (for HAL DMA housekeeping)
- `GPS_Init()` starts DMA via `HAL_UART_Receive_DMA(&huart3, buf, 512)`
- `GPS_Update()` polls DMA write head via NDTR each loop iteration

### CubeMX TIM4 Configuration (DSHOT300)

- Prescaler: 0 (no division)
- Counter Period (ARR): 639
- Clock Division: DIV1
- Auto-Reload Preload: Enabled
- PWM Mode 1 on CH1–CH4 (PD12–PD15, AF2)
- DMA: DMA1_Stream0, TIM4_UP request, Memory→Peripheral, word width, Normal mode
- `DSHOT_CCR_IDLE = 0` — outputs held idle-LOW between frames (Bluejay requirement)

### STM32H7 SPI Gotchas

- **NSSP Mode** must be disabled and **MasterKeepIOState** must be enabled —
  otherwise the peripheral pulses CS between bytes even with manual CS control.
- **3-byte transactions are unreliable.** The STM32H7 SPI peripheral drops the
  3rd byte on 3-byte transfers regardless of approach. Always use 4-byte
  transactions padded with a dummy byte.
- **RX FIFO does not drain on CS de-assert.** Call `SPI1_FlushRxFifo()`
  (disable + re-enable peripheral) between transactions to prevent stale data.
- **Soft reset mid-transaction** leaves the peripheral in an undefined state.
  Follow with `HAL_SPI_DeInit()` + `MX_SPI1_Init()` to recover cleanly.

### STM32H7 DMA / DSHOT Gotchas

- **`HAL_TIM_DMABurst_WriteStart` ignores NDTR** on STM32H7 HAL and runs
  continuously regardless of Normal mode setting. Do not use it for DSHOT.
  Use direct DMA register programming instead.
- **DSHOT idle-LOW required for Bluejay.** `DSHOT_CCR_IDLE` must be 0, not
  ARR+1. Bluejay's stricter state machine rejects all frames when idle is HIGH.
  BLHeli_S tolerates idle-HIGH but pulses/gallops; Bluejay produces silence.
- **CCR must be forced idle-LOW after each frame.** Set `CCR1–CCR4 = 0` in the
  DMA transfer-complete ISR to restore idle-LOW between frames.
- **Do not force-abort DMA mid-transfer.** `DSHOT_StartDMA()` must wait for the
  previous stream to be disabled (EN bit cleared by TC handler) before
  re-arming.
- **IRQ priority:** DMA1_Stream0 must be at higher priority than USART2.
  Use 0,0 for DMA1_Stream0 and 8,0 for USART2.

### STM32H7 UART Gotcha — HAL_UART_Receive_IT hangs

`HAL_UART_Receive_IT` hangs on STM32H7 HAL when called after peripheral init
in certain FIFO/lock states. Workaround for USART2 RC receiver:

1. Unlock handle: `__HAL_UNLOCK(&huart2)`
2. Clear error flags: `__HAL_UART_CLEAR_FLAG(...)`
3. Force state: `huart2.gState = HAL_UART_STATE_READY`
4. Arm RX directly: `SET_BIT(USART2->CR1, USART_CR1_RXNEIE_RXFNEIE)`
5. Handle in `USART2_IRQHandler` by reading `USART2->RDR` directly

### CubeMX Regeneration Gotchas

- **MPU_Config() gets wiped.** CubeMX regeneration replaces the custom
  MPU config with a default 4GB no-access region. The AXI SRAM non-cacheable
  region (Region 0, `0x24000000`, 512KB, TEX=1 C=0 B=0) required for DMA
  buffers must be manually restored after every regen.
- **USER CODE blocks:** All user code must be inside `/* USER CODE BEGIN */` /
  `/* USER CODE END */` markers. Includes, defines, and callbacks placed outside
  are silently wiped on regen.
- **`while(1)` closing brace** can be lost on regen — always verify `main.c`
  compiles cleanly after regeneration.

### Linker Script

File: `STM32H723XG_FLASH.ld`

A custom `.AXI_SRAM` output section was added to place DMA buffers in the
non-cacheable RAM_D1 region:

```ld
.AXI_SRAM (NOLOAD) :
{
  . = ALIGN(4);
  *(.AXI_SRAM)
  *(.AXI_SRAM*)
  . = ALIGN(4);
} >RAM_D1
```

Any buffer shared between CPU and DMA should use:
```c
__attribute__((section(".AXI_SRAM")))
```

---

## BMI323 IMU (Arvian Breakout Board — Amazon B0FG2ZFHNM)

- **Interface:** SPI (Mode 0, MSB first, up to 10 MHz — using 6 MHz)
- **VCC:** 3.3V — do NOT connect to 5V
- **CS:** Active-low, driven manually via GPIO

### Wiring (Arvian → FK723M1)

| Wire Color | Arvian Label | FK723M1 Pin |
|------------|--------------|-------------|
| Orange     | CSB          | PB0         |
| Brown      | SCX          | PA5 (SCK)   |
| Blue       | SDX          | PA7 (MOSI)  |
| Green      | SDO          | PA6 (MISO)  |
| Purple     | INT1         | PB1         |
| White      | VDD          | 3V3         |
| Gray       | GND          | GND         |

### SPI Protocol

**Read (4 bytes total):**
```
TX: [addr | 0x80]  [0x00 dummy]  [0x00]  [0x00]
RX: [ignored]      [ignored]     [LSB]   [MSB]
```

**Write (4 bytes total):**
```
TX: [addr & 0x7F]  [data_LSB]  [data_MSB]  [0x00 dummy]
```

Use `HAL_SPI_TransmitReceive` for both reads and writes — `HAL_SPI_Transmit`
truncates multi-byte writes on STM32H7.

### Confirmed Register Values

| Register | Address | Value Written | Expected | Meaning              |
|----------|---------|---------------|----------|----------------------|
| WHO_AM_I | 0x00    | —             | 0x43     | Chip ID              |
| ACC_CONF | 0x20    | 0x4028        | 0x4028   | 100Hz, ±8g, cont.    |
| GYR_CONF | 0x21    | 0x4048        | 0x4048   | 100Hz, ±2000dps, cont.|

---

## HGLRC M100-5883 GPS + Compass Module

Replaces both the GY-NEO6MV2 (NEO-6M) GPS and the GY-271 (QMC5883P) magnetometer.

### Specs

| Spec | Value |
|---|---|
| GPS chip | u-blox M10 (10th gen) |
| Channels | 72 |
| Constellations | GPS, GLONASS, BDS, Galileo, QZSS |
| Update rate | 10 Hz |
| Baud rate | 115200 (default) |
| Protocol | UBX binary |
| Compass chip | QMC5883L |
| Compass interface | I2C, address `0x0D` |
| Size / weight | 21×21×8 mm / 7.73 g |
| Power | 3.3–5V (power from 5V BEC — some units need ≥3.7V) |

### Wiring (M100-5883 → FK723M1)

| M100-5883 Pin | FK723M1 Pin      | Notes                              |
|---------------|------------------|------------------------------------|
| 5V            | PDB BEC 5V rail  | Never from STM32 3.3V pin          |
| GND           | Common GND       |                                    |
| TX            | PB11 (USART3 RX) | GPS data → STM32                   |
| RX            | PB10 (USART3 TX) | Optional config commands           |
| SDA           | PB7 (I2C1 SDA)   | QMC5883L — leave unsoldered for now |
| SCL           | PB6 (I2C1 SCL)   | QMC5883L — leave unsoldered for now |

### UBX NAV-PVT Fields Used

| Offset | Field    | Unit       | Converted to       |
|--------|----------|------------|--------------------|
| 20     | fixType  | —          | fix_type (0–5)     |
| 21     | flags    | —          | fix_valid (bit 0)  |
| 23     | numSV    | count      | satellites         |
| 24     | lon      | deg×1e-7   | longitude (float)  |
| 28     | lat      | deg×1e-7   | latitude (float)   |
| 36     | hMSL     | mm         | altitude_ft        |
| 60     | gSpeed   | mm/s       | speed_ms, speed_mph|
| 64     | headMot  | deg×1e-5   | course_deg         |

### GPS Driver Notes

- DMA buffer: 512 bytes in `.AXI_SRAM` (non-cacheable)
- Parser: UBX state machine in `gps.c`, no NMEA parsing
- Requires valid 3D fix (`fixType >= 3` AND `flags & 0x01`) before populating data
- Cold start: 30–120 seconds outdoors; will not fix indoors

### QMC5883L Compass (Deferred)

- I2C address: `0x0D`
- Different chip and register map from the retired QMC5883P (GY-271)
- Driver to be written once GPS is verified working and SDA/SCL are soldered
- Will replace `mag.c` (currently QMC5883P driver, returning `mag_ok=0`)

---

## Nano ESP32 (Quad-side) ✅ Flashed & Running

- **Role:** ESP-NOW receiver → UART bridge to STM32
- **MAC address: `D4:E9:F4:E6:E9:90`** ← current MAC after reflash
  - Previous MAC was `D4:E9:F4:E8:10:74` — changed after flash corruption
    caused by ST-Link reset glitching the shared USB rail during STM32 flash
- **UART wiring:**

| ESP32 Pin        | Direction | STM32 Pin       |
|------------------|-----------|-----------------|
| GPIO17 / A0 (TX) | →         | PA3 (USART2 RX) |
| GND              | shared    | GND             |

- GPIO18 (RX) wired to PA2 (USART2 TX) but unused in Phase 4
- Boot output confirms MAC address, ESP-NOW ready, diagnostic loop

> ⚠️ **Power:** Do not feed BEC 5V into Nano VIN. The ESP32-S3 requires 3.3V.
> Power from STM32 3.3V pin or a dedicated 3.3V regulator. The Nano may appear
> powered via VIN (LED on) but ESP-NOW will not function correctly.

> ⚠️ **Flash corruption risk:** Do not flash STM32 via ST-Link while the
> ESP32 is powered from the shared USB rail. ST-Link reset can corrupt ESP32
> flash mid-write, which silently changes the reported MAC address. Always
> disconnect or power-cycle the ESP32 before ST-Link flashing.

### Windows DFU Flash Note

First flash on a new Windows machine requires installing the Nano ESP32 board
package through Arduino IDE once — this provisions the WinUSB DFU driver that
PlatformIO's `dfu-util` needs. After that initial install, PlatformIO uploads
work standalone without Arduino IDE.

To enter DFU mode: double-tap the RST button. The RGB LED changes to indicate
DFU mode. Then immediately run the PlatformIO upload command.

---

## Nano ESP32 (Remote-side) 🟡 Firmware Built — Pending Wiring

- **Role:** Read M7 hall sensor gimbals + switches → transmit FLARE_RC_Packet_t
  to quad ESP32 via ESP-NOW at 100Hz
- **Target peer MAC:** `D4:E9:F4:E6:E9:90` (hardcoded in firmware as `kQuadMac`)
- **Pin assignments:** TBD — update `firmware/esp32_remote/src/main.cpp` once wired

### Planned Hardware

| Component   | Part                               | Notes                           |
|-------------|------------------------------------|---------------------------------|
| Gimbals     | FrSky M7 Hall Sensor (×2)         | Analog output, 4 axes, 0–3.3V  |
| OLED        | HiLetgo 2.42" SSD1309 128×64 SPI | Use SPI 7-pin variant           |
| Arm switch  | 2-position ON-ON toggle            | INPUT_PULLUP, LOW = armed       |
| Mode switch | 2-position ON-ON toggle            | INPUT_PULLUP, LOW = acro        |

### ADC Calibration (TBD after wiring)

```cpp
#define ADC_MIN      100    // measure at stick minimum
#define ADC_MAX      4000   // measure at stick maximum
#define ADC_DEADBAND 40     // raw counts either side of center
```

---

## SD Card Blackbox Logger (Planned — Phase 5.5)

- **Module:** Standard SPI micro-SD breakout, 3.3V (level-shifted)
- **Bus:** Shared SPI1 with BMI323 (PA5 SCK, PA6 MISO, PA7 MOSI)
- **CS:** PC0 — dedicated GPIO output (to be added in CubeMX)
- **Logs:** UART telemetry stream to `.txt` or `.csv` on SD card

### Wiring (SD module → FK723M1)

| SD Pin | FK723M1 Pin | Notes               |
|--------|-------------|---------------------|
| VCC    | 3.3V        |                     |
| GND    | Common GND  |                     |
| SCK    | PA5         | Shared with BMI323  |
| MOSI   | PA7         | Shared with BMI323  |
| MISO   | PA6         | Shared with BMI323  |
| CS     | PC0         | Dedicated           |

### CubeMX Changes Required

1. Add `PC0` as `GPIO_Output`, label `SD_CS`
2. No changes to SPI1 — configuration unchanged
3. Regenerate and audit `main.c` closing brace as always

---

## ESCs — Readytosky 35A BLHeli_S + Bluejay Firmware

- Signal wire: White
- Ground wire: Black
- **Firmware: Bluejay** (replaces stock BLHeli_S)
- Motor mapping (X-frame convention):

| Motor | Position    | Spin | DSHOT Pin |
|-------|-------------|------|-----------|
| M1    | Front-Left  | CCW  | PD12      |
| M2    | Front-Right | CW   | PD13      |
| M3    | Rear-Right  | CCW  | PD14      |
| M4    | Rear-Left   | CW   | PD15      |

### Bluejay-Specific Notes

- **CRC is non-inverted:** `(v ^ v>>4 ^ v>>8) & 0xF` — opposite of the
  published DSHOT spec (`~(v ^ v>>4 ^ v>>8) & 0xF`). FLARE firmware uses
  non-inverted CRC to match Bluejay.
- **Idle state must be LOW.** Bluejay's stricter frame sync rejects all frames
  if the signal idles HIGH between frames. `DSHOT_CCR_IDLE = 0`.
- **Telemetry bit:** kept at 0 — bidirectional DSHOT not enabled.
- **DSHOT throttle range:** 48 (minimum) to 2047 (maximum). Values 1–47 are
  reserved special commands. 0 = disarm/stop.

### BLHeliSuite 4-Way Interface Setup

- Arduino Nano (ATmega328P) flashed with 4-way interface firmware
- **Baud: 115200** (default does not work — must select 115200 explicitly)
- LGT8F328P clones fail the 4-way-if handshake — must use genuine ATmega328P
- Nano COM port: COM11
- FC/debug COM port: COM3 (CP2102)

---

## CP2102 USB-UART Adapter

| CP2102 | FK723M1          |
|--------|------------------|
| TX     | PA10 (USART1 RX) |
| RX     | PA9  (USART1 TX) |
| GND    | GND              |
| VCC    | unconnected      |

- COM port: COM3
- Baud: 115200 8N1

> ⚠️ Do not connect an explicit GND wire from CP2102 to STM32 if it could land
> on the RESET pin — shared ground is provided via USB to PC. A GND wire on
> RESET causes hard faults.

---

## General Notes

- **Soft-mount the FC stack.** Motor vibration aliasing into the IMU is the
  primary cause of PID instability in DIY flight controllers. Use silicone
  grommets or O-ring standoffs — never hard-mount.
- **3.3V logic throughout.** BMI323, QMC5883L, GPS TX, ESP32 UART, and
  STM32H7 GPIO are all 3.3V compatible. Do not expose any signal line to 5V.
- **GPS power:** Power GPS from 5V BEC, not the FC 3.3V rail.
- **Magnetometer placement:** Mount the M100-5883 away from motors, ESCs, and
  power wires. Calibrate QMC5883L after final mounting position is fixed.
- **ESC keepalive:** Bluejay disarms if no valid DSHOT frame is received for
  ~250ms. The main loop sends DSHOT 0 every iteration as a keepalive.
- **STM32 power during ESC work:** Power the STM32 via PDB 5V BEC (not USB-C)
  whenever ESCs are powered — both must share a common ground reference.
- **Do not flash STM32 via ST-Link while ESP32 shares USB power rail.** ST-Link
  reset can glitch the 3.3V rail and corrupt ESP32 flash mid-write, silently
  changing the Nano's reported MAC address.
- **Loose connections:** Dupont/header pin connections can work loose between
  sessions. If previously working hardware stops working, reseat all connectors
  before troubleshooting firmware.