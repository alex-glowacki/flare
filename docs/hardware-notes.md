# FLARE — Hardware Notes

---

## STM32H723ZGT6 (FK723M1 board)

- Cortex-M7 @ 192MHz SYSCLK (HSI PLL: PLLM=4, PLLN=12, PLLP=1)
- 1MB Flash, 564KB RAM, 144-pin LQFP
- **Clone ST-Link V2 (V2J37S7):** GUI connection in CubeProgrammer fails — use
  CLI only with `freq=100 reset=HWrst` flags. ITM SWO non-functional.

### Pin Assignments

| Function      | Pin            | Notes                             |
|---------------|----------------|-----------------------------------|
| SPI1 SCK      | PA5            | BMI323                            |
| SPI1 MISO     | PA6            | BMI323                            |
| SPI1 MOSI     | PA7            | BMI323                            |
| IMU CS        | PB0            | Active-low, manual GPIO           |
| IMU INT1      | PB1            | Not yet used                      |
| I2C1 SCL      | PB6            | Magnetometer                      |
| I2C1 SDA      | PB7            | Magnetometer                      |
| USART1 TX     | PA9            | CP2102 RX (debug)                 |
| USART1 RX     | PA10           | CP2102 TX (debug)                 |
| USART2 TX     | PA2            | Unused (available for future use) |
| USART2 RX     | PA3            | ESP32 quad-side UART TX           |
| USART3 TX     | PB10           | GPS module RX (optional/unused)   |
| USART3 RX     | PB11           | GPS module TX                     |
| User LED      | PG7 (LED_USER) |                                   |
| TIM4 CH1      | PD12           | DSHOT M1 (Front-Left)             |
| TIM4 CH2      | PD13           | DSHOT M2 (Front-Right)            |
| TIM4 CH3      | PD14           | DSHOT M3 (Rear-Right)             |
| TIM4 CH4      | PD15           | DSHOT M4 (Rear-Left)              |

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

- **9600 baud**, 8N1, TX/RX mode
- PB10 = TX (AF7), PB11 = RX (AF7)
- DMA: DMA1_Stream1, USART3_RX request, Peripheral→Memory, byte width,
  **Circular mode** — continuous ring buffer receive
- NVIC interrupt enabled (for HAL DMA housekeeping)
- `GPS_Init()` starts DMA via `HAL_UART_Receive_DMA(&huart3, buf, 256)`
- `GPS_Update()` polls DMA write head via NDTR each loop iteration

> ⚠️ Do not change USART3 baud from 9600 without first sending a UBX CFG-PRT
> message to reconfigure the NEO-6M module. The module defaults to 9600 on
> every cold start.

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

**Read (4 bytes):**
TX: [addr | 0x80]  [0x00 dummy]  [0x00]  [0x00]
RX: [ignored]      [ignored]     [LSB]   [MSB]
Reconstruction: `(uint16_t)(rx[2] | (rx[3] << 8))`

For 1-byte registers (e.g. WHO_AM_I), rx[3] contains garbage — mask:
`who_am_i & 0x00FF`

**Write (4 bytes — padded):**
TX: [addr & 0x7F]  [data_LSB]  [data_MSB]  [0x00 dummy]
The BMI323 clocks in the first 3 bytes and ignores the trailing dummy.
4-byte padding is required due to the STM32H7 3-byte truncation bug above.

**Soft Reset:**
- Write `0xDEAF` (LSB-first: `0xAF, 0xDE`) to register `0x7E`
- Use a short HAL timeout (10ms) — the BMI323 resets mid-transaction
- Follow with 50ms delay, then `HAL_SPI_DeInit()` + `MX_SPI1_Init()`
- Two dummy CHIP_ID reads required to re-select SPI mode

### Initialization Sequence

1. Soft reset → 50ms delay → SPI reinit
2. Two dummy reads of CHIP_ID
3. Verify `WHO_AM_I & 0x00FF == 0x43`
4. Read ACC_CONF default (expect `0x0028` = suspend mode)
5. **Initialize Feature Engine** (required before mode bits in ACC_CONF/GYR_CONF
   will be accepted — writes are silently ignored without this step)
6. Write ACC_CONF = `0x4028` (continuous, 100Hz, ±8g)
7. Write GYR_CONF = `0x4048` (continuous, 100Hz, ±2000 dps)
8. Read back and verify both registers

### Feature Engine Initialization

```c
WriteReg(0x10, 0x0000);  /* FEATURE_IO0 — clear          */
WriteReg(0x12, 0x012C);  /* FEATURE_IO2 — startup config */
WriteReg(0x14, 0x0001);  /* FEATURE_IO_STATUS — trigger  */
WriteReg(0x40, 0x0001);  /* FEATURE_CTRL — enable engine */
/* Poll FEATURE_IO1 until lower nibble == 0x01 (ready) or 0x03 (error) */
```

### Level Calibration

A 2-second calibration routine runs at boot (200 samples × 10ms). The last
50 samples are averaged to compute `imu_roll_offset` and `imu_pitch_offset`,
which are subtracted from fusion output every loop. Typical values at rest on
a flat bench: roll ≈ +1.4°, pitch ≈ −1.9°.

---

## QMC5883P Magnetometer (FORIOT GY-271 — Amazon B0CFLPKTP1)

> ⚠️ This is the **QMC5883P**, not the common QMC5883L. They are completely
> different chips with different register maps, I2C addresses, and chip IDs.
> Do not use QMC5883L libraries or documentation.

- **Interface:** I2C at 400 kHz
- **I2C address: `0x2C`** — ADDR pin pulled HIGH on this PCB
- **Chip ID: `0x80`** at register `0x00`
- **VCC:** 3.3V

### Register Map (used registers)

| Register | Address | Notes                                          |
|----------|---------|------------------------------------------------|
| CHIP_ID  | 0x00    | Returns 0x80                                   |
| XOUT_L   | 0x01    | Burst read 6 bytes for X/Y/Z (LSB first)       |
| XOUT_H   | 0x02    |                                                |
| YOUT_L   | 0x03    |                                                |
| YOUT_H   | 0x04    |                                                |
| ZOUT_L   | 0x05    |                                                |
| ZOUT_H   | 0x06    |                                                |
| STATUS   | 0x09    | Bit 0 = DRDY (data ready), Bit 1 = OVL        |
| CTRL     | 0x0A    | Mode, ODR, RNG, OSR config                    |

> ⚠️ QMC5883P has **no FBR register** (0x0B). Do not write to it.

### CTRL Configuration (0x0A)
CTRL = 0x09
- OSR1[7:6] = 00 → Over-sample ratio 1 (low power)
- RNG[5:4]  = 00 → ±30 Gauss full scale
- ODR[3:2]  = 10 → 100 Hz output data rate
- MODE[1:0] = 01 → Normal mode (continuous measurement)

### Initialization Sequence

1. Read CHIP_ID at `0x00` — validate against `0x80`; abort on mismatch
2. Write CTRL (`0x0A`) = `0x09` — enable normal mode at 100 Hz

### Data Format

- 6 bytes burst read from `0x01`: XL, XH, YL, YH, ZL, ZH
- Each axis: 16-bit signed, little-endian (LSB first), 2's complement
- Reconstruction: `(int16_t)(buf[0] | (buf[1] << 8))` for X, etc.

### Wiring (GY-271 → FK723M1)

| Wire Color | GY-271 Pin | FK723M1 Pin |
|------------|------------|-------------|
| Yellow     | SCL        | PB6         |
| Green      | SDA        | PB7         |
| Orange     | VCC        | 3V3         |
| Brown      | GND        | GND         |

DRDY pin not connected — polling used instead.

### Calibration Status

- Hard-iron calibration **not yet performed**
- `mag_cal.offset_x` and `mag_cal.offset_y` are currently `0.0f`
- Heading output will have hard-iron error until a rotation sweep is done
- To calibrate: rotate the FC slowly through 360° in yaw, record min/max
  for X and Y, then set `offset_x = (max_x + min_x) / 2`,
  `offset_y = (max_y + min_y) / 2`

---

## GY-NEO6MV2 GPS Module (u-blox NEO-6M)

- **Interface:** UART at 9600 baud (default, no configuration required)
- **Protocol:** NMEA 0183 — `$GPRMC` and `$GPGGA` sentences parsed
- **VCC:** 5V (onboard 3.3V LDO regulator) — powered from FC 5V header pin
  during bench testing; switch to PDB 5V BEC when flying on battery
- **Logic levels:** 3.3V — directly compatible with STM32H7, no level shifter
- **Antenna:** Passive ceramic patch — requires unobstructed sky view
- **Cold start:** 30–120 seconds to first fix outdoors; will not fix indoors

### Wiring (NEO-6M → FK723M1)

| GPS Pin | FK723M1 Pin | Notes                        |
|---------|-------------|------------------------------|
| VCC     | 5V header   | FC USB/PDB 5V rail           |
| GND     | GND         | Common ground                |
| TX      | PB11        | GPS transmits → STM32 RX     |
| RX      | unconnected | Not needed for receive-only  |

### Output Fields (SAE units)

| Field        | Unit           | Source sentence |
|--------------|----------------|-----------------|
| latitude     | decimal degrees | GPRMC / GNRMC  |
| longitude    | decimal degrees | GPRMC / GNRMC  |
| altitude_ft  | feet MSL        | GPGGA / GNGGA  |
| speed_mph    | miles per hour  | GPRMC / GNRMC  |
| course_deg   | degrees true    | GPRMC / GNRMC  |
| satellites   | count           | GPGGA / GNGGA  |
| fix_valid    | 0/1             | GPRMC status   |

### Mounting Notes

- Mount on top of quad with unobstructed 180° sky view
- Keep away from ESCs, high-current power wires, and carbon fiber
- Plan top-plate CAD mount alongside Ruko R111S Remote ID module

---

## Nano ESP32 (Quad-side) ✅ Flashed & Running

- **Role:** ESP-NOW receiver → UART bridge to STM32
- **MAC address: `20:6E:F1:32:70:3C`** — required for remote peer config
- **UART wiring:**

| ESP32 Pin        | Direction | STM32 Pin       |
|------------------|-----------|-----------------|
| GPIO17 / A0 (TX) | →         | PA3 (USART2 RX) |
| GND              | shared    | GND             |

- GPIO18 (RX) wired to PA2 (USART2 TX) but unused in Phase 4
- Boot output confirms MAC address, ESP-NOW ready, 5s diagnostic loop

### Windows DFU Flash Note

First flash on a new Windows machine requires installing the Nano ESP32 board
package through Arduino IDE once — this provisions the WinUSB DFU driver that
PlatformIO's `dfu-util` needs. After that initial install, PlatformIO uploads
work standalone without Arduino IDE.

To enter DFU mode: double-tap the RST button. The RGB LED changes to indicate
DFU mode. Then immediately run the PlatformIO upload command.

---

## Nano ESP32 (Remote-side) 🟡 Firmware Built — Pending Wiring

- **Role:** Read M7 hall sensor gimbals + switches → transmit FLARE_RC_Packet_t to quad ESP32 via ESP-NOW at 50Hz
- **Target peer MAC:** `20:6E:F1:32:70:3C` (hardcoded in firmware)
- **Pin assignments:** TBD — update `firmware/esp32_remote/src/main.cpp` once wired

### Planned Hardware

| Component   | Part                                   | Notes                              |
|-------------|----------------------------------------|------------------------------------|
| Gimbals     | FrSky M7 Hall Sensor (×2)             | Analog output, 4 axes, 0–3.3V     |
| OLED        | HiLetgo 2.42" SSD1309 128×64 SPI     | Arriving — use SPI 7-pin variant   |
| Arm switch  | 2-position ON-ON toggle               | INPUT_PULLUP, LOW = armed          |
| Mode switch | 2-position ON-ON toggle               | INPUT_PULLUP, LOW = acro           |

### ADC Calibration (TBD after wiring)

```cpp
#define ADC_MIN      100    // measure at stick minimum
#define ADC_MAX      4000   // measure at stick maximum
#define ADC_DEADBAND 40     // raw counts either side of center
```

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

---

## General Notes

- **Soft-mount the FC stack.** Motor vibration aliasing into the IMU is the
  primary cause of PID instability in DIY flight controllers. Use silicone
  grommets or O-ring standoffs — never hard-mount.
- **3.3V logic throughout.** BMI323, QMC5883P, NEO-6M TX, ESP32 UART, and
  STM32H7 GPIO are all 3.3V compatible. Do not expose any signal line to 5V.
- **GPS power:** Power GPS from 5V (FC header or PDB BEC), not the FC 3.3V
  rail. GPS draws ~50mA acquisition / ~20mA tracking — enough to brown out the
  STM32 logic rail under combined load.
- **Magnetometer placement:** Mount the GY-271 away from motors, ESCs, and
  power wires. Calibrate after final mounting position is fixed.
- **ESC keepalive:** Bluejay disarms if no valid DSHOT frame is received for
  ~250ms. The main loop sends DSHOT 0 every iteration as a keepalive.
- **STM32 power during ESC work:** Power the STM32 via PDB 5V BEC (not USB-C)
  whenever ESCs are powered — both must share a common ground reference.
- **Do not flash STM32 via ST-Link while ESP32 shares USB power rail.** ST-Link
  reset can glitch the 3.3V rail and corrupt ESP32 flash mid-write.
- **Loose connections:** Dupont/header pin connections can work loose between
  sessions. If previously working hardware stops working, reseat all connectors
  before troubleshooting firmware.