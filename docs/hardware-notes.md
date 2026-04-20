# FLARE — Hardware Notes

---

## STM32H723ZGT6 (FK723M1 board)

- Cortex-M7 @ 96 MHz SYSCLK (HSI PLL: PLLM=4, PLLN=12, PLLP=1)
- 1MB Flash, 564KB RAM, 144-pin LQFP
- **Clone ST-Link V2 (V2J37S7):** GUI connection in CubeProgrammer fails — use
  CLI only with `freq=100 reset=HWrst` flags. ITM SWO non-functional.

### Pin Assignments

| Function     | Pin            | Notes                        |
|--------------|----------------|------------------------------|
| SPI1 SCK     | PA5            |                              |
| SPI1 MISO    | PA6            |                              |
| SPI1 MOSI    | PA7            |                              |
| IMU CS       | PB0            | Active-low, manual GPIO      |
| IMU INT1     | PB1            | Not yet used                 |
| I2C1 SCL     | PB6            | Magnetometer                 |
| I2C1 SDA     | PB7            | Magnetometer                 |
| USART1 TX    | PA9            | CP2102 RX                    |
| USART1 RX    | PA10           | CP2102 TX                    |
| User LED     | PG7 (LED_USER) |                              |
| TIM4 CH1     | PD12           | DSHOT M1 (Front-Left)        |
| TIM4 CH2     | PD13           | DSHOT M2 (Front-Right)       |
| TIM4 CH3     | PD14           | DSHOT M3 (Rear-Right)        |
| TIM4 CH4     | PD15           | DSHOT M4 (Rear-Left)         |

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
```
TX: [addr | 0x80]  [0x00 dummy]  [0x00]  [0x00]
RX: [ignored]      [ignored]     [LSB]   [MSB]
```
Reconstruction: `(uint16_t)(rx[2] | (rx[3] << 8))`

For 1-byte registers (e.g. WHO_AM_I), rx[3] contains garbage — mask:
`who_am_i & 0x00FF`

**Write (4 bytes — padded):**
```
TX: [addr & 0x7F]  [data_LSB]  [data_MSB]  [0x00 dummy]
```
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
WriteReg(0x40, 0x0001);  /* FEATURE_CTRL — enable        */
/* Poll 0x11 (FEATURE_IO1) bit[3:0] == 0x01 for ready    */
```

### Register Map (used registers)

| Register       | Address | Notes                              |
|----------------|---------|------------------------------------|
| CHIP_ID        | 0x00    | WHO_AM_I — low byte = 0x43         |
| ACC_DATA_X     | 0x03    | Burst read 6 bytes for X/Y/Z       |
| GYR_DATA_X     | 0x06    | Burst read 6 bytes for X/Y/Z       |
| ACC_CONF       | 0x20    | Default 0x0028, target 0x4028      |
| GYR_CONF       | 0x21    | Default 0x0028, target 0x4048      |
| FEATURE_IO0    | 0x10    |                                    |
| FEATURE_IO1    | 0x11    | Poll for feature engine ready      |
| FEATURE_IO2    | 0x12    |                                    |
| FEATURE_IO_ST  | 0x14    |                                    |
| FEATURE_CTRL   | 0x40    |                                    |
| CMD            | 0x7E    | Soft reset: write 0xDEAF           |

### Post-write Timing

Per BMI323 datasheet: minimum 2 µs idle between transactions.
At 96 MHz: 200 NOPs ≈ 2.08 µs.

---

## QMC5883L Magnetometer (FORIOT GY-271 — Amazon B0CFLPKTP1)

- **Interface:** I2C (400 kHz Fast Mode)
- **VCC:** 3.3V — do NOT connect to 5V
- **Chip marking:** `HP 5883 5803` — QMC5883L-compatible register map
- **Module has onboard pull-up resistors** — no external resistors needed

### I2C Address

- **Expected (QMC5883L default):** `0x0D`
- **Actual on this module:** `0x2C` — ADDR pin is pulled high on the PCB
- **HAL 8-bit shifted address:** `0x2C << 1 = 0x58`

### Chip ID

- **QMC5883L spec:** register `0x0D` returns `0xFF`
- **This module:** returns `0x00` — ID check bypassed in driver, I2C comms
  confirmed working via bus scan

### Wiring (GY-271 → FK723M1)

| Wire Color | GY-271 Pin | FK723M1 Pin |
|------------|------------|-------------|
| Yellow     | SCL        | PB6         |
| Green      | SDA        | PB7         |
| Orange     | VCC        | 3V3         |
| Brown      | GND        | GND         |

DRDY pin not connected — polling used instead.

### Register Map (used registers)

| Register | Address | Notes                                      |
|----------|---------|--------------------------------------------|
| XOUT_L   | 0x00    | Burst read 6 bytes for X/Y/Z (LSB first)  |
| STATUS   | 0x06    | Bit 0 = DRDY (data ready)                 |
| CTRL1    | 0x09    | Mode, ODR, RNG, OSR config                |
| CTRL2    | 0x0A    | Soft reset, pointer roll-over             |
| FBR      | 0x0B    | SET/RESET period — must write 0x01 first  |
| CHIP_ID  | 0x0D    | Returns 0x00 on this module (non-standard)|

### CTRL1 Configuration

```
CTRL1 = 0x1D
  OSR[7:6] = 00  → Over-sample ratio 512 (lowest noise)
  RNG[5:4] = 01  → ±8 Gauss full scale
  ODR[3:2] = 11  → 200 Hz output data rate
  MODE[1:0]= 01  → Continuous measurement mode
```

### Initialization Sequence

1. Read CHIP_ID at `0x0D` — verify I2C comms (value ignored)
2. Write FBR (`0x0B`) = `0x01` — required by datasheet before continuous mode
3. Write CTRL1 (`0x09`) = `0x1D` — enable continuous measurement

### Data Format

- 6 bytes burst read from `0x00`: XL, XH, YL, YH, ZL, ZH
- Each axis: 16-bit signed, little-endian (LSB first), 2's complement
- Reconstruction: `(int16_t)(buf[0] | (buf[1] << 8))` for X, etc.

### Calibration Status

- Hard-iron calibration **not yet performed**
- `mag_cal.offset_x` and `mag_cal.offset_y` are currently `0.0f`
- Heading output will have hard-iron error until a rotation sweep is done
- To calibrate: rotate the FC slowly through 360° in yaw, record min/max
  for X and Y, then set `offset_x = (max_x + min_x) / 2`,
  `offset_y = (max_y + min_y) / 2`

---

## ESCs — Readytosky 35A BLHeli_S

- Signal wire: White
- Ground wire: Black
- Motor mapping (X-frame convention):

| Motor | Position    | Spin | DSHOT Pin |
|-------|-------------|------|-----------|
| M1    | Front-Left  | CCW  | PD12      |
| M2    | Front-Right | CW   | PD13      |
| M3    | Rear-Right  | CCW  | PD14      |
| M4    | Rear-Left   | CW   | PD15      |

- **Startup power not yet configured** — awaiting ATmega328P Nano (LUIRSAY,
  USB-C) for BLHeliSuite passthrough programming
- To configure: BLHeliSuite → Make Interfaces → Arduino BLHeli Bootloader
  → 115200 baud → connect ESC signal to D2 → Read Setup →
  set Startup Power ≥ 1.0 → Write Setup → repeat all 4 ESCs

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
- **3.3V logic throughout.** BMI323, QMC5883L, ESP32 UART, and STM32H7 GPIO
  are all 3.3V compatible. Do not expose any signal line to 5V.
- **Magnetometer placement:** Mount the GY-271 away from motors, ESCs, and
  power wires — these are sources of magnetic interference that will corrupt
  heading readings. Calibrate after final mounting position is fixed.