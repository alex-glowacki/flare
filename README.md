# FLARE
**Flight Lab for Avionics Research & Engineering**

A custom 450mm quadcopter with a DIY STM32H7-based flight controller and
ESP-NOW RC link. Built from scratch as a hands-on avionics and embedded
systems learning project.

---

## Project Status

| Phase | Goal                                              | Status |
|-------|---------------------------------------------------|--------|
| 1     | STM32H7 bring-up — blink, UART, toolchain         | ✅ Done |
| 2     | IMU bring-up — BMI323, sensor fusion              | ✅ Done |
| 2.5   | Magnetometer — QMC5883L, yaw fusion               | ✅ Done |
| 3     | PID loop — stabilization, DSHOT motor output      | ✅ Done |
| 4     | RC link — ESP-NOW, channel parsing, arming        | 🔄 In progress |
| 5     | Remote firmware — sticks, OLED, switches          | 🔲      |
| 6     | Flight testing & tuning                           | 🔲      |
| 7     | *(Future)* LiDAR mapping — RPLiDAR + Pi companion | 🔲      |

---

## Hardware

### Frame
- Custom PETG-CF printed, F450-class (450mm), self-modeled

### Flight Controller — FK723M1-ZGT6
- MCU: STM32H723ZGT6 (Cortex-M7, 192MHz)
- SPI1: PA5=SCK, PA6=MISO, PA7=MOSI
- I2C1: PB6=SCL, PB7=SDA
- CS (IMU): PB0
- INT1 (IMU): PB1
- USART1: PA9=TX, PA10=RX
- TIM4 DSHOT: PD12=M1, PD13=M2, PD14=M3, PD15=M4
- LED: PG7 (LED_USER)

### IMU — BMI323 (Arvian breakout, B0FG2ZFHNM)

| Wire color | Signal | Pin |
|------------|--------|-----|
| Orange     | CSB    | PB0 |
| Brown      | SCX    | PA5 |
| Blue       | SDX    | PA7 |
| Green      | SDO    | PA6 |
| Purple     | INT1   | PB1 |
| White      | VDD    | 3V3 |
| Gray       | GND    | GND |

### Magnetometer — QMC5883L (FORIOT GY-271, B0CFLPKTP1)

| Wire color | Signal | Pin |
|------------|--------|-----|
| Yellow     | SCL    | PB6 |
| Green      | SDA    | PB7 |
| Orange     | VCC    | 3V3 |
| Brown      | GND    | GND |

- I2C address: `0x2C` (ADDR pin pulled high on module — not the typical `0x0D`)
- Calibration not yet performed — hard-iron offsets are zero

### ESCs — Readytosky 35A BLHeli_S (×4)

| Motor | Position    | Spin | Pin  |
|-------|-------------|------|------|
| M1    | Front-Left  | CCW  | PD12 |
| M2    | Front-Right | CW   | PD13 |
| M3    | Rear-Right  | CCW  | PD14 |
| M4    | Rear-Left   | CW   | PD15 |

- Protocol: DSHOT300
- Demag Compensation: High (configured via BLHeliSuite + ATmega328P Nano)
- Config backed up: `config/esc_blheli_setup.ini`

### Power Distribution
- QWinOut PDB with built-in 5V and 12V BECs

### RC Link
- Nano ESP32 modules (ESP-NOW, quad-side and remote)

### Debug / Programming
- ST-Link V2 clone (V2J37S7) — SWD, CLI only
- CP2102 USB-UART adapter — COM3, 115200 baud
  - CP2102 TX → PA10, RX → PA9, GND → GND, VCC unconnected

---

## Repository Structure

```
flare/
├── firmware/
│   ├── fc/                        # STM32H7 flight controller firmware
│   │   ├── Core/
│   │   │   ├── Inc/
│   │   │   │   ├── main.h
│   │   │   │   ├── dshot.h        # DSHOT300 driver API
│   │   │   │   ├── imu_fusion.h   # Complementary filter API
│   │   │   │   ├── mag.h          # QMC5883L magnetometer driver API
│   │   │   │   ├── pid.h          # PID controller API
│   │   │   │   └── flare.h        # Motor mixing, arming logic API
│   │   │   └── Src/
│   │   │       ├── main.c         # Boot sequence, 100Hz loop, UART output
│   │   │       ├── imu_fusion.c   # Complementary filter (roll, pitch, yaw)
│   │   │       ├── mag.c          # QMC5883L I2C driver
│   │   │       ├── dshot.c        # DSHOT300 direct DMA output
│   │   │       ├── pid.c          # PID controller
│   │   │       ├── flare.c        # Motor mixing, arming logic
│   │   │       └── stm32h7xx_it.c # IRQ handlers (incl. DSHOT TC callback)
│   │   ├── cmake/stm32cubemx/     # CubeMX-generated CMake support
│   │   ├── CMakeLists.txt
│   │   └── build/Debug/           # Build output (gitignored)
│   ├── esp32_quad/                # ESP-NOW quad-side firmware (PlatformIO)
│   └── esp32_remote/              # ESP-NOW remote firmware (PlatformIO)
├── config/
│   └── esc_blheli_setup.ini       # BLHeli_S ESC configuration backup
└── docs/
    └── session_summaries/         # Per-session progress logs
```

---

## Toolchain

| Tool                | Version |
|---------------------|---------|
| arm-none-eabi-gcc   | 15.2.1  |
| CMake               | 4.3.1   |
| Ninja               | 1.13.2  |
| STM32CubeProgrammer | v2.22.0 |
| VS Code + clangd    | —       |
| PlatformIO          | —       |

---

## Build & Flash

Working directory: `firmware/fc`

**Build:**
```powershell
cmake --build build/Debug
```

**Flash:**
```powershell
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD freq=100 reset=HWrst -w build/Debug/fc.elf -rst
```

**clangd compile commands** are exported automatically via
`CMAKE_EXPORT_COMPILE_COMMANDS=TRUE` and pointed at `build/Debug` in `.clangd`.

---

## IMU Configuration

### BMI323 register values

| Register | Value    | Meaning                                        |
|----------|----------|------------------------------------------------|
| WHO_AM_I | `0x43`   | Chip ID (low byte only — high byte is garbage) |
| ACC_CONF | `0x4028` | Continuous mode, 100Hz ODR, ±8g range          |
| GYR_CONF | `0x4048` | Continuous mode, 100Hz ODR, ±2000dps range     |

### SPI protocol (confirmed working)

**Read (4 bytes):**
```
TX: [addr | 0x80]  [0x00 dummy]  [0x00]  [0x00]
RX: [ignored]      [ignored]     [LSB]   [MSB]
```
`return (uint16_t)(rx[2] | (rx[3] << 8));`

**Write (4 bytes):**
```
TX: [addr & 0x7F]  [data_LSB]  [data_MSB]  [0x00 dummy]
```
`HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 10)`

### Known STM32H7 SPI pitfalls

- **3-byte truncation:** STM32H7 SPI silently drops the 3rd byte on all
  3-byte transactions. All transactions must be padded to 4 bytes.
- **NSSP mode:** Pulses CS between bytes even with manual CS management —
  must be disabled. Enable Master Keep IO State in CubeMX.
- **Post-reset state:** After a BMI323 soft reset, the SPI peripheral must be
  fully re-initialized via `HAL_SPI_DeInit()` + `MX_SPI1_Init()`.

### BMI323 init sequence

1. `HAL_Delay(10)` — power-on settle
2. Soft reset via 3-byte `HAL_SPI_TransmitReceive` with 10ms timeout
3. `HAL_Delay(50)` — reset settle
4. `HAL_SPI_DeInit()` + `MX_SPI1_Init()` — recover SPI peripheral
5. Two dummy reads of `WHO_AM_I` — switches BMI323 from I2C to SPI mode
6. Verify `WHO_AM_I & 0x00FF == 0x43`
7. Initialize Feature Engine (required before ACC_CONF / GYR_CONF writes)
8. Write `ACC_CONF = 0x4028`, verify readback
9. Write `GYR_CONF = 0x4048`, verify readback

**The BMI323 silently ignores ACC_CONF/GYR_CONF mode bit writes until the
Feature Engine is initialized. This is undocumented in the datasheet.**

---

## Magnetometer Configuration

### QMC5883L (GY-271) key facts

| Property       | Value                                              |
|----------------|----------------------------------------------------|
| I2C address    | `0x2C` (ADDR pulled high — not the default `0x0D`)|
| Chip ID reg    | `0x0D` returns `0x00` (non-standard, check bypassed)|
| CTRL1 value    | `0x1D` (OSR=512, ±8G, 200Hz, continuous)          |
| FBR register   | Must write `0x01` before enabling continuous mode  |

### Hard-iron calibration (not yet done)

Rotate FC slowly through 360° in yaw, record min/max raw X and Y values, then:

```c
mag_cal.offset_x = (max_x + min_x) / 2.0f;
mag_cal.offset_y = (max_y + min_y) / 2.0f;
```

Calibrate after the sensor is in its final mounted position on the frame.

---

## DSHOT300 Configuration

### Timing (@ 192MHz SYSCLK, TIM4 ARR = 639)

| Symbol     | Value  | Meaning                  |
|------------|--------|--------------------------|
| Bit period | 3.33µs | 300 kbps                 |
| BIT_1      | 480    | 75% duty cycle — logic 1 |
| BIT_0      | 240    | 37.5% duty cycle — logic 0|
| Idle       | 640    | 100% duty — idle-high    |

### Key implementation facts

- **No HAL DMA burst API** — `HAL_TIM_DMABurst_WriteStart` ignores NDTR on
  STM32H7 and runs continuously. Direct DMA register programming used instead.
- **TIM4->DCR:** DBA=13 (CCR1 offset), DBL=3 (4 transfers per burst)
- **TIM4->DMAR:** peripheral address for burst access register
- **Buffer:** `dshot_buf[17][4]` uint32_t at `0x24000000` (D1 AXI SRAM)
- **Idle-high:** PD12–PD15 driven HIGH in `gpio.c` before TIM4 AF init;
  CCR1–CCR4 set to 640 in DMA TC interrupt after each frame
- **ESC keepalive:** `DSHOT_SendThrottle(0,0,0,0)` called every loop iteration
  — BLHeli_S disarms after ~250ms without a valid frame

---

## Sensor Fusion

A complementary filter runs at 100Hz producing roll, pitch, and yaw in degrees.

### Algorithm

```
// Roll and pitch — accel reference
roll_accel  = atan2(ay_g, az_g)  × (180/π)
pitch_accel = atan2(-ax_g, az_g) × (180/π)

// Gyro integration
roll_gyro  = roll_prev  + gx_dps × dt
pitch_gyro = pitch_prev + gy_dps × dt
yaw_gyro   = yaw_prev   + gz_dps × dt

// Complementary filter — roll and pitch
roll  = α × roll_gyro  + (1−α) × roll_accel
pitch = α × pitch_gyro + (1−α) × pitch_accel

// Complementary filter — yaw (shortest-path wrap-safe blend)
delta     = mag_heading − yaw_gyro        // normalised to (−180, +180]
yaw_fused = yaw_gyro + (1−β) × delta      // nudge gyro toward mag
```

### Parameters

| Parameter | Value  | Notes                                           |
|-----------|--------|-------------------------------------------------|
| α (alpha) | 0.96   | Roll/pitch gyro weight                          |
| β (beta)  | 0.90   | Yaw gyro weight — lower than α, mag is noisier  |
| dt        | 0.01s  | Matches 10ms HAL_Delay loop                     |
| ACC scale | `8.0 / 32768.0` g/LSB  | ±8g range                    |
| GYR scale | `2000.0 / 32768.0` dps/LSB | ±2000dps range           |

---

## UART Serial Output Format

115200 baud, USART1 (CP2102 on COM3).

Boot messages:
```
[DSHOT] driver ready
[DSHOT] ESCs armed
[FLARE] boot ok
[IMU] WHO_AM_I    = 0x43 (expect 0x43)
[IMU] ACC default = 0x0028
[IMU] ACC write   = 0x4028 (expect 0x4028)
[IMU] GYR write   = 0x4048 (expect 0x4048)
[FUSION] complementary filter ready
[MAG] chip ID = 0x00
[MAG] QMC5883L ready
[FLARE] PID controller ready
[IMU] starting 100Hz loop
```

100Hz data stream:
```
A:<ax> <ay> <az>  G:<gx> <gy> <gz>  R:<roll>  P:<pitch>  Y:<yaw>
```

---

## ST-Link V2 Clone Notes

Firmware: V2J37S7. Known limitations:
- GUI connection in STM32CubeProgrammer fails — use CLI only
- ITM SWO non-functional
- VS Code F5 flashing non-functional (requires `ST-LINK_gdbserver.exe`
  from CubeIDE, not installed)
- Always pass `freq=100 reset=HWrst` flags to CLI

**Memory read workaround** (when UART unavailable): promote variables to
`volatile` globals → find address with `arm-none-eabi-nm fc.elf | grep <symbol>`
→ read via STM32CubeProgrammer CLI `--upload` at the address.

---

## CubeMX Regeneration Warning

Regenerating code in CubeMX can:
- Wipe `#include` statements placed outside `USER CODE` blocks
- Introduce brace mismatches
- Move `/* USER CODE END WHILE */` outside the loop body

Always inspect `main.c` after any CubeMX regeneration before building.

---

## Commit Convention

```
<type>(<scope>): <description>

Types:  feat, fix, test, docs, refactor, chore
Scopes: fc/imu, fc/fusion, fc/mag, fc/pid, fc/dshot, fc/main, esp32/quad, esp32/remote, docs
```

Examples:
```
feat(fc/dshot): add DSHOT300 direct DMA driver with TIM4 burst
fix(fc/dshot): set CCR1-4 idle-high on DMA transfer complete via TC interrupt
fix(fc/main): send zero throttle each loop iteration to keep ESCs armed
chore(fc/main): remove bench test, motors confirmed spinning on all 4 channels
docs: update all docs for Phase 3 completion
```