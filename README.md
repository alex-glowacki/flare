# FLARE
**Flight Lab for Avionics Research & Engineering**

A custom 450mm quadcopter with a DIY STM32H7-based flight controller and
ESP-NOW RC link. Built from scratch as a hands-on avionics and embedded
systems learning project.

---

## Project Status

| Phase | Goal | Status |
|---|---|---|
| 1 | STM32H7 bring-up — blink, UART, toolchain         | ✅ Done |
| 2 | IMU bring-up — BMI323, sensor fusion              | ✅ Done |
| 3 | PID loop — stabilization, DSHOT motor output      | 🔲      |
| 4 | RC link — ESP-NOW, channel parsing, arming        | 🔲      |
| 5 | Remote firmware — sticks, OLED, switches          | 🔲      |
| 6 | Flight testing & tuning                           | 🔲      |
| 7 | *(Future)* LiDAR mapping — RPLiDAR + Pi companion | 🔲      |

---

## Hardware

### Frame
- Custom PETG-CF printed, F450-class (450mm), self-modeled

### Flight Controller — FK723M1-ZGT6
- MCU: STM32H723ZGT6 (Cortex-M7, 550MHz)
- SPI1: PA5=SCK, PA6=MISO, PA7=MOSI
- CS (IMU): PB0
- INT1 (IMU): PB1
- USART1: PA9=TX, PA10=RX
- LED: PG7 (LED_USER)

### IMU — BMI323 (Arvian breakout, B0FG2ZFHNM)
| Wire color | Signal | Pin |
|---|---|---|
| Orange | CSB | PB0 |
| Brown | SCX | PA5 |
| Blue | SDX | PA7 |
| Green | SDO | PA6 |
| Purple | INT1 | PB1 |
| White | VDD | 3V3 |
| Gray | GND | GND |

### ESCs
- BLHeli_S (4×)

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
│   │   │   │   └── imu_fusion.h   # Complementary filter API
│   │   │   └── Src/
│   │   │       ├── main.c         # IMU init, 100Hz loop, UART output
│   │   │       └── imu_fusion.c   # Complementary filter implementation
│   │   ├── cmake/stm32cubemx/     # CubeMX-generated CMake support
│   │   ├── CMakeLists.txt
│   │   └── build/Debug/           # Build output (gitignored)
│   ├── esp32_quad/                # ESP-NOW quad-side firmware (PlatformIO)
│   └── esp32_remote/              # ESP-NOW remote firmware (PlatformIO)
└── docs/
    └── session_summaries/         # Per-session progress logs
```

---

## Toolchain

| Tool | Version |
|---|---|
| arm-none-eabi-gcc | 15.2.1 |
| CMake | 4.3.1 |
| Ninja | 1.13.2 |
| STM32CubeProgrammer | v2.22.0 |
| VS Code + clangd | — |
| PlatformIO | — |

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

| Register | Value | Meaning |
|---|---|---|
| WHO_AM_I | `0x43` | Chip ID (low byte only — high byte is garbage) |
| ACC_CONF | `0x4028` | Continuous mode, 100Hz ODR, ±8g range |
| GYR_CONF | `0x4048` | Continuous mode, 100Hz ODR, ±2000dps range |

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

- **3-byte truncation:** The STM32H7 SPI peripheral silently drops the 3rd byte
  on all 3-byte transactions. All transactions must be padded to 4 bytes.
- **NSSP mode:** Pulses CS between bytes even with manual CS management — must
  be disabled. Enable Master Keep IO State in CubeMX.
- **Post-reset state:** After a BMI323 soft reset, the SPI peripheral must be
  fully re-initialized via `HAL_SPI_DeInit()` + `MX_SPI1_Init()`.
- **Soft reset transaction:** BMI323 resets mid-transaction. Use
  `HAL_SPI_TransmitReceive` with a short (10ms) timeout for the reset command
  only — do not use direct register access.

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
Feature Engine is initialized.** This is undocumented in the datasheet.

---

## Sensor Fusion

A complementary filter runs at 100Hz producing roll and pitch angles in degrees.
Yaw is omitted — without a magnetometer, gyro-only yaw drifts and is not useful
for stabilization.

### Algorithm

```
roll_accel  = atan2(ay_g, az_g)   × (180/π)
pitch_accel = atan2(-ax_g, az_g)  × (180/π)

roll_gyro   = roll_prev  + gx_dps × dt
pitch_gyro  = pitch_prev + gy_dps × dt

roll  = α × roll_gyro  + (1−α) × roll_accel
pitch = α × pitch_gyro + (1−α) × pitch_accel
```

### Parameters

| Parameter | Value | Notes |
|---|---|---|
| α (alpha) | 0.96 | Gyro weight — starting point, tune if needed |
| dt | 0.01s | Matches 10ms HAL_Delay loop |
| ACC scale | `8.0 / 32768.0` g/LSB | ±8g range |
| GYR scale | `2000.0 / 32768.0` dps/LSB | ±2000dps range |

### Verified behavior
- At rest: stable within ±0.01°
- During aggressive motion: angles track correctly, no runaway
- After motion stops: settles cleanly to stable value

---

## UART Serial Output Format

115200 baud, USART1 (CP2102 on COM3).

Boot messages:
```
[FLARE] boot ok
[IMU] WHO_AM_I    = 0x43 (expect 0x43)
[IMU] ACC default = 0x0028
[IMU] ACC write   = 0x4028 (expect 0x4028)
[IMU] GYR write   = 0x4048 (expect 0x4048)
[FUSION] complementary filter ready
[IMU] starting 100Hz loop
```

100Hz data stream:
```
A:<ax> <ay> <az>  G:<gx> <gy> <gz>  R:<roll>  P:<pitch>
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

Always inspect `main.c` after any CubeMX regeneration before building.

---

## Commit Convention

```
<type>(<scope>): <description>

Types:  feat, fix, test, docs, refactor, chore
Scopes: fc/imu, fc/fusion, fc/pid, esp32/quad, esp32/remote, docs
```

Examples:
```
feat(fc/fusion): add complementary filter for roll/pitch estimation
fix(fc/imu): fix BMI323 SPI write — pad to 4 bytes, add feature engine init
test(fc/imu): verify BMI323 accel axis response on physical tilt
```