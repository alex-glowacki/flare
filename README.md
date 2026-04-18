# FLARE
**Flight Lab for Avionics Research & Engineering**

Custom 450mm quadcopter with a DIY STM32H7 flight controller and custom ESP-NOW RC transmitter.

---

## Hardware

| Component     | Part                                  |
|---------------|---------------------------------------|
| Frame         | Custom PETG-CF, F450-class (printed)  |
| Props         | 10" 1045                              |
| FC MCU        | STM32H723ZGT6 (FK723M1 board)         |
| IMU           | BMI323 6-axis (Arvian breakout, SPI)  |
| ESCs          | BLHeli_S 30A (DSHOT300)               |
| RC Link       | ESP-NOW — Nano ESP32 × 2             |
| PDB           | QWinOut XT60 BEC 5V & 12V             |
| Battery       | 3S LiPo (dev), 4S (flight)            |

---

## Repository Layout

```
firmware/
  fc/            STM32H723 flight controller (CubeMX + CMake)
  esp32_quad/    ESP32 on the quadcopter (PlatformIO)
  esp32_remote/  ESP32 in the transmitter (PlatformIO)
docs/
  architecture.md
  hardware-notes.md
```

---

## Development Phases

| Phase | Goal                                              | Status         |
|-------|---------------------------------------------------|----------------|
| 1     | STM32H7 bring-up — blink, UART, toolchain         | ✅ Done         |
| 2     | IMU bring-up — BMI323, sensor fusion              | 🟡 In progress  |
| 3     | PID loop — stabilization, DSHOT motor output      | 🔲              |
| 4     | RC link — ESP-NOW, channel parsing, arming        | 🔲              |
| 5     | Remote firmware — sticks, OLED, switches          | 🔲              |
| 6     | Flight testing & tuning                           | 🔲              |
| 7     | *(Future)* LiDAR mapping — RPLiDAR + Pi companion | 🔲              |

---

## Phase 2 Progress

| Task                                        | Status |
|---------------------------------------------|--------|
| CubeMX SPI1 configuration                   | ✅     |
| BMI323 wiring                               | ✅     |
| WHO_AM_I = 0x43 confirmed                   | ✅     |
| Fix STM32H7 NSSP CS pulse bug               | ✅     |
| Fix SPI RX FIFO lag                         | ✅     |
| CP2102 UART output working                  | ✅     |
| Fix BMI323 WriteReg — 4-byte pad            | ✅     |
| Feature engine initialization               | ✅     |
| ACC_CONF / GYR_CONF verified (0x4028/0x4048)| ✅     |
| Raw accel/gyro streaming at 100Hz           | ✅     |
| Verify data changes on physical tilt        | 🔲     |
| Sensor fusion                               | 🔲     |

---

## Toolchain

| Tool                   | Purpose                              |
|------------------------|--------------------------------------|
| STM32CubeMX            | Clock config, peripheral init, CMake |
| arm-none-eabi-gcc      | ARM cross-compiler                   |
| CMake + Ninja          | Build system                         |
| STM32CubeProgrammer    | Flash via ST-Link                    |
| PlatformIO (VS Code)   | ESP32 build + flash                  |

### Build

```powershell
cd firmware/fc
cmake --build build/Debug
```

### Flash

```powershell
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD freq=100 reset=HWrst -w build/Debug/fc.elf -rst
```

---

## Getting Started

See [`docs/architecture.md`](docs/architecture.md) for the system overview and
[`docs/hardware-notes.md`](docs/hardware-notes.md) for component-specific gotchas.