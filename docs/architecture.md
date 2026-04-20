# FLARE — Architecture Overview

---

## System Block Diagram

```
[Hall Gimbals] ──► [ESP32 Remote]
                         │
                   ESP-NOW (2.4GHz)
                         │
                   [ESP32 Quad]
                         │
                        UART
                         │
                  [STM32H723 FC]
                   ├── BMI323 IMU (SPI1)
                   ├── QMC5883L Mag (I2C1)
                   ├── DSHOT300 → ESC × 4
                   └── Barometer (future)
```

---

## Component Responsibilities

| Component          | Role                                              |
|--------------------|---------------------------------------------------|
| STM32H723ZGT6      | PID, sensor fusion, DSHOT motor output            |
| BMI323             | 6-axis IMU (gyro + accel) over SPI1               |
| QMC5883L (GY-271)  | 3-axis magnetometer, yaw reference, over I2C1     |
| ESP32 (quad)       | ESP-NOW receive → UART bridge to STM32            |
| ESP32 (remote)     | Stick ADC read → ESP-NOW transmit, OLED display   |
| BLHeli_S 35A ESCs  | Motor drive via DSHOT300                          |

---

## Development Phases

| Phase | Goal                                              | Status          |
|-------|---------------------------------------------------|-----------------|
| 1     | STM32H7 bring-up — blink, UART, toolchain         | ✅ Done          |
| 2     | IMU bring-up — BMI323, sensor fusion              | ✅ Done          |
| 2.5   | Magnetometer — QMC5883L, yaw fusion               | ✅ Done          |
| 3     | PID loop — stabilization, DSHOT motor output      | 🔄 Blocked (ESC) |
| 4     | RC link — ESP-NOW, channel parsing, arming        | 🔲               |
| 5     | Remote firmware — sticks, OLED, switches          | 🔲               |
| 6     | Flight testing & tuning                           | 🔲               |
| 7     | *(Future)* LiDAR mapping — RPLiDAR + Pi companion | 🔲               |

---

## Firmware Structure

```
firmware/
├── fc/                          # STM32H723 flight controller
│   ├── Core/
│   │   ├── Inc/                 # Headers
│   │   │   ├── main.h
│   │   │   ├── imu_fusion.h     # Complementary filter API
│   │   │   └── mag.h            # QMC5883L driver API
│   │   └── Src/
│   │       ├── main.c           # Boot sequence, 100Hz loop, UART output
│   │       ├── imu_fusion.c     # Complementary filter (roll, pitch, yaw)
│   │       ├── mag.c            # QMC5883L I2C driver
│   │       ├── dshot.c          # DSHOT300 DMA output
│   │       ├── pid.c            # PID controller
│   │       ├── flare.c          # Motor mixing, arming logic
│   │       ├── spi.c            # CubeMX SPI1 init
│   │       ├── i2c.c            # CubeMX I2C1 init
│   │       ├── usart.c          # CubeMX USART1 init
│   │       ├── gpio.c           # CubeMX GPIO init
│   │       └── ...
│   ├── cmake/stm32cubemx/       # CubeMX CMake integration
│   ├── CMakeLists.txt
│   ├── CMakePresets.json        # Debug preset — build/Debug/
│   ├── fc.ioc                   # CubeMX project file
│   └── .clangd                  # clangd config for VS Code IntelliSense
├── esp32_quad/                  # Quad-side ESP32 (Phase 4)
│   ├── src/main.cpp
│   └── platformio.ini
└── esp32_remote/                # Remote-side ESP32 (Phase 5)
    ├── src/main.cpp
    └── platformio.ini
```

---

## FC Firmware Architecture (Phase 2.5 current state)

All flight controller application code lives in `firmware/fc/Core/Src/`.
Peripheral init (`spi.c`, `i2c.c`, `usart.c`, `gpio.c`) is CubeMX-generated
and must not be edited manually.

**Key principle:** All user code must remain inside `/* USER CODE BEGIN */` /
`/* USER CODE END */` markers to survive CubeMX regeneration. Always inspect
`main.c` after regeneration — CubeMX can wipe includes placed outside markers.

### main.c Boot Sequence

```
MX_*_Init()         — CubeMX peripheral init (GPIO, DMA, UART, SPI, TIM, I2C)
DSHOT_Init()        — start timer, send 2s zero-throttle arm sequence
BMI323_Init()       — SPI bring-up, feature engine, acc/gyr config
IMU_Fusion_Init()   — zero roll/pitch/yaw state
MAG_Init()          — I2C bring-up, FBR + CTRL1 config
FLARE_Init()        — PID state, motor mixing init
[BENCH TEST]        — 500 × throttle 1000 (remove after ESC config verified)
100Hz while loop:
  BMI323_ReadAccel / ReadGyro
  MAG_ReadHeading
  IMU_Fusion_Update  — complementary filter, all 3 axes
  FLARE_Update       — PID + motor mixing
  UART_Print         — live telemetry
```

---

## Toolchain

| Tool                  | Version  | Purpose                          |
|-----------------------|----------|----------------------------------|
| STM32CubeMX           | Latest   | Peripheral config + code gen     |
| arm-none-eabi-gcc     | 15.2.1   | ARM cross-compiler               |
| CMake                 | 4.3.1    | Build system                     |
| Ninja                 | 1.13.2   | Build backend                    |
| STM32CubeProgrammer   | v2.22.0  | Flash + RAM read via ST-Link     |
| VS Code + clangd      | —        | Editor + IntelliSense            |
| PlatformIO            | —        | ESP32 build + flash              |

### Build & Flash

```powershell
# Build
cmake --build build/Debug

# Flash
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" `
  -c port=SWD freq=100 reset=HWrst -w build/Debug/fc.elf -rst

# RAM read (for variable inspection without UART)
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" `
  -c port=SWD freq=100 -r32 <address> <length>

# Find variable addresses
arm-none-eabi-nm build/Debug/fc.elf | Select-String "variable_name"
```