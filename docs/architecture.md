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
                   ├── BMI323 IMU (SPI)
                   ├── DSHOT300 → ESC × 4
                   └── Barometer (future)
```

---

## Component Responsibilities

| Component          | Role                                              |
|--------------------|---------------------------------------------------|
| STM32H723ZGT6      | PID, sensor fusion, DSHOT motor output            |
| BMI323             | 6-axis IMU (gyro + accel) over SPI                |
| ESP32 (quad)       | ESP-NOW receive → UART bridge to STM32            |
| ESP32 (remote)     | Stick ADC read → ESP-NOW transmit, OLED display   |
| BLHeli_S 30A ESCs  | Motor drive via DSHOT300                          |

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

## Firmware Structure

```
firmware/
├── fc/                          # STM32H723 flight controller
│   ├── Core/
│   │   ├── Inc/                 # CubeMX-generated headers
│   │   └── Src/
│   │       ├── main.c           # All FC application code (Phase 1-2)
│   │       ├── spi.c            # CubeMX SPI1 init
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

## FC Firmware Architecture (Phase 2 current state)

All flight controller application code lives in `firmware/fc/Core/Src/main.c`.
Peripheral init (`spi.c`, `usart.c`, `gpio.c`) is CubeMX-generated and must
not be edited manually — use CubeMX to modify peripheral config, then regenerate.

**Key principle:** All user code must remain inside `/* USER CODE BEGIN */` /
`/* USER CODE END */` markers to survive CubeMX regeneration. Always inspect
`main.c` after regeneration — CubeMX can wipe includes placed outside markers.

### main.c Structure

```
Defines      — BMI323 register addresses, config values, timing macros
Globals      — volatile diagnostic variables (who_am_i_result, acc/gyr readbacks)
UART_Print   — thin wrapper around HAL_UART_Transmit
SPI helpers  — SPI1_FlushRxFifo
BMI323 driver:
  BMI323_ReadReg        — 4-byte SPI read
  BMI323_WriteReg       — 4-byte SPI write (3 data + 1 dummy pad)
  BMI323_BurstRead      — multi-word burst read for accel/gyro data
  BMI323_InitFeatureEngine — required before ACC_CONF/GYR_CONF writes
  BMI323_Init           — full init sequence
  BMI323_ReadAccel      — updates imu_acc_x/y/z globals
  BMI323_ReadGyro       — updates imu_gyr_x/y/z globals
main()       — init peripherals → BMI323_Init → 100Hz loop
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