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
| 3     | PID loop — stabilization, DSHOT motor output      | ✅ Done          |
| 4     | RC link — ESP-NOW, channel parsing, arming        | 🔄 In progress   |
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
│   │   │   ├── dshot.h          # DSHOT300 driver API
│   │   │   ├── imu_fusion.h     # Complementary filter API
│   │   │   ├── mag.h            # QMC5883L driver API
│   │   │   ├── pid.h            # PID controller API
│   │   │   └── flare.h          # Motor mixing, arming logic API
│   │   └── Src/
│   │       ├── main.c           # Boot sequence, 100Hz loop, UART output
│   │       ├── imu_fusion.c     # Complementary filter (roll, pitch, yaw)
│   │       ├── mag.c            # QMC5883L I2C driver
│   │       ├── dshot.c          # DSHOT300 direct DMA output (no HAL burst)
│   │       ├── pid.c            # PID controller
│   │       ├── flare.c          # Motor mixing, arming logic
│   │       ├── stm32h7xx_it.c   # IRQ handlers — includes DSHOT TC callback
│   │       ├── spi.c            # CubeMX SPI1 init
│   │       ├── i2c.c            # CubeMX I2C1 init
│   │       ├── usart.c          # CubeMX USART1 init
│   │       ├── gpio.c           # CubeMX GPIO init (includes DSHOT idle-high)
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

## FC Firmware Architecture (Phase 3 complete)

All flight controller application code lives in `firmware/fc/Core/Src/`.
Peripheral init (`spi.c`, `i2c.c`, `usart.c`, `gpio.c`) is CubeMX-generated
and must not be edited manually except inside `USER CODE` blocks.

**Key principle:** All user code must remain inside `/* USER CODE BEGIN */` /
`/* USER CODE END */` markers to survive CubeMX regeneration. Always inspect
`main.c` after regeneration — CubeMX can wipe includes placed outside markers.

### main.c Boot Sequence

```
MX_*_Init()         — CubeMX peripheral init (GPIO, DMA, UART, SPI, TIM, I2C)
DSHOT_Init()        — configure DMA1_Stream0, set CCRs idle-high, start PWM
                      send 200 × zero-throttle frames (2s arm sequence)
BMI323_Init()       — SPI bring-up, feature engine, acc/gyr config
IMU_Fusion_Init()   — zero roll/pitch/yaw state
MAG_Init()          — I2C bring-up, FBR + CTRL1 config
FLARE_Init()        — PID state, motor mixing init
100Hz while loop:
  BMI323_ReadAccel / ReadGyro
  MAG_ReadHeading
  IMU_Fusion_Update  — complementary filter, all 3 axes
  FLARE_Update       — PID + motor mixing
  UART_Print         — live telemetry
  DSHOT_SendThrottle(0,0,0,0)  — keep ESCs armed (zero throttle until RC live)
```

---

## DSHOT300 Implementation

### Approach
Direct DMA register programming — HAL DMA burst API (`HAL_TIM_DMABurst_WriteStart`)
was found to ignore NDTR on STM32H7 and run continuously regardless of Normal
mode setting. Direct register access bypasses this completely.

### Buffer Layout
`dshot_buf[17][4]` — 17 rows × 4 motors = 68 words, placed in D1 AXI SRAM
at `0x24000000`. 16 data rows (one bit per row) + 1 reset slot row (all zeros).

### Transfer Sequence (per frame)
1. `DSHOT_SendThrottle()` serialises 4 throttle values into `dshot_buf`
2. D-cache flushed via `SCB_CleanDCache_by_Addr`
3. `DSHOT_StartDMA()` reconfigures DMA1_Stream0 registers directly and enables
   `TIM4->DIER |= TIM_DIER_UDE`
4. On each TIM4 update event, DMA writes 4 words to `TIM4->DMAR`, which the
   timer routes to CCR1→CCR2→CCR3→CCR4 via the burst access register
5. After 68 words (17 update events), DMA stops (Normal mode)
6. `DMA1_Stream0_IRQHandler` TC callback fires:
   - Clears `TIM_DIER_UDE` to stop further DMA requests
   - Sets `CCR1–CCR4 = 640` (ARR+1 = 100% duty) to hold outputs idle-high

### Key Constants (@ 192MHz SYSCLK, ARR = 639)
| Symbol        | Value | Meaning                     |
|---------------|-------|-----------------------------|
| `DSHOT_BIT_1` | 480   | 75% duty — logic 1          |
| `DSHOT_BIT_0` | 240   | 37.5% duty — logic 0        |
| `DSHOT_CCR_IDLE` | 640 | 100% duty — idle-high      |
| Bit period    | 3.33µs | 300 kbps (DSHOT300)        |

### TIM4 DCR Configuration
```c
TIM4->DCR = (3U << TIM_DCR_DBL_Pos)   /* DBL=3 → 4 transfers per burst */
          | (13U << TIM_DCR_DBA_Pos);  /* DBA=13 → CCR1 word offset     */
```

### Idle-High Boot Sequence
`gpio.c` drives PD12–PD15 HIGH as `GPIO_OUTPUT_PP` immediately after GPIOD
clock enable — before `MX_TIM4_Init()` switches them to TIM4 AF. This prevents
ESC protocol auto-detection from seeing a floating/low signal at startup.

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
# Build (from firmware/fc/)
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