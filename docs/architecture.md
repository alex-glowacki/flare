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
                   UART (115200)
                   GPIO17 → PA3
                         │
                  [STM32H723 FC]
                   ├── BMI323 IMU (SPI1)
                   ├── QMC5883L Mag (I2C1)
                   ├── DSHOT300 → ESC × 4
                   └── Barometer (future)
```

---

## Component Responsibilities

| Component          | Role                                                        |
|--------------------|-------------------------------------------------------------|
| STM32H723ZGT6      | PID, sensor fusion, DSHOT motor output, RC packet parsing   |
| BMI323             | 6-axis IMU (gyro + accel) over SPI1                         |
| QMC5883L (GY-271)  | 3-axis magnetometer, yaw reference, over I2C1               |
| ESP32 (quad)       | ESP-NOW receive → UART bridge to STM32 (Phase 4 complete)   |
| ESP32 (remote)     | Stick ADC read → ESP-NOW transmit, OLED display (Phase 5)   |
| BLHeli_S 35A ESCs  | Motor drive via DSHOT300                                    |

---

## Development Phases

| Phase | Goal                                              | Status        |
|-------|---------------------------------------------------|---------------|
| 1     | STM32H7 bring-up — blink, UART, toolchain         | ✅ Done        |
| 2     | IMU bring-up — BMI323, sensor fusion              | ✅ Done        |
| 2.5   | Magnetometer — QMC5883L, yaw fusion               | ✅ Done        |
| 3     | PID loop — stabilization, DSHOT motor output      | ✅ Done        |
| 4     | RC link — ESP-NOW, channel parsing, arming        | ✅ Done        |
| 5     | Remote firmware — sticks, OLED, switches          | 🔲             |
| 6     | Flight testing & tuning                           | 🔲             |
| 7     | *(Future)* LiDAR mapping — RPLiDAR + Pi companion | 🔲             |

---

## Firmware Structure

```
firmware/
├── shared/                      # Headers shared between ESP32 and STM32
│   └── flare_protocol.h         # RC packet struct, CRC-8/MAXIM, constants
├── fc/                          # STM32H723 flight controller
│   ├── Core/
│   │   ├── Inc/                 # Headers
│   │   │   ├── main.h
│   │   │   ├── dshot.h          # DSHOT300 driver API
│   │   │   ├── imu_fusion.h     # Complementary filter API
│   │   │   ├── mag.h            # QMC5883L driver API
│   │   │   ├── pid.h            # PID controller API
│   │   │   ├── flare.h          # Motor mixing, arming logic API
│   │   │   └── rc.h             # USART2 RC receiver API
│   │   └── Src/
│   │       ├── main.c           # Boot sequence, 100Hz loop, UART output
│   │       ├── imu_fusion.c     # Complementary filter (roll, pitch, yaw)
│   │       ├── mag.c            # QMC5883L I2C driver
│   │       ├── dshot.c          # DSHOT300 direct DMA output (no HAL burst)
│   │       ├── pid.c            # PID controller
│   │       ├── flare.c          # Motor mixing, arming logic
│   │       ├── rc.c             # USART2 interrupt-driven RC receiver
│   │       ├── stm32h7xx_it.c   # IRQ handlers — DSHOT TC + USART2
│   │       ├── spi.c            # CubeMX SPI1 init
│   │       ├── i2c.c            # CubeMX I2C1 init
│   │       ├── usart.c          # CubeMX USART1 + USART2 init
│   │       ├── gpio.c           # CubeMX GPIO init (includes DSHOT idle-high)
│   │       └── ...
│   ├── cmake/stm32cubemx/       # CubeMX CMake integration
│   ├── CMakeLists.txt           # Includes firmware/shared via get_filename_component
│   ├── CMakePresets.json        # Debug preset — build/Debug/
│   ├── fc.ioc                   # CubeMX project file
│   └── .clangd                  # clangd config for VS Code IntelliSense
├── esp32_quad/                  # Quad-side ESP32 (Phase 4 complete)
│   ├── src/main.cpp             # ESP-NOW RX → UART bridge to STM32
│   └── platformio.ini           # lib_extra_dirs = ../../shared
└── esp32_remote/                # Remote-side ESP32 (Phase 5)
    ├── src/main.cpp             # Stub
    └── platformio.ini           # lib_extra_dirs = ../../shared
```

---

## FC Firmware Architecture (Phase 4 complete)

All flight controller application code lives in `firmware/fc/Core/Src/`.
Peripheral init (`spi.c`, `i2c.c`, `usart.c`, `gpio.c`) is CubeMX-generated
and must not be edited manually except inside `USER CODE` blocks.

**Key principle:** All user code must remain inside `/* USER CODE BEGIN */` /
`/* USER CODE END */` markers to survive CubeMX regeneration. Always inspect
`main.c` after regeneration — CubeMX can wipe includes placed outside markers
and can also wipe `MPU_Config()` Region 1.

### main.c Boot Sequence

```
MX_*_Init()         — CubeMX peripheral init (GPIO, DMA, UART1, UART2, SPI, TIM, I2C)
DSHOT_Init()        — configure DMA1_Stream0, set CCRs idle-high, start PWM
                      send 200 × zero-throttle frames (2s arm sequence)
BMI323_Init()       — SPI bring-up, feature engine, acc/gyr config
IMU_Fusion_Init()   — zero roll/pitch/yaw state
MAG_Init()          — I2C bring-up; sets mag_ok=0 if chip ID fails
FLARE_Init()        — PID state, motor mixing init
RC_Init()           — arm USART2 HAL_UART_Receive_IT, begin byte-by-byte RX

100Hz while loop:
  BMI323_ReadAccel / ReadGyro
  MAG_ReadHeading              — skipped if mag_ok == 0
  IMU_Fusion_Update            — complementary filter, all 3 axes
  RC_GetPacket()               — consume validated RC packet if available
  if RC healthy and armed:
    FLARE_Update               — PID + motor mixing
  DSHOT_SendThrottle(0,0,0,0)  — keepalive (zero throttle until Phase 5)
  UART_Print                   — live telemetry including RC:OK/LOST
```

---

## FLARE RC Protocol

Defined in `firmware/shared/flare_protocol.h` — included by both ESP32
PlatformIO projects (via `lib_extra_dirs`) and the STM32 CMake build
(via `get_filename_component(SHARED_INC ...)` in `CMakeLists.txt`).

### Packet Layout (14 bytes)

| Byte(s) | Field    | Type       | Notes                         |
|---------|----------|------------|-------------------------------|
| 0       | magic    | uint8_t    | `0xAF`                        |
| 1–2     | throttle | uint16_t   | 1000–2000                     |
| 3–4     | roll     | uint16_t   | 1000–2000, center 1500        |
| 5–6     | pitch    | uint16_t   | 1000–2000, center 1500        |
| 7–8     | yaw      | uint16_t   | 1000–2000, center 1500        |
| 9       | armed    | uint8_t    | 0 = disarmed, 1 = armed       |
| 10      | mode     | uint8_t    | 0 = angle, 1 = acro           |
| 11–12   | reserved | uint8_t[2] | Zero-padded                   |
| 13      | checksum | uint8_t    | CRC-8/MAXIM over bytes [0–12] |

### Checksum

CRC-8/MAXIM (Dallas/Maxim 1-Wire CRC), polynomial 0x31 reflected.
Computed via lookup table in `flare_crc8()`. `flare_checksum()` covers
`FLARE_PACKET_DATA_LEN = 13` bytes (all fields except checksum itself).

### STM32 RC Receiver (rc.c)

- `HAL_UART_Receive_IT()` armed for 1 byte at a time
- Sync recovery: first byte of each frame must be `FLARE_PACKET_MAGIC (0xAF)`
- On 14th byte: validate magic + CRC, copy to shadow buffer, set flag
- `RC_GetPacket()` — copies shadow buffer to caller, clears flag
- `RC_IsHealthy()` — returns 1 if last valid packet within `RC_TIMEOUT_MS (250)`
- `HAL_UART_RxCpltCallback` in `main.c` dispatches to `RC_UART_RxCpltCallback()`

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
3. `DSHOT_StartDMA()` waits for previous stream to disable (EN bit clear),
   then reconfigures DMA1_Stream0 registers and enables `TIM4->DIER |= TIM_DIER_UDE`
4. On each TIM4 update event, DMA writes 4 words to `TIM4->DMAR`
5. After 68 words, DMA stops (Normal mode)
6. `DMA1_Stream0_IRQHandler` TC callback fires:
   - Clears `TIM_DIER_UDE`
   - Sets `CCR1–CCR4 = 640` to hold outputs idle-high

### Key Constants (@ 192MHz SYSCLK, ARR = 639)
| Symbol           | Value | Meaning                     |
|------------------|-------|-----------------------------|
| `DSHOT_BIT_1`    | 480   | 75% duty — logic 1          |
| `DSHOT_BIT_0`    | 240   | 37.5% duty — logic 0        |
| `DSHOT_CCR_IDLE` | 640   | 100% duty — idle-high       |
| Bit period       | 3.33µs | 300 kbps (DSHOT300)        |

### IRQ Priorities
| Interrupt         | Priority | Reason                                    |
|-------------------|----------|-------------------------------------------|
| DMA1_Stream0 (TC) | 0, 0     | Highest — idle-high restore is time-critical |
| USART2 (RC RX)    | 8, 0     | Must not preempt DSHOT TC handler         |

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
cmake --build --preset Debug

# Flash
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" `
  -c port=SWD freq=100 reset=HWrst -w build/Debug/fc.elf -rst

# RAM read (for variable inspection without UART)
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" `
  -c port=SWD freq=100 -r32 <address> <length>

# Find variable addresses
arm-none-eabi-nm build/Debug/fc.elf | Select-String "variable_name"
```

### CMake shared include path

`firmware/shared/` is outside the STM32 CMake source tree. The path is
resolved at configure time using:

```cmake
get_filename_component(SHARED_INC
    "${CMAKE_SOURCE_DIR}/../shared"
    ABSOLUTE
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    ${SHARED_INC}
)
```

`CMAKE_SOURCE_DIR` resolves to `firmware/fc/`, so `../shared` lands at
`firmware/shared/`. `get_filename_component(... ABSOLUTE)` is required —
passing the relative path directly causes GCC on Windows to fail to resolve
the `..` segments.