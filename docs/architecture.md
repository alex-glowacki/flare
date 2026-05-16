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
                   ├── QMC5883P Mag (I2C1)
                   ├── DSHOT300 → ESC × 4
                   ├── NEO-6M GPS (USART3)
                   └── Barometer (future)
```

---

## Component Responsibilities

| Component          | Role                                                              |
|--------------------|-------------------------------------------------------------------|
| STM32H723ZGT6      | PID, sensor fusion, DSHOT motor output, RC packet parsing         |
| BMI323             | 6-axis IMU (gyro + accel) over SPI1                               |
| QMC5883P (GY-271)  | 3-axis magnetometer, yaw reference, over I2C1                     |
| NEO-6M (GY-NEO6MV2)| GPS position, speed, altitude over USART3 DMA                    |
| ESP32 (quad)       | ESP-NOW receive → UART bridge to STM32 ✅ flashed & verified      |
| ESP32 (remote)     | Stick ADC read → ESP-NOW transmit, OLED display (Phase 5)        |
| Readytosky 35A ESCs| Motor drive via DSHOT300, Bluejay firmware                        |

---

## Development Phases

| Phase | Goal                                              | Status        |
|-------|---------------------------------------------------|---------------|
| 1     | STM32H7 bring-up — blink, UART, toolchain         | ✅ Done        |
| 2     | IMU bring-up — BMI323, sensor fusion              | ✅ Done        |
| 2.5   | Magnetometer — QMC5883P, yaw fusion               | ✅ Done        |
| 3     | PID loop — stabilization, DSHOT motor output      | ✅ Done        |
| 4     | RC link — ESP-NOW, channel parsing, arming        | ✅ Done        |
| 4.5   | GPS — NEO-6M NMEA parsing, USART3 DMA             | ✅ Done        |
| 5     | Remote firmware — sticks, OLED, switches          | 🟡 In progress |
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
│   │   │   ├── mag.h            # QMC5883P driver API
│   │   │   ├── pid.h            # PID controller API
│   │   │   ├── flare.h          # Motor mixing, arming logic API
│   │   │   ├── rc.h             # USART2 RC receiver API
│   │   │   └── gps.h            # NEO-6M NMEA parser API
│   │   └── Src/
│   │       ├── main.c           # Boot sequence, 100Hz loop, UART output
│   │       ├── imu_fusion.c     # Complementary filter (roll, pitch, yaw)
│   │       ├── mag.c            # QMC5883P I2C driver
│   │       ├── dshot.c          # DSHOT300 direct DMA output (no HAL burst)
│   │       ├── pid.c            # PID controller
│   │       ├── flare.c          # Motor mixing, arming logic
│   │       ├── rc.c             # USART2 direct-register RC receiver
│   │       ├── gps.c            # NEO-6M NMEA parser, USART3 DMA ring buffer
│   │       ├── stm32h7xx_it.c   # IRQ handlers — DSHOT TC, USART2, USART3 DMA
│   │       ├── spi.c            # CubeMX SPI1 init
│   │       ├── i2c.c            # CubeMX I2C1 init
│   │       ├── usart.c          # CubeMX USART1 + USART2 + USART3 init
│   │       ├── gpio.c           # CubeMX GPIO init
│   │       └── ...
│   ├── cmake/stm32cubemx/       # CubeMX CMake integration
│   ├── CMakeLists.txt           # Includes firmware/shared via get_filename_component
│   ├── CMakePresets.json        # Debug preset — build/Debug/
│   ├── STM32H723XG_FLASH.ld    # Linker script — includes .AXI_SRAM section
│   ├── fc.ioc                   # CubeMX project file
│   └── .clangd                  # clangd config for VS Code IntelliSense
├── esp32_quad/                  # Quad-side ESP32 ✅ flashed & running
│   ├── src/main.cpp             # ESP-NOW RX → UART bridge to STM32
│   ├── extra_script.py          # SCons pre-build: injects firmware/shared into CPPPATH
│   └── platformio.ini
└── esp32_remote/                # Remote-side ESP32 🟡 built, pending wiring
    ├── src/main.cpp             # 50Hz ESP-NOW transmitter, stick/switch stubs
    ├── extra_script.py          # SCons pre-build: injects firmware/shared into CPPPATH
    └── platformio.ini
```

---

## FC Firmware Architecture (Phase 4.5 complete)

All flight controller application code lives in `firmware/fc/Core/Src/`.
Peripheral init (`spi.c`, `i2c.c`, `usart.c`, `gpio.c`) is CubeMX-generated
and must not be edited manually except inside `USER CODE` blocks.

**Key principle:** All user code must remain inside `/* USER CODE BEGIN */` /
`/* USER CODE END */` markers to survive CubeMX regeneration. Always inspect
`main.c` after regeneration — CubeMX can wipe the MPU config and the
`while(1)` closing brace.

### main.c Boot Sequence

```
MPU_Config()        — AXI SRAM non-cacheable (0x24000000, 512KB) for DMA buffers
MX_*_Init()         — CubeMX peripheral init (GPIO, DMA, UART1/2/3, SPI, TIM, I2C)
DSHOT_Init()        — configure DMA1_Stream0, set CCRs idle-LOW, start PWM
                      800 × 10ms = 8s of DSHOT 0 frames (Bluejay arm sequence)
BMI323_Init()       — SPI bring-up, feature engine, acc/gyr config
IMU_Fusion_Init()   — zero roll/pitch/yaw state
MAG_Init()          — I2C bring-up; sets mag_ok=0 if chip ID fails
FLARE_Init()        — PID state, motor mixing init
RC_Init()           — arm USART2 RX via direct register write
GPS_Init()          — start USART3 DMA circular receive (256-byte ring buffer)
IMU calibration     — 200 samples × 10ms, average last 50 for level offsets

100Hz while loop:
  BMI323_ReadAccel / ReadGyro
  MAG_ReadHeading              — skipped if mag_ok == 0
  IMU_Fusion_Update            — complementary filter, all 3 axes
  GPS_Update()                 — scan DMA ring buffer, parse new NMEA sentences
  RC_GetPacket()               — consume validated RC packet if available
  if RC healthy and mode != SAFE:
    FLARE_Update               — PID + motor mixing
  else:
    FLARE disarmed, DSHOT 0 keepalive
  UART_Print                   — live telemetry including RC:OK/LOST
  every 100 loops (1s):
    [GPS] print — fix, sats, lat, lon, alt (ft), speed (mph)
    [DIAG] print — DMA send/tc counters
```

---

## FLARE RC Protocol

Defined in `firmware/shared/flare_protocol.h`.

### Packet Layout (14 bytes)

| Byte(s) | Field    | Type       | Notes                         |
|---------|----------|------------|-------------------------------|
| 0       | magic    | uint8_t    | `0xFA`                        |
| 1–2     | throttle | uint16_t   | 1000–2000                     |
| 3–4     | roll     | uint16_t   | 1000–2000, center 1500        |
| 5–6     | pitch    | uint16_t   | 1000–2000, center 1500        |
| 7–8     | yaw      | uint16_t   | 1000–2000, center 1500        |
| 9       | armed    | uint8_t    | 0 = disarmed, 1 = armed       |
| 10      | mode     | uint8_t    | FLARE_MODE_* constants        |
| 11–12   | reserved | uint8_t[2] | Zero-padded, reserved         |
| 13      | checksum | uint8_t    | XOR checksum over bytes [0–12]|

### STM32 RC Receiver (rc.c)

- Direct register RX: `SET_BIT(USART2->CR1, USART_CR1_RXNEIE_RXFNEIE)`
- `USART2_IRQHandler` reads `USART2->RDR` directly, calls `RC_UART_RxCpltCallback()`
- Sync recovery: first byte must be `FLARE_PACKET_MAGIC`
- On full packet: validate magic + XOR checksum, copy to shadow buffer
- `RC_GetPacket()` — copies shadow buffer to caller, clears flag
- `RC_IsHealthy()` — returns 1 if last valid packet within `RC_TIMEOUT_MS (250)`

---

## GPS Driver (gps.c)

### Approach
Circular DMA receive into a 256-byte ring buffer in AXI SRAM (non-cacheable).
`GPS_Update()` is called each main loop iteration and walks new bytes from the
DMA write head, assembling NMEA sentences and dispatching complete ones to
parsers. No interrupts required beyond DMA housekeeping.

### Sentences Parsed

| Sentence      | Fields extracted                                    |
|---------------|-----------------------------------------------------|
| $GPRMC/$GNRMC | latitude, longitude, speed (mph), course, fix valid |
| $GPGGA/$GNGGA | altitude (ft), satellites in use                    |

All other sentence types ($GPGSV, $GPGLL, etc.) are silently dropped.

### Checksum
NMEA XOR checksum validated on every sentence before parsing. Sentences with
invalid checksums are silently discarded.

### Unit Conversions
- Speed: knots × 1.15078 = mph
- Altitude: metres × 3.28084 = feet

### DMA Buffer Placement
```c
static uint8_t gps_dma_buf[256] __attribute__((section(".AXI_SRAM")));
```
Placed in RAM_D1 via the `.AXI_SRAM` linker section. The MPU non-cacheable
region at `0x24000000` ensures CPU reads are coherent with DMA writes.

---

## DSHOT300 Implementation

### Approach
Direct DMA register programming — HAL DMA burst API ignored NDTR on STM32H7.

### Buffer Layout
`dshot_buf[17][4]` — 17 rows × 4 motors, placed in AXI SRAM at `0x24000000`.
16 data rows (one bit per row) + 1 reset slot row (all zeros).

### Key Constants (@ 192MHz SYSCLK, ARR = 639)
| Symbol           | Value | Meaning                      |
|------------------|-------|------------------------------|
| `DSHOT_BIT_1`    | 480   | 75% duty — logic 1           |
| `DSHOT_BIT_0`    | 240   | 37.5% duty — logic 0         |
| `DSHOT_CCR_IDLE` | 0     | 0% duty — idle-LOW (Bluejay) |
| Bit period       | 3.33µs | 300 kbps (DSHOT300)         |

### Bluejay CRC
Non-inverted: `crc = (v ^ (v>>4) ^ (v>>8)) & 0x0F`
Standard DSHOT spec uses inverted CRC — Bluejay does not.

### IRQ Priorities
| Interrupt         | Priority | Reason                                       |
|-------------------|----------|----------------------------------------------|
| DMA1_Stream0 (TC) | 0, 0     | Highest — idle-LOW restore is time-critical  |
| DMA1_Stream1      | 0, 0     | USART3 GPS DMA housekeeping                  |
| USART2 (RC RX)    | 8, 0     | Must not preempt DSHOT TC handler            |

---

## ESP32 Remote Firmware Architecture (Phase 5 — in progress)

### esp32_remote/src/main.cpp — 50Hz transmitter

```
setup():
  Serial.begin(115200)
  pinMode ARM_SWITCH, MODE_SWITCH → INPUT_PULLUP
  WiFi.mode(WIFI_STA), disconnect
  esp_now_init()
  esp_now_register_send_cb(on_packet_sent)
  esp_now_add_peer(kQuadMac)   ← 20:6E:F1:32:70:3C

loop() at 50Hz:
  analogRead() × 4 axes → map_stick() → 1000–2000
  digitalRead() × 2 switches → armed / mode flags
  build FLARE_RC_Packet_t
  flare_checksum() → pkt.checksum
  esp_now_send(kQuadMac, &pkt, 14)
```

### Pending Before Flash
- Wire M7 gimbals → assign GPIO ADC pins
- Wire arm + mode switches
- Measure per-axis ADC min/center/max
- Determine axis reversal per gimbal orientation
- Wire SSD1309 OLED (SPI) → add display driver

---

## Toolchain

| Tool                  | Version  | Purpose                          |
|-----------------------|----------|----------------------------------|
| STM32CubeMX           | 6.17.0   | Peripheral config + code gen     |
| arm-none-eabi-gcc     | 15.2.1   | ARM cross-compiler               |
| CMake                 | 4.3.1    | Build system                     |
| Ninja                 | 1.13.2   | Build backend                    |
| STM32CubeProgrammer   | v2.22.0  | Flash + RAM read via ST-Link     |
| VS Code + clangd      | —        | Editor + IntelliSense            |
| PlatformIO            | —        | ESP32 build + flash              |

### Build & Flash

```powershell
# STM32 — build (from firmware/fc/)
cmake --build --preset Debug

# STM32 — clean build
cmake --build --preset Debug --clean-first

# STM32 — flash
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" `
  -c port=SWD freq=100 reset=HWrst -w build/Debug/fc.elf -rst

# STM32 — RAM read (for variable inspection without UART)
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" `
  -c port=SWD freq=100 -r32 <address> <length>

# Find variable addresses
arm-none-eabi-nm build/Debug/fc.elf | Select-String "variable_name"

# ESP32 — build (from firmware/esp32_quad/ or esp32_remote/)
C:\Users\alexg\.platformio\penv\Scripts\platformio.exe run --environment arduino_nano_esp32

# ESP32 — flash (double-tap RST first)
C:\Users\alexg\.platformio\penv\Scripts\platformio.exe run --target upload --environment arduino_nano_esp32

# ESP32 — serial monitor
C:\Users\alexg\.platformio\penv\Scripts\platformio.exe device monitor --environment arduino_nano_esp32 --baud 115200
```

### CMake shared include path

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