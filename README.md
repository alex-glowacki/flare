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
| 2.5   | Magnetometer — QMC5883P hard-iron calibration     | ✅ Done |
| 3     | PID loop — stabilization, DSHOT motor output      | ✅ Done |
| 4     | RC link — ESP-NOW, channel parsing, arming        | ✅ Done |
| 4.5   | GPS — M100-5883 UBX binary parser, USART3 DMA     | ✅ Done |
| 4.75  | SD card blackbox logger                           | ✅ Done |
| 5     | Remote firmware — sticks, OLED, switches          | 🟡 In progress |
| 6     | Flight testing & PID tuning                       | 🟡 In progress |
| 7     | *(Future)* LiDAR mapping — RPLiDAR + Pi companion | 🔲      |

### Phase 6 — Flight Testing Detail

| Item | Status |
|---|---|
| First outdoor hover flight | ✅ Done (LOG000/LOG001) |
| IMU mounting offset calibration | ✅ Done — roll=+0.27°, pitch=−3.80°, bench-verified |
| Mag hard-iron calibration | ✅ Done — offset_x=−251.0, offset_y=+41.5 |
| SD blackbox logger operational | ✅ Done — LOG000–LOG008 captured |
| Yaw kd tuning — damping spin | ✅ Done — kd=0.05 confirmed (LOG001) |
| Yaw kp/output limit increase | ✅ Done — kp=1.2, out_limit=300 (LOG007) |
| Prop/motor direction verification | ✅ Done — all confirmed correct |
| Weak motor identification | ✅ Done — M1 (Front-Left, CCW) confirmed weaker than M3 via swap test |
| M1 motor replacement | 🔲 Pending — Readytosky 2212 920KV CCW needed |
| Software roll trim (interim) | 🔲 Pending — awaiting M1 swap back to original position |
| Stable hands-off hover | 🔲 Pending |
| ki_yaw = 0.01 | 🔲 Pending — after stable hover confirmed |

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
- USART1: PA9=TX, PA10=RX (debug / CP2102)
- USART2: PA2=TX, PA3=RX (ESP32 RC link)
- USART3: PB10=TX, PB11=RX (GPS)
- TIM4 DSHOT: PD12=M1, PD13=M2, PD14=M3, PD15=M4
- SD CS: PC0 (SPI1 shared with IMU)
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

### GPS + Compass — HGLRC M100-5883

| M100-5883 Pin | FK723M1 Pin | Notes |
|---------------|-------------|-------|
| 5V            | PDB BEC 5V  | Do not use STM32 3.3V pin |
| GND           | Common GND  | |
| TX            | PB11 (USART3 RX) | GPS data → STM32 |
| RX            | PB10 (USART3 TX) | Optional config commands |
| SDA           | PB7 (I2C1 SDA)   | QMC5883P compass |
| SCL           | PB6 (I2C1 SCL)   | QMC5883P compass |

- GPS chip: u-blox M10 (10th gen), 72 channels, GPS/GLONASS/BDS/Galileo/QZSS
- GPS protocol: UBX binary, NAV-PVT messages, 115200 baud, DMA ring buffer
- GPS fix confirmed indoors (8 satellites)
- Compass chip: QMC5883P, I2C address `0x2C`, chip ID `0x80`
- Hard-iron offsets: `offset_x = -251.0f`, `offset_y = 41.5f` (calibrated)
- Yaw stable at ~161–165° stationary

### ESCs — Readytosky 35A (×4)

| Motor | Position    | Spin | Cap color | Pin  |
|-------|-------------|------|-----------|------|
| M1    | Front-Left  | CCW  | Silver    | PD12 |
| M2    | Front-Right | CW   | Black     | PD13 |
| M3    | Rear-Right  | CCW  | Silver    | PD14 |
| M4    | Rear-Left   | CW   | Black     | PD15 |

- Firmware: Bluejay (replaces stock BLHeli_S)
- Protocol: DSHOT300
- Props: 1045R on CCW motors (M1/M3), 1045L on CW motors (M2/M4)
- Demag Compensation: High (configured via BLHeliSuite + ATmega328P Nano)
- Config backed up: `config/esc_blheli_setup.ini`

> ⚠️ M1 (Front-Left, CCW, silver cap) is a known weak motor — produces less
> thrust than M3 at equivalent DSHOT values. Confirmed via motor swap test
> (LOG007/LOG008). Replacement Readytosky 2212 920KV CCW unit required.

### Power Distribution
- QWinOut PDB with built-in 5V and 12V BECs
- 3S LiPo with XT60
- 4S upgrade planned post stable-flight (verify PDB BEC supports up to 16.8V first)

### RC Link
- Nano ESP32 modules (ESP-NOW, quad-side and remote)
- Quad-side ESP32: **MAC `D4:E9:F4:E6:E9:90`**, GPIO17 (TX) → STM32 PA3 (USART2 RX)
- Remote ESP32: FrSky M7 Hall Sensor Gimbals (×2), 2-position toggle switches for arm/mode
- Shared GND between ESP32 and STM32

> ⚠️ Do not power the quad-side Nano ESP32 from the BEC 5V → VIN pin. The
> ESP32-S3 core requires 3.3V. Power from STM32 3.3V pin or a dedicated 3.3V
> regulator.

> ⚠️ Do not flash STM32 via ST-Link while the ESP32 shares the USB power rail.
> ST-Link reset glitches can corrupt ESP32 flash mid-write, changing the
> reported MAC address.

### FAA Remote ID
- Ruko R111S — hardware mount completed

### SD Card Blackbox Logger
- WWZMDiB micro-SD module, SPI1 shared with BMI323, CS=PC0
- FatFS, 5Hz write / 1Hz flush
- f_printf has no `%f` support — all floats scaled to integers permanently
- Column names encode scale factor (e.g. `roll_x100`, `lat_x1e6`)
- Last log: LOG008.CSV

### Remote Controller Hardware (Phase 5 — in progress)
- **Gimbals:** FrSky M7 Hall Sensor (×2) — analog, 4 axes total
- **OLED:** HiLetgo 2.42" SSD1309 128×64, SPI 7-pin
- **Switches:** 2-position toggle — arming (dedicated), mode angle/acro (dedicated)
- Pin assignments TBD pending wiring

### Debug / Programming
- ST-Link V2 clone (V2J37S7) — SWD, CLI only
- CP2102 USB-UART adapter — COM3, 115200 baud
  - CP2102 TX → PA10, RX → PA9, GND → GND, VCC unconnected

---

## Repository Structure

```
flare/
├── firmware/
│   ├── shared/                    # Shared headers (ESP32 + STM32)
│   │   └── flare_protocol.h       # RC packet definition, CRC-8/MAXIM
│   ├── fc/                        # STM32H7 flight controller firmware
│   │   ├── Core/
│   │   │   ├── Inc/
│   │   │   │   ├── main.h
│   │   │   │   ├── dshot.h        # DSHOT300 driver API
│   │   │   │   ├── imu_fusion.h   # Complementary filter API
│   │   │   │   ├── mag.h          # QMC5883P magnetometer driver API
│   │   │   │   ├── pid.h          # PID controller API
│   │   │   │   ├── flare.h        # Motor mixing, arming logic API
│   │   │   │   ├── rc.h           # USART2 RC receiver API
│   │   │   │   └── gps.h          # UBX binary GPS parser API
│   │   │   └── Src/
│   │   │       ├── main.c         # Boot sequence, 100Hz loop, UART output
│   │   │       ├── imu_fusion.c   # Complementary filter (roll, pitch, yaw)
│   │   │       ├── mag.c          # QMC5883P magnetometer driver
│   │   │       ├── dshot.c        # DSHOT300 direct DMA output
│   │   │       ├── pid.c          # PID controller
│   │   │       ├── flare.c        # Motor mixing, arming logic
│   │   │       ├── rc.c           # USART2 interrupt-driven RC receiver
│   │   │       ├── gps.c          # UBX NAV-PVT parser, USART3 DMA ring buffer
│   │   │       └── stm32h7xx_it.c # IRQ handlers (incl. DSHOT TC callback)
│   │   ├── cmake/stm32cubemx/     # CubeMX-generated CMake support
│   │   ├── CMakeLists.txt
│   │   └── build/Debug/           # Build output (gitignored)
│   ├── esp32_quad/                # ESP-NOW quad-side firmware (PlatformIO)
│   │   ├── src/main.cpp           # ESP-NOW RX → UART bridge to STM32 ✅ flashed
│   │   ├── extra_script.py        # SCons path injection for shared header
│   │   └── platformio.ini
│   └── esp32_remote/              # ESP-NOW remote firmware (PlatformIO)
│       ├── src/main.cpp           # 50Hz transmitter stub — pending wiring
│       ├── extra_script.py        # SCons path injection for shared header
│       └── platformio.ini
├── config/
│   └── esc_blheli_setup.ini       # BLHeli_S ESC configuration backup
└── docs/
    ├── README.md
    ├── architecture.md
    ├── hardware-notes.md
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

### STM32 (working directory: `firmware/fc`)

**Build:**
```powershell
cmake --build --preset Debug
```

**Flash:**
```powershell
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD freq=100 reset=HWrst -w build/Debug/fc.elf -rst
```

### ESP32 (working directory: `firmware/esp32_quad` or `firmware/esp32_remote`)

**Build:**
```powershell
C:\Users\alexg\.platformio\penv\Scripts\platformio.exe run --environment arduino_nano_esp32
```

**Flash** (double-tap RST first to enter DFU mode):
```powershell
C:\Users\alexg\.platformio\penv\Scripts\platformio.exe run --target upload --environment arduino_nano_esp32
```

**Serial monitor:**
```powershell
C:\Users\alexg\.platformio\penv\Scripts\platformio.exe device monitor --environment arduino_nano_esp32 --baud 115200
```

---

## Current PID Gains

| Axis  | kp  | ki  | kd   | int_limit | out_limit |
|-------|-----|-----|------|-----------|-----------|
| Roll  | 0.5 | 0.0 | 0.10 | 20.0      | 150.0     |
| Pitch | 0.5 | 0.0 | 0.10 | 20.0      | 150.0     |
| Yaw   | 1.2 | 0.0 | 0.05 | 20.0      | 300.0     |

### Gain Tuning Decision Table

| Observation | Action |
|---|---|
| Yaw spin >500 °/s at hover throttle | Check motor torque balance — likely physical cause |
| Yaw drift ~1 °/s at hover | Add ki_yaw = 0.01 |
| Motors M1/M3 >> M2/M4 at hover | M1 weak — replace motor |
| Flip above ~43% throttle | Motor saturation + imbalance; do not push above 40% until M1 replaced |
| Roll/pitch oscillation after motor replacement | Re-evaluate kp; may need slight reduction |

---

## Magnetometer Notes

The QMC5883P on the HGLRC M100-5883 module is wired to I2C1 (PB6/PB7).

### Hard-iron calibration (completed)

```c
offset_x = -251.0f;
offset_y =   41.5f;
```

Applied in `main.c` before fusion. Yaw stable at ~161–165° stationary.

---

## RC Link — FLARE Protocol

### Packet format (`firmware/shared/flare_protocol.h`)

| Byte(s) | Field      | Type      | Notes                            |
|---------|------------|-----------|----------------------------------|
| 0       | magic      | uint8_t   | `0xFA` — sync byte               |
| 1–2     | throttle   | uint16_t  | 1000–2000 (1000 = min)           |
| 3–4     | roll       | uint16_t  | 1000–2000 (1500 = center)        |
| 5–6     | pitch      | uint16_t  | 1000–2000 (1500 = center)        |
| 7–8     | yaw        | uint16_t  | 1000–2000 (1500 = center)        |
| 9       | armed      | uint8_t   | 0 = disarmed, 1 = armed          |
| 10      | mode       | uint8_t   | 0 = angle, 1 = acro              |
| 11–12   | reserved   | uint8_t[2]| Zero-padded, future use          |
| 13      | checksum   | uint8_t   | XOR checksum over bytes [0–12]   |

Total: **14 bytes**. Shared header used by both ESP32 and STM32 firmware.

### STM32 RC receiver (rc.c / rc.h)

- USART2, 115200 8N1, PA3 = RX
- Direct register RX: `SET_BIT(USART2->CR1, USART_CR1_RXNEIE_RXFNEIE)`
- Sync recovery: discards bytes until magic is seen
- XOR checksum validated on complete 14-byte frame
- `RC_GetPacket()` — returns new packet to main loop (clears flag)
- `RC_IsHealthy()` — returns 1 if packet received within `RC_TIMEOUT_MS` (250ms)
- USART2 IRQ priority: 8,0 (below DMA1_Stream0 at 0,0)

### ESP32 quad-side firmware (esp32_quad/src/main.cpp) ✅ flashed

- ESP-NOW receiver, Station mode
- MAC address: **`D4:E9:F4:E6:E9:90`** ← updated after reflash (flash corruption
  during ST-Link programming changed MAC from `D4:E9:F4:E8:10:74`)
- `on_packet_received()` callback: validates magic + length + checksum, forwards
  raw 14-byte packet over UART to STM32
- UART: GPIO17 (TX) → STM32 PA3, 115200 baud

### ESP32 remote-side firmware (esp32_remote/src/main.cpp) 🟡 built, pending wiring

- ESP-NOW transmitter targeting `D4:E9:F4:E6:E9:90`
- 100Hz transmit loop, ADC stick read, deadband, XOR checksum
- Pin assignments placeholders — update once hardware is wired
- Arming and mode via `INPUT_PULLUP` toggle switches

### Shared header path resolution (Windows PlatformIO)

Both ESP32 projects use `extra_script.py` (SCons pre-build hook) to inject
the absolute path to `firmware/shared/` into `CPPPATH`:

```python
shared_path = os.path.abspath(
    os.path.join(env["PROJECT_DIR"], "..", "shared")
)
env.Append(CPPPATH=[shared_path])
```

`platformio.ini` references it via `extra_scripts = pre:extra_script.py`.
Relative `build_flags = -I` paths do not resolve correctly on Windows.

---

## DSHOT300 Configuration

### Timing (@ 192MHz SYSCLK, TIM4 ARR = 639)

| Symbol     | Value  | Meaning                   |
|------------|--------|---------------------------|
| Bit period | 3.33µs | 300 kbps                  |
| BIT_1      | 480    | 75% duty cycle — logic 1  |
| BIT_0      | 240    | 37.5% duty cycle — logic 0|
| Idle       | 0      | 0% duty — idle-LOW (Bluejay) |

### Key implementation facts

- **No HAL DMA burst API** — `HAL_TIM_DMABurst_WriteStart` ignores NDTR on
  STM32H7 and runs continuously. Direct DMA register programming used instead.
- **TIM4->DCR:** DBA=13 (CCR1 offset), DBL=3 (4 transfers per burst)
- **Buffer:** `dshot_buf[17][4]` uint32_t at `0x24000000` (D1 AXI SRAM)
- **Idle-LOW:** `DSHOT_CCR_IDLE = 0` — required for Bluejay firmware
- **Bluejay CRC:** non-inverted: `(v ^ (v>>4) ^ (v>>8)) & 0x0F`
- **ESC keepalive:** `DSHOT_SendThrottle(0,0,0,0)` called every loop iteration
- **DMA1_Stream0 IRQ priority:** 0,0 (highest)

---

## Sensor Fusion

A complementary filter runs at 100Hz producing roll, pitch, and yaw in degrees.

### Algorithm

```
// Roll and pitch — accel reference (with mounting offsets applied)
roll_accel  = atan2(ay_g, az_g)  × (180/π) - IMU_ROLL_OFFSET
pitch_accel = atan2(-ax_g, az_g) × (180/π) - IMU_PITCH_OFFSET

// Gyro integration
roll_gyro  = roll_prev  + gx_dps × dt
pitch_gyro = pitch_prev + gy_dps × dt
yaw_gyro   = yaw_prev   + gz_dps × dt

// Complementary filter — roll and pitch
roll  = α × roll_gyro  + (1−α) × roll_accel
pitch = α × pitch_gyro + (1−α) × pitch_accel

// Yaw — gyro only until mag-fused yaw hold implemented
yaw = yaw_gyro   (drifts over time)
```

### Parameters

| Parameter | Value  | Notes                                           |
|-----------|--------|-------------------------------------------------|
| α (alpha) | 0.96   | Roll/pitch gyro weight                          |
| dt        | 0.01s  | Matches 10ms loop tick                          |
| IMU_ROLL_OFFSET  | +0.27° | Bench-calibrated mounting offset       |
| IMU_PITCH_OFFSET | −3.80° | Bench-calibrated mounting offset       |
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
[MAG] chip ID = 0x80
[MAG] QMC5883P ready
[FLARE] PID controller ready
[RC] init status=0 (0=OK)
[RC] USART2 receiver ready
[GPS] USART3 DMA receiver ready
[GPS] waiting for fix (may take 30-120s outdoors)...
[CAL] Hold still — levelling IMU (2s)...
[CAL] level offset: roll=X.XX  pitch=XX.XX
[IMU] starting 100Hz loop
```

100Hz data stream:
```
A:<ax> <ay> <az>  G:<gx> <gy> <gz>  R:<roll>  P:<pitch>  Y:<yaw>  RC:<OK|LOST>
```

1Hz GPS print:
```
[GPS] fix=<0|1> sats=<n> lat=<deg> lon=<deg> alt=<ft> spd=<mph>
```

---

## ST-Link V2 Clone Notes

Firmware: V2J37S7. Known limitations:
- GUI connection in STM32CubeProgrammer fails — use CLI only
- ITM SWO non-functional
- VS Code F5 flashing non-functional
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
- Wipe MPU region config from `MPU_Config()` — always restore Region 1
  (AXI SRAM at `0x24000000`) after regen
- Wipe NVIC lines for USART2 — must be kept inside USER CODE blocks

Always inspect `main.c` after any CubeMX regeneration before building.

---

## Known Issues & Gotchas

- **SD/IMU SPI sharing:** SD writes on SPI1 can cause single-sample BMI323 accel
  dropouts. Mitigation: last-known-good accel guard in fusion loop.
- **f_printf float support:** FatFS `f_printf` has no `%f` — all floats scaled
  to integers permanently. Never attempt `%f` in log writes.
- **GPIO43/44 on Nano ESP32:** Must never be assigned to hardware UART — corrupts
  USB Serial/JTAG stack unrecoverably without chip erase.
- **ESC arming window:** DSHOT 0 frames must reach ESC within ~300ms of power-on.
  Arm immediately after `DSHOT_Init()`, before IMU init delays.
- **M1 motor weakness:** M1 (Front-Left, CCW) produces less thrust than M3 at
  equivalent DSHOT values. Causes persistent left roll bias under aerodynamic load.
  Replacement required before stable hover is achievable.

---

## Commit Convention

```
<type>(<scope>): <description>

Types:  feat, fix, tune, test, docs, refactor, chore
Scopes: fc/imu, fc/fusion, fc/mag, fc/pid, fc/dshot, fc/main, fc/rc, fc/gps,
        esp32/quad, esp32/remote, shared, docs, control
```

Examples:
```
feat(fc/gps): replace NEO-6M NMEA parser with UBX binary for M100-5883
fix(fc/imu): apply IMU mounting offset correction in complementary filter
tune(control): increase yaw kp and output limit for authority at hover
docs: update README for session 22 — flight testing progress
```