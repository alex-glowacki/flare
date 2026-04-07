# FLARE — Architecture Overview

## System Block Diagram
[Hall Gimbals] ──► [ESP32 Remote]
│
ESP-NOW (2.4GHz)
│
[ESP32 Quad]
│
UART
│
[STM32H723 FC]
├── ICM-42688 (SPI)
├── DSHOT → ESC × 4
└── Barometer (future)

## Component Responsibilities

| Component         | Role                                              |
|-------------------|---------------------------------------------------|
| STM32H723ZGT6     | PID, sensor fusion, DSHOT motor output            |
| ICM-42688-P       | 6-axis IMU (gyro + accel) over SPI                |
| ESP32 (quad)      | ESP-NOW receive → UART bridge to STM32            |
| ESP32 (remote)    | Stick ADC read → ESP-NOW transmit, OLED display   |
| BLHeli_S 30A ESCs | Motor drive via DSHOT300                          |

## Development Phases

See `../README.md` for the full phase table.