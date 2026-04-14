# FLARE — Hardware Notes

## STM32H723ZGT6

- Cortex-M7 @ 550MHz, double-precision FPU
- 1MB Flash, 564KB RAM, 144-pin LQFP
- **DMA / cache coherency:** Always call `SCB_CleanDCache` before DMA TX
  and `SCB_InvalidateDCache` after DMA RX. Forgetting this causes silent
  data corruption that is extremely hard to debug.

## BMI323 IMU (Arvian Breakout Board)

- **Interface:** SPI1 — do NOT use I2C (too slow for high-rate PID loops)
- No I2C/SPI selection jumper on the Arvian breakout — SPI mode is
  selected automatically on the first CS transaction
- VCC is 3.3V only — do NOT connect to 5V rail
- CS pin (CSB) is active-low; drive low before SPI transaction, high after
- INT1 signals data-ready — use this to trigger IMU reads (do not poll)
- INT2 — leave unconnected for now
- Soft-mount the FC stack. Motor vibration aliasing into the IMU is the
  leading cause of instability in DIY flight controllers. Use O-rings or
  foam standoffs.
- WHO_AM_I register: 0x00, expected response: 0x43 — **confirmed ✅**
- SPI read protocol requires one dummy byte after the address byte before
  valid data begins (3-byte transaction total for a 16-bit register read)

### Confirmed SPI Wiring (FK723M1 pin labels)

| BMI323 Breakout | Wire Color | FK723M1 Pin |
|-----------------|------------|-------------|
| VDD             | White      | 3.3V        |
| GND             | Gray       | GND         |
| SCK             | Brown      | A5          |
| SDX (MOSI)      | Blue       | A7          |
| SDO (MISO)      | Green      | A6          |
| CSB             | Orange     | B0          |
| INT1            | Purple     | B1          |
| INT2            | —          | Unconnected |

### CubeMX SPI1 Configuration

| Parameter        | Value           |
|------------------|-----------------|
| Mode             | Full-Duplex Master |
| Hardware NSS     | Disabled (manual CS via GPIO) |
| Frame Format     | Motorola        |
| Data Size        | 8 Bits          |
| First Bit        | MSB First       |
| Prescaler        | 16 (~2.75 MHz)  |
| CPOL             | Low             |
| CPHA             | 1 Edge          |

## BLHeli_S 30A ESCs

- Use DSHOT300 to start (safe timer config on H723)
- DSHOT is digital — no ESC calibration step required

## ESP-NOW Link

- Peer-to-peer, no router needed
- ~1–2ms latency, reliable to 200–300m outdoors
- ESP32 on the quad acts as a dumb UART bridge — STM32 does all parsing

## Power System

- QWinOut PDB XT60 with built-in 5V BEC (2A) and 12V BEC (500mA @ 4S only)
- 12V BEC is not usable on 3S — leave unconnected until 4S
- 5V BEC powers FK723M1 and Nano ESP32
- Use XT60 connectors on the main power path
- Always charge in a LiPo safety bag