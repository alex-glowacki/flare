# FLARE — Hardware Notes

## STM32H723ZGT6

- Cortex-M7 @ 550MHz, double-precision FPU
- 1MB Flash, 564KB RAM, 144-pin LQFP
- **DMA / cache coherency:** Always call `SCB_CleanDCache` before DMA TX
  and `SCB_InvalidateDCache` after DMA RX. Forgetting this causes silent
  data corruption that is extremely hard to debug.

## ICM-42688-P (IMU)

- SPI preferred (faster than I2C, needed for high sample rates)
- **Soft-mount the FC stack.** Motor vibration aliasing into the IMU is
  the leading cause of instability in DIY flight controllers. Use O-rings
  or foam standoffs.

## BLHeli_S 30A ESCs

- Use DSHOT300 to start (safe timer config on H723)
- DSHOT is digital — no ESC calibration step required

## ESP-NOW Link

- Peer-to-peer, no router needed
- ~1–2ms latency, reliable to 200–300m outdoors
- ESP32 on the quad acts as a dumb UART bridge — STM32 does all parsing

## Power System

- Start on 3S LiPo during development
- F450 integrated PDB does **not** regulate voltage — add a dedicated 5V BEC
- Use XT60 connectors on the main power path
- Always charge in a LiPo safety bag