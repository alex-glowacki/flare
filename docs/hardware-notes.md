# FLARE — Hardware Notes

---

## STM32H723ZGT6 (FK723M1 board)

- Cortex-M7 @ 96 MHz SYSCLK (HSI PLL: PLLM=4, PLLN=12, PLLP=1)
- 1MB Flash, 564KB RAM, 144-pin LQFP
- **Clone ST-Link V2 (V2J37S7):** GUI connection in CubeProgrammer fails — use
  CLI only with `freq=100 reset=HWrst` flags. ITM SWO non-functional.

### Pin Assignments

| Function     | Pin  | Notes                        |
|--------------|------|------------------------------|
| SPI1 SCK     | PA5  |                              |
| SPI1 MISO    | PA6  |                              |
| SPI1 MOSI    | PA7  |                              |
| IMU CS       | PB0  | Active-low, manual GPIO      |
| IMU INT1     | PB1  | Not yet used                 |
| USART1 TX    | PA9  | CP2102 RX                    |
| USART1 RX    | PA10 | CP2102 TX                    |
| User LED     | LED_USER (PG7) |                   |

### CubeMX SPI1 Configuration (confirmed working)

```c
hspi1.Init.Mode                    = SPI_MODE_MASTER;
hspi1.Init.Direction               = SPI_DIRECTION_2LINES;
hspi1.Init.DataSize                = SPI_DATASIZE_8BIT;
hspi1.Init.CLKPolarity             = SPI_POLARITY_LOW;
hspi1.Init.CLKPhase                = SPI_PHASE_1EDGE;
hspi1.Init.NSS                     = SPI_NSS_SOFT;
hspi1.Init.BaudRatePrescaler       = SPI_BAUDRATEPRESCALER_16;  /* 6 MHz */
hspi1.Init.FirstBit                = SPI_FIRSTBIT_MSB;
hspi1.Init.NSSPMode                = SPI_NSS_PULSE_DISABLE;
hspi1.Init.FifoThreshold           = SPI_FIFO_THRESHOLD_01DATA;
hspi1.Init.MasterKeepIOState       = SPI_MASTER_KEEP_IO_STATE_ENABLE;
hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
```

### STM32H7 SPI Gotchas

- **NSSP Mode** must be disabled and **MasterKeepIOState** must be enabled —
  otherwise the peripheral pulses CS between bytes even with manual CS control.
- **3-byte transactions are unreliable.** The STM32H7 SPI peripheral drops the
  3rd byte on 3-byte transfers regardless of approach (HAL_SPI_Transmit,
  HAL_SPI_TransmitReceive, or direct TSIZE register access). Always use 4-byte
  transactions padded with a dummy byte.
- **RX FIFO does not drain on CS de-assert.** Call `SPI1_FlushRxFifo()`
  (disable + re-enable peripheral) between transactions to prevent stale data.
- **Soft reset mid-transaction** leaves the peripheral in an undefined state.
  Follow with `HAL_SPI_DeInit()` + `MX_SPI1_Init()` to recover cleanly.

---

## BMI323 IMU (Arvian Breakout Board — Amazon B0FG2ZFHNM)

- **Interface:** SPI (Mode 0, MSB first, up to 10 MHz — using 6 MHz)
- **VCC:** 3.3V — do NOT connect to 5V
- **CS:** Active-low, driven manually via GPIO

### Wiring (Arvian → FK723M1)

| Wire Color | Arvian Label | FK723M1 Pin |
|------------|--------------|-------------|
| Orange     | CSB          | PB0         |
| Brown      | SCX          | PA5 (SCK)   |
| Blue       | SDX          | PA7 (MOSI)  |
| Green      | SDO          | PA6 (MISO)  |
| Purple     | INT1         | PB1         |
| White      | VDD          | 3V3         |
| Gray       | GND          | GND         |

### SPI Protocol

**Read (4 bytes):**
```
TX: [addr | 0x80]  [0x00 dummy]  [0x00]  [0x00]
RX: [ignored]      [ignored]     [LSB]   [MSB]
```
Reconstruction: `(uint16_t)(rx[2] | (rx[3] << 8))`

For 1-byte registers (e.g. WHO_AM_I), rx[3] contains garbage — mask:
`who_am_i & 0x00FF`

**Write (4 bytes — padded):**
```
TX: [addr & 0x7F]  [data_LSB]  [data_MSB]  [0x00 dummy]
```
The BMI323 clocks in the first 3 bytes and ignores the trailing dummy.
4-byte padding is required due to the STM32H7 3-byte truncation bug above.

**Soft Reset:**
- Write `0xDEAF` (LSB-first: `0xAF, 0xDE`) to register `0x7E`
- Use a short HAL timeout (10ms) — the BMI323 resets mid-transaction
- Follow with 50ms delay, then `HAL_SPI_DeInit()` + `MX_SPI1_Init()`
- Two dummy CHIP_ID reads required to re-select SPI mode

### Initialization Sequence

1. Soft reset → 50ms delay → SPI reinit
2. Two dummy reads of CHIP_ID
3. Verify `WHO_AM_I & 0x00FF == 0x43`
4. Read ACC_CONF default (expect `0x0028` = suspend mode)
5. **Initialize Feature Engine** (required before mode bits in ACC_CONF/GYR_CONF
   will be accepted — writes are silently ignored without this step)
6. Write ACC_CONF = `0x4028` (continuous, 100Hz, ±8g)
7. Write GYR_CONF = `0x4048` (continuous, 100Hz, ±2000 dps)
8. Read back and verify both registers

### Feature Engine Initialization

```c
WriteReg(0x10, 0x0000);  /* FEATURE_IO0 — clear         */
WriteReg(0x12, 0x012C);  /* FEATURE_IO2 — startup config */
WriteReg(0x14, 0x0001);  /* FEATURE_IO_STATUS — trigger  */
WriteReg(0x40, 0x0001);  /* FEATURE_CTRL — enable        */
/* Poll 0x11 (FEATURE_IO1) bit[3:0] == 0x01 for ready    */
```

### Register Map (used registers)

| Register       | Address | Notes                              |
|----------------|---------|------------------------------------|
| CHIP_ID        | 0x00    | WHO_AM_I — low byte = 0x43         |
| ACC_DATA_X     | 0x03    | Burst read 6 bytes for X/Y/Z       |
| GYR_DATA_X     | 0x06    | Burst read 6 bytes for X/Y/Z       |
| ACC_CONF       | 0x20    | Default 0x0028, target 0x4028      |
| GYR_CONF       | 0x21    | Default 0x0028, target 0x4048      |
| FEATURE_IO0    | 0x10    |                                    |
| FEATURE_IO1    | 0x11    | Poll for feature engine ready      |
| FEATURE_IO2    | 0x12    |                                    |
| FEATURE_IO_ST  | 0x14    |                                    |
| FEATURE_CTRL   | 0x40    |                                    |
| CMD            | 0x7E    | Soft reset: write 0xDEAF           |

### Post-write Timing

Per BMI323 datasheet: minimum 2 µs idle between transactions.
At 96 MHz: 200 NOPs ≈ 2.08 µs.

---

## CP2102 USB-UART Adapter

| CP2102 | FK723M1  |
|--------|----------|
| TX     | PA10 (USART1 RX) |
| RX     | PA9  (USART1 TX) |
| GND    | GND      |
| VCC    | unconnected      |

- COM port: COM3
- Baud: 115200 8N1

---

## General Notes

- **Soft-mount the FC stack.** Motor vibration aliasing into the IMU is the
  primary cause of PID instability in DIY flight controllers. Use silicone
  grommets or O-ring standoffs — never hard-mount.
- **3.3V logic throughout.** BMI323, ESP32 UART, and STM32H7 GPIO are all
  3.3V compatible. Do not expose any signal line to 5V.