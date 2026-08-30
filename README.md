# OpenPilotESP32

An ESP32 port of the NinjaPilot flight code: a slim, rate-mode-only target for
a bare **ESP32-WROOM-32E** module.

This project lives **outside** the NinjaPilot tree on purpose. It consumes that
tree's flight code and does not modify it — the handful of upstream changes a
HAL port unavoidably needs are kept as a patch you apply and revert on demand.

## Status

**Runs on hardware. The GCS talks to it over serial.** Verified on an
ESP32-D0WD-V3 (rev 3.0) with ESP-IDF v5.3.2 / GCC 13.2 on macOS arm64.

- 75-second soak, all six modules: 0 resets, 0 panics, 0 asserts
- `pyuavtalk`: framing, handshake, object read and object write all verified
- NinjaPilotGCS over serial UART: NO LINK cleared, TELEMETRY green, 0 CRC errors
- ~303 KB image, 29% of the 1 MB app partition, ~280 KB heap free

**Not yet done:** no IMU has been connected, so nothing on the sensor, attitude
or control path has ever executed. No servo output has been scoped. Bring-up
starts at step 4 of the list at the end of this file.

```
.flash.text   168246      code executing XIP from flash
.iram0.text    60319
.dram0.data     8444 + _uavo_handles
.dram0.bss      7216
```

## Why this is worth doing

CopterControl runs the same flight code at `PIOS_SENSOR_RATE 500.0f` on an
STM32F103CBT: 72 MHz Cortex-M3, 20 KB SRAM, **no FPU** — every float software
emulated. Its firmware bank is 116 KB.

A WROOM-32E is 240 MHz with a hardware single-precision FPU and 520 KB of SRAM.
The build backs the premise up: **168 KB of `.flash.text`** on a part with
520 KB of SRAM, so the whole executable would fit on-chip with room to spare.
The XIP cache-thrash that caps ArduPilot's ESP32 port near 150 Hz is not the
constraint here — ArduPilot is pushing ~2 MB through the same cache.

## Layout

```
pios/esp32/            the PiOS backend (~2,200 lines): sys, delay, irq, led,
                       wdg, usart/COM, spi, i2c, servo (MCPWM), exti, ppm (RMT)
targets/esp32wroom/    board definition + the ESP-IDF project
tools/                 link check, patch apply/revert helpers
patches/               the flight-tree changes this port needs
CLAUDE.md              rules and traps — read before changing anything
SKILL.md               operational recipes (build, flash, talk to it)
```

## Quick start

```bash
tools/apply-ninjapilot-patch.sh /path/to/NinjaPilot-15.02.ninja
```

Then follow SKILL.md for generating UAVObjects, building, flashing and talking
to the board. `tools/revert-ninjapilot-patch.sh` puts the flight tree back.

## About the patch

A HAL port has to hook the architecture dispatch point, so `flight/pios/pios.h`
gains a third branch next to the existing `USE_SIM_POSIX` one. Everything in
the patch is additive and guarded, so coptercontrol / simposix / realposix
compile to the same code as before. Two entries are genuine upstream bug fixes
found by this work:

- `pios_callbackscheduler.c` — when a scheduler task for a priority already
  existed, `Create()` returned without updating `stackSize`, so the shared
  stack was whatever the *first* caller asked for. Now takes the maximum.
  **This does increase stack allocation on other targets.**
- `attitude.c` — the ADXL345-only read path was not guarded by
  `PIOS_INCLUDE_ADXL345` (every other use of that driver in the file is), and
  the task stack was hardcoded at 540 bytes with no per-board override.

The GCS entries make it talk to an ESP32 at all: deassert DTR/RTS on open (they
are wired to EN/RESET on these adapters), and make the auto-connect preference
overridable instead of hardcoded to UDP — default behaviour is unchanged.

`patches/mpu6000-i2c-and-icm20602.patch` is separate and **not applied**: it
adds an I2C transport to the MPU6000 driver, for breakouts that do not bring
SDO/AD0 out to a pin and so cannot do 4-wire SPI.

## Hardware

3.3 V only. The WROOM-32 has **no USB peripheral** — the USB socket on a devkit
is a CP2102/CH340 UART bridge, so UAVTalk runs over UART0 through it. There is
no USB HID or CDC on this chip; that arrived with the S3.

| Function | GPIO | Notes |
| --- | --- | --- |
| UART0 TX / RX | 1 / 3 | UAVTalk to the GCS, via the USB bridge |
| SPI3 SCLK / MISO / MOSI | 18 / 19 / 23 | sensor bus |
| IMU CS | 5 | driven by PiOS, not the peripheral |
| IMU INT (data ready) | 34 | input-only pin, deliberately |
| PPM in | 4 | decoded by RMT |
| Servo 1–6 | 25, 27, 33, 32, 22, 21 | MCPWM, quad X on 1–4 |
| LED | 2 | |
| IDF console TX | 13 | UART1 — kept off UART0 on purpose |

### IMU

Use an **ICM-20602**. It is MPU-6500 family, so it shares the MPU6000 register
map byte-for-byte in everything the driver touches — same 14-byte burst from
`ACCEL_XOUT_H`, same config registers, same scaling. The only change it needed
was widening the `WHO_AM_I` gate to accept 0x12 alongside 0x68 and 0x70.

Wire SDO to GPIO19 and **leave SA0 unconnected** — on Invensense parts AD0/SDO
is often one net internally, and grounding SA0 would short MISO.

Not recommended: the **ICM-20948** is a different architecture (banked
registers, `WHO_AM_I` 0xEA, mag behind an internal I2C master) and needs a new
driver. The **BNO08x** does its own sensor fusion on an onboard Cortex-M0+ and
speaks SHTP; it is excellent for robotics orientation and the wrong shape for a
flight controller that wants raw high-rate gyro for its own filter.

### SPI or I2C

| | pins | 15-byte sensor read | of a 500 Hz period |
| --- | --- | --- | --- |
| I2C @ 400 kHz | 2 | ~390 µs | ~19% |
| SPI @ 8 MHz | 4 | ~15 µs | ~0.8% |

I2C wins on pins, SPI on speed — by ~25×. Both work at 500 Hz. Prefer SPI
anyway: it is push-pull and deterministic, and has no equivalent of I2C bus
lockup, which is a classic source of intermittent sensor faults on airframes.

## Known gaps

- **No IMU has ever been attached.** Everything sensor-side is untested.
- **`HwSettings` requests are answered with a NACK** while every other settings
  object responds normally. A NACK means `UAVObjGetByID()` returned NULL, yet
  the object is compiled, linked, present in the handle table, has
  `UAVOBJ_INIT_hwsettings` defined, and its ID matches the GCS's exactly.
  Unresolved; needs the registration return value instrumented on hardware.
- **Settings are not persistent.** No `PIOS_FLASHFS` equivalent yet; the board
  boots to defaults every time. The `settings` partition is already reserved so
  adding NVS later will not move anyone's flash layout.
- No WiFi, no I2C peripherals, no baro/mag/GPS, no DShot.
- `make gcs` cannot work on macOS — see SKILL.md.

## First bring-up order

1. Console banner and free-heap line
2. LED heartbeat
3. GCS connect over UART0; UAVTalk objects populate
4. IMU: `WHO_AM_I` reads 0x12, boot alarm clears
5. `GyroSensor` / `AccelSensor` sane at rest (gyro ≈ 0, accel Z ≈ −9.81)
6. Gyro bias zeroed with the board still
7. `AttitudeState` Roll/Pitch level, and tracking when tilted — the leveling test
8. Servo output **with props off**, on a scope, before anything else
