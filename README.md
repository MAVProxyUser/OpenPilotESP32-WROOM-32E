# OpenPilotESP32

An ESP32 port of the NinjaPilot flight code: a slim, rate-mode-only target for
a bare **ESP32-WROOM-32E** module.

This project lives **outside** the NinjaPilot tree on purpose. It consumes that
tree's flight code and does not modify it — the handful of upstream changes a
HAL port unavoidably needs are kept as a patch you apply and revert on demand.

## Status

**Flies the whole control chain on the bench.** SparkFun ESP32 Thing Plus
(ESP32-D0WD-V3 rev 3.0), ESP-IDF v5.3.2 / GCC 13.2, built on macOS arm64.
Both cores are in use: every flight task is pinned to core 1, WiFi and lwIP
stay on core 0.

- ICM-20602 at 500 Hz over SPI with a data-ready interrupt; complementary
  filter attitude (`AttitudeState` tracks tilt correctly, both signs)
- Spektrum DSMX satellite (SPM9745) live on RX1; all channels calibrated,
  including per-channel reversal, via `tools/rc_check.py --apply`
- Arms from the transmitter (yaw-right hold), stabilizes, spins motors on the
  QuadX mixer, disarms; verified end to end with `tools/motor_watch.py`
- Settings persist across power cycles: `PIOS_FLASHFS` on NVS (its own
  partition; WiFi credentials live separately in the default NVS partition)
- WiFi telemetry: UAVTalk over TCP :9000, UDP discovery beacon on :9999,
  credentials provisioned offline by `tools/wifi_setup.py`. Modem power save
  is off -- deterministic latency beats standby current on a flight board
- Motor idle points come from the wizard's output-calibration page over
  WiFi (battery power, no USB). True ESC range calibration is the
  BOARD_ESC_CAL boot mode: flash it over USB with no battery, unplug,
  connect the battery, and the board boots straight into the max-then-min
  sequence inside the ESCs' calibration window
- The NinjaPilot GCS is fully board-aware: the Setup Wizard recognises
  0x1202, locks input to the Spektrum satellite, draws the Thing Plus in
  its connection diagram, and never offers ESC range calibration (it
  needs the controller alive before the ESCs power, impossible on a
  single-rail board -- output calibration is offered on network links
  instead). The vehicle, output and attitude config pages all carry the
  real pin names and the CC-style calibration screens; the hardware tab
  is a reference card. The firmware reports a full version blob (git
  hash, date, UAVO-set sha1) and honors the GCS's IAP reboot sequence,
  so post-save reboots happen automatically instead of ending in a
  "Reboot failed" dialog
- Telemetry serves TCP and UDP on port 9000; UDP is preferred (a connected
  TCP client arms lwIP's 250 ms tcp fast timer -- the residual scheduler
  stall source; a UDP client never starts it). The UDP stream stays with
  its holder (a new sender takes over only after 3 s of holder silence),
  and the flight view is genuinely live: AttitudeState at 25 Hz,
  ManualControlCommand at 4 Hz
- GCS connects are cheap: metaobjects fetch lazily on first use instead of
  at connect (157 -> 46 objects), and a SettingsGeneration token lets
  reconnects skip refetching unchanged settings entirely (down to ~11)
- The stabilization loop watchdog judges wall-clock pass gap (warn 12 ms /
  critical 30 ms) instead of counting missed samples, so the characterized
  ~3.6 ms scheduler stalls no longer blink STAB at an idle bench
- 60 s characterization with WiFi telemetry active, full flight stack:
  0 Attitude alarm events, 7 Stabilization warnings/min, CPU 34 %

**Not yet done:** no baro / mag / GPS (rate and attitude flight only), no
DShot, no flight testing off the bench.

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
- `uavtalk.c` — the receive state machine's checksum state read the CRC
  byte without checking the feed buffer still had one. A frame whose
  header+payload ends exactly on a receive-chunk boundary (118-byte
  ActuatorSettings against a 128-byte telemetry read) consumed a stale
  byte as its CRC and was dropped with no NACK, on every attempt. Every
  other state already guarded against feed starvation.
- `attitude.c` — the ADXL345-only read path was not guarded by
  `PIOS_INCLUDE_ADXL345` (every other use of that driver in the file is), and
  the task stack was hardcoded at 540 bytes with no per-board override.

The GCS entries make it talk to an ESP32 at all: deassert DTR/RTS on open (they
are wired to EN/RESET on these adapters), and make the auto-connect preference
overridable instead of hardcoded to UDP — default behaviour is unchanged.

`patches/mpu6000-i2c-and-icm20602.patch` is separate and **not applied**: it
adds an I2C transport to the MPU6000 driver, for breakouts that do not bring
SDO/AD0 out to a pin and so cannot do 4-wire SPI.

## Simulation twin: fw_simwroom

The flight tree gains a posix target that is this board, minus the silicon:
`flight/targets/boards/simwroom/` builds `fw_simwroom.elf`, a native process
that runs the SAME control stack as the hardware — the CC-style Attitude
complementary filter, the same module set (Receiver, ManualControl,
Stabilization, Attitude, Actuator, Telemetry), no Revo estimator, no nav
stack. It exists so a change can be flown in Gazebo before it is flown in the
back yard.

What makes it a twin rather than another simposix:

- **Same board identity.** It reports board id 0x1202 (type 0x12 rev 0x02)
  through FirmwareIAPObj, with a real "OpFw" description blob (git hash,
  build time, UAVO-set sha1), so the GCS gives it the same config screens,
  the same silkscreen pin labels and the same version check as the hardware.
- **Same IMU contract.** attitude.c compiles down the same
  `PIOS_INCLUDE_ICM20602` branch the ESP32 build takes. The posix driver
  (`flight/pios/posix/pios_icm20602_sim.c`) registers as
  `PIOS_ICM20602_Driver` and advertises the REAL board's scale factors
  (±2000 dps → 1/16.4, ±8 g → g/4096, from board_hw_defs.c), so calibration
  settings behave identically. Sensor data arrives as GyroSensor/AccelSensor
  UAVObjects — exactly what the Gazebo bridge publishes — and is packed into
  the same int16 queue records the hardware driver produces.
- **Same reboot protocol.** The GCS IAP sequence (1122/2233/3344) ends the
  process cleanly instead of restarting a chip.

Build and try it (in the NinjaPilot tree, via the space-free symlink):

```bash
cd /tmp/njp && make ROOT_DIR=$PWD fw_simwroom
build/fw_simwroom/fw_simwroom.elf   # binds UAVTalk on UDP :9000
```

Verified: boots, hands the GCS/pyuavtalk board id 0x1202 and the version
blob, streams AttitudeState at 25 Hz, and integrates streamed GyroSensor
data through the complementary filter (a 20 deg/s roll feed rolls the
attitude, accel pulls it level again — the filter behaving as on hardware).

Not wired up yet: the Gazebo bridge itself is Revo-flavored (RevoSettings,
PathFollower missions); flying the twin in Gazebo needs a stabilized-flight
harness mode that speaks only what this board has. That is the next step,
not a limitation of the target.

## Hardware

3.3 V only. The WROOM-32 has **no USB peripheral** — the USB socket is a
CP2102/CH340 UART bridge, so serial UAVTalk runs over UART0 through it (or
skip the cable entirely and use WiFi telemetry).

The deployed board is a **SparkFun ESP32 Thing Plus**; pins below are its
silkscreen names where it has them.

| Function | GPIO | Notes |
| --- | --- | --- |
| UART0 TX / RX | 1 / 3 | serial UAVTalk via the USB bridge |
| SPI3 SCLK / MISO / MOSI | 5 / 19 / 18 | sensor bus |
| IMU CS | 14 | plain GPIO, driven by PiOS |
| IMU INT (data ready) | 34 | input-only pin, deliberately |
| DSM satellite in | 16 (RX1) | UART2, Spektrum serial stream |
| Motor 1–4 | 15, 33, 27, 12 | MCPWM quad X: NW, NE, SE, SW |
| LED | 13 | slow blink disarmed, fast strobe armed |
| BOOT button (0) | 0 | hold ~3 s at power-up: erase stored settings |

GPIO12 (Motor 4) is MTDI, a flash-voltage strapping pin; using it as an
output required burning the `XPD_SDIO_FORCE` eFuse (one-way, per-chip) so
the strap is ignored. PPM input exists in the tree but is compiled out:
its RMT receiver on an unconnected pin collects coupled noise edges, and
servicing them periodically starved the gyro interrupt. Re-enable only
with a real PPM source wired.

### Power — the one hard rule

**Never connect the flight battery and USB at the same time.** The ESC/BEC
5 V line feeds VUSB on this board, so two supplies end up on one rail.
Everything that needs motors powered (RF motor calibration, the wizard's
ESC/output calibration over WiFi) is designed to run on battery power with
no USB attached; everything on USB runs with motors unpowered.

There is no firmware interlock for this, deliberately: the only signal a
detector could read (the CP2102 waking up on the VUSB rail) is also
energized by the BEC in the flight configuration, so it cannot tell a USB
cable from battery power and would block arming in flight. The rule is
procedural.

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

## Known gaps and characterized behavior

- **No baro / mag / GPS.** Rate and attitude flight only; no altitude hold or
  navigation. No DShot (MCPWM speaks plain PWM; OneShot requests from the GCS
  wizard are clamped to 400 Hz PWM).
- **Periodic ~4 ms scheduler stalls exist and are tolerated, not eliminated.**
  lwIP timer work on core 0 collides with flight tasks through the FreeRTOS
  SMP kernel lock (~2/s idle, ~4/s with a TCP telemetry client). Sensor
  samples are queued, not lost; the attitude consumer waits 5 sample periods
  before alarming. Unicore builds had the same stalls — the tick froze inside
  the windows, so nothing could see them.
- **WiFi is a bench feature.** Erase credentials before flying
  (`tools/wifi_setup.py erase`); the radio's task load is characterized but
  the airframe RF environment is not.
- `make gcs` cannot work on macOS — see SKILL.md for the qmake route.

## Bench bring-up, as it actually went

1. Console banner, LED heartbeat, GCS link over UART0
2. ICM-20602 answers `WHO_AM_I` 0x12; sensors sane at rest
3. `AttitudeState` level and tracking both signs — the leveling test
4. DSM satellite streaming; channels mapped, reversed and calibrated
   (`tools/rc_check.py`, `tools/rc_calibrate.py`, `tools/rc_monitor.py`)
5. Arm from the transmitter; motors on the QuadX mixer with props off
   (`tools/motor_watch.py` shows commanded vs actual per channel)
6. Motor idle points set with the wizard's output calibration over WiFi
7. WiFi telemetry on; 60 s characterization with the full stack live
