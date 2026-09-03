# LiteWing (ESP32-S3) — brushed nano quad port

> **The code described here is not in this repository, and not on `claude`.**
> It lives on the [`litewing`](https://github.com/MAVProxyUser/NinjaPilot-15.02.ninja/tree/litewing)
> branch of the **NinjaPilot** repo, because the work is a flight-tree board
> target (`flight/targets/boards/simlitewing/`), not an ESP-IDF project. This
> file sits on `main` here so the findings are discoverable instead of buried
> on a side branch.

Status 2026-09-03: **the posix twin builds, its output model and its control
loop are both verified, and the flight firmware compiles clean for ESP32-S3
silicon with zero source changes.** No LiteWing hardware has run any of it.

### The S3 compiles as-is

```bash
cd targets/esp32wroom/esp-idf
cmake -G Ninja -B build-s3 -DIDF_TARGET=esp32s3 \
      -DSDKCONFIG_DEFAULTS=sdkconfig.defaults -DSDKCONFIG=sdkconfig.s3 \
      -DPYTHON_DEPS_CHECKED=1 . && ninja -C build-s3
```

Configures and builds with **no errors and no source edits**: an 879,696-byte
image. That is the concrete version of the claim above — the S2 needed an LEDC
servo backend, core-count-aware task creation and a relocated pin map before it
would even build, and the S3 needed nothing.

It is emphatically **not** a flashable LiteWing firmware. It compiles; it has
never run. What it still carries:

- the **Thing Plus pin map**, so motors are on GPIO 15/33/27/12 instead of
  LiteWing's 5/6/3/4, and GPIO 26–32 are flash/PSRAM territory on S3 modules;
- the **console on GPIO 22/23, which do not exist on the S3 at all** —
  `SOC_GPIO_VALID_GPIO_MASK` in the IDF masks out bits 22–25;
- **brushless servo-pulse output**, not the brushed duty model below;
- the **ICM-20602 SPI** sensor path, not MPU6050 over I2C.

So the remaining work to a real LiteWing image is the pin map, the brushed LEDC
backend, an MPU6050 I2C transport and the console pins — not an architecture
fight.

## Why the S3 is portable when the S2 is not

From the IDF's own `soc_caps.h`, not from marketing:

| | ESP32 (flew) | ESP32-S2 | **ESP32-S3 (LiteWing)** |
| --- | --- | --- | --- |
| CPU cores | 2 | 1 | **2** |
| MCPWM | yes | **none** | **yes** |
| Native USB | no | yes | yes |
| Hardware UARTs | 3 | 2 | 3 |
| SRAM | 520 KB | 320 KB | 512 KB |

The dual core is the whole argument. This codebase pins every flight task to
core 1 and leaves WiFi and lwIP on core 0, and the characterised ~4 ms
scheduler stalls are tolerable only because of that split. The S2 has one core
and destroys it. The S3 keeps it, and keeps MCPWM as well, so the S3 is close
to a retarget-plus-pinmap where the S2 was a rewrite. See [ESP32-S2.md](ESP32-S2.md).

## The hardware

LiteWing V2.6.C, from CircuitDigest / Semicon Media. ESP32-S3-WROOM-1, ~45 g,
100 × 100 mm, 1S LiPo, four 720-size **coreless brushed** motors on 55/65 mm
props. Schematic: `jobitjoseph/LiteWing`, `hardware/LieWingV2.6.C`.

Pin map transcribed from the KiCad schematic, not the wiki (which does not
publish the motor pins):

| Function | GPIO |
| --- | --- |
| Motors MOT_1 / MOT_2 / MOT_3 / MOT_4 | 5 / 6 / 3 / 4 |
| MPU6050 (I2C0) SCL / SDA / INT | 10 / 11 / 12 |
| Battery sense ADC_BAT | 2 |
| LEDs blue / red / green | 7 / 8 / 9 |
| Buzzer BUZ1 / BUZ2 | 38 / 39 |
| I2C1 header (for a ToF module) SDA1 / SCL1 | 40 / 41 |
| SPI header (for a flow module) MOSI / CLK / MISO / CS | 35 / 36 / 37 / 42 |
| UART0 (CH340K bridge) | TXD0 / RXD0 |

Expansion header also brings out nine free GPIOs (1, 13, 15–20, 48), UART0,
both I2C buses, the flow SPI, 3V3 and VBUS.

**The base board carries exactly one sensor: the MPU6050.** The V2.6.C
schematic contains no PMW3901, VL53L1X, MS5611 or HMC5883 symbol — only bare
connectors (`Conn_01x06`, `Conn_01x09`) where such parts would attach. The
last two rows of that table are *header pinouts*, not fitted devices. The ToF
and optical flow live on a separate **LiteWing Drone Positioning Module**
(its own board and schematic in the same hardware repo); baro and mag would go
on I2C0 via the expansion header. Plan for a bare IMU and add from there.

## The real work: a brushed output model

Every other target here drives brushless ESCs, which want a 1000–2000 µs pulse
and an idle above their stop or they never spin. LiteWing drives coreless
motors through low-side **IRLML6344** N-channel MOSFETs. The gate takes a
**duty cycle**, 0–100 %, at roughly a 20 kHz carrier. There is no pulse width,
no arming ritual, no ESC range calibration.

So the range is reinterpreted rather than reused. `ChannelMin`/`Max` become
`0..1000` = 0.0–100.0 % duty; `ChannelNeutral` is 0 and
`MotorsSpinWhileArmed` is FALSE, because a brushed motor at zero duty simply
stops. On hardware the backend turns that number into an LEDC duty instead of
a pulse; the actuator pipeline above it is untouched.

That last part is worth more than convenience. It removes **by construction**
the failure that tipped the ESP32 quad onto its tail on 2026-09-02, where four
equal idle thrusts lifted the light (nose) end while the FC applied no
stabilisation at all because throttle was below zero. Here an armed,
throttle-down quad sits still.

Measured on the twin:

| stick | ActuatorCommand |
| --- | --- |
| armed, throttle down | `[0, 0, 0, 0]` |
| 50 % | `[469, 469, 469, 469]` — 46.9 % duty |
| 100 % | `[992, 992, 992, 992]` — 99.2 % duty |

The control loop drives that model in the right direction on both axes too —
tilt injected via the sensor stream at 50 % throttle, reading
`ActuatorCommand` (M1 NW, M2 NE, M3 SE, M4 SW):

| stimulus | ActuatorCommand | result |
| --- | --- | --- |
| level | `[456, 483, 456, 483]` | balanced |
| nose down | `[956, 983, 0, 0]` | front pair up |
| nose up | `[0, 0, 956, 983]` | rear pair up |
| roll left | `[708, 0, 204, 1000]` | left pair (M1/M4) up |
| roll right | `[0, 729, 1000, 236]` | right pair (M2/M3) up |

Each correction raises the dropped side, which is the whole game. The rail-to-
rail saturation is expected: nothing feeds vehicle dynamics back, so a sustained
20° error winds the integrator exactly as it does props-off on real hardware.

## Traps

- **`board_rev` must stay `0x02`.** `attitude.c` has
  `#define BOARDISCC3D (bdinfo->board_rev == 0x02)` and keys the MPU6000-family
  sensor path off it. Any other revision silently routes to the ADXL345 path,
  `updateSensors()` returns −1 forever and Attitude sits at Error with no
  sample ever reaching the filter. The board id is `0x1302`: `0x13` is
  LiteWing, `0x02` is a sensor-path selector, not a revision number.
- **The airframe defaults are written in two places.** A second block later in
  `PIOS_Board_Init` re-writes `ChannelMin/Neutral/Max` after
  `simlitewing_apply_default_airframe()` runs. It silently overwrote the
  brushed 0/0/1000 back to servo 1000/1000/2000, and the first probe duly
  reported servo values on a brushed board. Both sites must agree.
- **GPIO 35/36/37 are the octal-PSRAM pins** on ESP32-S3-WROOM-1 variants that
  carry PSRAM — the schematic brackets them "PSRAM" for exactly this reason —
  and this board routes them to the SPI header the flow module plugs into. A
  part with octal PSRAM cannot drive that header at all. Check the module
  suffix; do not enable PSRAM.
- **Which physical corner is MOT_1 is NOT in the schematic.** The netlist gives
  gate-to-GPIO only; the corner assignment is on the PCB silkscreen. The mixer
  in the twin is the stock OpenPilot QuadX convention as an explicit
  **placeholder**. Verify it against the board before anything spins — a motor
  order or IMU frame that disagrees with the mixer is positive feedback on both
  axes, and it is what flipped the ESP32 quad twice on 2026-09-01.

## What already exists, and what does not

Already in the flight tree, so these are *enable-and-wire*, not *write*:

- `flight/pios/common/pios_ms5611.c` — MS5611 barometer, and its init is
  `PIOS_MS5611_Init(cfg, int32_t i2c_device)`, which takes an I2C handle
  directly.
- `flight/pios/common/pios_hmc5x83.c` — HMC5883/5983 magnetometer family.
- `pios_i2c.c` already exists in the ESP32 port.

The LiteWing expansion header lists MS5611 and HMC5883 as intended I2C0
devices. So altitude hold and magnetic heading are reachable on this airframe
in a way they are not on the Thing Plus build, which is rate-and-attitude
only — provided you fit the parts, because nothing but the MPU6050 is on the
base board.

Not in the tree, and would need writing: **VL53L1X** (ToF) and **PMW3901**
(optical flow) — both of which are on the separate Positioning Module anyway,
so they are two purchases and two drivers away, not one `#define`.

Also still to do: an **MPU6050-over-I2C** transport. The driver logic largely
survives — our MPU/ICM driver already accepts `WHO_AM_I` 0x68, which is the
MPU6050, and the register map is MPU6000-family for everything it touches —
but the existing path is SPI.

**I2C budget caution.** Baro and mag would share I2C0 with the IMU. A 15-byte
read at 400 kHz costs ~390 µs, about 19 % of a 500 Hz period, and MS5611
conversions run ~10 ms. Budget the rates (baro 20–50 Hz, mag 50–100 Hz, both
normal) — and note this is precisely why Revo and CC3D put their IMU on SPI.

## Next steps

1. Confirm the MOT_n → physical corner mapping from the PCB.
2. Confirm the module's PSRAM suffix before trusting the flow SPI pins.
3. MPU6050 I2C transport.
4. Re-tune Bank1 for 45 g — the current gains are 4-inch-class and meaningless
   here. Do it on the twin.
5. Only then an ESP-IDF target for the S3.
