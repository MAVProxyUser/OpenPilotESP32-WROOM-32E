# LiteWing (ESP32-S3) — brushed nano quad port

> **The code described here is not in this repository, and not on `claude`.**
> It lives on the [`litewing`](https://github.com/MAVProxyUser/NinjaPilot-15.02.ninja/tree/litewing)
> branch of the **NinjaPilot** repo, because the work is a flight-tree board
> target (`flight/targets/boards/simlitewing/`), not an ESP-IDF project. This
> file sits on `main` here so the findings are discoverable instead of buried
> on a side branch.

Status 2026-09-03: **the twin hovers in Gazebo on a physically-real LiteWing
model, its output model and control loop are verified, and the flight firmware
compiles clean for ESP32-S3 silicon with zero source changes.** No LiteWing
hardware has run any of it.

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

## It flies in Gazebo

`ground/gazebo_bridge/worlds/litewing.sdf` on the branch is generated from the
airframe's real numbers, not copied from the X3: 55 g, motors at ±35 mm for a
99 mm diagonal, `motorConstant` 2.4e-08 so a 720 coreless on a 55 mm prop makes
~22 g at 3000 rad/s — **thrust-to-weight 1.60**. `tools/litewing_bridge.py`
closes the loop: Gazebo's IMU becomes GyroSensor/AccelSensor, the firmware's
brushed `ActuatorCommand` drives the rotors, and only the collective is the
bridge's — every attitude correction comes from the firmware.

macOS needs the server and GUI as separate processes (gz-sim#44); run both
under `GZ_PARTITION` so another Gazebo on the machine is undisturbed.

```
  t     alt      vz     roll   pitch   ActuatorCommand
  1.0   0.015   +0.09    +0.0    +0.0   [692, 694, 692, 694]
  5.1   0.966   +0.06    +0.0    -0.0   [644, 644, 644, 644]
 11.2   1.038   +0.00    -0.0    -0.0   [647, 647, 647, 647]
 25.4   1.039   +0.00    +0.0    +0.0   [647, 647, 647, 647]
 39.5   1.039   +0.00    +0.0    +0.0   [646, 647, 648, 647]
```

Thirty-plus seconds at 1.039 m, `vz` 0.00, roll and pitch pinned at ±0.0°, all
four motors within ±1. The model predicts hover at **624** duty from first
principles; the aircraft hovers at **647** — agreement to 4 %, so the brushed
0..1000 duty model and the physics are consistent.

### The retune, measured rather than asserted

Bank1's stock 4-inch-class gains **do** diverge on a 55 g airframe: a
rock-steady ten-second hover, then the rails, reproducibly at t≈17 s. A nano
has a fraction of the inertia for comparable authority.

| | stock (4-inch) | LiteWing |
| --- | --- | --- |
| Roll/Pitch rate Kp | 0.0032 | **0.0012** |
| rate Ki | 0.0075 | **0.0022** |
| rate Kd | 0.00005 | **0.00002** |
| attitude Kp | 3.2 | **2.5** |

`LITEWING_STOCK_GAINS=1` reproduces the divergence.

### Landing: command a descent, never cut throttle

The first version dropped throttle to zero when the flight timer expired. From
1.5 m that is a free fall — about 0.55 s, arriving near **5.4 m/s** — and it
put the airframe on its back, where it then sat unable to take off (roll
−180°, motors pushing it into the ground) until the pose was reset. On real
hardware that is how arms break.

The bridge now walks the altitude setpoint down at 0.35 m/s, lets the same PD
fly it, and only cuts power once the airframe is resting and still, then
disarms:

```
  land  0.5  alt  1.483  vz -0.04   land  4.6  alt  0.735  vz -0.27
  land  2.0  alt  1.264  vz -0.20   land  6.7  alt  0.113  vz -0.20
  land  3.6  alt  0.944  vz -0.32   land  7.7  alt  0.006  vz -0.00
  touchdown at 0.006 m, vz +0.00 m/s   disarmed: Disarmed
```

### Flight trail

Translucent cyan **CYLINDER** segments via the `/marker` service, so the flown
path persists in the scene and take-off is visible after the fact. Cylinders,
not `LINE_STRIP`: gz renders LINE_STRIP at 1 px regardless of scale and it is
invisible at scene distance — the same lesson `gazebo_bridge.py` already
records. 40 mm tubes suit a 100 mm airframe, and the `/marker` service replies
with `Empty`, not `Boolean`.

### Three traps that each cost an afternoon

**1. `ActuatorCommand` telemetry is periodic at 1000 ms.** Publishing the
cached value drove Gazebo's rotors at 1 Hz against 1 kHz physics: the airframe
pogoed floor-to-6 m and flipped. It reads exactly like a tuning problem and is
not one. Poll the object explicitly at loop rate.

**2. A paused world looks exactly like a firmware failure.** A GUI connecting
to a running server can leave the world paused, and a paused world steps no
physics and publishes no IMU — so the aircraft sits motionless at its spawn
pose while a naive bridge happily prints an altitude column. The tell is the
number: a paused model sits at its **spawn** z (0.03 here), whereas a landed
one rests at half its body thickness (**0.006**). Those differ, and that
difference is the whole diagnosis. The bridge now forces `pause:false` via
`/world/<w>/control` before anything else, and **exits** if zero IMU frames
arrive rather than printing numbers that would not be a real flight. Verify
independently of your own bridge with
`gz topic -e -t /world/litewing/dynamic_pose/info`.

**3. macOS cannot run `gz sim` server and GUI in one process** (gz-sim#44), and
the Ruby `gz` wrapper **exits on stdin EOF** — so a detached server dies the
moment whatever held its stdin goes away. Spawn it with a pipe held open, and
run everything under `GZ_PARTITION` so a second Gazebo on the machine is
undisturbed.

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

## Sensors on the expansion header

Nothing but the MPU6050 is fitted, so every capability below is a part you add.

### Barometer — the cheap win, and a driver now exists

`flight/pios/common/pios_bmp280.c` + `pios_bmp280.h` were written for this
airframe (on the `litewing` branch). There was no Bosch driver in the tree at
all before — only `pios_bmp085.c` and `pios_ms5611.c` — which is why altitude
hold kept being described as needing GPS.

Mass is the whole argument: a **BMP280 is ~1 g**; a Matek M9N-5883 is
**14.5 g**, which on a 55 g airframe takes AUW to ~70 g and thrust-to-weight
from **1.60 to 1.24**.

It is simpler than the MS5611 on purpose. The MS5611 needs a state machine
because temperature and pressure are separately commanded conversions; the
BMP280 in NORMAL mode converts both continuously, so `poll()` is "read six
bytes and compensate" with no FSM to get wrong.

**The units trap.** `handleBaro()` in `modules/Sensors/sensors.c` compares
against `PIOS_CONST_MKS_STD_ATMOSPHERE_F` = 1.01325e5, so the sample must be
**Pascals** — even though `barosensor.xml` labels the field "kPa". Bosch's
compensation returns Q24.8, hence the `/256`. Emitting kPa as the label
suggests would report a 1.5 m hover as **32,423 m**.

What is verified, and what is not:

| | status |
| --- | --- |
| Bosch compensation maths | **verified** against the datasheet's worked example: 25.08 °C, 100653.3 Pa |
| Units contract, driver → `handleBaro` | **verified**, round-trips 0–1000 m to under 0.2 mm |
| Compiles into the target | **yes**, twin still boots 0x1302 with brushed endpoints intact |
| I2C transactions, real silicon | **untested** — no BMP280 on a posix twin's nonexistent bus |

### GPS + compass — Matek M9N-5883

u-blox NEO-M9N on UART, **QMC5883L** compass on I2C, JST-GH 6-pin
(`5V, G, TX, RX, DA, CL`), 32×32×10 mm, 14.5 g, 38400 baud default.

| module pin | LiteWing |
| --- | --- |
| TX (GNSS) | any free GPIO as UART RX, e.g. **GPIO18** |
| RX (GNSS) | any free GPIO as UART TX, e.g. **GPIO17** |
| DA / CL | **GPIO40 / GPIO41** (I2C1) — *not* I2C0, keep the 500 Hz IMU bus clear |
| G | GND |
| 5V | **see below** |

Two blockers before ordering a cable:

- **Power.** The module wants 4–6 V. LiteWing has no 5 V rail in flight: the
  header offers 3V3 and VBUS, and VBUS only exists with USB plugged in. On a
  1S pack this needs a boost converter.
- **The compass is QMC5883L**, which despite the name is a different part from
  the HMC5883L that `pios_hmc5x83.c` covers. It needs its own driver.

The GPS side is the easy half: `modules/GPS` already has `UBX.c` and
`ubx_autoconfig.c`, so a u-blox M9N is enable-and-wire. GPS is also what turns
the Remote ID broadcast from standards-shaped into actually compliant.

### What altitude hold actually requires

Having a baro driver is necessary and not sufficient. This twin's module list
is `ManualControl Stabilization Attitude Telemetry Actuator Receiver Logging
Flip RemoteID` — there is **no Sensors module and no AltitudeHold**, so nothing
would consume a barometer even if one were fitted. Closing that loop means
compiling both modules in plus a simulated baro feeding `BaroSensor` from
Gazebo's altitude, the same trick `pios_icm20602_sim.c` plays for the IMU.
That is a piece of work, not a `#define`.

## Next steps

1. Confirm the MOT_n → physical corner mapping from the PCB.
2. Confirm the module's PSRAM suffix before trusting the flow SPI pins.
3. MPU6050 I2C transport.
4. Re-tune Bank1 for 45 g — the current gains are 4-inch-class and meaningless
   here. Do it on the twin.
5. Sensors + AltitudeHold modules and a simulated baro, to close the altitude
   loop in the twin before any hardware carries a BMP280.
6. Only then an ESP-IDF target for the S3.
