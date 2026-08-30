# Working on OpenPilotESP32

Rules and traps for this port. Every entry below cost real bench time to find.

## RULE: do not modify the NinjaPilot tree in place

This project is deliberately separate. It consumes the NinjaPilot flight code
and must leave that checkout clean. The changes a HAL port cannot avoid live in
`patches/ninjapilot-shared-changes.patch`, applied and reverted with the
scripts in `tools/`.

If you need another upstream change, add it to the patch — do not edit the
flight tree and leave it dirty. The user watches that tree.

## The chip has NO USB peripheral

The original ESP32 (D0WD-V3) has no USB controller — that arrived with the S3.
The USB socket on a WROOM-32 devkit is a CP2102/CH340 **USB-to-UART bridge**.

So there is no USB HID, no USB CDC, and no `PIOS_INCLUDE_USB` on this target.
"UAVTalk over USB" here means UAVTalk over UART0 through that bridge, which is
exactly what the GCS and `ground/pyuavtalk/` talk to. This is silicon, not
configuration — do not go looking for a HID option to enable.

## RULE: the IDF console stays OFF UART0

`sdkconfig.defaults` moves it to UART1/GPIO13. Do not move it back except as a
deliberate, temporary debugging step (see SKILL.md).

**Why:** UART0 carries UAVTalk. With the console sharing it, log text
interleaves into the middle of UAVTalk frames. The receiver reports
`failed CRC check` in a loop and the GCS never links. `pyuavtalk` hides this —
it silently drops bad frames and limps along — so the symptom presents as
*flaky request/reply* rather than corruption. The GCS is stricter and simply
shows NO LINK.

## RULE: PIOS_MPU6000_Register() is called UNCONDITIONALLY

Even when no IMU answered. This looks wrong and has already been "fixed" once,
which broke the board.

**Why:** `Register()` publishes the PIOS_SENSORS instance the sensor consumers
wait on. Skip it and module init hangs before telemetry ever starts — the board
looks completely dead, with no console output and no core dump. Report the
missing IMU with `SYSTEMALARMS_ALARM_BOOTFAULT` (visible over UAVTalk) and let
the board boot. A board that comes up, says what is wrong and refuses to arm is
far more useful than one that silently wedges.

## Every stack size in this tree is sized for a Cortex-M3

They are too small on Xtensa — the windowed ABI spills register windows to the
stack and IDF's printf/driver layers are far hungrier than the STM32
equivalents. See the `PIOS_*_STACK_SIZE` block in `pios_board.h`.

**The failure mode is the problem:** these do NOT report as stack overflows.
They corrupt the heap or the scheduler ready-list and crash somewhere
unrelated. Telemetry at 3072 bytes corrupted the *event dispatcher's* periodic
object list; Attitude's hardcoded 540 crashed inside `vTaskSwitchContext`.

**How to find one:** comment modules out of `firmware/InitMods.c` down to none,
confirm the board is stable, then add them back one at a time. Do not guess.

## Two DTR/RTS traps

On these adapters DTR and RTS are wired to EN/RESET and GPIO0 — that is how
esptool reboots the chip. Both pyserial and Qt's QSerialPort **assert them when
they open a port**, which holds the ESP32 in reset.

Symptom: zero bytes at every baud while esptool still talks to the chip
perfectly. Already fixed in `ground/pyuavtalk/uavtalk_client.py`
(`SerialTransport`) and in the GCS `serialconnection` plugin. If you write new
tooling, deassert both after opening.

## PIOS_Assert() is a SILENT infinite spin

`DEBUG` is not defined for this target, so `PIOS_Assert(x)` expands to
`if (!(x)) { while (1) {;} }` — no message, no core dump, no reset. It is
indistinguishable from a dead board.

When bisecting a hang, remember an assert is a candidate even though nothing
was printed. `PIOS_DEBUG_Panic()` is the loud alternative, but note it aborts
and reboots, which turns a wedge into a boot loop — pick deliberately.

## Things that must not drift

- **`BOARD_REVISION` must stay `0x02`.** `modules/Attitude/attitude.c` derives
  its sensor path from `BOARDISCC3D (bdinfo->board_rev == 0x02)`. Anything else
  selects the CopterControl analog-gyro + ADXL345 path, which this board does
  not have. It compiles fine and then never produces a sample.
- **`UAVOBJ_INIT_*` must be defined** for every object you want initialised
  (see `esp-idf/main/CMakeLists.txt`). `UAVObjectsInitializeAll()` wraps every
  call in `#ifdef UAVOBJ_INIT_<name>`; with none defined it is an EMPTY
  FUNCTION that links and runs silently. Keep the list slim — initialising all
  111 objects makes telemetry stream everything and saturates a 57600 link.
- **`esp-idf/main/uavo_handles.ld` is load-bearing.** Read its header before
  touching it: the section must be writable *and* keep its name, and the two
  obvious fixes each break one of those.

## Sensor data-ready runs in a TASK, not an ISR

`pios/esp32/pios_exti.c`. Two independent reasons, both hard:

- IDF's `spi_device_polling_transmit()` is not ISR-safe, and the MPU6000
  data-ready handler reads the sensor over SPI.
- The Xtensa FPU is unusable in an ISR — IDF does not save/restore FPU context
  there — and `PIOS_MPU6000_HandleData()` does float math on that exact path.

This is why `pios/common/pios_mpu6000.c` is used unmodified.

## Do not try to build realposix here

It needs `linux/can.h` and `sys/prctl.h` — SocketCAN, Linux-only by design.
That is why `make gcs` cannot work on macOS (it depends on OPFW_RESOURCE, which
builds all firmware). See SKILL.md for the way around it.

## Open: HwSettings requests get a NACK

Every other settings object answers a `TYPE_OBJ_REQ` normally;
`HwSettings` returns `NACK` with an empty payload. A NACK from uavtalk.c means
`UAVObjGetByID()` returned NULL — the object manager does not have it.

Already ruled out, with evidence:

- object ID matches the GCS's exactly (`0xA65C5CD0` both sides)
- `hwsettings.c` is compiled, linked, and present in `_uavo_handles`
- `UAVOBJ_INIT_hwsettings=1` is defined, and `pios_board.c` also calls
  `HwSettingsInitialize()` explicitly
- its generated `Initialize()` is structurally identical to
  `StabilizationSettingsBank1Initialize()`, which works
- the handle table does not overlap the heap
  (`__stop__uavo_handles == _heap_start`)

Next step is to instrument the return value of `HwSettingsInitialize()` /
`UAVObjRegister()` on hardware. Do not write this off as cosmetic — an object
the GCS cannot read is an object the user cannot configure.
