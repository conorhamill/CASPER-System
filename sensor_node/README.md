# sensor_node — the ESP32 sensor sketch

Three limb joint angles, the platform orientation, and the translation-stage
encoder, all sent to the MasterMACS on CAN2.

```
sensor_node/sensor_node_esp32/sensor_node_esp32.ino
```

All the sensor maths happens on the ESP32. The MACS receives finished numbers
in the units its kinematics wants, so its 2 ms interrupt spends its budget on
the inverse kinematics rather than on quaternions.

Madgwick filters update at 100 Hz; frames go out at 50 Hz.

## IMU mapping (as wired)

| mux channel | what is on it |
|---|---|
| ch0 | **limb 1** |
| ch1 | **platform** (XYZ intrinsic Euler) |
| ch2 | **limb 2** |
| ch3 | **limb 3** |

Bench-confirmed: moving a limb by hand moves the matching theta.

**The CAN frames are ARM-ORDERED.** theta1 *is* arm 1, theta2 *is* arm 2,
theta3 *is* arm 3. The channel shuffle happens once, in `ARM_CHANNEL`, so
nothing downstream has to know the loom.

The `NHATS` rotation axes were measured **per limb**, so they follow the limb
rather than the channel — they are indexed by arm, not by mux channel.

Home angles are not here. They are a MACS concern — `HOME_ARM1/2/3_CDEG` in
`main/3R1T_Config.mh` — and the sketch does not use them.

## Frames

| ID | contents | used by |
|---|---|---|
| 0x6E4 | theta1, theta2 — int32 cdeg each | 3R, 3R1T |
| 0x6E5 | theta3 int32 cdeg, status, seq | 3R, 3R1T |
| 0x6E8 | platform Euler X/Y/Z int16 cdeg, status, seq | 3R, 3R1T |
| 0x6E9 | position int32 µm, velocity int16 0.1 mm/s, status, seq | 1T, 3R1T |
| 0x6EA | REF rising / falling edge positions, int32 µm | 1T, 3R1T |

The limb angles are **continuous and signed**. They are unwrapped through the
360/0 seam and accumulated, so a limb turning past 360° keeps counting and one
sitting at its home angle reads a steady figure instead of flicking between 0
and 360. That is what lets the MACS average and difference them, and it is why
they are `int32` across two frames rather than `uint16` in one.

Each REF edge latches the **encoder count inside the interrupt**, so both
positions are exact whatever speed the stage crossed at. Stage homing takes the
first new edge after reversing off the end of travel as its datum.

## Platform yaw drifts, and cannot be fixed

6-DOF filter: Euler X and Y are gravity-referenced and hold indefinitely. **Z
has no absolute reference** — it starts wherever the platform was at power-up
and drifts a few degrees per minute.

There is nothing on the rig that can datum it, so eulerZ is always relative and
the "yaw datum set" status bit always reads 0. An absolute heading needs the
AK8963 magnetometer (a status bit says whether one is fitted), a 9-DOF filter
and a magnetic survey. Until that exists, do not use eulerZ for anything that
has to be right after a few minutes.

## Serial (115200 8N1)

Output only — there is no command interface. The sketch prints a one-line
status at 1 Hz and a CAN diagnosis at start-up.

`umPerCount`, `sendHz` and `canBitrate` are constants at the top of the sketch.
Changing one means reflashing, which for a calibration figure is the right way
round anyway.
