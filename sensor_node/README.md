# sensor_node — the one common ESP32 sketch

One sketch serves **3R**, **1T** and **3R1T**. Flash it once and leave it.
Each MACS program listens only to the frames it cares about.

```
sensor_node/sensor_node_esp32/sensor_node_esp32.ino
```

## IMU mapping (as wired)

| mux channel | what is on it |
|---|---|
| ch0 | **limb 3** (home 120°) |
| ch1 | **platform** (XYZ intrinsic Euler) |
| ch2 | **limb 2** (home 240°) |
| ch3 | **limb 1** (home 0°) |

**The CAN frame is ARM-ORDERED.** theta1 *is* arm 1, theta2 *is* arm 2,
theta3 *is* arm 3. The channel shuffle happens here once, in `ARM_CHANNEL`,
so nothing downstream has to remember the loom.

This changed: the older `fable/homing` build had theta1 mean *mux channel 0*
and undid the shuffle on the MACS with `HOMING_SRC_ARM1 = theta3`. If you flash
this sketch, that program's `HOMING_SRC_*` defines must be straightened or
arm 1 and arm 3 will be swapped. `fable/3R1T` has already been corrected.

The `NHATS` rotation axes were measured **per limb**, so they follow the limb,
not the channel — they are indexed by arm here, not by mux channel.

## Frames

| ID | contents | used by |
|---|---|---|
| 0x6E4 | theta1/2/3 uint16 cdeg, status, seq | 3R, 3R1T |
| 0x6E8 | platform Euler X/Y/Z int16 cdeg, status | 3R, 3R1T |
| 0x6E9 | position int32 µm, velocity int16 0.1 mm/s, status | 1T, 3R1T |
| 0x6EA | REF rising / falling edge positions, int32 µm | 1T, 3R1T |

Both REF edges are latched **in the interrupt**, so the MACS gets the mark's
true extent without polling fast enough to catch it.

## Platform yaw drifts

6-DOF filter: Euler X and Y are gravity-referenced and hold. **Z has no
absolute reference** — it starts wherever the platform was at power-up and
drifts a few degrees per minute. Press `y` on the serial console at a known
pose to datum it, or move to a 9-DOF filter using the AK8963.

## Commands (115200 8N1)

`h` help · `c` recalibrate gyro (keep still) · `y` zero yaw datum ·
`z` zero encoder · `s <um>` µm per count · `f <hz>` send rate ·
`b <bps>` bitrate · `p` status print
