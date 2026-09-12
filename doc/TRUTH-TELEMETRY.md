# Truth telemetry — reading the model state from the harness

The harness reports its truth state as **TRUTH_* pseudocan frames on its
USB CDC at 20 Hz** — not over the CAN bus, not MAVLink, not a file. This
is what you compare the autopilot's estimate against, and what any
external consumer (a display, a logger, a second navigator under test)
should read.

## Transport: pseudocan lines

Text lines over the CDC-ACM port at any baud:

```
ID-A[.ID-B][R]:hexpayload[:crc16][ port[ fmi]]\n
```

ID-A/ID-B are the two halves of a 29-bit CAN identifier as printed by
`fmtcan` (a 32-bit priority-preserving header: ID-A in bits 30:20, RTR
bit 19, EXT bit 18, ID-B in bits 17:0; the line prints header>>18 and
header&0x3ffff in hex). Payloads are big-endian. The harness emits
crc-less lines and verifies a crc when the host sends one. Reference
decoder and encoder: `rb01tool/main.go` (`Header`, `parseLine`, and
the TRUTH_* cases); `tools/bench/drive.go` is the minimal encoder.

The 29-bit id is a bit-field (src/canmsg.h): LCC [28:26] = 1 for
measurements, MSGID [25:19], PRV bit 16 set, SRCID [15:8] hashed from
the harness UID, 0x9 in [7:4], and a 4-bit per-message sequence in
[3:0].

## The frames

| msgid | name         | payload (8 bytes, big-endian)                                  |
| ----- | ------------ | -------------------------------------------------------------- |
| 0x44  | TRUTH_POSVEL | h cm i32 (above origin), ḣ cm/s i16, groundspeed cm/s i16     |
| 0x45  | TRUTH_ATT    | q_nb w, x, y, z as i16 × 2¹⁵                                   |
| 0x46  | TRUTH_AIR    | IAS 0.1 m/s u16, TAS 0.1 m/s u16, α 0.01° i16, β 0.01° i16     |
| 0x47  | TRUTH_CTRL   | δa, δe, δr 0.01° i16 (post-lag), δt 0.1 % u16                  |
| 0x49  | TRUTH_POS    | N cm i32, E cm i32 from the origin                             |
| 0x4A  | TRUTH_VEL    | vN, vE, vD cm/s i16, u16 pad                                   |

Full 6-DOF pose = 0x49 (N, E) + 0x44 (h) + 0x45 (the quaternion, served
directly — no Euler conversion on the harness). Also on the link: 0x42
STATUS at 10 Hz (u32 harness µs clock, psi 0.01°, flags), 0x40/0x41
PWM captures, 0x43 DIAG at 1 Hz, 0x48 PARAM_VAL readbacks. Truth frames
are emitted only in six-DOF mode.

## Gotchas

- **0x4A VEL closes each 20 Hz burst.** All six frames go out
  back-to-back; use VEL's arrival as the burst-complete barrier and
  latch a pose there, never mid-burst (rb01tool does exactly this).
- **No timestamp.** Only the 4-bit per-msgid sequence (drop detection,
  wraps at 16). Host arrival time is the stamp; if you need the
  harness's own clock, STATUS carries it at 10 Hz.
- 20 Hz is a decimation of the 1 kHz model; raising it is one constant
  in src/main.c (~180 bytes per burst as text = 3.6 kB/s at 20 Hz).
- **Do not tap the CAN bus for pose.** What the DUT sees there is the
  deliberately degraded DroneCAN Fix2 + RawAirData feed (5 Hz, lagged
  150 ms by default, quantized). Using it as truth hands a consumer a
  lagged, quantized pose on purpose.
- The origin is the harness's feeder origin (src/main.c: 45.52688 N,
  1.667291 E). N/E are metres from it in a local tangent plane; `drive
  setpos` moves the aircraft, not the origin.
- Sharing the port: the CDC is one device with one reader. A truth
  consumer and a command sender must go through one process (rb01tool
  does both), or through a bridge (`bench serbridge` on a second port
  works for the harness CDC too).

## Comparing with the autopilot

ArduPilot: ATTITUDE (roll/pitch/yaw, rad), GLOBAL_POSITION_INT or
LOCAL_POSITION_NED, VFR_HUD (airspeed). PX4: `vehicle_attitude`,
`vehicle_local_position`, `airspeed_validated`. Convert the truth
quaternion to Euler on the host (rb01tool's `fromQuat`), align by host
arrival time, and remember the estimator is allowed the GPS lag plus
its own. On the healthy checkride bench the EKF roll tracked truth roll
with r = 1.000 and no measurable lag — a truth-vs-estimate divergence
is a finding, not noise ([F5-TESTREPORT-2026-07-15.md](F5-TESTREPORT-2026-07-15.md), night 7).
