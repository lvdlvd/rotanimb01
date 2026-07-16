#!/usr/bin/env python3
# End-to-end SITL rig validation: TAKEOFF-mode departure on the JSON-backend
# fdm, climb to TKOFF_ALT, then hands-off FBWA cruise. Sim time from
# time_boot_ms (wall clock runs at --speedup).
import sys, time
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from mav import connect
from pymavlink import mavutil

MODE_FBWA, MODE_TAKEOFF = 5, 13

m = connect('tcp:127.0.0.1:5760')
print("connected")

sim_ms = [0]
def pump(want=None, tmo=2.0):
    t0 = time.time()
    while time.time() - t0 < tmo:
        r = m.recv_match(blocking=True, timeout=0.5)
        if r is None:
            continue
        if hasattr(r, 'time_boot_ms'):
            sim_ms[0] = max(sim_ms[0], r.time_boot_ms)
        t = r.get_type()
        if t == 'STATUSTEXT':
            print(f"  [{sim_ms[0]/1000:7.1f}] {r.text}")
        if want and t == want:
            return r
    return None

def rc_neutral():
    m.mav.rc_channels_override_send(m.target_system, 1,
        1500, 1500, 1000, 1500, 0, 0, 0, 0)

def set_mode(n):
    m.mav.set_mode_send(m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, n)

# wait for EKF to settle (GPS glitch/yaw align texts), ~40 s sim time
print("waiting for EKF to settle...")
t0 = time.time()
while sim_ms[0] < 42000 and time.time() - t0 < 90:
    rc_neutral()
    pump(tmo=1.0)

set_mode(MODE_TAKEOFF)
hb = pump('HEARTBEAT', 3)
print(f"mode = {hb.custom_mode if hb else '?'} (want {MODE_TAKEOFF})")

# arm, with retries against transient prearm states
armed = False
for i in range(30):
    rc_neutral()
    m.mav.command_long_send(m.target_system, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    r = pump('COMMAND_ACK', 3)
    if r and r.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and r.result == 0:
        armed = True
        break
    time.sleep(0.5)
if not armed:
    print("ARM FAILED, giving up")
    sys.exit(1)
print(f"ARMED at sim t={sim_ms[0]/1000:.1f}")

# monitor the departure: expect ground roll, rotate ~26, climb at TECS_CLMB_MAX
t_arm = sim_ms[0]
last_print = 0
ok = False
while sim_ms[0] - t_arm < 120000 and time.time() - t0 < 300:
    rc_neutral()
    v = pump('VFR_HUD', 2)
    if v is None:
        continue
    if sim_ms[0] - last_print > 5000:
        last_print = sim_ms[0]
        print(f"  t+{(sim_ms[0]-t_arm)/1000:5.1f} ias {v.airspeed:5.1f} alt {v.alt:6.1f} "
              f"climb {v.climb:5.2f} thr {v.throttle:3d}")
    if v.alt > 140:
        ok = True
        break
if not ok:
    print("NEVER REACHED 140 m")
    sys.exit(1)
print(f"reached {140} m at sim t+{(sim_ms[0]-t_arm)/1000:.1f} s — switching FBWA hands-off")

set_mode(MODE_FBWA)
t_fbwa = sim_ms[0]
worst_roll = worst_pitch = 0.0
while sim_ms[0] - t_fbwa < 60000 and time.time() - t0 < 420:
    rc_neutral()
    a = pump('ATTITUDE', 2)
    if a is None:
        continue
    import math
    worst_roll = max(worst_roll, abs(math.degrees(a.roll)))
    worst_pitch = max(worst_pitch, abs(math.degrees(a.pitch)))
v = pump('VFR_HUD', 3)
print(f"FBWA 60 s hands-off: worst |roll| {worst_roll:.1f} deg, worst |pitch| {worst_pitch:.1f} deg, "
      f"ias {v.airspeed:.1f} alt {v.alt:.1f}" if v else "no VFR_HUD at end")
print("RIG VALIDATION PASS" if worst_roll < 25 and worst_pitch < 25 else "RIG VALIDATION MARGINAL")
m.close()
