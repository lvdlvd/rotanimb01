#!/usr/bin/env python3
# Stage the f5-bench tuning params on the SITL instance (JSON backend on our
# fdm.c), then reboot to latch. Mirrors doc/f5-bench.parm minus the
# DroneCAN/INS-cal bench plumbing (SITL has its own sensors from truth).
import sys, time
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from mav import connect
from pymavlink import mavutil

PARAMS = [
    # airframe / speeds (model truth: cruise 38 @ 45%, stall ~20)
    ('AIRSPEED_CRUISE', 38), ('AIRSPEED_MIN', 21), ('AIRSPEED_MAX', 46),
    ('TRIM_THROTTLE', 45),
    # TECS energy limits from golden climb data
    ('TECS_CLMB_MAX', 5), ('TECS_SINK_MIN', 2.8), ('TECS_SINK_MAX', 6),
    # nav + attitude limits
    ('NAVL1_PERIOD', 17), ('ROLL_LIMIT_DEG', 40),
    ('PTCH_LIM_MAX_DEG', 20), ('PTCH_LIM_MIN_DEG', -18),
    ('WP_LOITER_RAD', 553),
    # AUTOTUNE-learned roll gains (bench night 4, A/B validated)
    ('RLL_RATE_FF', 3.0047), ('RLL_RATE_P', 4.8389), ('RLL_RATE_D', 0.3832),
    # TAKEOFF mode (bench night 6 values)
    ('TKOFF_ROTATE_SPD', 26), ('TKOFF_ALT', 150), ('TKOFF_LVL_ALT', 30),
    ('TKOFF_DIST', 800), ('TKOFF_LVL_PITCH', 12),
    # THR_MINSPD MUST be 0 for a wheeled standing start: nonzero suppresses
    # throttle until GPS speed exceeds it (hand-launch feature) = deadlock
    ('TKOFF_THR_MAX', 100), ('TKOFF_THR_MINSPD', 0),
    # no DCM fallback (the night-6 takeoff fix)
    ('AHRS_OPTIONS', 1),
    # bench comforts: no safety switch, no RC failsafing, GCS overrides
    ('BRD_SAFETY_DEFLT', 0), ('RC_OVERRIDE_TIME', 3),
    ('THR_FAILSAFE', 0), ('FS_SHORT_ACTN', 0), ('FS_LONG_ACTN', 0),
    ('FS_GCS_ENABL', 0), ('RTL_AUTOLAND', 2),
    # SITL airspeed sensor from the JSON 'airspeed' field
    ('ARSPD_TYPE', 100), ('ARSPD_USE', 1),
    ('LOG_DISARMED', 0),
]

def stage(m):
    bad = []
    for name, want in PARAMS:
        for attempt in range(4):
            m.mav.param_set_send(m.target_system, 1, name.encode(), float(want),
                                 mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            t0 = time.time()
            got = None
            while time.time() - t0 < 1.5:
                r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                if r is None:
                    continue
                rid = r.param_id if isinstance(r.param_id, str) else r.param_id.decode()
                if rid.strip('\x00') == name:
                    got = r.param_value
                    break
            if got is not None and abs(got - float(want)) < max(1e-3, abs(want) * 1e-4):
                break
        else:
            bad.append((name, want, got))
            print(f"  FAIL {name}: wanted {want}, got {got}")
    return bad

m = connect('tcp:127.0.0.1:5760')
print("connected, staging", len(PARAMS), "params")
bad = stage(m)
if bad:
    print("STAGING INCOMPLETE:", bad)
    sys.exit(1)
print("all params verified; rebooting SITL to latch")
m.mav.command_long_send(m.target_system, 1,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(2)
m.close()
print("done")
