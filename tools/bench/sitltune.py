#!/usr/bin/env python3
# SITL pitch autotune to completion. Lessons encoded from the bench nights:
#  - AUTOTUNE_AXES=2: pitch only, don't touch the validated roll gains
#  - events need stick REVERSALS crossing trim (rate demand + attitude error)
#  - AP_AutoTune::stop() RESTORES gains on mode exit unless BOTH D and P
#    limits were found -> read gains while still in AUTOTUNE
#  - the bench D/P ladders died at the altitude floor: recover IN-MODE
#    (AUTOTUNE flies like FBWA) with neutral stick + full throttle
import sys, time, math
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from mav import connect
from pymavlink import mavutil

MODE_FBWA, MODE_AUTOTUNE, MODE_TAKEOFF = 5, 8, 13
ALT_WORK, ALT_FLOOR, ALT_RECOVERED = 400, 280, 380

m = connect('tcp:127.0.0.1:5760')
print("connected")

sim_ms = [0]
state = {'alt': 0.0, 'ias': 0.0, 'pitch': 0.0, 'roll': 0.0}
texts = []

def pump(tmo=0.5):
    t0 = time.time()
    while time.time() - t0 < tmo:
        r = m.recv_match(blocking=True, timeout=0.2)
        if r is None:
            continue
        if hasattr(r, 'time_boot_ms'):
            sim_ms[0] = max(sim_ms[0], r.time_boot_ms)
        t = r.get_type()
        if t == 'VFR_HUD':
            state['alt'], state['ias'] = r.alt, r.airspeed
        elif t == 'ATTITUDE':
            state['pitch'], state['roll'] = math.degrees(r.pitch), math.degrees(r.roll)
        elif t == 'STATUSTEXT':
            texts.append((sim_ms[0], r.text))
            print(f"  [{sim_ms[0]/1000:7.1f}] {r.text}")

def rc(roll=1500, pitch=1500, thr=1500, yaw=1500):
    m.mav.rc_channels_override_send(m.target_system, 1, roll, pitch, thr, yaw, 0, 0, 0, 0)

def set_mode(n):
    m.mav.set_mode_send(m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, n)

def setp(name, v):
    m.mav.param_set_send(m.target_system, 1, name.encode(), float(v),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.15)

def getp(name):
    for _ in range(4):
        m.mav.param_request_read_send(m.target_system, 1, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < 2:
            r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if r is None:
                continue
            rid = r.param_id if isinstance(r.param_id, str) else r.param_id.decode()
            if rid.strip('\x00') == name:
                return r.param_value
    return None

# hold sim-time: keep overriding + pumping for ms of SIM time
def hold(ms, roll=1500, pitch=1500, thr=1500):
    t0 = sim_ms[0]
    while sim_ms[0] - t0 < ms:
        rc(roll, pitch, thr)
        pump(0.15)

setp('TKOFF_ALT', ALT_WORK)
setp('AUTOTUNE_AXES', 2)
setp('TKOFF_THR_MINSPD', 0)

# --- wait for EKF, depart
print("waiting for EKF...")
t0 = time.time()
while sim_ms[0] < 42000 and time.time() - t0 < 120:
    rc(thr=1000)
    pump(0.5)
set_mode(MODE_TAKEOFF)
time.sleep(0.5)
for i in range(40):
    rc(thr=1000)
    m.mav.command_long_send(m.target_system, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    r = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
    if r and r.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and r.result == 0:
        break
    pump(0.3)
else:
    print("ARM FAILED"); sys.exit(1)
print(f"armed t={sim_ms[0]/1000:.1f}, climbing to {ALT_WORK}")
t_arm = sim_ms[0]
while state['alt'] < ALT_WORK - 15 and sim_ms[0] - t_arm < 300000:
    rc(thr=1000)   # TAKEOFF mode owns throttle
    pump(0.3)
if state['alt'] < ALT_WORK - 15:
    print("never reached working altitude"); sys.exit(1)
print(f"at {state['alt']:.0f} m t+{(sim_ms[0]-t_arm)/1000:.0f}s — entering AUTOTUNE (pitch only)")

set_mode(MODE_AUTOTUNE)
hold(1000, thr=1450)

# --- reversal campaign
finished = False
n_rev = 0
t_tune0 = sim_ms[0]
while not finished and n_rev < 200 and sim_ms[0] - t_tune0 < 900000:
    # altitude floor: recover in-mode, wings level, full throttle, gentle up
    if state['alt'] < ALT_FLOOR:
        print(f"  floor! alt {state['alt']:.0f} — in-mode recovery")
        while state['alt'] < ALT_RECOVERED:
            rc(pitch=1350, thr=2000)   # ~40% up stick, full power
            pump(0.2)
        print(f"  recovered to {state['alt']:.0f}")
    # energy: pick throttle by ias
    thr = 1700 if state['ias'] < 30 else (1300 if state['ias'] > 42 else 1500)
    # one reversal pair: full-up then full-down, 1.6 s sim each
    hold(1600, pitch=1200, thr=thr)
    hold(1600, pitch=1800, thr=thr)
    n_rev += 1
    # settle briefly so error re-arms
    hold(800, pitch=1500, thr=thr)
    for tms, txt in texts[-6:]:
        if 'Pitch: Finished' in txt or 'AUTOTUNE: Success' in txt:
            finished = True
    if n_rev % 10 == 0:
        print(f"  {n_rev} reversals, alt {state['alt']:.0f} ias {state['ias']:.1f}")

print(f"reversals: {n_rev}, finished: {finished}")

# --- read gains IN-MODE
gains = {}
for p in ['PTCH_RATE_FF', 'PTCH_RATE_P', 'PTCH_RATE_I', 'PTCH_RATE_D',
          'PTCH2SRV_TCONST', 'PTCH_RATE_FLTD', 'PTCH_RATE_FLTT', 'PTCH_RATE_SMAX']:
    gains[p] = getp(p)
print("IN-MODE GAINS:", {k: (round(v, 4) if v is not None else None) for k, v in gains.items()})

# progress texts summary
print("--- tune texts ---")
for tms, txt in texts:
    if 'itch' in txt or 'AUTOTUNE' in txt:
        print(f"  [{tms/1000:7.1f}] {txt}")

# if finished, exit mode (gains persist); else stay noted
if finished:
    set_mode(MODE_FBWA)
    hold(2000, thr=1450)
    print("finished tune — gains latched, back to FBWA")
m.close()
