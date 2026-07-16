#!/usr/bin/env python3
# Instrumented bench takeoff: record pitch demand vs achieved vs elevator
# servo from ARM onward, auto-abort on departure. Diagnosing the no-rotation
# with the SITL-derived pitch gains (suspect: PTCH_RATE_I windup on ground).
import sys, time, math
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from mav import connect
from pymavlink import mavutil

HOST = sys.argv[1] if len(sys.argv) > 1 else 'tcp:192.168.88.41:5760'
TAG = sys.argv[2] if len(sys.argv) > 2 else 'run'
MODE_FBWA, MODE_TAKEOFF = 5, 13

m = connect(HOST)
print("connected", flush=True)

sim_ms = [0]
st = {'alt': 0., 'ias': 0., 'thr': 0, 'roll': 0., 'pitch': 0., 'q': 0.,
      'nav_pitch': 0., 'servo2': 0}

def pump(tmo=0.3):
    t0 = time.time()
    while time.time() - t0 < tmo:
        r = m.recv_match(blocking=True, timeout=0.15)
        if r is None:
            continue
        if hasattr(r, 'time_boot_ms'):
            sim_ms[0] = max(sim_ms[0], r.time_boot_ms)
        t = r.get_type()
        if t == 'VFR_HUD':
            st['alt'], st['ias'], st['thr'] = r.alt, r.airspeed, r.throttle
        elif t == 'ATTITUDE':
            st['roll'], st['pitch'], st['q'] = math.degrees(r.roll), math.degrees(r.pitch), math.degrees(r.pitchspeed)
        elif t == 'NAV_CONTROLLER_OUTPUT':
            st['nav_pitch'] = r.nav_pitch
        elif t == 'SERVO_OUTPUT_RAW':
            st['servo2'] = r.servo2_raw
        elif t == 'STATUSTEXT':
            print(f"  [{sim_ms[0]/1000:7.1f}] {r.text}", flush=True)

def rc():
    m.mav.rc_channels_override_send(m.target_system, 1, 1500, 1500, 1000, 1500, 0, 0, 0, 0)

def set_mode(n):
    m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, n)

def msg_interval(msgid, hz):
    m.mav.command_long_send(m.target_system, 1,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msgid, 1e6 / hz, 0, 0, 0, 0, 0)
    time.sleep(0.1)

print("waiting for EKF...", flush=True)
t0 = time.time()
while time.time() - t0 < 60:
    rc(); pump(0.5)

# moderate streams BEFORE arm — we need the ground roll
msg_interval(30, 10)   # ATTITUDE
msg_interval(36, 5)    # SERVO_OUTPUT_RAW
msg_interval(62, 5)    # NAV_CONTROLLER_OUTPUT
msg_interval(74, 5)    # VFR_HUD

set_mode(MODE_TAKEOFF); time.sleep(0.5)
armed = False
for i in range(60):
    rc()
    m.mav.command_long_send(m.target_system, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    r = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
    if r and r.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and r.result == 0:
        armed = True
        break
    pump(1.0)
if not armed:
    print("ARM FAILED", flush=True); sys.exit(1)
print(f"ARMED t={sim_ms[0]/1000:.1f}", flush=True)

t_arm = sim_ms[0]
rows = []
last = 0
verdict = 'timeout'
while sim_ms[0] - t_arm < 120000:
    rc(); pump(0.1)
    rows.append((sim_ms[0] - t_arm, st['ias'], st['alt'], st['pitch'], st['nav_pitch'],
                 st['q'], st['servo2'], st['thr'], st['roll']))
    if sim_ms[0] - last > 2000:
        last = sim_ms[0]
        print(f"  t+{(sim_ms[0]-t_arm)/1000:5.1f} ias {st['ias']:5.1f} alt {st['alt']:6.1f} "
              f"pit {st['pitch']:6.1f} dem {st['nav_pitch']:6.1f} q {st['q']:6.1f} "
              f"sv2 {st['servo2']:4d} thr {st['thr']:3d}", flush=True)
    if st['alt'] > 100:
        verdict = 'CLIMBING OK'
        break
    if st['ias'] > 42 and st['alt'] < 5:
        verdict = 'NO ROTATION (overspeed on ground)'
        break
    if st['pitch'] < -12 or abs(st['roll']) > 60:
        verdict = 'DEPARTURE'
        break
print("VERDICT:", verdict, flush=True)
if verdict != 'CLIMBING OK':
    m.mav.command_long_send(m.target_system, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
    print("force-disarmed", flush=True)

fn = f"bench-takeoff-{TAG}.csv"
with open(fn, 'w') as f:
    f.write("t_ms,ias,alt,pitch,nav_pitch,q,servo2,thr,roll\n")
    for r_ in rows:
        f.write(','.join(f"{x}" for x in r_) + '\n')
print("wrote", fn, flush=True)
m.close()
