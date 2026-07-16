#!/usr/bin/env python3
# Bench validation of the SITL-derived pitch tune: stage gains on the DUT,
# TAKEOFF-mode departure to 300 m, then the two loiter legs (shortened to
# 240 s each — real time on the bench), demand-vs-achieved stats + CSV.
import sys, time, math
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from mav import connect
from pymavlink import mavutil

HOST = 'tcp:192.168.88.41:5760'
MODE_FBWA, MODE_LOITER, MODE_TAKEOFF = 5, 12, 13

GAINS = [
    # SITL-derived pitch tune (AUTOTUNE completed, A/B validated in SITL)
    ('PTCH_RATE_FF', 1.6387), ('PTCH_RATE_P', 8.3447),
    ('PTCH_RATE_I', 6.2585), ('PTCH_RATE_D', 0.4981),
    ('PTCH2SRV_TCONST', 0.75), ('PTCH_RATE_FLTT', 2.1221),
    # ROLL gains MUST be staged too: a completed AUTOTUNE keeps them in
    # RAM only and any DUT reboot reverts them (the night-5/7 L1 mystery)
    ('RLL_RATE_FF', 3.0047), ('RLL_RATE_P', 4.8389),
    ('RLL_RATE_I', 0.15), ('RLL_RATE_D', 0.3832),
    ('NAVL1_PERIOD', 17), ('NAVL1_DAMPING', 0.75),
    ('ARSPD_RATIO', 1.6327),  # 2/rho0: harness dp is exactly 0.5*rho0*ias^2
    # wheeled standing start: nonzero suppresses throttle until GPS speed
    ('TKOFF_THR_MINSPD', 0),
    ('TKOFF_ALT', 300),
]

m = connect(HOST)
print("connected to bench")

sim_ms = [0]
state = {'alt': 0., 'ias': 0., 'thr': 0, 'roll': 0., 'pitch': 0.,
         'nav_roll': 0., 'nav_pitch': 0., 'lat': 0., 'lon': 0., 'servo1': 0, 'servo2': 0}

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
            state['alt'], state['ias'], state['thr'] = r.alt, r.airspeed, r.throttle
        elif t == 'ATTITUDE':
            state['roll'], state['pitch'] = math.degrees(r.roll), math.degrees(r.pitch)
        elif t == 'NAV_CONTROLLER_OUTPUT':
            state['nav_roll'], state['nav_pitch'] = r.nav_roll, r.nav_pitch
        elif t == 'SERVO_OUTPUT_RAW':
            state['servo1'], state['servo2'] = r.servo1_raw, r.servo2_raw
        elif t == 'GLOBAL_POSITION_INT':
            state['lat'], state['lon'] = r.lat / 1e7, r.lon / 1e7
        elif t == 'STATUSTEXT':
            print(f"  [{sim_ms[0]/1000:7.1f}] {r.text}")

def rc(pitch=1500, thr=1000):
    m.mav.rc_channels_override_send(m.target_system, 1, 1500, pitch, thr, 1500, 0, 0, 0, 0)

def set_mode(n):
    m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, n)

def setp_verified(name, want):
    for attempt in range(4):
        m.mav.param_set_send(m.target_system, 1, name.encode(), float(want),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < 2:
            r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if r is None:
                continue
            rid = r.param_id if isinstance(r.param_id, str) else r.param_id.decode()
            if rid.strip('\x00') == name and abs(r.param_value - float(want)) < max(1e-3, abs(want) * 1e-3):
                return True
    print(f"PARAM FAIL {name}")
    return False

def msg_interval(msgid, hz):
    m.mav.command_long_send(m.target_system, 1,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msgid, 1e6 / hz, 0, 0, 0, 0, 0)
    time.sleep(0.1)

print("staging pitch tune on DUT...")
ok = all(setp_verified(n, v) for n, v in GAINS)
if not ok:
    sys.exit(1)
print("gains verified on DUT")

# let EKF settle post power-cycle, keep overrides warm
print("waiting for EKF...")
t0 = time.time()
while time.time() - t0 < 60:
    rc(); pump(0.5)

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
    print("ARM FAILED after 60 tries"); sys.exit(1)
print(f"ARMED t={sim_ms[0]/1000:.1f}, TAKEOFF to 300 m")

t_arm = sim_ms[0]
last = 0
while state['alt'] < 285 and sim_ms[0] - t_arm < 300000:
    rc(); pump(0.3)
    if sim_ms[0] - last > 10000:
        last = sim_ms[0]
        print(f"  t+{(sim_ms[0]-t_arm)/1000:5.0f} ias {state['ias']:5.1f} alt {state['alt']:6.1f} thr {state['thr']:3d}")
if state['alt'] < 285:
    print("never reached 285 m"); sys.exit(1)
print("airborne at working altitude — streams up")
msg_interval(30, 10); msg_interval(62, 5); msg_interval(74, 5); msg_interval(33, 2); msg_interval(36, 5)

def leg(name, speed, rad, dur_ms):
    print(f"--- leg {name}: {speed} m/s R{rad} ---")
    setp_verified('AIRSPEED_CRUISE', speed)
    setp_verified('WP_LOITER_RAD', rad)
    set_mode(MODE_FBWA); pump(0.5)
    set_mode(MODE_LOITER)
    t0 = sim_ms[0]
    rows = []
    last = 0
    while sim_ms[0] - t0 < dur_ms:
        rc(); pump(0.2)
        rows.append((sim_ms[0], state['ias'], state['alt'], state['roll'],
                     state['nav_roll'], state['pitch'], state['nav_pitch'],
                     state['thr'], state['lat'], state['lon'], time.time(), state['servo1'], state['servo2']))
        if sim_ms[0] - last > 30000:
            last = sim_ms[0]
            print(f"  t+{(sim_ms[0]-t0)/1000:5.0f} ias {state['ias']:5.1f} alt {state['alt']:5.0f} "
                  f"roll {state['roll']:6.1f} dem {state['nav_roll']:6.1f} thr {state['thr']:3d}")
    fn = f"bench-loiter-{name}.csv"
    with open(fn, 'w') as f:
        f.write("t_ms,ias,alt,roll,nav_roll,pitch,nav_pitch,thr,lat,lon,wall,servo1,servo2\n")
        for r_ in rows:
            f.write(','.join(f"{x}" for x in r_) + '\n')
    tail = [r_ for r_ in rows if r_[0] > sim_ms[0] - 120000]
    ias = [r_[1] for r_ in tail]; roll = [r_[3] for r_ in tail]; dem = [r_[4] for r_ in tail]
    la0 = tail[0][8]
    xs = [(r_[8] - la0) * 111320.0 for r_ in tail]
    ys = [(r_[9] - tail[0][9]) * 111320.0 * math.cos(math.radians(la0)) for r_ in tail]
    n = len(xs)
    mx, my = sum(xs)/n, sum(ys)/n
    u = [x - mx for x in xs]; v = [y - my for y in ys]
    suu = sum(a*a for a in u); svv = sum(a*a for a in v); suv = sum(a*b for a, b in zip(u, v))
    det = suu * svv - suv * suv
    R = Rstd = float('nan')
    if abs(det) > 1e-6:
        suuu = sum(a**3 for a in u); svvv = sum(a**3 for a in v)
        suvv = sum(a*b*b for a, b in zip(u, v)); svuu = sum(b*a*a for a, b in zip(u, v))
        uc = (svv * (suuu + suvv) - suv * (svvv + svuu)) / (2 * det)
        vc = (suu * (svvv + svuu) - suv * (suuu + suvv)) / (2 * det)
        rs = [math.hypot(a - uc, b - vc) for a, b in zip(u, v)]
        R = sum(rs) / n
        Rstd = (sum((r_ - R)**2 for r_ in rs) / n) ** 0.5
    mean = lambda a: sum(a) / len(a)
    sd = lambda a: (sum((x - mean(a))**2 for x in a) / len(a)) ** 0.5
    print(f"  LAST 120 s: ias {mean(ias):.1f}±{sd(ias):.1f} (cmd {speed}), "
          f"roll {mean(roll):.1f}±{sd(roll):.1f}, demand {mean(dem):.1f}±{sd(dem):.1f}, "
          f"R {R:.0f}±{Rstd:.0f} m (cmd {rad})")

leg('std', 29, 553, 240000)
leg('fast', 38, 363, 240000)
print("bench validation done — leaving it in LOITER, disarm skipped (bench)")
m.close()
