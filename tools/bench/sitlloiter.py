#!/usr/bin/env python3
# SITL loiter study: fly the two owner-defined loiter legs and record
# demand-vs-achieved. Usage: sitlloiter.py <tag> [pitch=new|old]
#  - standard: 3 deg/s -> 29 m/s, R 553   - fast: 6 deg/s -> 38 m/s, R 363
# Records CSV: t, ias, alt, roll, nav_roll, pitch, nav_pitch, thr, lat, lon
# Then prints capture stats incl. circle-fit radius over the last 120 s.
import sys, time, math
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from mav import connect
from pymavlink import mavutil

TAG = sys.argv[1] if len(sys.argv) > 1 else 'run'
PITCH = sys.argv[2] if len(sys.argv) > 2 else 'new'
MODE_FBWA, MODE_LOITER, MODE_TAKEOFF = 5, 12, 13

NEW_PITCH = {'PTCH_RATE_FF': 1.6387, 'PTCH_RATE_P': 8.3447,
             'PTCH_RATE_I': 6.2585, 'PTCH_RATE_D': 0.4981,
             'PTCH2SRV_TCONST': 0.75, 'PTCH_RATE_FLTT': 2.1221}
OLD_PITCH = {'PTCH_RATE_FF': 0.0, 'PTCH_RATE_P': 0.345,
             'PTCH_RATE_I': 0.3, 'PTCH_RATE_D': 0.04,
             'PTCH2SRV_TCONST': 0.45, 'PTCH_RATE_FLTT': 3.0}

m = connect('tcp:127.0.0.1:5760')
sim_ms = [0]
state = {'alt': 0., 'ias': 0., 'thr': 0, 'roll': 0., 'pitch': 0.,
         'nav_roll': 0., 'nav_pitch': 0., 'lat': 0., 'lon': 0.}

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
        elif t == 'GLOBAL_POSITION_INT':
            state['lat'], state['lon'] = r.lat / 1e7, r.lon / 1e7
        elif t == 'STATUSTEXT':
            print(f"  [{sim_ms[0]/1000:7.1f}] {r.text}")

def rc(pitch=1500, thr=1500):
    m.mav.rc_channels_override_send(m.target_system, 1, 1500, pitch, thr, 1500, 0, 0, 0, 0)

def set_mode(n):
    m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, n)

def setp(name, v):
    m.mav.param_set_send(m.target_system, 1, name.encode(), float(v),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.15)

def msg_interval(msgid, hz):
    m.mav.command_long_send(m.target_system, 1,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msgid, 1e6 / hz, 0, 0, 0, 0, 0)
    time.sleep(0.1)

gains = NEW_PITCH if PITCH == 'new' else OLD_PITCH
print(f"pitch gains: {PITCH}")
for k, v in gains.items():
    setp(k, v)
setp('TKOFF_ALT', 300)
setp('TKOFF_THR_MINSPD', 0)

print("waiting for EKF...")
t0 = time.time()
while sim_ms[0] < 42000 and time.time() - t0 < 120:
    rc(thr=1000); pump(0.5)
set_mode(MODE_TAKEOFF); time.sleep(0.5)
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
print(f"armed t={sim_ms[0]/1000:.1f}")
t_arm = sim_ms[0]
while state['alt'] < 285 and sim_ms[0] - t_arm < 300000:
    rc(thr=1000); pump(0.3)
print(f"at {state['alt']:.0f} m — streams up, starting legs")
# streams only once airborne (bench lesson)
msg_interval(30, 25)   # ATTITUDE
msg_interval(62, 10)   # NAV_CONTROLLER_OUTPUT
msg_interval(74, 10)   # VFR_HUD
msg_interval(33, 5)    # GLOBAL_POSITION_INT

def leg(name, speed, rad, dur_ms):
    print(f"--- leg {name}: {speed} m/s R{rad} ---")
    setp('AIRSPEED_CRUISE', speed)
    setp('WP_LOITER_RAD', rad)
    set_mode(MODE_FBWA); pump(0.5)
    set_mode(MODE_LOITER)          # radius latches at entry
    t0 = sim_ms[0]
    rows = []
    last = 0
    while sim_ms[0] - t0 < dur_ms:
        rc(thr=1000)               # LOITER owns throttle; sticks neutral
        pump(0.2)
        rows.append((sim_ms[0], state['ias'], state['alt'], state['roll'],
                     state['nav_roll'], state['pitch'], state['nav_pitch'],
                     state['thr'], state['lat'], state['lon']))
        if sim_ms[0] - last > 30000:
            last = sim_ms[0]
            print(f"  t+{(sim_ms[0]-t0)/1000:5.0f} ias {state['ias']:5.1f} alt {state['alt']:5.0f} "
                  f"roll {state['roll']:6.1f} dem {state['nav_roll']:6.1f} thr {state['thr']:3d}")
    fn = f"sitl-loiter-{TAG}-{name}.csv"
    with open(fn, 'w') as f:
        f.write("t_ms,ias,alt,roll,nav_roll,pitch,nav_pitch,thr,lat,lon\n")
        for r_ in rows:
            f.write(','.join(f"{x}" for x in r_) + '\n')
    # stats over the last 120 s (post-capture, if it captured)
    tail = [r_ for r_ in rows if r_[0] > sim_ms[0] - 120000]
    ias = [r_[1] for r_ in tail]; roll = [r_[3] for r_ in tail]; dem = [r_[4] for r_ in tail]
    # circle fit (Kasa) on lat/lon in meters
    la0 = tail[0][8]
    xs = [(r_[8] - la0) * 111320.0 for r_ in tail]
    ys = [(r_[9] - tail[0][9]) * 111320.0 * math.cos(math.radians(la0)) for r_ in tail]
    n = len(xs)
    mx, my = sum(xs)/n, sum(ys)/n
    u = [x - mx for x in xs]; v = [y - my for y in ys]
    suu = sum(a*a for a in u); svv = sum(a*a for a in v); suv = sum(a*b for a, b in zip(u, v))
    suuu = sum(a**3 for a in u); svvv = sum(a**3 for a in v)
    suvv = sum(a*b*b for a, b in zip(u, v)); svuu = sum(b*a*a for a, b in zip(u, v))
    det = suu * svv - suv * suv
    R = float('nan')
    if abs(det) > 1e-6:
        uc = (svv * (suuu + suvv) - suv * (svvv + svuu)) / (2 * det)
        vc = (suu * (svvv + svuu) - suv * (suuu + suvv)) / (2 * det)
        rs = [math.hypot(a - uc, b - vc) for a, b in zip(u, v)]
        R = sum(rs) / n
        Rstd = (sum((r_ - R)**2 for r_ in rs) / n) ** 0.5
    mean = lambda a: sum(a) / len(a)
    sd = lambda a: (sum((x - mean(a))**2 for x in a) / len(a)) ** 0.5
    print(f"  LAST 120 s: ias {mean(ias):.1f}±{sd(ias):.1f} (cmd {speed}), "
          f"roll {mean(roll):.1f}±{sd(roll):.1f}, demand {mean(dem):.1f}±{sd(dem):.1f}, "
          f"R {R:.0f}±{Rstd:.0f} m (cmd {rad})" if not math.isnan(R) else "  circle fit failed")
    return fn

leg('std', 29, 553, 420000)
leg('fast', 38, 363, 420000)
print("done")
m.close()
