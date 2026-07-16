#!/usr/bin/env python3
# Re-run the two loiter legs on an ALREADY-AIRBORNE bench aircraft (no
# takeoff). Used after fixing NAVL1_* left over from the night-5 sweep.
import sys, time, math
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from mav import connect
from pymavlink import mavutil

HOST = sys.argv[1] if len(sys.argv) > 1 else 'tcp:192.168.88.41:5760'
MODE_FBWA, MODE_LOITER = 5, 12

m = connect(HOST)
print("connected", flush=True)

sim_ms = [0]
state = {'alt': 0., 'ias': 0., 'thr': 0, 'roll': 0., 'nav_roll': 0., 'lat': 0., 'lon': 0.}

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
            state['roll'] = math.degrees(r.roll)
        elif t == 'NAV_CONTROLLER_OUTPUT':
            state['nav_roll'] = r.nav_roll
        elif t == 'GLOBAL_POSITION_INT':
            state['lat'], state['lon'] = r.lat / 1e7, r.lon / 1e7
        elif t == 'STATUSTEXT':
            print(f"  [{sim_ms[0]/1000:7.1f}] {r.text}", flush=True)

def rc():
    m.mav.rc_channels_override_send(m.target_system, 1, 1500, 1500, 1000, 1500, 0, 0, 0, 0)

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
    print(f"PARAM FAIL {name}", flush=True)
    return False

for n, v in [('RLL_RATE_FF', 3.0047), ('RLL_RATE_P', 4.8389),
             ('RLL_RATE_I', 0.15), ('RLL_RATE_D', 0.3832),
             ('ARSPD_RATIO', 1.6327)]:
    assert setp_verified(n, v), n
assert setp_verified('NAVL1_PERIOD', 17)
assert setp_verified('NAVL1_DAMPING', 0.75)
print("L1 params restored (17 / 0.75)", flush=True)

def leg(name, speed, rad, dur_ms):
    print(f"--- leg {name}: {speed} m/s R{rad} ---", flush=True)
    setp_verified('AIRSPEED_CRUISE', speed)
    setp_verified('WP_LOITER_RAD', rad)
    set_mode(MODE_FBWA); pump(1.0)
    set_mode(MODE_LOITER)
    t0 = sim_ms[0]
    rows = []
    last = 0
    while sim_ms[0] - t0 < dur_ms:
        rc(); pump(0.2)
        rows.append((sim_ms[0], state['ias'], state['alt'], state['roll'],
                     state['nav_roll'], state['thr'], state['lat'], state['lon']))
        if sim_ms[0] - last > 30000:
            last = sim_ms[0]
            print(f"  t+{(sim_ms[0]-t0)/1000:5.0f} ias {state['ias']:5.1f} alt {state['alt']:5.0f} "
                  f"roll {state['roll']:6.1f} dem {state['nav_roll']:6.1f} thr {state['thr']:3d}", flush=True)
    fn = f"bench-relegs-{name}.csv"
    with open(fn, 'w') as f:
        f.write("t_ms,ias,alt,roll,nav_roll,thr,lat,lon\n")
        for r_ in rows:
            f.write(','.join(f"{x}" for x in r_) + '\n')
    tail = [r_ for r_ in rows if r_[0] > sim_ms[0] - 120000]
    ias = [r_[1] for r_ in tail]; roll = [r_[3] for r_ in tail]; dem = [r_[4] for r_ in tail]
    la0 = tail[0][6]
    xs = [(r_[6] - la0) * 111320.0 for r_ in tail]
    ys = [(r_[7] - tail[0][7]) * 111320.0 * math.cos(math.radians(la0)) for r_ in tail]
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
          f"R {R:.0f}±{Rstd:.0f} m (cmd {rad})", flush=True)

leg('std', 29, 553, 300000)
leg('fast', 38, 363, 300000)
print("relegs done", flush=True)
m.close()
