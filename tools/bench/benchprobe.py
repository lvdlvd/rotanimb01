#!/usr/bin/env python3
# Straight-line consistency probe (bench, already airborne): FBWA wings
# level 120 s, record yaw vs GPS course vs EKF velocity vs airspeed.
# Discriminates: EKF/GPS input wander (L1's food) vs control loops.
import sys, time, math
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from mav import connect
from pymavlink import mavutil

HOST = sys.argv[1] if len(sys.argv) > 1 else 'tcp:192.168.88.41:5760'
MODE_FBWA = 5
m = connect(HOST)
print("connected", flush=True)

sim_ms = [0]
st = {'yaw': None, 'roll': 0., 'ias': 0., 'alt': 0.,
      'gps_cog': None, 'gps_v': 0., 'ekf_vx': 0., 'ekf_vy': 0.}
rows = []

def pump(tmo=0.3, rec=False):
    t0 = time.time()
    while time.time() - t0 < tmo:
        r = m.recv_match(blocking=True, timeout=0.15)
        if r is None:
            continue
        if hasattr(r, 'time_boot_ms'):
            sim_ms[0] = max(sim_ms[0], r.time_boot_ms)
        t = r.get_type()
        if t == 'ATTITUDE':
            st['yaw'], st['roll'] = math.degrees(r.yaw) % 360, math.degrees(r.roll)
        elif t == 'GPS_RAW_INT':
            st['gps_cog'], st['gps_v'] = r.cog / 100.0, r.vel / 100.0
        elif t == 'GLOBAL_POSITION_INT':
            st['ekf_vx'], st['ekf_vy'] = r.vx / 100.0, r.vy / 100.0
        elif t == 'VFR_HUD':
            st['ias'], st['alt'] = r.airspeed, r.alt
        elif t == 'STATUSTEXT':
            print(f"  [{sim_ms[0]/1000:7.1f}] {r.text}", flush=True)
        if rec and t == 'GPS_RAW_INT' and st['yaw'] is not None:
            rows.append((sim_ms[0], st['yaw'], st['gps_cog'], st['gps_v'],
                         st['ekf_vx'], st['ekf_vy'], st['ias'], st['alt'], st['roll']))

def rc(thr=1450):
    m.mav.rc_channels_override_send(m.target_system, 1, 1500, 1500, thr, 1500, 0, 0, 0, 0)

m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_FBWA)
# GPS_RAW_INT at 5 Hz
m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0, 24, 200000, 0, 0, 0, 0, 0)
time.sleep(0.2)
print("FBWA straight, settling 10 s...", flush=True)
t0 = sim_ms[0]
while sim_ms[0] - t0 < 10000 or sim_ms[0] == 0:
    rc(); pump(0.2)
print("recording 120 s...", flush=True)
t0 = sim_ms[0]
last = 0
while sim_ms[0] - t0 < 120000:
    rc(); pump(0.2, rec=True)
    if sim_ms[0] - last > 20000:
        last = sim_ms[0]
        d = rows[-1] if rows else None
        if d:
            ang = lambda a: (a + 180) % 360 - 180
            ev = math.degrees(math.atan2(d[5], d[4])) % 360
            print(f"  yaw {d[1]:6.1f} cog {d[2]:6.1f} ekfv {ev:6.1f} |gps| {d[3]:5.1f} ias {d[6]:5.1f} roll {d[8]:5.1f}", flush=True)

with open('bench-probe.csv', 'w') as f:
    f.write("t_ms,yaw,gps_cog,gps_v,ekf_vx,ekf_vy,ias,alt,roll\n")
    for r_ in rows:
        f.write(','.join(f"{x}" for x in r_) + '\n')

ang = lambda a: (a + 180) % 360 - 180
dcog = [ang(r_[2] - r_[1]) for r_ in rows]
ekfd = [ang(math.degrees(math.atan2(r_[5], r_[4])) % 360 - r_[1]) for r_ in rows]
dv = [r_[3] - r_[6] for r_ in rows]
mean = lambda a: sum(a) / len(a)
sd = lambda a: (sum((x - mean(a))**2 for x in a) / len(a)) ** 0.5
print(f"cog-yaw {mean(dcog):.1f}±{sd(dcog):.1f} deg | ekfvel-yaw {mean(ekfd):.1f}±{sd(ekfd):.1f} deg "
      f"| gpsV-ias {mean(dv):.1f}±{sd(dv):.1f} m/s | n={len(rows)}", flush=True)
m.close()
