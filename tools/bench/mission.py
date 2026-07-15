import sys, time, threading, math, subprocess
sys.path.insert(0, '/private/tmp/claude-501/-Users-lvd-Project-stm32/50614b33-98b9-4f22-93e0-3259987dd941/scratchpad/bench')
from mav import connect
from pymavlink import mavutil
m = connect('tcp:192.168.88.41:5760')
rc = [1500, 1500, 1000, 1500, 0, 0, 0, 0]
def rc_thread():
    while True:
        try: m.mav.rc_channels_override_send(m.target_system, 1, *rc)
        except Exception: return
        time.sleep(0.2)
threading.Thread(target=rc_thread, daemon=True).start()
time.sleep(1.5)
def hud(): return m.recv_match(type='VFR_HUD', blocking=True, timeout=2)
def att_deg():
    a = m.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    return (math.degrees(a.roll), math.degrees(a.pitch)) if a else (99,99)
def mode(n):
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, n, 0,0,0,0,0)
    m.recv_match(type='COMMAND_ACK', blocking=True, timeout=4)

def takeoff(n):
    rc[0], rc[1], rc[2], rc[3] = 1500, 1500, 1000, 1500
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0,0,0,0,0)
    # settle: estimator level before arming
    t0=time.time()
    while time.time()-t0<40:
        r, p = att_deg()
        if abs(r) < 3 and abs(p) < 3: break
        time.sleep(1)
    time.sleep(2)
    mode(5)
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0)
    t0=time.time(); armed=False
    while time.time()-t0<8:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and hb.base_mode & 128: armed=True; break
    if not armed: print(f"  attempt {n}: arm refused"); return False
    rc[2] = 2000
    rotated=False; t0=time.time(); peak=0; pki=0
    while time.time()-t0<60:
        h = hud()
        if not h: continue
        peak=max(peak,h.alt); pki=max(pki,h.airspeed)
        if not rotated and h.airspeed > 25: rc[1] = 1200; rotated=True
        if rotated and h.alt > 40:
            # hand the climb to TECS: it knows the airframe speeds
            rc[1] = 1500; rc[2] = 1500
            mode(12)
            h2 = hud()
            m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE, 0, 160, 3, 0,0,0,0,0)
            print(f"  attempt {n}: airborne, TECS climb to 160")
            t1=time.time()
            while time.time()-t1<90:
                h3 = hud()
                if h3 and h3.alt > 140: print(f"  attempt {n}: AT ALTITUDE"); return True
                if h3 and h3.alt < 1 and h3.airspeed < 3: break
                time.sleep(0.5)
            break
        time.sleep(0.3)
    print(f"  attempt {n}: failed (peak alt {peak:.0f} ias {pki:.0f})"); return False

ok=False
for n in range(1, 6):
    if takeoff(n): ok=True; break
    time.sleep(3)
if not ok: print("NO TAKEOFF"); sys.exit(1)

def sample(secs, tag, positions=None):
    t0=time.time(); last=None; maxroll=0; alts=[]
    while time.time()-t0<secs:
        a = m.recv_match(type='ATTITUDE', blocking=True, timeout=2)
        h = hud()
        gp = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if gp is not None and positions is not None: positions.append((gp.lat/1e7, gp.lon/1e7))
        if a and h:
            last=(round(math.degrees(a.roll),1), round(math.degrees(a.pitch),1), round(h.airspeed,1), round(h.alt,1))
            maxroll=max(maxroll, abs(math.degrees(a.roll))); alts.append(h.alt)
        time.sleep(0.4)
    print(f"  [{tag}] end {last} max|roll| {maxroll:.0f} alt {min(alts):.0f}..{max(alts):.0f}")

pos_calm=[]; sample(90, "LOITER calm", pos_calm)
subprocess.run(['ssh','slon@slon.local','python3 /home/slon/drive.py wind 0 5 150 3; exit 0'], capture_output=True)
print("  wind 5 m/s E + gusts in")
pos_wind=[]; sample(110, "LOITER wind", pos_wind)
def center(ps): return (sum(p[0] for p in ps)/len(ps), sum(p[1] for p in ps)/len(ps)) if ps else (0,0)
cc, cw = center(pos_calm), center(pos_wind)
drift = math.hypot((cw[0]-cc[0])*111319, (cw[1]-cc[1])*111319*math.cos(math.radians(52)))
print(f"  loiter center drift: {drift:.0f} m")
h = hud(); base = h.alt
m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE, 0, base+100, 3, 0,0,0,0,0)
print(f"  TECS climb {base:.0f} -> {base+100:.0f}")
ias_lo, ias_hi = 99.0, 0.0; t0=time.time(); reached=False
while time.time()-t0<150:
    h = hud()
    if h:
        if h.airspeed > 5: ias_lo=min(ias_lo,h.airspeed); ias_hi=max(ias_hi,h.airspeed)
        if h.alt > base + 90: reached=True; break
    time.sleep(0.5)
print(f"  TECS: reached {reached} in {round(time.time()-t0)}s ias {ias_lo:.1f}..{ias_hi:.1f}")
sample(20, "hold")
print("=== DONE ===")
m.close()
