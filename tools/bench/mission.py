import sys, time, threading, math, subprocess
sys.path.insert(0, '/private/tmp/claude-501/-Users-lvd-Project-stm32/50614b33-98b9-4f22-93e0-3259987dd941/scratchpad/bench')
from mav import connect
from pymavlink import mavutil
m = connect('tcp:192.168.88.41:5760')
rc = [1500, 1500, 1000, 1500, 0, 0, 0, 0]
def rc_thread():
    while True:
        try: m.mav.rc_channels_override_send(m.target_system, 1, *rc)
        except Exception: pass
        time.sleep(0.2)
threading.Thread(target=rc_thread, daemon=True).start()

def reboot_dut():
    global m
    import subprocess as sp
    print("  (attitude stuck: rebooting DUT)")
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1,0,0,0,0,0,0)
    try: m.close()
    except Exception: pass
    time.sleep(15)
    sp.run(['ssh','slon@slon.local',
        'fuser -k /dev/serial/by-id/usb-ArduPilot_NucleoF767ZI_240044000451323232383933-if00 2>/dev/null; sleep 1; setsid nohup python3 /home/slon/serbridge.py /dev/serial/by-id/usb-ArduPilot_NucleoF767ZI_240044000451323232383933-if00 5760 </dev/null >/home/slon/serbridge.log 2>&1 & sleep 1.5; exit 0'], capture_output=True)
    m = connect('tcp:192.168.88.41:5760')
    t0 = time.time()
    while time.time()-t0 < 30:
        g = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if g and g.fix_type >= 3: break
    time.sleep(5)
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
    # settle: estimator level and quiet before arming
    t0=time.time(); level=False
    while time.time()-t0<25:
        r, p = att_deg()
        if abs(r) < 2 and abs(p) < 2: level=True; break
        time.sleep(1)
    if not level:
        reboot_dut()
    time.sleep(10)
    mode(5)
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0)
    t0=time.time(); armed=False
    while time.time()-t0<8:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and hb.base_mode & 128: armed=True; break
    if not armed: print(f"  attempt {n}: arm refused"); return False
    rc[2] = 2000
    rotated=False; t0=time.time(); peak=0; pki=0; ias_f=0.0
    while time.time()-t0<120:
        h = hud()
        if not h: continue
        peak=max(peak,h.alt); pki=max(pki,h.airspeed)
        ias_f = 0.6*ias_f + 0.4*h.airspeed  # EMA against sensor noise
        if not rotated and ias_f > 24: rc[1] = 1280; rotated=True
        if rotated:
            # proportional Vy hold: pitch for 27 m/s, rate-limited
            tgt = 1350 - 10.0*(ias_f - 27.0)
            tgt = max(1180, min(1500, tgt))
            rc[1] += max(-25, min(25, int(tgt - rc[1])))
            if h.alt > 150:
                rc[1] = 1500; rc[2] = 1500
                mode(12)
                print(f"  attempt {n}: AT ALTITUDE"); return True
            if h.alt < 1 and ias_f < 3 and time.time()-t0 > 25: break
        time.sleep(0.3)
    print(f"  attempt {n}: failed (peak alt {peak:.0f} ias {pki:.0f})"); return False

ok=False
for n in range(1, 6):
    if takeoff(n): ok=True; break
    time.sleep(3)
if not ok: print("NO TAKEOFF"); sys.exit(1)

def sample(secs, tag, positions=None):
    t0=time.time(); last=None; maxroll=0; alts=[]; trk=[]; dmd=[]
    while time.time()-t0<secs:
        a = m.recv_match(type='ATTITUDE', blocking=True, timeout=2)
        h = hud()
        nav = m.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=False)
        gp = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if gp is not None and positions is not None: positions.append((gp.lat/1e7, gp.lon/1e7))
        if a and h:
            roll = math.degrees(a.roll)
            last=(round(roll,1), round(math.degrees(a.pitch),1), round(h.airspeed,1), round(h.alt,1))
            maxroll=max(maxroll, abs(roll)); alts.append(h.alt)
            if nav is not None:
                trk.append(abs(roll - nav.nav_roll)); dmd.append(abs(nav.nav_roll))
        time.sleep(0.4)
    if trk:
        trk.sort(); dmd.sort()
        print(f"  [{tag}] end {last} max|roll| {maxroll:.0f} alt {min(alts):.0f}..{max(alts):.0f}"
              f" | demand p50/max {dmd[len(dmd)//2]:.0f}/{dmd[-1]:.0f} track-err p50/max {trk[len(trk)//2]:.1f}/{trk[-1]:.0f}")
    else:
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
