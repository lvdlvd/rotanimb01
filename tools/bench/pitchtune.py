import sys, time, threading, math
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
    print("  (rebooting DUT)")
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1,0,0,0,0,0,0)
    try: m.close()
    except Exception: pass
    time.sleep(15)
    sp.run(['ssh','slon@slon.local','fuser -k /dev/serial/by-id/usb-ArduPilot_NucleoF767ZI_240044000451323232383933-if00 2>/dev/null; sleep 1; setsid nohup python3 /home/slon/serbridge.py /dev/serial/by-id/usb-ArduPilot_NucleoF767ZI_240044000451323232383933-if00 5760 </dev/null >/home/slon/serbridge.log 2>&1 & sleep 1.5; exit 0'], capture_output=True)
    m = connect('tcp:192.168.88.41:5760')
    t0=time.time()
    while time.time()-t0<30:
        g = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if g and g.fix_type >= 3: break
    time.sleep(5)
def hud(): return m.recv_match(type='VFR_HUD', blocking=True, timeout=2)
def att():
    a = m.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    return (math.degrees(a.roll), math.degrees(a.pitch)) if a else (0,0)
def mode(n):
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, n, 0,0,0,0,0)
    m.recv_match(type='COMMAND_ACK', blocking=True, timeout=4)
def getp(name):
    for _ in range(3):
        m.param_fetch_one(name)
        t0=time.time()
        while time.time()-t0<3:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg and msg.param_id == name: return round(msg.param_value, 4)
    return None
def takeoff(n, top=340):
    rc[0], rc[1], rc[2], rc[3] = 1500, 1500, 1000, 1500
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0,0,0,0,0)
    t0=time.time(); level=False
    while time.time()-t0<25:
        r, p = att()
        if abs(r) < 2 and abs(p) < 2: level=True; break
        time.sleep(1)
    if not level: reboot_dut()
    time.sleep(10)
    mode(5)
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0)
    t0=time.time(); armed=False
    while time.time()-t0<8:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and hb.base_mode & 128: armed=True; break
    if not armed: print(f"  attempt {n}: arm refused"); return False
    rc[2] = 2000
    rotated=False; t0=time.time(); ias_f=0.0; peak=0
    while time.time()-t0<180:
        h = hud()
        if not h: continue
        peak=max(peak,h.alt)
        ias_f = 0.6*ias_f + 0.4*h.airspeed
        if not rotated and ias_f > 24: rc[1] = 1280; rotated=True
        if rotated:
            tgt = max(1180, min(1500, 1350 - 10.0*(ias_f - 27.0)))
            rc[1] += max(-25, min(25, int(tgt - rc[1])))
            if h.alt > top:
                print(f"  attempt {n}: AT {top} m"); return True
            if h.alt < 1 and ias_f < 3 and time.time()-t0 > 25: break
        time.sleep(0.3)
    print(f"  attempt {n}: failed (peak {peak:.0f})"); return False

ok=False
for n in range(1, 7):
    if takeoff(n): ok=True; break
    time.sleep(3)
if not ok: print("NO TAKEOFF"); sys.exit(1)

rc[1] = 1500; rc[2] = 1600
mode(8)
time.sleep(1)
print("=== PITCH AUTOTUNE from 340 m, floor 120 m (300 s max) ===")
t0=time.time(); demand=1200; rc[1]=demand
while time.time()-t0<300:
    r, p = att()
    h = hud()
    if demand == 1200 and p > 15: demand=1750; rc[1]=demand
    elif demand == 1750 and p < -10: demand=1200; rc[1]=demand
    if h and h.alt < 120: demand=1200; rc[1]=demand   # nose up, full throttle
    if h and h.alt < 120: rc[2]=2000
    elif h: rc[2]=1600
    if h and h.alt > 380: demand=1750; rc[1]=demand
    msg = m.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        print(f"    TEXT: {msg.text}")
        if 'Finished' in msg.text: break
    if h and h.alt < 5: print("  DOWN"); break
    time.sleep(0.15)
rc[1]=1500
print("pitch gains IN MODE:", {p: getp(p) for p in ['PTCH_RATE_FF','PTCH_RATE_P','PTCH_RATE_D','PTCH_RATE_I']})
mode(5)
print("=== PITCH TUNE SESSION END ===")
m.close()
