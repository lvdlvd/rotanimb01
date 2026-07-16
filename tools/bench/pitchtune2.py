import sys, time, threading, math, csv, subprocess
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
    print("  (rebooting DUT)")
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1,0,0,0,0,0,0)
    try: m.close()
    except Exception: pass
    time.sleep(15)
    subprocess.run(['ssh','slon@slon.local','fuser -k /dev/serial/by-id/usb-ArduPilot_NucleoF767ZI_240044000451323232383933-if00 2>/dev/null; sleep 1; setsid nohup python3 /home/slon/serbridge.py /dev/serial/by-id/usb-ArduPilot_NucleoF767ZI_240044000451323232383933-if00 5760 </dev/null >/home/slon/serbridge.log 2>&1 & sleep 1.5; exit 0'], capture_output=True)
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
def setp(name, v):
    m.mav.param_set_send(m.target_system, 1, name.encode(), v, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.4)
def getp(name):
    for _ in range(3):
        m.param_fetch_one(name)
        t0=time.time()
        while time.time()-t0<3:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg and msg.param_id == name: return round(msg.param_value, 4)
    return None

def auto_takeoff(n):
    rc[0], rc[1], rc[2], rc[3] = 1500, 1500, 1000, 1500
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0,0,0,0,0)
    t0=time.time(); level=False
    while time.time()-t0<25:
        r, p = att()
        if abs(r) < 2 and abs(p) < 2: level=True; break
        time.sleep(1)
    if not level: reboot_dut()
    time.sleep(8)
    mode(13)
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0)
    t0=time.time(); armed=False
    while time.time()-t0<8:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and hb.base_mode & 128: armed=True; break
    if not armed: print(f"  attempt {n}: arm refused"); return False
    t0=time.time()
    while time.time()-t0<120:
        h = hud()
        if h and h.alt > 140: print(f"  attempt {n}: airborne at 150"); return True
        if h and h.alt < 1 and h.airspeed < 3 and time.time()-t0 > 30: break
        time.sleep(0.4)
    print(f"  attempt {n}: takeoff failed"); return False

ok=False; refusals=0
for n in range(1, 7):
    if auto_takeoff(n): ok=True; break
    refusals += 1
    if refusals >= 2: reboot_dut(); refusals=0
    time.sleep(3)
if not ok: print("NO TAKEOFF"); sys.exit(1)

# TECS climb to 400 in LOITER (give it time; pitch is untuned, that's the point)
mode(12)
m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE, 0, 400, 3, 0,0,0,0,0)
print("TECS climb to 400...")
t0=time.time()
while time.time()-t0<300:
    h = hud()
    if h and h.alt > 380: break
    time.sleep(1)
h = hud(); print(f"at {h.alt:.0f} m — pitch AUTOTUNE, floor 180")

mode(8); time.sleep(1)
t0=time.time(); demand=1200; rc[1]=demand
while time.time()-t0<300:
    r, p = att()
    h = hud()
    if demand == 1200 and p > 15: demand=1750; rc[1]=demand
    elif demand == 1750 and p < -10: demand=1200; rc[1]=demand
    if h and h.alt < 180: demand=1200; rc[1]=demand; rc[2]=2000
    elif h: rc[2]=1600
    if h and h.alt > 430: demand=1750; rc[1]=demand
    msg = m.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        print(f"    TEXT: {msg.text}")
        if 'Finished' in msg.text: break
    if h and h.alt < 5: print("  DOWN"); break
    time.sleep(0.15)
rc[1]=1500
gains = {p: getp(p) for p in ['PTCH_RATE_FF','PTCH_RATE_P','PTCH_RATE_D','PTCH_RATE_I']}
print("pitch gains IN MODE:", gains)
mode(5)
# re-apply explicitly (stop() restores unless finished)
for k, v in gains.items():
    if v is not None: setp(k, v)
print("gains re-applied post-exit")
print("=== PITCH TUNE COMPLETE — still flying ===")
m.close()
