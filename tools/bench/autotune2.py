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
def verify_mode():
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    return hb.custom_mode if hb else -1
def texts():
    while True:
        msg = m.recv_match(type='STATUSTEXT', blocking=False)
        if msg is None: return
        print(f"    TEXT: {msg.text}")
def getp(name):
    for _ in range(3):
        m.param_fetch_one(name)
        t0=time.time()
        while time.time()-t0<3:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg and msg.param_id == name: return round(msg.param_value, 4)
    return None

def takeoff(n):
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
    rotated=False; t0=time.time()
    while time.time()-t0<120:
        h = hud()
        if not h: continue
        if not rotated and h.airspeed > 25: rc[1] = 1200; rotated=True
        if rotated:
            if h.airspeed > 29 and rc[1] > 1180: rc[1] -= 12
            elif h.airspeed < 25 and rc[1] < 1480: rc[1] += 12
            if h.alt > 150:
                rc[1] = 1500; rc[2] = 1600
                print(f"  attempt {n}: AT ALTITUDE"); return True
            if h.alt < 1 and h.airspeed < 3 and time.time()-t0 > 25: return False
        time.sleep(0.3)
    return False

ok=False
for n in range(1, 7):
    if takeoff(n): ok=True; break
    time.sleep(3)
if not ok: print("NO TAKEOFF"); sys.exit(1)

print("gains before:", {p: getp(p) for p in ['RLL_RATE_FF','RLL_RATE_P','RLL_RATE_D']})
mode(8)
time.sleep(1)
print("mode now:", verify_mode(), "(8 = AUTOTUNE)")

# ROLL: aggressive reversals — flip demand as the bank passes through +-20
print("=== AUTOTUNE roll reversals (240 s max) ===")
t0=time.time(); demand=1900
rc[0]=demand
finished=False
while time.time()-t0<240:
    r, p = att()
    h = hud()
    if demand == 1900 and r > 20: demand=1100; rc[0]=demand
    elif demand == 1100 and r < -20: demand=1900; rc[0]=demand
    # altitude keeper
    if h and h.alt < 80: rc[1] = 1330
    elif h and h.alt > 260: rc[1] = 1560
    else: rc[1] = 1470
    msg = m.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        print(f"    TEXT: {msg.text}")
        if 'Finished' in msg.text: finished=True; break
    if h and h.alt < 5: print("  DOWN during roll tune"); break
    time.sleep(0.15)
rc[0]=1500
print("roll gains IN MODE:", {p: getp(p) for p in ['RLL_RATE_FF','RLL_RATE_P','RLL_RATE_D']})

# PITCH reversals
print("=== AUTOTUNE pitch reversals (240 s max) ===")
t0=time.time(); demand=1200
rc[1]=demand
while time.time()-t0<240:
    r, p = att()
    h = hud()
    if demand == 1200 and p > 15: demand=1750; rc[1]=demand
    elif demand == 1750 and p < -10: demand=1200; rc[1]=demand
    if h and h.alt < 80: demand=1200; rc[1]=demand
    if h and h.alt > 300: demand=1750; rc[1]=demand
    msg = m.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        print(f"    TEXT: {msg.text}")
        if 'Finished' in msg.text: break
    if h and h.alt < 5: print("  DOWN during pitch tune"); break
    time.sleep(0.15)
rc[1]=1500
print("pitch gains IN MODE:", {p: getp(p) for p in ['PTCH_RATE_FF','PTCH_RATE_P','PTCH_RATE_D']})
texts()
mode(5)
print("=== AUTOTUNE SESSION END ===")
m.close()
