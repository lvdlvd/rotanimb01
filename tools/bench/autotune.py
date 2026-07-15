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

# proven takeoff
def takeoff():
    rc[0], rc[1], rc[2], rc[3] = 1500, 1500, 1000, 1500
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0,0,0,0,0)
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
    if not armed: return False
    rc[2] = 2000
    rotated=False; t0=time.time()
    while time.time()-t0<120:
        h = hud()
        if not h: continue
        if not rotated and h.airspeed > 25: rc[1] = 1200; rotated=True
        if rotated:
            if h.airspeed > 29 and rc[1] > 1180: rc[1] -= 12
            elif h.airspeed < 25 and rc[1] < 1480: rc[1] += 12
            if h.alt > 150: rc[1] = 1500; rc[2] = 1600; return True
            if h.alt < 1 and h.airspeed < 3 and time.time()-t0 > 25: return False
        time.sleep(0.3)
    return False

ok=False
for n in range(4):
    if takeoff(): ok=True; break
    time.sleep(3)
print("AT ALTITUDE:", ok)
if not ok: sys.exit(1)

# read gains before
def getp(name):
    for _ in range(3):
        m.param_fetch_one(name)
        t0=time.time()
        while time.time()-t0<3:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg and msg.param_id == name: return msg.param_value
    return None
before = {p: getp(p) for p in ['RLL_RATE_FF','RLL_RATE_P','PTCH_RATE_FF','PTCH_RATE_P']}
print("gains before:", before)

mode(8)  # AUTOTUNE
print("AUTOTUNE: roll doublets")
t0=time.time()
while time.time()-t0<100:
    rc[0] = 1800 if int(time.time()-t0) % 8 < 4 else 1200
    h = hud()
    if h and h.alt < 60: rc[1] = 1350  # don't let it sink away
    elif h: rc[1] = 1500
    msg = m.recv_match(type='STATUSTEXT', blocking=False)
    if msg and 'utotune' in msg.text: print("  TEXT:", msg.text)
    time.sleep(0.4)
rc[0] = 1500
print("AUTOTUNE: pitch doublets")
t0=time.time()
while time.time()-t0<100:
    h = hud()
    base = 1500
    if h and h.alt < 70: base = 1400
    if h and h.alt > 250: base = 1560
    rc[1] = base - 180 if int(time.time()-t0) % 8 < 4 else base + 180
    msg = m.recv_match(type='STATUSTEXT', blocking=False)
    if msg and 'utotune' in msg.text: print("  TEXT:", msg.text)
    time.sleep(0.4)
rc[1] = 1500
mode(5)
after = {p: getp(p) for p in ['RLL_RATE_FF','RLL_RATE_P','PTCH_RATE_FF','PTCH_RATE_P']}
print("gains after:", after)
print("=== AUTOTUNE DONE (still flying) ===")
m.close()
