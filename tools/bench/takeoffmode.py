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

# TAKEOFF-mode parameters for this airframe
for n, v in [('TKOFF_ROTATE_SPD', 26.0), ('TKOFF_ALT', 150.0), ('TKOFF_LVL_ALT', 30.0),
             ('TKOFF_DIST', 800.0), ('TKOFF_LVL_PITCH', 12.0)]:
    setp(n, v)

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
    mode(13)  # TAKEOFF
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0)
    t0=time.time(); armed=False
    while time.time()-t0<8:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and hb.base_mode & 128: armed=True; break
    if not armed: print(f"  attempt {n}: arm refused"); return False
    t0=time.time(); peak=0; pki=0
    while time.time()-t0<120:
        h = hud()
        if not h: continue
        peak=max(peak,h.alt); pki=max(pki,h.airspeed)
        msg = m.recv_match(type='STATUSTEXT', blocking=False)
        if msg: print(f"    TEXT: {msg.text}")
        if h.alt > 140: print(f"  attempt {n}: TAKEOFF MODE SUCCESS (peak ias {pki:.0f})"); return True
        if h.alt < 1 and h.airspeed < 3 and time.time()-t0 > 30: break
        time.sleep(0.4)
    print(f"  attempt {n}: failed (peak {peak:.0f} ias {pki:.0f})"); return False

ok=False; refusals=0
for n in range(1, 7):
    if auto_takeoff(n): ok=True; break
    refusals += 1
    if refusals >= 2: reboot_dut(); refusals=0
    time.sleep(3)
if not ok: print("NO TAKEOFF"); sys.exit(1)

# streams on, then the L1 capture study: 3 loiter entries, one per config
for msgid, hz in [(30, 25), (62, 10), (74, 10), (33, 5)]:
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msgid, int(1e6/hz), 0,0,0,0,0)
    time.sleep(0.2)
w = csv.writer(open('/private/tmp/claude-501/-Users-lvd-Project-stm32/50614b33-98b9-4f22-93e0-3259987dd941/scratchpad/bench/l1study.csv','w'))
w.writerow(['t','leg','msg','a','b','c','d'])
def record(secs, leg):
    t0=time.time(); down=False
    while time.time()-t0<secs:
        msg = m.recv_match(type=['ATTITUDE','NAV_CONTROLLER_OUTPUT','VFR_HUD','GLOBAL_POSITION_INT'], blocking=True, timeout=1)
        if msg is None: continue
        t = round(time.time()-t0, 2); k = msg.get_type()
        if k=='ATTITUDE': w.writerow([t,leg,'ATT',round(math.degrees(msg.roll),2),round(math.degrees(msg.pitch),2),'',''])
        elif k=='NAV_CONTROLLER_OUTPUT': w.writerow([t,leg,'NAV',round(msg.nav_roll,2),round(msg.nav_pitch,2),msg.wp_dist,round(msg.alt_error,1)])
        elif k=='VFR_HUD':
            w.writerow([t,leg,'HUD',round(msg.airspeed,2),round(msg.alt,1),round(msg.climb,2),msg.throttle])
            if msg.alt < 5 and msg.airspeed < 5: down=True; break
        elif k=='GLOBAL_POSITION_INT': w.writerow([t,leg,'POS',msg.lat,msg.lon,'',''])
    print(f"  [{leg}] {'DOWN' if down else 'ok'}")
    return not down

def leg(tag, period, damping, radius, spd, secs=170):
    setp('NAVL1_PERIOD', period); setp('NAVL1_DAMPING', damping)
    setp('WP_LOITER_RAD', radius); setp('AIRSPEED_CRUISE', spd)
    mode(5); time.sleep(1); mode(12)  # re-enter to latch radius
    print(f"LEG {tag}: P{period} D{damping} R{radius} @{spd}")
    return record(secs, tag)

if not leg('p17', 17.0, 0.75, 553.0, 29.0): sys.exit(1)
if not leg('p13', 13.0, 0.80, 553.0, 29.0): sys.exit(1)
if not leg('p10', 10.0, 0.85, 553.0, 29.0): sys.exit(1)
print("=== L1 STUDY RECORDED ===")
m.close()
