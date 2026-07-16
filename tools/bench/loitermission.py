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

def setup_streams():
    for msgid, hz in [(30, 25), (62, 10), (74, 10), (33, 5)]:  # ATTITUDE, NAV_CTRL, VFR_HUD, GLOBAL_POS
        m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msgid, int(1e6/hz), 0,0,0,0,0)
        time.sleep(0.2)

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
def speed(v):
    setp('AIRSPEED_CRUISE', v)  # TECS reads it live; DO_CHANGE_SPEED doesn't take in LOITER
def reloiter(radius, v):
    # WP_LOITER_RAD is latched at LOITER entry: set params, then re-enter
    setp('WP_LOITER_RAD', radius)
    speed(v)
    mode(5); time.sleep(1); mode(12)

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
    rotated=False; t0=time.time(); ias_f=0.0; peak=0
    while time.time()-t0<120:
        h = hud()
        if not h: continue
        peak=max(peak,h.alt)
        ias_f = 0.6*ias_f + 0.4*h.airspeed
        if not rotated and ias_f > 24: rc[1] = 1280; rotated=True
        if rotated:
            tgt = max(1180, min(1500, 1350 - 10.0*(ias_f - 27.0)))
            rc[1] += max(-25, min(25, int(tgt - rc[1])))
            if h.alt > 150:
                rc[1] = 1500; rc[2] = 1500
                mode(12)
                print(f"  attempt {n}: AT ALTITUDE"); return True
            if h.alt < 1 and ias_f < 3 and time.time()-t0 > 25: break
        time.sleep(0.3)
    print(f"  attempt {n}: failed (peak {peak:.0f})"); return False

ok=False; refusals=0
for n in range(1, 8):
    r = takeoff(n)
    if r: ok=True; break
    refusals += 1
    if refusals >= 2:
        # DCM drifted off the bias walk (EKF fine): only a reboot resets DCM
        reboot_dut(); refusals = 0
    time.sleep(3)
if not ok: print("NO TAKEOFF"); sys.exit(1)

setup_streams()  # high-rate streams only once airborne — they starve the takeoff loop
w = csv.writer(open('/private/tmp/claude-501/-Users-lvd-Project-stm32/50614b33-98b9-4f22-93e0-3259987dd941/scratchpad/bench/loiter.csv','w'))
w.writerow(['t','leg','msg','a','b','c','d'])
def record(secs, leg):
    t0=time.time(); down=False
    while time.time()-t0<secs:
        msg = m.recv_match(type=['ATTITUDE','NAV_CONTROLLER_OUTPUT','VFR_HUD','GLOBAL_POSITION_INT'], blocking=True, timeout=1)
        if msg is None: continue
        t = round(time.time()-t0, 2); k = msg.get_type()
        if k=='ATTITUDE': w.writerow([t,leg,'ATT',round(math.degrees(msg.roll),2),round(math.degrees(msg.pitch),2),round(math.degrees(msg.yaw),1),''])
        elif k=='NAV_CONTROLLER_OUTPUT': w.writerow([t,leg,'NAV',round(msg.nav_roll,2),round(msg.nav_pitch,2),msg.wp_dist,round(msg.alt_error,1)])
        elif k=='VFR_HUD':
            w.writerow([t,leg,'HUD',round(msg.airspeed,2),round(msg.alt,1),round(msg.climb,2),msg.throttle])
            if msg.alt < 5 and msg.airspeed < 5: down=True; break
        elif k=='GLOBAL_POSITION_INT': w.writerow([t,leg,'POS',msg.lat,msg.lon,'',''])
    print(f"  [{leg}] {'DOWN' if down else 'complete'}")
    return not down

# corrected to the MODEL's true cruise (~38 m/s at 45% throttle = the owner's
# 90 kt Kitfox): standard = 3 deg/s at (38+19.8)/2 = 29 -> R 553, bank 8.8;
# fast = 6 deg/s at 38 -> R 363, bank 22. The first attempt commanded 22 m/s
# — BELOW the accelerated stall at the banks L1 demanded.
reloiter(553.0, 29.0)
print("LEG standard: R553 @ 29 m/s (geom bank 8.8 deg)")
if not record(180, 'std'): sys.exit(1)
reloiter(363.0, 38.0)
print("LEG fast: R363 @ 38 m/s (geom bank 22 deg)")
if not record(180, 'fast'): sys.exit(1)
reloiter(553.0, 29.0)
subprocess.run(['ssh','slon@slon.local','python3 /home/slon/drive.py wind 0 5 150 3; exit 0'], capture_output=True)
print("LEG standard+wind")
record(150, 'stdwind')
subprocess.run(['ssh','slon@slon.local','python3 /home/slon/drive.py wind 0 0; exit 0'], capture_output=True)
print("=== RECORDED ===")
m.close()
