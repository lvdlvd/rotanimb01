from pymavlink import mavutil as _mu
import time
_orig = _mu.add_message
def _safe(messages, mtype, msg):
    try: _orig(messages, mtype, msg)
    except TypeError: messages[mtype] = msg
_mu.add_message = _safe

def connect(port='/dev/cu.usbmodem1301'):
    m = _mu.mavlink_connection(port, baud=115200, source_system=255)
    t0=time.time()
    while time.time()-t0<15:
        if m.recv_match(type='HEARTBEAT', blocking=True, timeout=2): return m
    raise RuntimeError('no heartbeat')
