import serial, time, sys, re

HARN_CMD = '/dev/serial/by-id/usb-rotanimb01_hitl-harness_2038334D46325004004F0038-if00'
HARN_CON = '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0668FF545589564867022446-if02'
MAV_PORT = '/dev/cu.usbmodem1301'

def id29(msgid, seq): return (6<<26) | ((msgid&0x7f)<<19) | (1<<16) | (0xb0<<8) | (0x9<<4) | (seq & 0xf)
def pline(msgid, payload, seq=0):
    i = id29(msgid, seq)
    return f"{i>>18:03x}.{i&0x3ffff:05x}:{payload.hex()}\n".encode()

def harness_cmd(msgid, payload, seq=0):
    s = serial.Serial(HARN_CMD, 115200, timeout=0.2)
    s.write(pline(msgid, payload, seq)); s.flush(); s.close()

def console_tail(secs):
    s = serial.Serial(HARN_CON, 115200, timeout=0.3)
    data = b''; t0 = time.time()
    while time.time() - t0 < secs: data += s.read(2048)
    s.close()
    return [l for l in data.decode(errors='replace').split('\n') if l.startswith('t ') or 'ctl' in l]

if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'mode':
        harness_cmd(0x43, bytes([int(sys.argv[2])]) + bytes(7), seq=1)
        for l in console_tail(2.5)[-3:]: print(l)
    elif cmd == 'airstart':
        import struct
        p = struct.pack('>HHHH', int(sys.argv[2]), int(float(sys.argv[3])*10), 0, 0)
        harness_cmd(0x44, p, seq=2)
        for l in console_tail(2.5)[-3:]: print(l)
    elif cmd == 'wind':
        import struct
        # i16 N/E/D cm/s, u8 gust sigma cm/s, u8 gust tau s
        p = struct.pack('>hhh', int(float(sys.argv[2])*100), int(float(sys.argv[3])*100), 0) + bytes([int(sys.argv[4]) if len(sys.argv)>4 else 0, int(sys.argv[5]) if len(sys.argv)>5 else 0])
        harness_cmd(0x46, p, seq=3)
        print("wind sent")
    elif cmd == 'cal':
        # PWM_CAL deflections for an ArduPilot DUT. The harness default cal
        # (controls.c) is all-positive: pwm HIGH = positive deflection, which
        # per fdm.c convention (Cmde < 0) is NOSE-DOWN elevator. ArduPilot
        # outputs ch2 HIGH for nose-UP, so the elevator wants a NEGATIVE full
        # deflection. This cal lives in harness RAM ONLY — resend after EVERY
        # harness reboot (incl. uhubctl power cycles hitting shared hub
        # ports), or the DUT flies with an inverted elevator: no rotation on
        # takeoff, elevator railed, ground-roll overspeed.
        import struct
        for chan, cdeg in ((0, 2000), (1, -2500), (3, 2500)):  # ail, ELE FLIPPED, rud
            harness_cmd(0x45, bytes([chan, 1]) + struct.pack('>h', cdeg) + bytes(4), seq=4+chan)
            time.sleep(0.1)
        for l in console_tail(2.5)[-2:]: print(l)
    elif cmd == 'tail':
        for l in console_tail(float(sys.argv[2])): print(l)
