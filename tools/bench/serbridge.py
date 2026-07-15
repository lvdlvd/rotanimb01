import serial, socket, select, sys
dev, port = sys.argv[1], int(sys.argv[2])
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", port)); srv.listen(1)
while True:
    conn, addr = srv.accept()
    print("client", addr, flush=True)
    s = serial.Serial(dev, 115200, timeout=0)
    try:
        while True:
            r, _, _ = select.select([conn, s], [], [], 1)
            if s in r:
                d = s.read(4096)
                if d: conn.sendall(d)
            if conn in r:
                d = conn.recv(4096)
                if not d: break
                s.write(d)
    except (ConnectionResetError, BrokenPipeError, OSError) as e:
        print("drop:", e, flush=True)
    finally:
        s.close(); conn.close()
