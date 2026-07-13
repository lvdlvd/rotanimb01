// rb01tool — console cockpit for the rotanimb01 HITL harness.
//
// Connects to the harness's USB CDC-ACM port (pseudocan lines, lib/fmtcan
// format), shows the live system state and lets single keys steer the
// physics: speed, climb and turn-rate commands (CMD_STATE, auto-repeated so
// the harness never sees them go stale) and the environment (CMD_ENV).
//
//	rb01tool [-p /dev/cu.usbmodemXXX]
//
// With no -p the single /dev/cu.usbmodem* is used; multiple candidates are
// listed instead. The display repaints in place ~10x/s; 'q' quits.
//
// Wire format (lib/fmtcan): ID-A['.'ID-B]['R'] ':' hexpayload [':'crc16]
// [' 'port[' 'fmi]] '\n' — crc16 poly 0xc599 msb-first over the 4 header
// bytes then the payload, priority-preserving uint32 header representation
// (lib/can.h). The harness emits crc-less lines; we send with crc (the
// harness verifies a crc when present). Dictionary: src/canmsg.h.
package main

import (
	"bufio"
	"encoding/binary"
	"encoding/hex"
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"

	"golang.org/x/term"
)

// ---- pseudocan wire representation ------------------------------------------

// Header is the priority-preserving uint32 representation of a CAN id
// (lib/can.h): ID-A in bits 30:20, RTR bit 19, EXT bit 18, ID-B in 17:0.
type Header uint32

const (
	hdrEXT Header = 1 << 18
	hdrRTR Header = 1 << 19
)

func mkHeader29(id29 uint32) Header {
	return Header((id29&0x1ffc0000)<<2) | hdrEXT | Header(id29&0x3ffff)
}

func (h Header) id29() uint32 { return uint32((h&(0x7ff<<20))>>2 | h&0x3ffff) }
func (h Header) isExt() bool  { return h&hdrEXT != 0 }

func (h Header) String() string {
	ida := uint32(h>>20) & 0x7ff
	if !h.isExt() {
		return fmt.Sprintf("%03x", ida)
	}
	return fmt.Sprintf("%03x.%05x", ida, uint32(h&0x3ffff))
}

func crc16Next(crc uint16, b byte) uint16 {
	crc ^= uint16(b) << 8
	for i := 0; i < 8; i++ {
		if crc&0x8000 != 0 {
			crc ^= 0xc599
		}
		crc <<= 1
	}
	return crc
}

func checksum(h Header, payload []byte) uint16 {
	var crc uint16
	for s := 24; s >= 0; s -= 8 {
		crc = crc16Next(crc, byte(h>>s))
	}
	for _, b := range payload {
		crc = crc16Next(crc, b)
	}
	return crc
}

// parseLine accepts the full grammar with crc, port and fmi each optional;
// a present crc must verify.
func parseLine(line string) (Header, []byte, error) {
	line = strings.TrimRight(line, "\r\n")
	if line == "" {
		return 0, nil, errEmpty
	}
	colon := strings.IndexByte(line, ':')
	if colon < 0 {
		return 0, nil, fmt.Errorf("no ':' in %q", line)
	}
	hs, rest := line[:colon], line[colon+1:]

	rtr := strings.HasSuffix(hs, "R")
	hs = strings.TrimSuffix(hs, "R")
	var h Header
	if dot := strings.IndexByte(hs, '.'); dot >= 0 {
		ida, err := strconv.ParseUint(hs[:dot], 16, 11)
		if err != nil {
			return 0, nil, fmt.Errorf("id-a in %q: %v", line, err)
		}
		idb, err := strconv.ParseUint(hs[dot+1:], 16, 18)
		if err != nil {
			return 0, nil, fmt.Errorf("id-b in %q: %v", line, err)
		}
		h = mkHeader29(uint32(ida)<<18 | uint32(idb))
	} else {
		ida, err := strconv.ParseUint(hs, 16, 11)
		if err != nil {
			return 0, nil, fmt.Errorf("id in %q: %v", line, err)
		}
		h = Header(ida << 20)
	}
	if rtr {
		h |= hdrRTR
	}

	// rest = hexpayload [':'crc] [' 'port [' 'fmi]]
	var crcs string
	if c := strings.IndexByte(rest, ':'); c >= 0 {
		crcs = rest[c+1:]
		rest = rest[:c]
	}
	payloadHex := rest
	if sp := strings.IndexByte(rest, ' '); sp >= 0 {
		payloadHex = rest[:sp] // trailing " port fmi" ignored
	}
	payload, err := hex.DecodeString(payloadHex)
	if err != nil || len(payload) > 8 {
		return 0, nil, fmt.Errorf("payload in %q", line)
	}
	if crcs != "" {
		if sp := strings.IndexByte(crcs, ' '); sp >= 0 {
			crcs = crcs[:sp]
		}
		want, err := strconv.ParseUint(crcs, 16, 16)
		if err != nil {
			return 0, nil, fmt.Errorf("crc in %q", line)
		}
		if checksum(h, payload) != uint16(want) {
			return 0, nil, fmt.Errorf("bad crc in %q", line)
		}
	}
	return h, payload, nil
}

var errEmpty = fmt.Errorf("empty line")

// ---- the harness dictionary (src/canmsg.h) -----------------------------------

const (
	lccMEAS = 1
	lccTMC  = 6

	cmdSTATE = 0x40 // TMC: i16 V cm/s, i16 hdot cm/s, i16 psidot mrad/s, u16 flags
	cmdENV   = 0x41 // TMC: u16 QNH Pa/10, u16 T0 0.1K, u16 B 0.01uT, i16 incl 0.01deg

	measPWM14  = 0x40 // 4 x u16 us
	measPWM58  = 0x41
	measSTATUS = 0x42 // u32 time us, u16 psi 0.01deg, u16 flags (bit0 cmd stale)
	measDIAG   = 0x43 // 4 x u16 (usb transport: 0, usb bad, usb drop, pwm errs)
)

func id29(lcc, msgid, srcid, seq uint32) uint32 {
	return lcc<<26 | (msgid&0x7f)<<19 | 1<<16 | (srcid&0xff)<<8 | 0x9<<4 | seq&0xf
}

func lccOf(id uint32) uint32   { return id >> 26 & 7 }
func msgidOf(id uint32) uint32 { return id >> 19 & 0x7f }
func srcidOf(id uint32) uint32 { return id >> 8 & 0xff }

// ---- live state ---------------------------------------------------------------

type counter struct {
	n      uint64
	window []time.Time // for the rate, last few seconds
}

func (c *counter) hit(now time.Time) {
	c.n++
	c.window = append(c.window, now)
	cut := now.Add(-3 * time.Second)
	i := 0
	for i < len(c.window) && c.window[i].Before(cut) {
		i++
	}
	c.window = c.window[i:]
}

func (c *counter) rate() float64 { return float64(len(c.window)) / 3.0 }

type state struct {
	sync.Mutex
	lines, errs uint64
	lastErr     string
	lastRx      time.Time

	srcid  uint32
	status struct {
		t   time.Time
		us  uint32
		psi float64
		flg uint16
	}
	pwm  [8]uint16
	diag [4]uint16

	byID map[uint32]*counter // per 29-bit-id (seq masked) traffic summary
}

// ---- commands -----------------------------------------------------------------

type commands struct {
	v, hdot, psidot    float64 // m/s, m/s, deg/s
	qnh, t0, b, incl   float64 // Pa, K, uT, deg
	sentState, sentEnv uint64
	seq                uint32
}

func defaultCommands() commands {
	return commands{qnh: 101325, t0: 288.15, b: 48.33, incl: 65.6} // physics.c defaults
}

func (c *commands) statePayload() []byte {
	p := make([]byte, 8)
	binary.BigEndian.PutUint16(p[0:], uint16(int16(c.v*100)))                      // cm/s
	binary.BigEndian.PutUint16(p[2:], uint16(int16(c.hdot*100)))                   // cm/s
	binary.BigEndian.PutUint16(p[4:], uint16(int16(c.psidot*1000*3.14159265/180))) // mrad/s
	return p
}

func (c *commands) envPayload() []byte {
	p := make([]byte, 8)
	binary.BigEndian.PutUint16(p[0:], uint16(c.qnh/10))          // Pa/10
	binary.BigEndian.PutUint16(p[2:], uint16(c.t0*10))           // 0.1 K
	binary.BigEndian.PutUint16(p[4:], uint16(c.b*100))           // 0.01 uT
	binary.BigEndian.PutUint16(p[6:], uint16(int16(c.incl*100))) // 0.01 deg
	return p
}

func send(w *os.File, msgid uint32, payload []byte, seq uint32) error {
	h := mkHeader29(id29(lccTMC, msgid, 0xb0, seq)) // srcid 0xb0: this tool
	_, err := fmt.Fprintf(w, "%v:%s:%04x 1 0\n", h, hex.EncodeToString(payload), checksum(h, payload))
	return err
}

// ---- terminal -----------------------------------------------------------------

const (
	esc     = "\x1b["
	home    = esc + "H"
	clrEOS  = esc + "J"
	clrEOL  = esc + "K"
	bold    = esc + "1m"
	dim     = esc + "2m"
	inverse = esc + "7m"
	normal  = esc + "0m"
)

func findPort(flagged string) (string, error) {
	if flagged != "" {
		return flagged, nil
	}
	m, _ := filepath.Glob("/dev/cu.usbmodem*")
	switch len(m) {
	case 0:
		return "", fmt.Errorf("no /dev/cu.usbmodem* found; pass -p")
	case 1:
		return m[0], nil
	}
	return "", fmt.Errorf("multiple candidates, pass -p:\n  %s", strings.Join(m, "\n  "))
}

func main() {
	fPort := flag.String("p", "", "serial port (default: the single /dev/cu.usbmodem*)")
	flag.Parse()

	port, err := findPort(*fPort)
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	dev, err := os.OpenFile(port, os.O_RDWR, 0)
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	defer dev.Close()
	devRestore, err := term.MakeRaw(int(dev.Fd())) // kill the line discipline
	if err != nil {
		fmt.Fprintf(os.Stderr, "raw mode on %s: %v\n", port, err)
		os.Exit(1)
	}
	defer term.Restore(int(dev.Fd()), devRestore)

	inRestore, err := term.MakeRaw(int(os.Stdin.Fd()))
	if err != nil {
		fmt.Fprintf(os.Stderr, "raw mode on stdin: %v\n", err)
		os.Exit(1)
	}
	defer term.Restore(int(os.Stdin.Fd()), inRestore)
	defer fmt.Print(normal + "\r\n")

	st := &state{byID: map[uint32]*counter{}}
	cmd := defaultCommands()
	var cmdMu sync.Mutex

	// reader: harness lines -> state
	go func() {
		sc := bufio.NewScanner(dev)
		sc.Buffer(make([]byte, 4096), 4096)
		for sc.Scan() {
			h, p, err := parseLine(sc.Text())
			now := time.Now()
			st.Lock()
			st.lines++
			st.lastRx = now
			if err != nil {
				if err != errEmpty {
					st.errs++
					st.lastErr = err.Error()
				}
				st.Unlock()
				continue
			}
			id := h.id29() &^ 0xf // fold the seq nibble
			c := st.byID[id]
			if c == nil {
				c = &counter{}
				st.byID[id] = c
			}
			c.hit(now)
			if h.isExt() && lccOf(h.id29()) == lccMEAS {
				st.srcid = srcidOf(h.id29())
				switch m := msgidOf(h.id29()); {
				case m == measSTATUS && len(p) == 8:
					st.status.t = now
					st.status.us = binary.BigEndian.Uint32(p)
					st.status.psi = float64(binary.BigEndian.Uint16(p[4:])) / 100
					st.status.flg = binary.BigEndian.Uint16(p[6:])
				case m == measPWM14 && len(p) == 8:
					for i := 0; i < 4; i++ {
						st.pwm[i] = binary.BigEndian.Uint16(p[2*i:])
					}
				case m == measPWM58 && len(p) == 8:
					for i := 0; i < 4; i++ {
						st.pwm[4+i] = binary.BigEndian.Uint16(p[2*i:])
					}
				case m == measDIAG && len(p) == 8:
					for i := 0; i < 4; i++ {
						st.diag[i] = binary.BigEndian.Uint16(p[2*i:])
					}
				}
			}
			st.Unlock()
		}
	}()

	// keys: adjust commands; quit
	quit := make(chan struct{})
	go func() {
		buf := make([]byte, 1)
		for {
			if n, err := os.Stdin.Read(buf); err != nil || n == 0 {
				close(quit)
				return
			}
			cmdMu.Lock()
			switch buf[0] {
			case 'q', 'Q', 3: // 3 = ctrl-c in raw mode
				cmdMu.Unlock()
				close(quit)
				return
			case 'v':
				cmd.v -= 1
			case 'V':
				cmd.v += 1
			case 'h':
				cmd.hdot -= 0.5
			case 'H':
				cmd.hdot += 0.5
			case 'r':
				cmd.psidot -= 1
			case 'R':
				cmd.psidot += 1
			case ' ':
				cmd.v, cmd.hdot, cmd.psidot = 0, 0, 0
			case 'p':
				cmd.qnh -= 100
			case 'P':
				cmd.qnh += 100
			case 't':
				cmd.t0 -= 1
			case 'T':
				cmd.t0 += 1
			case 'e', 'E':
				env := defaultCommands()
				cmd.qnh, cmd.t0, cmd.b, cmd.incl = env.qnh, env.t0, env.b, env.incl
			}
			cmdMu.Unlock()
		}
	}()

	// senders: CMD_STATE at ~3 Hz keeps the harness's stale timer fed;
	// CMD_ENV every 2 s
	stateTick := time.Tick(300 * time.Millisecond)
	envTick := time.Tick(2 * time.Second)
	paint := time.Tick(100 * time.Millisecond)
	fmt.Print(home + clrEOS)

	for {
		select {
		case <-quit:
			return
		case <-stateTick:
			cmdMu.Lock()
			cmd.seq++
			err := send(dev, cmdSTATE, cmd.statePayload(), cmd.seq)
			cmd.sentState++
			cmdMu.Unlock()
			if err != nil {
				return
			}
		case <-envTick:
			cmdMu.Lock()
			cmd.seq++
			send(dev, cmdENV, cmd.envPayload(), cmd.seq)
			cmd.sentEnv++
			cmdMu.Unlock()
		case <-paint:
			repaint(port, st, &cmd, &cmdMu)
		}
	}
}

func repaint(port string, st *state, cmd *commands, cmdMu *sync.Mutex) {
	var b strings.Builder
	line := func(format string, args ...interface{}) {
		fmt.Fprintf(&b, format, args...)
		b.WriteString(clrEOL + "\r\n")
	}

	st.Lock()
	cmdMu.Lock()

	b.WriteString(home)
	line("%srb01tool%s — rotanimb01 cockpit on %s", bold, normal, port)
	age := "never"
	if !st.lastRx.IsZero() {
		age = fmt.Sprintf("%.1fs ago", time.Since(st.lastRx).Seconds())
	}
	line("link   %d lines, %d errors, last rx %s", st.lines, st.errs, age)
	if st.lastErr != "" {
		line("%slast error: %.70s%s", dim, st.lastErr, normal)
	}
	line("")

	stale := ""
	if st.status.flg&1 != 0 {
		stale = inverse + " CMD STALE " + normal
	}
	sAge := time.Since(st.status.t).Seconds()
	line("%sSTATUS%s  t=%9.3f s   psi=%7.2f°   flags=%04x %s   (%.1fs ago, src %02x)",
		bold, normal, float64(st.status.us)/1e6, st.status.psi, st.status.flg, stale, sAge, st.srcid)
	line("%sPWM%s     1:%5d  2:%5d  3:%5d  4:%5d", bold, normal, st.pwm[0], st.pwm[1], st.pwm[2], st.pwm[3])
	line("        5:%5d  6:%5d  7:%5d  8:%5d", st.pwm[4], st.pwm[5], st.pwm[6], st.pwm[7])
	line("%sDIAG%s    usb bad %d  drop %d  pwm errs %d", bold, normal, st.diag[1], st.diag[2], st.diag[3])
	line("")
	line("%sCMD%s     V=%6.1f m/s   hdot=%+5.1f m/s   psidot=%+5.1f °/s   (sent %d)",
		bold, normal, cmd.v, cmd.hdot, cmd.psidot, cmd.sentState)
	line("%sENV%s     QNH=%6.0f Pa   T0=%5.1f K   B=%5.2f µT   incl=%4.1f°   (sent %d)",
		bold, normal, cmd.qnh, cmd.t0, cmd.b, cmd.incl, cmd.sentEnv)
	line("")

	ids := make([]uint32, 0, len(st.byID))
	for id := range st.byID {
		ids = append(ids, id)
	}
	sort.Slice(ids, func(i, j int) bool { return ids[i] < ids[j] })
	line("%straffic%s        id   lcc msgid src      count    rate", bold, normal)
	for _, id := range ids {
		c := st.byID[id]
		line("        %03x.%05x   %d    %02x   %02x %10d %7.1f/s",
			id>>18&0x7ff, id&0x3ffff, lccOf(id), msgidOf(id), srcidOf(id), c.n, c.rate())
	}
	line("")
	line("%skeys%s  V/v speed±1  H/h climb±.5  R/r turn±1  SPACE level  P/p qnh±100  T/t temp±1  e env-reset  q quit",
		dim, normal)
	b.WriteString(clrEOS)

	cmdMu.Unlock()
	st.Unlock()

	os.Stdout.WriteString(b.String())
}
