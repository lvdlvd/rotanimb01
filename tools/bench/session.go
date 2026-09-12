// session — a MAVLink GCS session over TCP (serbridge or SITL) with the
// helpers every mission needs. source_system MUST be 255: ArduPilot silently
// ignores RC overrides from anyone but SYSID_MYGCS.
package main

import (
	"fmt"
	"math"
	"net"
	"strings"
	"time"
)

const (
	gcsSys  = 255
	gcsComp = 190

	ModeManual   = 0
	ModeFBWA     = 5
	ModeAutotune = 8
	ModeRTL      = 11
	ModeLoiter   = 12
	ModeTakeoff  = 13
	ModeHdgAlt   = 27 // PLANE_MODE_HDGALT: experimental, NOT stock ArduPlane — branch
	// hdgaltmode of https://github.com/lvdlvd/ardupilot (hdgalt_dev.xml), no
	// warranty; nothing in this repository needs it (README)

	cmdArmDisarm          = 400
	cmdRebootShutdown     = 246
	cmdSetMessageInterval = 511
)

// State is the continuously-pumped picture of the aircraft.
type State struct {
	SimMs               uint32 // max time_boot_ms seen: sim time on SITL, boot time on bench
	Alt, IAS, Climb     float64
	Throttle            int
	Roll, Pitch, Yaw    float64 // deg
	NavRoll, NavPitch   float64
	AltError, AspdError float64
	Lat, Lon            float64
	GPSFix              int
	GPSVel, GPSCog      float64
	Vx, Vy              float64
	Servo               [8]uint16
	Armed               bool
	Mode                uint32
	HaveHB              bool
}

type Session struct {
	conn      net.Conn
	parser    Parser
	seq       byte
	target    byte // target system id
	St        State
	Debug     bool
	QuietText bool
	// textHook, when set, sees every STATUSTEXT after printing.
	textHook func(string)
}

func Dial(addr string) (*Session, error) {
	c, err := net.DialTimeout("tcp", addr, 5*time.Second)
	if err != nil {
		return nil, err
	}
	return &Session{conn: c, target: 1}, nil
}

func (s *Session) Close() { s.conn.Close() }

func (s *Session) send(msgid uint32, payload []byte) error {
	f := EncodeFrame(s.seq, gcsSys, gcsComp, msgid, payload)
	s.seq++
	_, err := s.conn.Write(f)
	return err
}

// Pump reads frames for the wall-clock duration, updating St and printing
// STATUSTEXTs. It is the only reader — call it often.
func (s *Session) Pump(d time.Duration) {
	deadline := time.Now().Add(d)
	buf := make([]byte, 4096)
	for time.Now().Before(deadline) {
		s.conn.SetReadDeadline(time.Now().Add(150 * time.Millisecond))
		n, err := s.conn.Read(buf)
		if n > 0 {
			s.parser.Feed(buf[:n])
			for f := s.parser.Next(); f != nil; f = s.parser.Next() {
				s.handle(f)
			}
		}
		if err != nil {
			if ne, ok := err.(net.Error); ok && ne.Timeout() {
				continue
			}
			return
		}
	}
}

func (s *Session) bumpSim(ms uint32) {
	if ms > s.St.SimMs {
		s.St.SimMs = ms
	}
}

func (s *Session) handle(f *Frame) {
	switch f.MsgID {
	case MsgHeartbeat:
		if f.CompID != 1 { // only the autopilot, not peripherals
			return
		}
		hb := decHeartbeat(f)
		s.St.Mode = hb.CustomMode
		s.St.Armed = hb.BaseMode&128 != 0
		s.St.HaveHB = true
		s.target = f.SysID
	case MsgAttitude:
		a := decAttitude(f)
		s.bumpSim(a.TimeBootMs)
		s.St.Roll = float64(a.Roll) * 180 / math.Pi
		s.St.Pitch = float64(a.Pitch) * 180 / math.Pi
		yaw := float64(a.Yaw) * 180 / math.Pi
		if yaw < 0 {
			yaw += 360
		}
		s.St.Yaw = yaw
	case MsgVFRHUD:
		v := decVFRHUD(f)
		s.St.IAS = float64(v.Airspeed)
		s.St.Alt = float64(v.Alt)
		s.St.Climb = float64(v.Climb)
		s.St.Throttle = int(v.Throttle)
	case MsgGPSRawInt:
		g := decGPSRawInt(f)
		s.St.GPSFix = int(g.FixType)
		s.St.GPSVel = float64(g.Vel) / 100
		s.St.GPSCog = float64(g.Cog) / 100
	case MsgGlobalPositionInt:
		g := decGlobalPositionInt(f)
		s.bumpSim(g.TimeBootMs)
		s.St.Lat = float64(g.Lat) / 1e7
		s.St.Lon = float64(g.Lon) / 1e7
		s.St.Vx = float64(g.Vx) / 100
		s.St.Vy = float64(g.Vy) / 100
	case MsgNavControllerOutput:
		n := decNavControllerOutput(f)
		s.St.NavRoll = float64(n.NavRoll)
		s.St.NavPitch = float64(n.NavPitch)
		s.St.AltError = float64(n.AltError)
		s.St.AspdError = float64(n.AspdError)
	case MsgServoOutputRaw:
		s.St.Servo = decServoOutputRaw(f).Servo
	case MsgStatustext:
		st := decStatustext(f)
		if !s.QuietText {
			fmt.Printf("  [%7.1f] %s\n", float64(s.St.SimMs)/1000, st.Text)
		}
		if s.textHook != nil {
			s.textHook(st.Text)
		}
	}
}

// WaitHeartbeat pumps until the autopilot heartbeat is seen.
func (s *Session) WaitHeartbeat(timeout time.Duration) error {
	t0 := time.Now()
	for time.Since(t0) < timeout {
		s.Pump(200 * time.Millisecond)
		if s.St.HaveHB {
			return nil
		}
	}
	return fmt.Errorf("no heartbeat within %v", timeout)
}

// RCNeutral keeps the override channels warm: sticks centered, throttle low
// (auto-throttle modes own it anyway).
func (s *Session) RCNeutral() {
	s.RCOverride(1500, 1500, 1000, 1500)
}

func (s *Session) RCOverride(ail, ele, thr, rud uint16) {
	s.send(MsgRCChannelsOverride, payRCOverride(s.target, 1, [8]uint16{ail, ele, thr, rud}))
}

func (s *Session) SetMode(mode uint32) {
	s.send(MsgSetMode, paySetMode(s.target, mode))
}

// GetParam reads one parameter with retries. The horizon (6 x 3 s) rides
// out the multi-second wifi outages the bench link actually has — 4 x 2 s
// fit entirely inside one and aborted two campaigns on 2026-08-06.
func (s *Session) GetParam(name string) (float32, error) {
	for try := 0; try < 6; try++ {
		s.send(MsgParamRequestRead, payParamRequestRead(s.target, 1, name))
		if v, ok := s.waitParam(name, 3*time.Second); ok {
			return v, nil
		}
	}
	return 0, fmt.Errorf("param %s: no reply", name)
}

// SetParam sets and verifies (PARAM_VALUE replies cross-contaminate: always
// match the id). ArduPilot persists MAVLink param writes to storage.
func (s *Session) SetParam(name string, v float64) error {
	tol := math.Max(1e-3, math.Abs(v)*1e-3)
	for try := 0; try < 6; try++ {
		s.send(MsgParamSet, payParamSet(s.target, 1, name, float32(v)))
		if got, ok := s.waitParam(name, 3*time.Second); ok &&
			math.Abs(float64(got)-v) < tol {
			return nil
		}
	}
	return fmt.Errorf("param %s: set to %g failed", name, v)
}

func (s *Session) waitParam(name string, timeout time.Duration) (float32, bool) {
	deadline := time.Now().Add(timeout)
	buf := make([]byte, 4096)
	for time.Now().Before(deadline) {
		s.conn.SetReadDeadline(time.Now().Add(150 * time.Millisecond))
		n, err := s.conn.Read(buf)
		if n > 0 {
			s.parser.Feed(buf[:n])
			for f := s.parser.Next(); f != nil; f = s.parser.Next() {
				if f.MsgID == MsgParamValue {
					pv := decParamValue(f)
					if strings.TrimRight(pv.ID, "\x00") == name {
						return pv.Value, true
					}
					continue
				}
				s.handle(f)
			}
		}
		if err != nil {
			if ne, ok := err.(net.Error); ok && ne.Timeout() {
				continue
			}
			return 0, false
		}
	}
	return 0, false
}

// Command sends a COMMAND_LONG and waits for its ack.
func (s *Session) Command(cmd uint16, p1, p2, p3, p4, p5, p6, p7 float32) (byte, error) {
	s.send(MsgCommandLong, payCommandLong(s.target, 1, cmd, p1, p2, p3, p4, p5, p6, p7))
	deadline := time.Now().Add(3 * time.Second)
	buf := make([]byte, 4096)
	for time.Now().Before(deadline) {
		s.conn.SetReadDeadline(time.Now().Add(150 * time.Millisecond))
		n, err := s.conn.Read(buf)
		if n > 0 {
			s.parser.Feed(buf[:n])
			for f := s.parser.Next(); f != nil; f = s.parser.Next() {
				if f.MsgID == MsgCommandAck {
					ack := decCommandAck(f)
					if ack.Command == cmd {
						return ack.Result, nil
					}
					continue
				}
				s.handle(f)
			}
		}
		if err != nil {
			if ne, ok := err.(net.Error); ok && ne.Timeout() {
				continue
			}
			return 0, err
		}
	}
	return 0, fmt.Errorf("cmd %d: no ack", cmd)
}

// Arm retries against transient prearm states, keeping overrides warm.
func (s *Session) Arm(tries int) error {
	for i := 0; i < tries; i++ {
		s.RCNeutral()
		if res, err := s.Command(cmdArmDisarm, 1, 0, 0, 0, 0, 0, 0); err == nil && res == 0 {
			return nil
		}
		s.Pump(time.Second)
	}
	return fmt.Errorf("arm failed after %d tries", tries)
}

func (s *Session) ForceDisarm() {
	s.Command(cmdArmDisarm, 0, 21196, 0, 0, 0, 0, 0)
}

func (s *Session) Reboot() {
	s.Command(cmdRebootShutdown, 1, 0, 0, 0, 0, 0, 0)
}

// MsgInterval requests a stream rate. Bench lesson: raise rates only once
// airborne — pre-takeoff floods starve the climb loop.
// SendHdgAltCommand sends HDGALT_COMMAND (dev-fork msg 52100). flags:
// 1 heading, 2 turn rate, 4 altitude, 8 climb rate.
func (s *Session) SendHdgAltCommand(flags uint16, headingDeg, turnRateDps, altitudeM, climbRateMps float64) error {
	p := make([]byte, 22)
	le32 := func(off int, v uint32) {
		p[off] = byte(v)
		p[off+1] = byte(v >> 8)
		p[off+2] = byte(v >> 16)
		p[off+3] = byte(v >> 24)
	}
	lef := func(off int, v float64) { le32(off, math.Float32bits(float32(v))) }
	le32(0, 0) // start_time_boot_ms: execute on receipt
	lef(4, headingDeg)
	lef(8, turnRateDps)
	lef(12, altitudeM)
	lef(16, climbRateMps)
	p[20] = byte(flags)
	p[21] = byte(flags >> 8)
	return s.send(MsgHdgAltCommand, p)
}

func (s *Session) MsgInterval(msgid uint32, hz float32) {
	s.Command(cmdSetMessageInterval, float32(msgid), 1e6/hz, 0, 0, 0, 0, 0)
}
