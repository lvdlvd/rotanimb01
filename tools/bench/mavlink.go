// mavlink — minimal MAVLink2 codec, just the dictionary this bench speaks.
// Handwritten wire layouts (MAVLink sorts fields by type size; extensions
// append unsorted; trailing zero bytes truncate). Byte-pinned against
// pymavlink reference frames in mavlink_test.go.
package main

import (
	"encoding/binary"
	"fmt"
	"math"
)

const (
	MsgHeartbeat           = 0
	MsgSetMode             = 11
	MsgParamRequestRead    = 20
	MsgParamValue          = 22
	MsgParamSet            = 23
	MsgGPSRawInt           = 24
	MsgAttitude            = 30
	MsgGlobalPositionInt   = 33
	MsgServoOutputRaw      = 36
	MsgNavControllerOutput = 62
	MsgRCChannelsOverride  = 70
	MsgVFRHUD              = 74
	MsgCommandLong         = 76
	MsgCommandAck          = 77
	MsgStatustext          = 253
	MsgHdgAltCommand       = 52100
)

// crcExtra seeds the X.25 checksum per message; a wrong value here makes the
// other end drop us silently — that is what the pinned tests are for.
var crcExtra = map[uint32]byte{
	MsgHeartbeat: 50, MsgSetMode: 89, MsgParamRequestRead: 214,
	MsgParamValue: 220, MsgParamSet: 168, MsgGPSRawInt: 24,
	MsgAttitude: 39, MsgGlobalPositionInt: 104, MsgServoOutputRaw: 222,
	MsgNavControllerOutput: 183, MsgRCChannelsOverride: 124,
	MsgVFRHUD: 20, MsgCommandLong: 152, MsgCommandAck: 143,
	MsgStatustext: 83, MsgHdgAltCommand: 161,
}

func crcX25(crc uint16, b byte) uint16 {
	b ^= byte(crc)
	b ^= b << 4
	return crc>>8 ^ uint16(b)<<8 ^ uint16(b)<<3 ^ uint16(b)>>4
}

func crcX25Buf(crc uint16, p []byte) uint16 {
	for _, b := range p {
		crc = crcX25(crc, b)
	}
	return crc
}

// Frame is a decoded MAVLink2 frame; Payload is zero-padded back to full
// length by the per-message decoders, not here.
type Frame struct {
	Seq, SysID, CompID byte
	MsgID              uint32
	Payload            []byte
}

// EncodeFrame wraps a payload; trailing zeros truncate to >= 1 byte.
func EncodeFrame(seq, sysid, compid byte, msgid uint32, payload []byte) []byte {
	n := len(payload)
	for n > 1 && payload[n-1] == 0 {
		n--
	}
	f := make([]byte, 0, 12+n)
	f = append(f, 0xfd, byte(n), 0, 0, seq, sysid, compid,
		byte(msgid), byte(msgid>>8), byte(msgid>>16))
	f = append(f, payload[:n]...)
	crc := crcX25Buf(0xffff, f[1:])
	crc = crcX25(crc, crcExtra[msgid])
	return append(f, byte(crc), byte(crc>>8))
}

// Parser is a resyncing streaming decoder. Unknown message ids (no crcExtra)
// are dropped: their checksum cannot be verified.
type Parser struct {
	buf []byte
}

func (p *Parser) Feed(data []byte) { p.buf = append(p.buf, data...) }

// Next returns the next valid frame, or nil when more bytes are needed.
func (p *Parser) Next() *Frame {
	for {
		for len(p.buf) > 0 && p.buf[0] != 0xfd {
			p.buf = p.buf[1:]
		}
		if len(p.buf) < 12 {
			return nil
		}
		plen := int(p.buf[1])
		if p.buf[2] != 0 { // incompat flags (signing): not ours, resync
			p.buf = p.buf[1:]
			continue
		}
		total := 12 + plen
		if len(p.buf) < total {
			return nil
		}
		f := p.buf[:total]
		msgid := uint32(f[7]) | uint32(f[8])<<8 | uint32(f[9])<<16
		extra, known := crcExtra[msgid]
		if known {
			crc := crcX25Buf(0xffff, f[1:total-2])
			crc = crcX25(crc, extra)
			if byte(crc) == f[total-2] && byte(crc>>8) == f[total-1] {
				fr := &Frame{Seq: f[4], SysID: f[5], CompID: f[6], MsgID: msgid,
					Payload: append([]byte(nil), f[10:total-2]...)}
				p.buf = p.buf[total:]
				return fr
			}
			// bad crc: false sync, shift one byte
			p.buf = p.buf[1:]
			continue
		}
		// unknown id: assume framing is honest and skip the whole frame
		p.buf = p.buf[total:]
	}
}

// ---- payload cursor helpers ------------------------------------------------

type wbuf struct{ b []byte }

func (w *wbuf) u8(v byte)     { w.b = append(w.b, v) }
func (w *wbuf) u16(v uint16)  { w.b = binary.LittleEndian.AppendUint16(w.b, v) }
func (w *wbuf) u32(v uint32)  { w.b = binary.LittleEndian.AppendUint32(w.b, v) }
func (w *wbuf) i16(v int16)   { w.u16(uint16(v)) }
func (w *wbuf) i32(v int32)   { w.u32(uint32(v)) }
func (w *wbuf) f32(v float32) { w.u32(math.Float32bits(v)) }
func (w *wbuf) str(s string, n int) {
	b := make([]byte, n)
	copy(b, s)
	w.b = append(w.b, b...)
}

type rbuf struct {
	b []byte
	i int
}

// rd pads the truncated payload back to full length.
func rd(f *Frame, full int) *rbuf {
	b := f.Payload
	if len(b) < full {
		b = append(append([]byte(nil), b...), make([]byte, full-len(b))...)
	}
	return &rbuf{b: b}
}
func (r *rbuf) u8() byte     { v := r.b[r.i]; r.i++; return v }
func (r *rbuf) u16() uint16  { v := binary.LittleEndian.Uint16(r.b[r.i:]); r.i += 2; return v }
func (r *rbuf) u32() uint32  { v := binary.LittleEndian.Uint32(r.b[r.i:]); r.i += 4; return v }
func (r *rbuf) u64() uint64  { v := binary.LittleEndian.Uint64(r.b[r.i:]); r.i += 8; return v }
func (r *rbuf) i16() int16   { return int16(r.u16()) }
func (r *rbuf) i32() int32   { return int32(r.u32()) }
func (r *rbuf) f32() float32 { return math.Float32frombits(r.u32()) }
func (r *rbuf) str(n int) string {
	b := r.b[r.i : r.i+n]
	r.i += n
	for i, c := range b {
		if c == 0 {
			return string(b[:i])
		}
	}
	return string(b)
}

// ---- messages we send ------------------------------------------------------

func payHeartbeat(typ, autopilot, baseMode byte, customMode uint32, status byte) []byte {
	var w wbuf
	w.u32(customMode)
	w.u8(typ)
	w.u8(autopilot)
	w.u8(baseMode)
	w.u8(status)
	w.u8(3) // mavlink_version
	return w.b
}

func paySetMode(sys byte, customMode uint32) []byte {
	var w wbuf
	w.u32(customMode)
	w.u8(sys)
	w.u8(1) // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
	return w.b
}

func payParamRequestRead(sys, comp byte, name string) []byte {
	var w wbuf
	w.i16(-1)
	w.u8(sys)
	w.u8(comp)
	w.str(name, 16)
	return w.b
}

func payParamSet(sys, comp byte, name string, v float32) []byte {
	var w wbuf
	w.f32(v)
	w.u8(sys)
	w.u8(comp)
	w.str(name, 16)
	w.u8(9) // MAV_PARAM_TYPE_REAL32
	return w.b
}

func payCommandLong(sys, comp byte, cmd uint16, p1, p2, p3, p4, p5, p6, p7 float32) []byte {
	var w wbuf
	for _, v := range []float32{p1, p2, p3, p4, p5, p6, p7} {
		w.f32(v)
	}
	w.u16(cmd)
	w.u8(sys)
	w.u8(comp)
	w.u8(0) // confirmation
	return w.b
}

func payRCOverride(sys, comp byte, ch [8]uint16) []byte {
	var w wbuf
	for _, v := range ch {
		w.u16(v)
	}
	w.u8(sys)
	w.u8(comp)
	return w.b
}

// ---- messages we receive ---------------------------------------------------

type Heartbeat struct {
	CustomMode                        uint32
	Type, Autopilot, BaseMode, Status byte
}

func decHeartbeat(f *Frame) Heartbeat {
	r := rd(f, 9)
	return Heartbeat{CustomMode: r.u32(), Type: r.u8(), Autopilot: r.u8(),
		BaseMode: r.u8(), Status: r.u8()}
}

type ParamValue struct {
	Value        float32
	Count, Index uint16
	ID           string
}

func decParamValue(f *Frame) ParamValue {
	r := rd(f, 25)
	return ParamValue{Value: r.f32(), Count: r.u16(), Index: r.u16(), ID: r.str(16)}
}

type GPSRawInt struct {
	TimeUsec           uint64
	Lat, Lon, Alt      int32
	Eph, Epv, Vel, Cog uint16
	FixType, Sats      byte
}

func decGPSRawInt(f *Frame) GPSRawInt {
	r := rd(f, 30)
	return GPSRawInt{TimeUsec: r.u64(), Lat: r.i32(), Lon: r.i32(), Alt: r.i32(),
		Eph: r.u16(), Epv: r.u16(), Vel: r.u16(), Cog: r.u16(),
		FixType: r.u8(), Sats: r.u8()}
}

type Attitude struct {
	TimeBootMs                                        uint32
	Roll, Pitch, Yaw, RollSpeed, PitchSpeed, YawSpeed float32
}

func decAttitude(f *Frame) Attitude {
	r := rd(f, 28)
	return Attitude{TimeBootMs: r.u32(), Roll: r.f32(), Pitch: r.f32(), Yaw: r.f32(),
		RollSpeed: r.f32(), PitchSpeed: r.f32(), YawSpeed: r.f32()}
}

type GlobalPositionInt struct {
	TimeBootMs                 uint32
	Lat, Lon, Alt, RelativeAlt int32
	Vx, Vy, Vz                 int16
	Hdg                        uint16
}

func decGlobalPositionInt(f *Frame) GlobalPositionInt {
	r := rd(f, 28)
	return GlobalPositionInt{TimeBootMs: r.u32(), Lat: r.i32(), Lon: r.i32(),
		Alt: r.i32(), RelativeAlt: r.i32(), Vx: r.i16(), Vy: r.i16(), Vz: r.i16(),
		Hdg: r.u16()}
}

type ServoOutputRaw struct {
	TimeUsec uint32
	Servo    [8]uint16
}

func decServoOutputRaw(f *Frame) ServoOutputRaw {
	r := rd(f, 21)
	var m ServoOutputRaw
	m.TimeUsec = r.u32()
	for i := range m.Servo {
		m.Servo[i] = r.u16()
	}
	return m
}

type NavControllerOutput struct {
	NavRoll, NavPitch, AltError, AspdError, XtrackError float32
	NavBearing, TargetBearing                           int16
	WpDist                                              uint16
}

func decNavControllerOutput(f *Frame) NavControllerOutput {
	r := rd(f, 26)
	return NavControllerOutput{NavRoll: r.f32(), NavPitch: r.f32(),
		AltError: r.f32(), AspdError: r.f32(), XtrackError: r.f32(),
		NavBearing: r.i16(), TargetBearing: r.i16(), WpDist: r.u16()}
}

type VFRHUD struct {
	Airspeed, Groundspeed, Alt, Climb float32
	Heading                           int16
	Throttle                          uint16
}

func decVFRHUD(f *Frame) VFRHUD {
	r := rd(f, 20)
	return VFRHUD{Airspeed: r.f32(), Groundspeed: r.f32(), Alt: r.f32(),
		Climb: r.f32(), Heading: r.i16(), Throttle: r.u16()}
}

type CommandAck struct {
	Command uint16
	Result  byte
}

func decCommandAck(f *Frame) CommandAck {
	r := rd(f, 3)
	return CommandAck{Command: r.u16(), Result: r.u8()}
}

type Statustext struct {
	Severity byte
	Text     string
}

func decStatustext(f *Frame) Statustext {
	r := rd(f, 51)
	return Statustext{Severity: r.u8(), Text: r.str(50)}
}

// hexdump aids -debug output.
func hexdump(p []byte) string {
	return fmt.Sprintf("%x", p)
}
