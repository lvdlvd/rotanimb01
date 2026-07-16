// Byte-pinned against pymavlink (MAVLink2, ardupilotmega dialect): every
// frame below was generated with srcSystem=255 srcComponent=190 seq=7.
// Generator kept in the session scratchpad (genvectors.py); the vectors are
// the spec — if an encoder or crcExtra entry drifts, these fail.
package main

import (
	"encoding/hex"
	"math"
	"testing"
)

var refFrames = map[string]string{
	"heartbeat":             "fd09000007ffbe000000000000000608c00403a7c1",
	"param_request_read":    "fd0f000007ffbe140000ffff0101524c4c5f524154455f46462177",
	"param_set":             "fd17000007ffbe17000050fcd03f010141525350445f524154494f0000000000098f10",
	"param_value":           "fd19000007ffbe16000000008841780541034e41564c315f504552494f4400000000091c68",
	"set_mode":              "fd06000007ffbe0b00000d0000000101bccc",
	"command_long":          "fd20000007ffbe4c00000000803f0098a546000000000000000000000000000000000000000090010101304e",
	"command_ack":           "fd02000007ffbe4d00009001778b",
	"rc_channels_override":  "fd12000007ffbe460000dc05dc05e803dc050000000000000000010121b4",
	"attitude":              "fd1c000007ffbe1e000040e20100cdcccc3dcdcc4cbd000020400ad7233c0ad7a3bc8fc2f53c3e40",
	"vfr_hud":               "fd13000007ffbe4a000000001a42cdcc204233b38d439a9999bf5f012d7ab5",
	"gps_raw_int":           "fd1e000007ffbe180000d2029649000000000092fe1ec0320a03785104007900c8000a0f3089030c2677",
	"global_position_int":   "fd1c000007ffbe21000040e201000092fe1ec0320a0378510400904d0400d80e88ff1e0030893f08",
	"nav_controller_output": "fd1a000007ffbe3e00009a99114133331340000048c1cdcccc3e00400ac4ab002d002902e324",
	"servo_output_raw":      "fd0c000007ffbe240000d2029649ec05a7065106e405824d",
	"statustext":            "fd10000007ffbefd00000650697463683a2046696e6973686564aef4",
}

func ref(t *testing.T, name string) []byte {
	b, err := hex.DecodeString(refFrames[name])
	if err != nil || len(b) == 0 {
		t.Fatalf("bad vector %q", name)
	}
	return b
}

func checkEncode(t *testing.T, name string, msgid uint32, payload []byte) {
	t.Helper()
	got := EncodeFrame(7, 255, 190, msgid, payload)
	want := ref(t, name)
	if string(got) != string(want) {
		t.Errorf("%s:\n got %x\nwant %x", name, got, want)
	}
}

func TestEncode(t *testing.T) {
	checkEncode(t, "heartbeat", MsgHeartbeat, payHeartbeat(6, 8, 192, 0, 4))
	checkEncode(t, "param_request_read", MsgParamRequestRead,
		payParamRequestRead(1, 1, "RLL_RATE_FF"))
	checkEncode(t, "param_set", MsgParamSet, payParamSet(1, 1, "ARSPD_RATIO", 1.6327))
	checkEncode(t, "set_mode", MsgSetMode, paySetMode(1, 13))
	checkEncode(t, "command_long", MsgCommandLong,
		payCommandLong(1, 1, 400, 1, 21196, 0, 0, 0, 0, 0))
	checkEncode(t, "rc_channels_override", MsgRCChannelsOverride,
		payRCOverride(1, 1, [8]uint16{1500, 1500, 1000, 1500, 0, 0, 0, 0}))
}

func parseOne(t *testing.T, name string) *Frame {
	t.Helper()
	var p Parser
	p.Feed(ref(t, name))
	f := p.Next()
	if f == nil {
		t.Fatalf("%s: no frame", name)
	}
	if f.Seq != 7 || f.SysID != 255 || f.CompID != 190 {
		t.Fatalf("%s: bad header %+v", name, f)
	}
	return f
}

func feq(a, b float32) bool { return math.Abs(float64(a-b)) < 1e-4 }

func TestDecode(t *testing.T) {
	hb := decHeartbeat(parseOne(t, "heartbeat"))
	if hb.Type != 6 || hb.Autopilot != 8 || hb.BaseMode != 192 || hb.CustomMode != 0 || hb.Status != 4 {
		t.Errorf("heartbeat: %+v", hb)
	}
	pv := decParamValue(parseOne(t, "param_value"))
	if pv.ID != "NAVL1_PERIOD" || !feq(pv.Value, 17.0) || pv.Count != 1400 || pv.Index != 833 {
		t.Errorf("param_value: %+v", pv)
	}
	at := decAttitude(parseOne(t, "attitude"))
	if at.TimeBootMs != 123456 || !feq(at.Roll, 0.1) || !feq(at.Pitch, -0.05) ||
		!feq(at.Yaw, 2.5) || !feq(at.PitchSpeed, -0.02) {
		t.Errorf("attitude: %+v", at)
	}
	vh := decVFRHUD(parseOne(t, "vfr_hud"))
	if !feq(vh.Airspeed, 38.5) || !feq(vh.Groundspeed, 40.2) || !feq(vh.Alt, 283.4) ||
		!feq(vh.Climb, -1.2) || vh.Heading != 351 || vh.Throttle != 45 {
		t.Errorf("vfr_hud: %+v", vh)
	}
	gp := decGPSRawInt(parseOne(t, "gps_raw_int"))
	if gp.TimeUsec != 1234567890 || gp.FixType != 3 || gp.Lat != 520000000 ||
		gp.Lon != 51000000 || gp.Alt != 283000 || gp.Vel != 3850 || gp.Cog != 35120 || gp.Sats != 12 {
		t.Errorf("gps_raw_int: %+v", gp)
	}
	gi := decGlobalPositionInt(parseOne(t, "global_position_int"))
	if gi.Lat != 520000000 || gi.RelativeAlt != 282000 || gi.Vx != 3800 ||
		gi.Vy != -120 || gi.Vz != 30 || gi.Hdg != 35120 {
		t.Errorf("global_position_int: %+v", gi)
	}
	nc := decNavControllerOutput(parseOne(t, "nav_controller_output"))
	if !feq(nc.NavRoll, 9.1) || !feq(nc.NavPitch, 2.3) || nc.NavBearing != 171 ||
		nc.TargetBearing != 45 || nc.WpDist != 553 || !feq(nc.XtrackError, -553.0) {
		t.Errorf("nav_controller_output: %+v", nc)
	}
	so := decServoOutputRaw(parseOne(t, "servo_output_raw"))
	if so.TimeUsec != 1234567890 || so.Servo[0] != 1516 || so.Servo[1] != 1703 ||
		so.Servo[2] != 1617 || so.Servo[3] != 1508 || so.Servo[7] != 0 {
		t.Errorf("servo_output_raw: %+v", so)
	}
	ca := decCommandAck(parseOne(t, "command_ack"))
	if ca.Command != 400 || ca.Result != 0 {
		t.Errorf("command_ack: %+v", ca)
	}
	st := decStatustext(parseOne(t, "statustext"))
	if st.Severity != 6 || st.Text != "Pitch: Finished" {
		t.Errorf("statustext: %+v", st)
	}
}

// The parser must resync across garbage and split feeds.
func TestParserResync(t *testing.T) {
	var p Parser
	junk := []byte{0x00, 0xfd, 0x03, 0x99} // fake sync that fails crc
	stream := append(append(junk, ref(t, "attitude")...), ref(t, "vfr_hud")...)
	// feed one byte at a time
	var got []uint32
	for _, b := range stream {
		p.Feed([]byte{b})
		for f := p.Next(); f != nil; f = p.Next() {
			got = append(got, f.MsgID)
		}
	}
	if len(got) != 2 || got[0] != MsgAttitude || got[1] != MsgVFRHUD {
		t.Errorf("resync: got %v", got)
	}
}
