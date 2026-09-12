package main

import "testing"

// Pinned wire vectors for GPS_CFG (canmsg.h 0x48): the id29 header was
// derived by hand from the layout comment (LCC 6 | msgid | PRV | src 0xb0
// | 0x9 | seq 4) and the payload from "u8 enable, u8 lag/10ms, zero-pad".
func TestGPSCfgLine(t *testing.T) {
	cases := []struct {
		enable bool
		lagMs  int
		want   string
	}{
		{true, 150, "690.1b094:010f000000000000\n"},
		{false, 150, "690.1b094:000f000000000000\n"},
		{true, 0, "690.1b094:0100000000000000\n"},
	}
	for _, c := range cases {
		p, err := gpsCfgPayload(c.enable, c.lagMs)
		if err != nil {
			t.Fatal(err)
		}
		if got := pline(canGPSCfg, 4, p); got != c.want {
			t.Errorf("gps %v %d: got %q want %q", c.enable, c.lagMs, got, c.want)
		}
	}
	if _, err := gpsCfgPayload(true, 3000); err == nil {
		t.Error("lag 3000 ms accepted; the u8 field holds at most 2550")
	}
}

// Pinned wire vectors for CMD_NOISE (canmsg.h 0x42): header derived the same
// way as above (LCC 6 | msgid 0x42 | PRV | src 0xb0 | 0x9 | seq 12), payload
// from "u8 gyro/accel/mag/baro/bias scale in 1/16 of nominal, u8 flags".
// 0x10 = 16 = the 1.0x the harness boots with.
func TestNoiseLine(t *testing.T) {
	cases := []struct {
		name                         string
		gyro, accel, mag, baro, bias float64
		redraw, zero                 bool
		want                         string
	}{
		{"default", 1, 1, 1, 1, 1, false, false, "684.1b09c:1010101010000000\n"},
		{"off", 0, 0, 0, 0, 0, false, false, "684.1b09c:0000001000000000\n"}, // baro floored
		{"4x", 4, 4, 4, 4, 4, false, false, "684.1b09c:4040404040000000\n"},
		{"redraw", 1, 1, 1, 1, 1, true, false, "684.1b09c:1010101010010000\n"},
		{"zero", 1, 1, 1, 1, 0, false, true, "684.1b09c:1010101000020000\n"},
		{"gyro only", 8, 1, 1, 1, 1, false, false, "684.1b09c:8010101010000000\n"},
	}
	for _, c := range cases {
		p, err := noisePayload(c.gyro, c.accel, c.mag, c.baro, c.bias, c.redraw, c.zero)
		if err != nil {
			t.Fatalf("%s: %v", c.name, err)
		}
		if got := pline(canNoise, 12, p); got != c.want {
			t.Errorf("noise %s: got %q want %q", c.name, got, c.want)
		}
	}
	if _, err := noisePayload(20, 1, 1, 1, 1, false, false); err == nil {
		t.Error("gyro scale 20 accepted; the 1/16 u8 field holds at most 15.9375")
	}
	if _, err := noisePayload(-1, 1, 1, 1, 1, false, false); err == nil {
		t.Error("negative gyro scale accepted")
	}
}
