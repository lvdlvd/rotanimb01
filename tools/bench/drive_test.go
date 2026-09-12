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
// from "u8 gyro/accel/mag/baro scale, u8 bias, u8 flags, u8 pitot, u8 gps,
// all in 1/16 of nominal". 0x10 = 16 = the 1.0x the harness boots with.
func TestNoiseLine(t *testing.T) {
	all := func(x float64) noiseCfg {
		return noiseCfg{gyro: x, accel: x, mag: x, baro: x, bias: x, pitot: x, gps: x}
	}
	cases := []struct {
		name string
		cfg  noiseCfg
		want string
	}{
		{"default", defaultNoise(), "684.1b09c:1010101010001010\n"},
		// every scale off; the baro floors at 1.0x on the way out
		{"off", noiseCfg{}, "684.1b09c:0000001000000000\n"},
		{"4x", all(4), "684.1b09c:4040404040004040\n"},
		{"redraw", func() noiseCfg { c := defaultNoise(); c.redraw = true; return c }(),
			"684.1b09c:1010101010011010\n"},
		{"zero the biases", func() noiseCfg { c := defaultNoise(); c.bias, c.zero = 0, true; return c }(),
			"684.1b09c:1010101000021010\n"},
		{"gyro only", func() noiseCfg { c := defaultNoise(); c.gyro = 8; return c }(),
			"684.1b09c:8010101010001010\n"},
		// the aiding sensors are independent of the four on the SPI bus
		{"aiding only", func() noiseCfg { c := defaultNoise(); c.pitot, c.gps = 2, 4; return c }(),
			"684.1b09c:1010101010002040\n"},
	}
	for _, c := range cases {
		p, err := noisePayload(c.cfg)
		if err != nil {
			t.Fatalf("%s: %v", c.name, err)
		}
		if got := pline(canNoise, 12, p); got != c.want {
			t.Errorf("noise %s: got %q want %q", c.name, got, c.want)
		}
	}
	if _, err := noisePayload(all(20)); err == nil {
		t.Error("scale 20 accepted; the 1/16 u8 field holds at most 15.9375")
	}
	if _, err := noisePayload(all(-1)); err == nil {
		t.Error("negative scale accepted")
	}
	if _, err := noisePayload(noiseCfg{baro: 1, gps: 20}); err == nil {
		t.Error("gps scale 20 accepted; the 1/16 u8 field holds at most 15.9375")
	}
}
