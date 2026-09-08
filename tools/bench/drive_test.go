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
