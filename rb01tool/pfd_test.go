package main

import (
	"math"
	"strings"
	"testing"
)

// render at a known attitude and eyeball via `go test -v -run PFD`
func TestPFDRender(t *testing.T) {
	f := &flight{valid: true, ias: 25.2, tas: 26.0, alt: 98.4, hdot: -1.2, gs: 24.8,
		da: -2.1, de: -4.5, dr: 0.3, dt: 0.56}
	// 15 deg right roll, 5 deg nose up, heading 065
	f.fromQuat(quatFromEuler(15*math.Pi/180, 5*math.Pi/180, 65*math.Pi/180))

	lines := renderPFD(f)
	if len(lines) != pfdH+3 {
		t.Fatalf("%d lines", len(lines))
	}
	for _, l := range lines {
		t.Log(strings.ReplaceAll(strings.ReplaceAll(l, bold, "«"), normal, "»"))
	}
	// horizon must be tilted: marks present both above and below center row
	above, below := false, false
	for i, l := range lines[:pfdH] {
		if strings.ContainsAny(l[tapeW:tapeW+horizW], "-/\\") {
			if i < pfdH/2-1 {
				above = true
			}
			if i > pfdH/2+1 {
				below = true
			}
		}
	}
	if !above || !below {
		t.Fatal("horizon not tilted across the center")
	}
}

func TestPFDLevel(t *testing.T) {
	f := &flight{valid: true, ias: 25, alt: 100}
	f.fromQuat(quatFromEuler(0, 0, 0))
	lines := renderPFD(f)
	// level: the horizon shares the center row with the aircraft symbol
	if !strings.Contains(lines[pfdH/2], "-") || !strings.Contains(lines[pfdH/2], "W") {
		t.Fatalf("level horizon missing from center: %q", lines[pfdH/2])
	}
	for _, l := range lines {
		t.Log(strings.ReplaceAll(strings.ReplaceAll(l, bold, "«"), normal, "»"))
	}
}

func quatFromEuler(roll, pitch, yaw float64) (w, x, y, z float64) {
	cr, sr := math.Cos(roll/2), math.Sin(roll/2)
	cp, sp := math.Cos(pitch/2), math.Sin(pitch/2)
	cy, sy := math.Cos(yaw/2), math.Sin(yaw/2)
	w = cy*cp*cr + sy*sp*sr
	x = cy*cp*sr - sy*sp*cr
	y = cy*sp*cr + sy*cp*sr
	z = sy*cp*cr - cy*sp*sr
	return
}
