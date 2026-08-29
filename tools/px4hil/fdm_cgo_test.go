package main

import "testing"

// Smoke: the cgo-wrapped FDM trims and steps — the integration risk retired
// before any MAVLink plumbing.
func TestFdmStepsLevel(t *testing.T) {
	f := NewFdm()
	c, err := f.Trim(100.0, 30.0, 0.0)
	if err != nil {
		t.Fatalf("trim: %v", err)
	}
	for i := 0; i < 2000; i++ { // 1 s at 2 kHz
		f.Step(c, 0.0005)
	}
	tr := f.Truth()
	if tr.H < 80 || tr.H > 120 {
		t.Fatalf("trimmed level flight drifted: h=%.1f", tr.H)
	}
}
