package main

// pfd — an ASCII-art primary flight display, toggled with 'f'. Airspeed tape
// on the left, altitude tape on the right (current values boxed and bold), a
// compass tape along the bottom, and the horizon line through the center
// showing pitch and roll, fed by the harness's TRUTH_* telemetry.

import (
	"fmt"
	"math"
	"strings"
)

const (
	pfdW = 72 // canvas columns
	pfdH = 19 // canvas rows (plus the compass tape below)

	tapeW   = 8              // columns reserved per side tape
	horizW  = pfdW - 2*tapeW // horizon field width
	pitchPx = 2.5            // rows per 10 degrees of pitch
)

type flight struct {
	valid            bool
	roll, pitch, yaw float64 // rad
	ias, tas         float64 // m/s
	alt, hdot, gs    float64 // m, m/s, m/s
	da, de, dr, dt   float64 // deg, deg, deg, 0..1
}

func (f *flight) fromQuat(w, x, y, z float64) {
	f.roll = math.Atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
	f.pitch = math.Asin(clamp(2*(w*y-z*x), -1, 1))
	f.yaw = math.Atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
}

func clamp(v, lo, hi float64) float64 { return math.Max(lo, math.Min(hi, v)) }

// renderPFD returns the display as terminal lines (no trailing newline).
func renderPFD(f *flight) []string {
	grid := make([][]byte, pfdH)
	for i := range grid {
		grid[i] = []byte(strings.Repeat(" ", pfdW))
	}
	cy, cx := pfdH/2, tapeW+horizW/2

	// horizon: one mark per column across the field, offset by pitch,
	// tilted by roll. Character cells are ~2:1, so half the slope per col.
	// Pitch up moves the horizon down the screen; right roll tilts it
	// counter-clockwise as seen from the cockpit.
	tan := math.Tan(f.roll)
	off := f.pitch * (180 / math.Pi) / 10 * pitchPx
	for x := tapeW; x < tapeW+horizW; x++ {
		dy := off - float64(x-cx)*tan*0.5
		y := cy + int(math.Round(dy))
		if y >= 0 && y < pfdH {
			ch := byte('-')
			if tan*0.5 > 0.25 {
				ch = '\\'
			} else if tan*0.5 < -0.25 {
				ch = '/'
			}
			grid[y][x] = ch
		}
	}
	// fixed aircraft symbol
	for _, d := range []struct {
		dx int
		ch byte
	}{{-3, '_'}, {-2, '_'}, {2, '_'}, {3, '_'}, {0, 'W'}} {
		grid[cy][cx+d.dx] = d.ch
	}

	// side tapes: airspeed left (2 m/s per row), altitude right (5 m per row)
	for row := 0; row < pfdH; row++ {
		d := cy - row
		spd := f.ias + float64(d)*2
		if spd >= 0 && (int(math.Round(spd))%10 <= 1 || d == 0) {
			s := fmt.Sprintf("%4.0f-", spd)
			copy(grid[row][0:], []byte(s))
		}
		alt := f.alt + float64(d)*5
		if int(math.Round(alt))%25 <= 2 || d == 0 {
			s := fmt.Sprintf("-%5.0f", alt)
			copy(grid[row][pfdW-len(s):], []byte(s))
		}
	}

	lines := make([]string, 0, pfdH+3)
	for row := 0; row < pfdH; row++ {
		l := string(grid[row])
		if row == cy { // current values, boxed and bold
			ias := fmt.Sprintf("%s|%5.1f>%s", bold, f.ias, normal)
			alt := fmt.Sprintf("%s<%6.1f|%s", bold, f.alt, normal)
			l = ias + l[7:pfdW-8] + alt
		}
		lines = append(lines, l)
	}

	// compass tape: one column per 2 degrees, ticks every 10, labels every 30
	hdg := math.Mod(f.yaw*180/math.Pi+360, 360)
	tape := []byte(strings.Repeat(" ", pfdW))
	for x := 0; x < pfdW; x++ {
		d := math.Mod(hdg+float64(x-pfdW/2)*2+3600, 360)
		r := int(math.Round(d)) % 360
		switch {
		case r%30 < 2: // label once; the neighbour column gets the tick
			lbl := fmt.Sprintf("%02d", ((r+1)/30*30/10)%36)
			if x+1 < pfdW {
				copy(tape[x:], lbl)
			}
		case r%10 < 2 || r%10 >= 29:
			tape[x] = '\''
		}
	}
	lines = append(lines, string(tape),
		strings.Repeat(" ", pfdW/2-3)+bold+fmt.Sprintf("[%05.1f]", hdg)+normal)
	lines = append(lines, fmt.Sprintf("%sIAS %4.1f  TAS %4.1f  ALT %6.1f  VSI %+5.1f  GS %4.1f   ctl a%+5.1f e%+5.1f r%+5.1f t%3.0f%%%s",
		dim, f.ias, f.tas, f.alt, f.hdot, f.gs, f.da, f.de, f.dr, f.dt*100, normal))
	return lines
}
