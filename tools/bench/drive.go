// drive — harness pseudocan commands over the local serial ports (runs ON
// the bench host, next to the USB devices). Wire format nlib/fmtcan, dictionary
// src/canmsg.h; the harness accepts crc-less lines.
package main

import (
	"encoding/binary"
	"encoding/hex"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"
)

// Device defaults. The environment variable wins; otherwise the by-id glob
// must match exactly one device (Linux keeps /dev/serial/by-id/ stable
// across reboots and port moves, ttyACM numbers are not). With two ST-Links
// on one host the console glob is ambiguous — set the variable.
//
//	ROTANIMB01_HARNESS   the harness pseudocan CDC (commands + TRUTH_* telemetry)
//	ROTANIMB01_CONSOLE   the harness ST-Link VCP (heartbeat/console tail)
//	ROTANIMB01_DUT       the DUT's MAVLink CDC (serbridge)
func devDefault(env, glob string) string {
	if v := os.Getenv(env); v != "" {
		return v
	}
	if m, _ := filepath.Glob(glob); len(m) == 1 {
		return m[0]
	}
	return glob // open fails naming what was looked for
}

func defHarnCmd() string {
	return devDefault("ROTANIMB01_HARNESS", "/dev/serial/by-id/usb-rotanimb01_hitl-harness_*-if00")
}

func defHarnCon() string {
	return devDefault("ROTANIMB01_CONSOLE", "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_*-if02")
}

func defDutDev() string {
	if v := os.Getenv("ROTANIMB01_DUT"); v != "" {
		return v
	}
	// either autopilot's CDC name, whichever one is attached
	var m []string
	for _, g := range []string{"/dev/serial/by-id/usb-ArduPilot_*-if00", "/dev/serial/by-id/usb-PX4_*-if00"} {
		hits, _ := filepath.Glob(g)
		m = append(m, hits...)
	}
	if len(m) == 1 {
		return m[0]
	}
	return "/dev/serial/by-id/usb-{ArduPilot,PX4}_*-if00"
}

const (
	canMode     = 0x43
	canAirstart = 0x44
	canFDMPos   = 0x49
	canParam    = 0x47
	canPWMCal   = 0x45
	canWind     = 0x46
	canGPSCfg   = 0x48
)

// gpsCfgPayload encodes GPS_CFG (canmsg.h 0x48): u8 enable, u8 feeder lag
// in 10 ms units, zero-padded to the 8 bytes every command frame carries.
// The harness default is enable=1, lag 150 ms.
func gpsCfgPayload(enable bool, lagMs int) ([]byte, error) {
	if lagMs < 0 || lagMs > 2550 {
		return nil, fmt.Errorf("gps lag %d ms out of range 0..2550", lagMs)
	}
	p := make([]byte, 8)
	if enable {
		p[0] = 1
	}
	p[1] = byte(lagMs / 10)
	return p, nil
}

// id29: LCC 6 (TMC) | msgid | PRV | srcid 0xb0 | 0x9 | seq
func id29(msgid, seq uint32) uint32 {
	return 6<<26 | (msgid&0x7f)<<19 | 1<<16 | 0xb0<<8 | 0x9<<4 | seq&0xf
}

func pline(msgid, seq uint32, payload []byte) string {
	i := id29(msgid, seq)
	return fmt.Sprintf("%03x.%05x:%s\n", i>>18, i&0x3ffff, hex.EncodeToString(payload))
}

func harnessCmd(dev string, msgid, seq uint32, payload []byte) error {
	f, err := openSerial(dev, 115200)
	if err != nil {
		return err
	}
	defer f.Close()
	_, err = f.WriteString(pline(msgid, seq, payload))
	return err
}

// consoleTail reads the harness ST-Link console for secs, returning the
// heartbeat and control lines.
func consoleTail(dev string, secs float64) ([]string, error) {
	f, err := openSerial(dev, 115200)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	deadline := time.Now().Add(time.Duration(secs * float64(time.Second)))
	var data []byte
	buf := make([]byte, 2048)
	for time.Now().Before(deadline) {
		f.SetReadDeadline(time.Now().Add(300 * time.Millisecond))
		n, _ := f.Read(buf)
		if n > 0 {
			data = append(data, buf[:n]...)
		}
	}
	var out []string
	for _, l := range strings.Split(string(data), "\n") {
		if strings.HasPrefix(l, "t ") || strings.Contains(l, "ctl") {
			out = append(out, strings.TrimRight(l, "\r"))
		}
	}
	return out, nil
}

func tailPrint(con string, secs float64, last int) error {
	lines, err := consoleTail(con, secs)
	if err != nil {
		return err
	}
	if last > 0 && len(lines) > last {
		lines = lines[len(lines)-last:]
	}
	for _, l := range lines {
		fmt.Println(l)
	}
	return nil
}

func cmdDrive(cmd, dev, con string, args []string) error {
	switch cmd {
	case "mode":
		if len(args) < 1 {
			return fmt.Errorf("drive mode <0|1>")
		}
		m, _ := strconv.Atoi(args[0])
		p := make([]byte, 8)
		p[0] = byte(m)
		if err := harnessCmd(dev, canMode, 1, p); err != nil {
			return err
		}
		return tailPrint(con, 2.5, 3)

	case "airstart":
		if len(args) < 2 {
			return fmt.Errorf("drive airstart <alt_m> <speed_mps> [hdg_deg] [north_m east_m]")
		}
		alt, _ := strconv.Atoi(args[0])
		spd, _ := strconv.ParseFloat(args[1], 64)
		if len(args) >= 5 { // rehome first, then trim at the new spot
			n, _ := strconv.ParseFloat(args[3], 64)
			e, _ := strconv.ParseFloat(args[4], 64)
			q := make([]byte, 8)
			binary.BigEndian.PutUint32(q[0:], uint32(int32(n*100)))
			binary.BigEndian.PutUint32(q[4:], uint32(int32(e*100)))
			if err := harnessCmd(dev, canFDMPos, 7, q); err != nil {
				return err
			}
		}
		p := make([]byte, 8)
		binary.BigEndian.PutUint16(p[0:], uint16(alt))
		binary.BigEndian.PutUint16(p[2:], uint16(spd*10))
		if len(args) >= 3 { // heading was silently dropped before 2026-08-28
			hd, _ := strconv.ParseFloat(args[2], 64)
			binary.BigEndian.PutUint16(p[4:], uint16(hd*100))
		}
		if err := harnessCmd(dev, canAirstart, 2, p); err != nil {
			return err
		}
		return tailPrint(con, 2.5, 3)

	case "setpos":
		if len(args) < 2 {
			return fmt.Errorf("drive setpos <north_m> <east_m>")
		}
		n, _ := strconv.ParseFloat(args[0], 64)
		e, _ := strconv.ParseFloat(args[1], 64)
		q := make([]byte, 8)
		binary.BigEndian.PutUint32(q[0:], uint32(int32(n*100)))
		binary.BigEndian.PutUint32(q[4:], uint32(int32(e*100)))
		if err := harnessCmd(dev, canFDMPos, 7, q); err != nil {
			return err
		}
		return tailPrint(con, 2.5, 3)

	case "gps":
		// enable/disable the harness's on-board DroneCAN GPS feeder
		// (GPS-denied segments); optional lag in ms (default 150).
		if len(args) < 1 || (args[0] != "0" && args[0] != "1") {
			return fmt.Errorf("drive gps <0|1> [lag_ms]")
		}
		lag := 150
		if len(args) > 1 {
			var err error
			if lag, err = strconv.Atoi(args[1]); err != nil {
				return fmt.Errorf("drive gps: lag %q: %v", args[1], err)
			}
		}
		p, err := gpsCfgPayload(args[0] == "1", lag)
		if err != nil {
			return err
		}
		if err := harnessCmd(dev, canGPSCfg, 4, p); err != nil {
			return err
		}
		fmt.Printf("gps feeder enable=%s lag=%d ms sent\n", args[0], lag)
		return nil

	case "wind":
		if len(args) < 2 {
			return fmt.Errorf("drive wind <north_mps> <east_mps> [gust_sigma_cms] [gust_tau_s]")
		}
		n, _ := strconv.ParseFloat(args[0], 64)
		e, _ := strconv.ParseFloat(args[1], 64)
		p := make([]byte, 8)
		binary.BigEndian.PutUint16(p[0:], uint16(int16(n*100)))
		binary.BigEndian.PutUint16(p[2:], uint16(int16(e*100)))
		if len(args) > 2 {
			g, _ := strconv.Atoi(args[2])
			p[6] = byte(g)
		}
		if len(args) > 3 {
			tau, _ := strconv.Atoi(args[3])
			p[7] = byte(tau)
		}
		if err := harnessCmd(dev, canWind, 3, p); err != nil {
			return err
		}
		fmt.Println("wind sent")
		return nil

	case "engine":
		// preset the FDM engine params (power_w 41, t_static_n 42,
		// crit_alt_m 43). 912iS = the model's default, naturally aspirated;
		// 915iS = turbo, rated to FL150 — the high-altitude bench choice.
		if len(args) < 1 {
			return fmt.Errorf("drive engine <912|915>")
		}
		var vals [3]float64
		switch args[0] {
		case "912":
			vals = [3]float64{74600, 1601, 0}
		case "915":
			vals = [3]float64{105000, 2000, 4572}
		default:
			return fmt.Errorf("unknown engine %q (912|915)", args[0])
		}
		for i, v := range vals {
			p := make([]byte, 8)
			binary.BigEndian.PutUint16(p[0:], uint16(41+i))
			binary.BigEndian.PutUint32(p[2:], math.Float32bits(float32(v)))
			if err := harnessCmd(dev, canParam, uint32(8+i), p); err != nil {
				return err
			}
			time.Sleep(50 * time.Millisecond)
		}
		fmt.Printf("engine %s set\n", args[0])
		return nil

	case "param":
		// any FDM parameter by table index (fdm/fdm.h, doc/FDM-TUNING.md);
		// the harness echoes PARAM_VAL on the pseudocan link, which only a
		// reader of that port (rb01tool) sees — this side tails the console
		if len(args) < 2 {
			return fmt.Errorf("drive param <index> <value>")
		}
		idx, err := strconv.Atoi(args[0])
		if err != nil || idx < 0 || idx >= 0x8000 {
			return fmt.Errorf("drive param: index %q", args[0])
		}
		v, err := strconv.ParseFloat(args[1], 32)
		if err != nil {
			return fmt.Errorf("drive param: value %q: %v", args[1], err)
		}
		p := make([]byte, 8)
		binary.BigEndian.PutUint16(p[0:], uint16(idx))
		binary.BigEndian.PutUint32(p[2:], math.Float32bits(float32(v)))
		if err := harnessCmd(dev, canParam, 11, p); err != nil {
			return err
		}
		fmt.Printf("param %d = %g sent\n", idx, v)
		return tailPrint(con, 2.5, 2)

	case "cal":
		// PWM_CAL deflections, per autopilot. The harness default cal
		// (controls.c) is all-positive: pwm HIGH = positive deflection, and
		// per fdm.c's classic aero signs a positive deflection is nose-DOWN
		// (Cmde -1.2) and nose-LEFT (Cndr -0.08); aileron matches (Clda +0.17
		// = roll right). Whichever axes the autopilot drives the other way
		// need a NEGATIVE full deflection here.
		//
		// This cal lives in harness RAM ONLY — resend after EVERY harness
		// reboot (incl. uhubctl power cycles hitting shared hub ports) and
		// after every harness reflash, or the DUT flies with a reversed
		// surface: no rotation on takeoff, elevator railed, ground-roll
		// overspeed.
		//
		//   ardupilot (default): ail +, ELE FLIPPED, rud +
		//   px4:                 ail +, ELE FLIPPED, RUD FLIPPED
		//
		// PX4 drives its outputs as "+ = positive body torque" in FRD, so
		// +pitch is nose-UP and +yaw is nose-RIGHT — both opposite to the
		// fdm. Same mapping the flight-verified px4hil SITL bridge applies
		// (tools/px4hil/main.go: negate ele + rud, aileron matches).
		profile := "ardupilot"
		if len(args) > 0 {
			profile = args[0]
		}
		var cal []struct {
			chan_ byte
			cdeg  int16
		}
		switch profile {
		case "ardupilot":
			cal = []struct {
				chan_ byte
				cdeg  int16
			}{{0, 2000}, {1, -2500}, {3, 2500}}
		case "px4":
			cal = []struct {
				chan_ byte
				cdeg  int16
			}{{0, 2000}, {1, -2500}, {3, -2500}}
		default:
			return fmt.Errorf("drive cal [ardupilot|px4]: unknown profile %q", profile)
		}
		names := [4]string{"ail", "ele", "thr", "rud"}
		for i, c := range cal {
			p := make([]byte, 8)
			p[0] = c.chan_
			p[1] = 1 // part 1 = deflection, 0.01 deg units
			binary.BigEndian.PutUint16(p[2:], uint16(c.cdeg))
			if err := harnessCmd(dev, canPWMCal, uint32(4+i), p); err != nil {
				return err
			}
			fmt.Printf("cal %s: %s = %+.2f deg\n", profile, names[c.chan_], float64(c.cdeg)/100)
			time.Sleep(100 * time.Millisecond)
		}
		return tailPrint(con, 2.5, 2)

	case "tail":
		secs := 3.0
		if len(args) > 0 {
			secs, _ = strconv.ParseFloat(args[0], 64)
		}
		return tailPrint(con, secs, 0)
	}
	return fmt.Errorf("drive: unknown command %q (mode|airstart|setpos|gps|wind|engine|param|cal|tail)", cmd)
}
