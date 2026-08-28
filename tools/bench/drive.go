// drive — harness pseudocan commands over the local serial ports (runs ON
// the pi, next to the USB devices). Wire format lib/fmtcan, dictionary
// src/canmsg.h; the harness accepts crc-less lines.
package main

import (
	"encoding/binary"
	"encoding/hex"
	"fmt"
	"math"
	"strconv"
	"strings"
	"time"
)

// the pi's stable by-id paths
const (
	defHarnCmd = "/dev/serial/by-id/usb-rotanimb01_hitl-harness_2038334D46325004004F0038-if00"
	defHarnCon = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066DFF535550755187243416-if02"
)

const (
	canMode     = 0x43
	canAirstart = 0x44
	canFDMPos   = 0x49
	canParam    = 0x47
	canPWMCal   = 0x45
	canWind     = 0x46
)

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
		// crit_alt_m 43). 912iS = the owner's actual engine (default);
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

	case "cal":
		// PWM_CAL deflections for an ArduPilot DUT. The harness default cal
		// (controls.c) is all-positive: pwm HIGH = positive deflection, which
		// per fdm.c convention (Cmde < 0) is NOSE-DOWN elevator. ArduPilot
		// outputs ch2 HIGH for nose-UP, so the elevator wants a NEGATIVE full
		// deflection. This cal lives in harness RAM ONLY — resend after EVERY
		// harness reboot (incl. uhubctl power cycles hitting shared hub
		// ports), or the DUT flies with an inverted elevator: no rotation on
		// takeoff, elevator railed, ground-roll overspeed.
		for i, c := range []struct {
			chan_ byte
			cdeg  int16
		}{{0, 2000}, {1, -2500}, {3, 2500}} { // ail, ELE FLIPPED, rud
			p := make([]byte, 8)
			p[0] = c.chan_
			p[1] = 1 // part 1 = deflection, 0.01 deg units
			binary.BigEndian.PutUint16(p[2:], uint16(c.cdeg))
			if err := harnessCmd(dev, canPWMCal, uint32(4+i), p); err != nil {
				return err
			}
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
	return fmt.Errorf("drive: unknown command %q (mode|airstart|wind|cal|tail)", cmd)
}
