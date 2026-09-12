// bench — the rotanimb01 flight-campaign driver, one binary for both rigs:
// the SITL rig (fdm/sitljson + arduplane --model JSON) and the real bench
// (serbridge on the bench host). See README.md for session procedures.
//
//	bench arm       [-c addr]
//	bench mode      [-c addr] manual|fbwa|autotune|rtl|loiter|takeoff
//	bench param     [-c addr] NAME [VALUE]        (get, or set then read back)
//	bench params    [-c addr] -profile bench|sitl [-reboot]
//	bench fly       [-c addr] [-alt 150]
//	bench loiter    [-c addr] [-tag run] [-takeoff] [-dur 5m]
//	bench probe     [-c addr] [-dur 2m]
//	bench tune      [-c addr] [-rev 200]
//	bench disarm    [-c addr]        (force)
//	bench reboot    [-c addr]
//	bench watch     [-c addr] [-dur 2m]
//	bench mode      [-c addr] hdgalt                 (experimental fork mode, README)
//	bench hdgaltcmd [-c addr] [-hdg -trate -alt -crate]
//	bench drive     <mode|airstart|setpos|gps|wind|engine|param|cal|tail> [args]   (on the bench host)
//	bench serbridge [-dev path] [-port 5760]                                 (on the bench host)
//	bench usbreset  [-dev path]                        (Linux, root: USBDEVFS_RESET of a wedged CDC)
//
// Defaults: -c 127.0.0.1:5760 (SITL). The bench is the same port through
// serbridge on the host the USB devices hang off (mDNS names don't resolve
// from Go sockets on macOS — use the host's IP). Device defaults: drive.go.
package main

import (
	"flag"
	"fmt"
	"os"
	"strconv"
	"time"
)

func dial(addr string) *Session {
	s, err := Dial(addr)
	if err != nil {
		fmt.Fprintf(os.Stderr, "connect %s: %v\n", addr, err)
		os.Exit(1)
	}
	if err := s.WaitHeartbeat(15 * time.Second); err != nil {
		fmt.Fprintf(os.Stderr, "%s: %v\n", addr, err)
		os.Exit(1)
	}
	fmt.Printf("connected to %s (sys %d, mode %d, armed %v)\n",
		addr, s.target, s.St.Mode, s.St.Armed)
	return s
}

func main() {
	if len(os.Args) < 2 {
		fmt.Fprintln(os.Stderr, "usage: bench <arm|mode|param|params|fly|loiter|probe|tune|watch|disarm|reboot|drive|serbridge|usbreset> [flags]")
		os.Exit(2)
	}
	cmd, args := os.Args[1], os.Args[2:]
	fs := flag.NewFlagSet(cmd, flag.ExitOnError)
	addr := fs.String("c", "127.0.0.1:5760", "MAVLink TCP address (serbridge or SITL)")

	var err error
	switch cmd {
	case "params":
		profile := fs.String("profile", "", "bench | sitl")
		reboot := fs.Bool("reboot", false, "reboot after staging to latch")
		fs.Parse(args)
		err = cmdParams(dial(*addr), *profile, *reboot)

	case "fly":
		alt := fs.Float64("alt", 150, "takeoff altitude, m")
		fs.Parse(args)
		err = cmdFly(dial(*addr), *alt)

	case "loiter":
		tag := fs.String("tag", "run", "CSV filename tag")
		takeoff := fs.Bool("takeoff", false, "depart first (else assumes airborne)")
		dur := fs.Duration("dur", 5*time.Minute, "per-leg duration (sim time)")
		fs.Parse(args)
		err = cmdLoiter(dial(*addr), *tag, *takeoff, *dur)

	case "probe":
		dur := fs.Duration("dur", 2*time.Minute, "recording duration (sim time)")
		fs.Parse(args)
		err = cmdProbe(dial(*addr), *dur)

	case "tune":
		rev := fs.Int("rev", 200, "max elevator reversals")
		fs.Parse(args)
		err = cmdTune(dial(*addr), *rev)

	case "param":
		fs.Parse(args)
		rest := fs.Args()
		if len(rest) < 1 {
			fmt.Fprintln(os.Stderr, "usage: bench param NAME [VALUE]")
			os.Exit(2)
		}
		s := dial(*addr)
		if len(rest) >= 2 {
			var v float64
			if v, err = strconv.ParseFloat(rest[1], 64); err != nil {
				break
			}
			if err = s.SetParam(rest[0], v); err != nil {
				break
			}
		}
		var got float32
		if got, err = s.GetParam(rest[0]); err == nil {
			fmt.Printf("%s = %g\n", rest[0], got)
		}

	case "mode":
		fs.Parse(args)
		rest := fs.Args()
		modes := map[string]uint32{"manual": ModeManual, "fbwa": ModeFBWA, "hdgalt": ModeHdgAlt,
			"autotune": ModeAutotune, "rtl": ModeRTL, "loiter": ModeLoiter,
			"takeoff": ModeTakeoff}
		if len(rest) < 1 || modes[rest[0]] == 0 && rest[0] != "manual" {
			fmt.Fprintln(os.Stderr, "usage: bench mode manual|fbwa|autotune|rtl|loiter|takeoff")
			os.Exit(2)
		}
		s := dial(*addr)
		s.SetMode(modes[rest[0]])
		s.Pump(time.Second)
		fmt.Printf("mode set: %s (now %d)\n", rest[0], s.St.Mode)

	case "watch":
		dur := fs.Duration("dur", 30*time.Second, "how long to watch")
		fs.Parse(args)
		s := dial(*addr)
		s.QuietText = false
		end := time.Now().Add(*dur)
		for time.Now().Before(end) {
			s.Pump(time.Second)
			st := &s.St
			fmt.Printf("mode %2d alt %6.0f clb %+5.1f ias %4.1f thr %3d%% pit %+5.1f/%+5.1f rll %+6.1f nav %+5.1f aerr %+7.1f serr %+5.1f\n",
				st.Mode, st.Alt, st.Climb, st.IAS, st.Throttle, st.Pitch, st.NavPitch, st.Roll, st.NavRoll, st.AltError, st.AspdError)
		}

	case "hdgaltcmd":
		hdg := fs.Float64("hdg", -1, "heading deg (unset: leave)")
		trate := fs.Float64("trate", 0, "turn rate deg/s (0: leave)")
		alt := fs.Float64("alt", -1, "altitude m MSL (unset: leave)")
		crate := fs.Float64("crate", 0, "climb rate m/s (0: leave)")
		fs.Parse(args)
		var flags uint16
		if *hdg >= 0 {
			flags |= 1
		}
		if *trate != 0 {
			flags |= 2
		}
		if *alt >= 0 {
			flags |= 4
		}
		if *crate != 0 {
			flags |= 8
		}
		s := dial(*addr)
		if err = s.SendHdgAltCommand(flags, *hdg, *trate, *alt, *crate); err == nil {
			s.Pump(time.Second)
			fmt.Printf("hdgalt command sent (flags %#x)\n", flags)
		}

	case "arm":
		fs.Parse(args)
		s := dial(*addr)
		if err = s.Arm(40); err == nil {
			fmt.Println("armed")
		}

	case "disarm":
		fs.Parse(args)
		s := dial(*addr)
		s.ForceDisarm()
		fmt.Println("force-disarmed")

	case "reboot":
		fs.Parse(args)
		s := dial(*addr)
		s.Reboot()
		fmt.Println("reboot sent")

	case "drive":
		dev := fs.String("dev", defHarnCmd(), "harness pseudocan CDC device ($ROTANIMB01_HARNESS)")
		con := fs.String("con", defHarnCon(), "harness ST-Link console device ($ROTANIMB01_CONSOLE)")
		fs.Parse(args)
		rest := fs.Args()
		if len(rest) < 1 {
			fmt.Fprintln(os.Stderr, "usage: bench drive <mode|airstart|setpos|gps|wind|engine|param|cal|tail> [args]")
			os.Exit(2)
		}
		err = cmdDrive(rest[0], *dev, *con, rest[1:])

	case "serbridge":
		dev := fs.String("dev", defDutDev(), "DUT MAVLink serial device ($ROTANIMB01_DUT)")
		port := fs.Int("port", 5760, "TCP listen port")
		fs.Parse(args)
		err = cmdSerbridge(*dev, *port)

	case "usbreset":
		dev := fs.String("dev", defHarnCmd(), "tty device whose USB parent to reset ($ROTANIMB01_HARNESS)")
		fs.Parse(args)
		err = usbReset(*dev)

	default:
		fmt.Fprintf(os.Stderr, "unknown command %q\n", cmd)
		os.Exit(2)
	}
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
