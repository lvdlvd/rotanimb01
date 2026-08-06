// bench — the rotanimb01 flight-campaign driver, one binary for both rigs:
// the SITL rig (fdm/sitljson + arduplane --model JSON) and the real bench
// (serbridge on the pi). See README.md for session procedures.
//
//	bench param     [-c addr] NAME [VALUE]        (get, or set then read back)
//	bench params    [-c addr] -profile bench|sitl [-reboot]
//	bench fly       [-c addr] [-alt 150]
//	bench loiter    [-c addr] [-tag run] [-takeoff] [-dur 5m]
//	bench probe     [-c addr] [-dur 2m]
//	bench tune      [-c addr] [-rev 200]
//	bench disarm    [-c addr]        (force)
//	bench reboot    [-c addr]
//	bench drive     <mode|airstart|wind|cal|tail> [args]   (on the pi)
//	bench serbridge [-dev path] [-port 5760]               (on the pi)
//
// Defaults: -c 127.0.0.1:5760 (SITL). The bench is the same port through
// serbridge on the pi (mDNS names don't resolve from Go/python sockets on
// macOS — use the pi's IP).
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
		fmt.Fprintln(os.Stderr, "usage: bench <param|params|fly|loiter|probe|tune|disarm|reboot|drive|serbridge> [flags]")
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
		dev := fs.String("dev", defHarnCmd, "harness pseudocan CDC device")
		con := fs.String("con", defHarnCon, "harness ST-Link console device")
		fs.Parse(args)
		rest := fs.Args()
		if len(rest) < 1 {
			fmt.Fprintln(os.Stderr, "usage: bench drive <mode|airstart|wind|cal|tail> [args]")
			os.Exit(2)
		}
		err = cmdDrive(rest[0], *dev, *con, rest[1:])

	case "serbridge":
		dev := fs.String("dev", "/dev/serial/by-id/usb-ArduPilot_NucleoF767ZI_240044000451323232383933-if00", "serial device")
		port := fs.Int("port", 5760, "TCP listen port")
		fs.Parse(args)
		err = cmdSerbridge(*dev, *port)

	default:
		fmt.Fprintf(os.Stderr, "unknown command %q\n", cmd)
		os.Exit(2)
	}
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
