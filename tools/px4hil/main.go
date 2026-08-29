// px4hil — PX4 SITL simulator bridge around the SAME fdm.c the harness
// flies (sitljson's pattern with a MAVLink skin). We LISTEN; PX4 SITL
// connects as TCP client (its simulator-mavlink TCP mode) and lockstep
// follows the sensor timestamps we generate.
//
//	make px4hil && ./px4hil            # then: make px4_sitl_default none
//	                                   # (PX4 side needs the airframe config)
//
// v0.2 scope: HEARTBEAT + HIL_SENSOR (250 Hz, dithered) + HIL_GPS
// (5 Hz) out, HIL_ACTUATOR_CONTROLS in (PX4 torque signs mapped to the
// fdm's aero signs), harness actuator-lag model, ground start at the
// bench anchor, -truth JSON UDP tap (sitljson format) for the vnav
// driver. Flight-verified against px4_sitl_default 5100_px4hil_kitfox:
// auto runway takeoff to 100 m + loiter, EKF2 healthy.
package main

import (
	"flag"
	"fmt"
	"log"
	"math"
	"net"
	"sync"
	"time"

	gomav "github.com/lvdlvd/gomavlink"
	apm "github.com/lvdlvd/gomavlink/ardupilotmega"
)

const (
	sensorDt   = 0.004  // 250 Hz HIL_SENSOR cadence
	physDt     = 0.0005 // 2 kHz fdm substep, same as the target
	gpsEveryN  = 50     // 5 Hz
	beatEveryN = 250    // 1 Hz
)

// PX4's sensors-module DataValidator declares a source STALE after 100
// bit-identical samples, and the fdm's noiseless truth is exactly
// constant on a parked aircraft. Dither at roughly the emulated
// sensors' own noise floors keeps the voters alive.
const (
	accN   = 0.02   // m/s^2
	gyroN  = 0.002  // rad/s
	magN   = 0.0002 // gauss (~20 nT, RM3100-ish)
	pressN = 0.03   // hPa (~3 Pa, BMP390-ish)
	diffN  = 0.02   // hPa
	altN   = 0.25   // m
)

type dither struct{ s uint32 }

func (d *dither) u(a float32) float32 { // ~U(-a, a)
	d.s ^= d.s << 13
	d.s ^= d.s >> 17
	d.s ^= d.s << 5
	return a * float32(int32(d.s)) * (1.0 / 2147483648.0)
}

// harness controls.c actuator model: tau 60 ms + 300 deg/s rate limit,
// throttle tau 0.3 s (mirrors sitljson.c)
func lag(y, u, dt, tau, rate float32) float32 {
	a := dt / (tau + dt)
	ny := y + a*(u-y)
	dmax := rate * dt
	d := ny - y
	if d > dmax {
		d = dmax
	}
	if d < -dmax {
		d = -dmax
	}
	return y + d
}

type shared struct {
	mu  sync.Mutex
	tgt Controls // demanded, from HIL_ACTUATOR_CONTROLS
}

func main() {
	listen := flag.String("listen", ":4560", "TCP listen address for PX4 SITL")
	lat0 := flag.Float64("lat0", 45.52688, "origin latitude (bench anchor)")
	lon0 := flag.Float64("lon0", 1.667291, "origin longitude (bench anchor)")
	speedup := flag.Float64("speedup", 1, "sim speed vs wall clock (0 = unthrottled)")
	truthPort := flag.Int("truth", 0, "UDP truth tap port on localhost, sitljson JSON format, ~10 Hz (0 = off)")
	flag.Parse()

	// truth tap: same JSON the sitljson wrapper mirrors, so the vnav
	// driver reads identical truth whichever autopilot is flying
	var truth net.Conn
	if *truthPort != 0 {
		var err error
		truth, err = net.Dial("udp", fmt.Sprintf("127.0.0.1:%d", *truthPort))
		if err != nil {
			log.Fatal(err)
		}
	}

	ln, err := net.Listen("tcp", *listen)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("px4hil: waiting for PX4 SITL on %s (anchor %.5f,%.5f)", *listen, *lat0, *lon0)
	conn, err := ln.Accept()
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("px4hil: PX4 connected from %s", conn.RemoteAddr())

	f := NewFdm()
	sh := &shared{}

	// rx: actuator controls -> demanded deflections. PX4 generic FW HIL
	// layout: 0 aileron, 1 elevator, 2 rudder, 3 throttle, all -1..1
	// (throttle 0..1). Calibrate against the airframe config at bring-up.
	go func() {
		dec := gomav.NewDecoder(conn, apm.Dialect)
		for {
			msg, _, err := dec.Decode()
			if err != nil {
				log.Printf("px4hil: rx: %v", err)
				return
			}
			if m, ok := msg.(*apm.HilActuatorControls); ok {
				const maxAil, maxEle, maxRud = 20, 25, 25 // deg, harness defaults
				// PX4 outputs are "+ = positive body torque"; the fdm
				// uses classic aero signs (Cmde<0, Cndr<0: + deflection
				// = nose down / nose left) -> negate ele + rud.
				// Aileron matches (Clda>0). Verified at bring-up.
				sh.mu.Lock()
				sh.tgt.Da = m.Controls[0] * maxAil * math.Pi / 180
				sh.tgt.De = -m.Controls[1] * maxEle * math.Pi / 180
				sh.tgt.Dr = -m.Controls[2] * maxRud * math.Pi / 180
				thr := m.Controls[3]
				if thr < 0 {
					thr = 0
				}
				sh.tgt.Thr = thr
				sh.mu.Unlock()
			}
		}
	}()

	enc := gomav.NewEncoder(conn, gomav.Stream(42, 1, 0))
	dth := &dither{s: 0x2545F491}
	var act Controls // lagged, what the fdm sees
	var simUsec uint64
	n := 0
	wall0 := time.Now()
	for {
		sh.mu.Lock()
		tgt := sh.tgt
		sh.mu.Unlock()
		for i := 0; i < 8; i++ { // sensorDt/physDt
			act.Da = lag(act.Da, tgt.Da, physDt, 0.060, 300*math.Pi/180)
			act.De = lag(act.De, tgt.De, physDt, 0.060, 300*math.Pi/180)
			act.Dr = lag(act.Dr, tgt.Dr, physDt, 0.060, 300*math.Pi/180)
			act.Thr = lag(act.Thr, tgt.Thr, physDt, 0.300, 10)
			f.Step(act, physDt)
		}
		simUsec += uint64(sensorDt * 1e6)
		tr := f.Truth()

		if err := enc.Encode(&apm.HilSensor{
			TimeUsec: simUsec,
			Xacc:     tr.SForce[0] + dth.u(accN), Yacc: tr.SForce[1] + dth.u(accN), Zacc: tr.SForce[2] + dth.u(accN),
			Xgyro: tr.Rate[0] + dth.u(gyroN), Ygyro: tr.Rate[1] + dth.u(gyroN), Zgyro: tr.Rate[2] + dth.u(gyroN),
			Xmag: tr.Mag[0]/100 + dth.u(magN), Ymag: tr.Mag[1]/100 + dth.u(magN), Zmag: tr.Mag[2]/100 + dth.u(magN), // uT -> gauss
			AbsPressure:  tr.PPa/100 + dth.u(pressN), // Pa -> hPa
			DiffPressure: tr.Qbar/100 + dth.u(diffN),
			PressureAlt:  tr.H + dth.u(altN),
			Temperature:  tr.TDegc,
			FieldsUpdated: apm.HilSensorUpdatedFlags(0x1FFF),
		}); err != nil {
			log.Fatalf("px4hil: tx: %v", err)
		}

		if n%gpsEveryN == 0 {
			// int 1e-7 deg math like the harness feeder (float32 can't):
			// PosCm is cm; 1e-7 deg lat = 1.113195 cm
			lat := int32(*lat0*1e7) + int32(float64(tr.PosCm[0])/1.113195)
			lon := int32(*lon0*1e7) + int32(float64(tr.PosCm[1])/(1.113195*math.Cos(*lat0*math.Pi/180)))
			vn, ve, vd := tr.VNed[0], tr.VNed[1], tr.VNed[2]
			gs := math.Sqrt(float64(vn*vn + ve*ve))
			cog := math.Atan2(float64(ve), float64(vn)) * 180 / math.Pi
			if cog < 0 {
				cog += 360
			}
			enc.Encode(&apm.HilGps{
				TimeUsec: simUsec, Lat: lat, Lon: lon,
				Alt: int32(tr.H * 1000), Eph: 100, Epv: 150,
				Vel: uint16(gs * 100), Vn: int16(vn * 100), Ve: int16(ve * 100), Vd: int16(vd * 100),
				Cog: uint16(cog * 100), FixType: 3, SatellitesVisible: 12,
			})
		}
		if n%beatEveryN == 0 {
			enc.Encode(&apm.Heartbeat{Type: apm.MAV_TYPE_GENERIC, Autopilot: apm.MAV_AUTOPILOT_INVALID})
		}
		if truth != nil && n%25 == 0 { // 10 Hz at the 250 Hz sensor cadence
			fmt.Fprintf(truth,
				"{\"timestamp\":%.6f,"+
					"\"imu\":{\"gyro\":[%.6f,%.6f,%.6f],\"accel_body\":[%.5f,%.5f,%.5f]},"+
					"\"position\":[%.4f,%.4f,%.4f],"+
					"\"velocity\":[%.4f,%.4f,%.4f],"+
					"\"quaternion\":[%.6f,%.6f,%.6f,%.6f],"+
					"\"airspeed\":%.3f}\n",
				float64(simUsec)*1e-6,
				tr.Rate[0], tr.Rate[1], tr.Rate[2],
				tr.SForce[0], tr.SForce[1], tr.SForce[2],
				float64(tr.PosCm[0])*0.01, float64(tr.PosCm[1])*0.01, float64(tr.PosCm[2])*0.01,
				tr.VNed[0], tr.VNed[1], tr.VNed[2],
				tr.Quat[0], tr.Quat[1], tr.Quat[2], tr.Quat[3],
				// sitljson's "airspeed" is EAS at sea-level density
				math.Sqrt(float64(2*tr.Qbar/1.225)))
		}
		n++

		if *speedup > 0 {
			simWall := time.Duration(float64(simUsec) * 1000 / *speedup)
			if ahead := simWall - time.Since(wall0); ahead > time.Millisecond {
				time.Sleep(ahead)
			}
		}
		_ = fmt.Sprint() // keep fmt for future prints
	}
}
