// missions — the campaign flights, ported 1:1 from the python originals that
// flew the F5 checkride (CSV formats kept identical). All run against either
// rig: SITL (127.0.0.1:5760) or the bench (serbridge on the pi).
package main

import (
	"fmt"
	"math"
	"os"
	"sort"
	"strings"
	"time"
)

// ---- parameter profiles ------------------------------------------------------

// paramsCommon mirrors doc/f5-bench.parm's tuning core. Roll AND pitch gains
// must both be staged: a completed AUTOTUNE keeps gains in RAM only and any
// DUT reboot reverts them (the night-5/7 L1 mystery).
var paramsCommon = [][2]interface{}{
	{"AIRSPEED_CRUISE", 38.0}, {"AIRSPEED_MIN", 21.0}, {"AIRSPEED_MAX", 46.0},
	{"TRIM_THROTTLE", 45.0},
	{"TECS_CLMB_MAX", 5.0}, {"TECS_SINK_MIN", 2.8}, {"TECS_SINK_MAX", 6.0},
	{"NAVL1_PERIOD", 17.0}, {"NAVL1_DAMPING", 0.75},
	{"ROLL_LIMIT_DEG", 40.0}, {"PTCH_LIM_MAX_DEG", 20.0}, {"PTCH_LIM_MIN_DEG", -18.0},
	{"WP_LOITER_RAD", 553.0},
	{"RLL_RATE_FF", 3.0047}, {"RLL_RATE_P", 4.8389},
	{"RLL_RATE_I", 0.15}, {"RLL_RATE_D", 0.3832},
	{"PTCH_RATE_FF", 1.6387}, {"PTCH_RATE_P", 8.3447},
	{"PTCH_RATE_I", 6.2585}, {"PTCH_RATE_D", 0.4981},
	{"PTCH2SRV_TCONST", 0.75}, {"PTCH_RATE_FLTT", 2.1221},
	{"AHRS_OPTIONS", 1.0}, // no DCM fallback: the gyro bias walk corrupts it
	{"TKOFF_ROTATE_SPD", 26.0}, {"TKOFF_ALT", 150.0}, {"TKOFF_LVL_ALT", 30.0},
	{"TKOFF_DIST", 800.0}, {"TKOFF_LVL_PITCH", 12.0}, {"TKOFF_THR_MAX", 100.0},
	// MUST be 0 for a wheeled standing start (hand-launch feature = deadlock)
	{"TKOFF_THR_MINSPD", 0.0},
	{"BRD_SAFETY_DEFLT", 0.0}, {"RC_OVERRIDE_TIME", 3.0},
	{"THR_FAILSAFE", 0.0}, {"FS_SHORT_ACTN", 0.0}, {"FS_LONG_ACTN", 0.0},
	{"FS_GCS_ENABL", 0.0}, {"RTL_AUTOLAND", 2.0}, {"LOG_DISARMED", 0.0},
	{"ARSPD_USE", 1.0},
}

// bench: the harness feeds DroneCAN GPS/airspeed; ARSPD_RATIO must be
// 2/rho0 = 1.6327 (harness dp is exactly 0.5*rho0*IAS^2) — the 2.0 default
// read 10.7% high. SITL keeps its own default-consistent sensors.
var paramsBench = [][2]interface{}{
	{"CAN_P1_DRIVER", 1.0}, {"CAN_D1_PROTOCOL", 1.0}, {"GPS1_TYPE", 9.0},
	{"ARSPD_TYPE", 8.0}, {"ARSPD_SKIP_CAL", 1.0}, {"ARSPD_OFFSET", 0.0},
	{"ARSPD_RATIO", 1.6327},
	{"INS_ACCOFFS_X", 0.001}, {"INS_ACCOFFS_Y", 0.001}, {"INS_ACCOFFS_Z", 0.001},
	{"INS_ACCSCAL_X", 1.001}, {"INS_ACCSCAL_Y", 1.001}, {"INS_ACCSCAL_Z", 1.001},
	{"INS_ACC1_CALTEMP", 25.0}, {"ARMING_CRSDP_IGN", 1.0},
}

var paramsSITL = [][2]interface{}{
	{"ARSPD_TYPE", 100.0},
}

func cmdParams(s *Session, profile string, reboot bool) error {
	set := append([][2]interface{}{}, paramsCommon...)
	switch profile {
	case "bench":
		set = append(set, paramsBench...)
	case "sitl":
		set = append(set, paramsSITL...)
	default:
		return fmt.Errorf("profile %q: want bench or sitl", profile)
	}
	fmt.Printf("staging %d params (%s)\n", len(set), profile)
	// One flaky-wifi outage must not abort the campaign: collect failures
	// and give the stragglers a second full pass before giving up.
	var failed [][2]interface{}
	for _, p := range set {
		if err := s.SetParam(p[0].(string), p[1].(float64)); err != nil {
			fmt.Printf("  %v — will retry\n", err)
			failed = append(failed, p)
		}
	}
	for _, p := range failed {
		if err := s.SetParam(p[0].(string), p[1].(float64)); err != nil {
			return err
		}
	}
	fmt.Println("all params verified")
	if reboot {
		fmt.Println("rebooting to latch")
		s.Reboot()
	}
	return nil
}

// ---- shared flight phases ------------------------------------------------

// holdSim keeps overrides warm for ms of SIM time (wall time scales with
// SITL --speedup automatically).
func (s *Session) holdSim(ms uint32, ail, ele, thr uint16) {
	t0 := s.St.SimMs
	for s.St.SimMs-t0 < ms {
		s.RCOverride(ail, ele, thr, 1500)
		s.Pump(150 * time.Millisecond)
	}
}

// waitEKF gives the estimator its settling time after boot.
func (s *Session) waitEKF() {
	fmt.Println("waiting for EKF...")
	t0 := time.Now()
	for (s.St.SimMs < 42000 || !s.St.HaveHB) && time.Since(t0) < 2*time.Minute {
		s.RCNeutral()
		s.Pump(500 * time.Millisecond)
	}
}

// takeoff departs in TAKEOFF mode and climbs to within 15 m of alt.
func (s *Session) takeoff(alt float64) error {
	if err := s.SetParam("TKOFF_ALT", alt); err != nil {
		return err
	}
	s.SetMode(ModeTakeoff)
	s.Pump(500 * time.Millisecond)
	if err := s.Arm(40); err != nil {
		return err
	}
	fmt.Printf("ARMED t=%.1f, TAKEOFF to %.0f m\n", float64(s.St.SimMs)/1000, alt)
	tArm := s.St.SimMs
	var last uint32
	for s.St.Alt < alt-15 {
		if s.St.SimMs-tArm > 300000 {
			return fmt.Errorf("takeoff: never reached %.0f m", alt)
		}
		s.RCNeutral() // TAKEOFF owns the throttle
		s.Pump(300 * time.Millisecond)
		if s.St.SimMs-last > 10000 {
			last = s.St.SimMs
			fmt.Printf("  t+%5.0f ias %5.1f alt %6.1f thr %3d\n",
				float64(s.St.SimMs-tArm)/1000, s.St.IAS, s.St.Alt, s.St.Throttle)
		}
		if s.St.IAS > 42 && s.St.Alt < 5 {
			return fmt.Errorf("takeoff: NO ROTATION (overspeed on ground) — check the harness PWM cal")
		}
		if s.St.Pitch < -12 || math.Abs(s.St.Roll) > 60 {
			return fmt.Errorf("takeoff: DEPARTURE (pitch %.0f roll %.0f)", s.St.Pitch, s.St.Roll)
		}
	}
	return nil
}

// ---- stats helpers ---------------------------------------------------------

func meanSD(a []float64) (float64, float64) {
	if len(a) == 0 {
		return 0, 0
	}
	var m float64
	for _, x := range a {
		m += x
	}
	m /= float64(len(a))
	var v float64
	for _, x := range a {
		v += (x - m) * (x - m)
	}
	return m, math.Sqrt(v / float64(len(a)))
}

// circleFit is the Kasa fit over a lat/lon track, in meters.
func circleFit(lat, lon []float64) (r, rsd float64) {
	n := len(lat)
	if n < 8 {
		return math.NaN(), math.NaN()
	}
	la0 := lat[0]
	xs := make([]float64, n)
	ys := make([]float64, n)
	for i := range lat {
		xs[i] = (lat[i] - la0) * 111320
		ys[i] = (lon[i] - lon[0]) * 111320 * math.Cos(la0*math.Pi/180)
	}
	var mx, my float64
	for i := range xs {
		mx += xs[i]
		my += ys[i]
	}
	mx /= float64(n)
	my /= float64(n)
	var suu, svv, suv, suuu, svvv, suvv, svuu float64
	for i := range xs {
		u, v := xs[i]-mx, ys[i]-my
		suu += u * u
		svv += v * v
		suv += u * v
		suuu += u * u * u
		svvv += v * v * v
		suvv += u * v * v
		svuu += v * u * u
	}
	det := suu*svv - suv*suv
	if math.Abs(det) < 1e-6 {
		return math.NaN(), math.NaN()
	}
	uc := (svv*(suuu+suvv) - suv*(svvv+svuu)) / (2 * det)
	vc := (suu*(svvv+svuu) - suv*(suuu+suvv)) / (2 * det)
	rs := make([]float64, n)
	for i := range xs {
		rs[i] = math.Hypot(xs[i]-mx-uc, ys[i]-my-vc)
	}
	return meanSD(rs)
}

// ---- loiter legs -----------------------------------------------------------

type row struct {
	simMs                   uint32
	ias, alt, roll, navRoll float64
	pitch, navPitch         float64
	thr                     int
	lat, lon                float64
	wall                    float64
	servo1, servo2          uint16
}

func writeCSV(fn string, rows []row) error {
	f, err := os.Create(fn)
	if err != nil {
		return err
	}
	defer f.Close()
	fmt.Fprintln(f, "t_ms,ias,alt,roll,nav_roll,pitch,nav_pitch,thr,lat,lon,wall,servo1,servo2")
	for _, r := range rows {
		fmt.Fprintf(f, "%d,%g,%g,%g,%g,%g,%g,%d,%g,%g,%.3f,%d,%d\n",
			r.simMs, r.ias, r.alt, r.roll, r.navRoll, r.pitch, r.navPitch,
			r.thr, r.lat, r.lon, r.wall, r.servo1, r.servo2)
	}
	return nil
}

func (s *Session) leg(tag, name string, speed, rad float64, dur time.Duration) error {
	fmt.Printf("--- leg %s: %.0f m/s R%.0f ---\n", name, speed, rad)
	if err := s.SetParam("AIRSPEED_CRUISE", speed); err != nil {
		return err
	}
	if err := s.SetParam("WP_LOITER_RAD", rad); err != nil {
		return err
	}
	s.SetMode(ModeFBWA)
	s.Pump(time.Second)
	s.SetMode(ModeLoiter) // radius latches at entry
	t0 := s.St.SimMs
	durMs := uint32(dur.Milliseconds())
	var rows []row
	var last uint32
	for s.St.SimMs-t0 < durMs {
		s.RCNeutral()
		s.Pump(200 * time.Millisecond)
		rows = append(rows, row{s.St.SimMs, s.St.IAS, s.St.Alt, s.St.Roll,
			s.St.NavRoll, s.St.Pitch, s.St.NavPitch, s.St.Throttle,
			s.St.Lat, s.St.Lon, float64(time.Now().UnixNano()) / 1e9,
			s.St.Servo[0], s.St.Servo[1]})
		if s.St.SimMs-last > 30000 {
			last = s.St.SimMs
			fmt.Printf("  t+%5.0f ias %5.1f alt %5.0f roll %6.1f dem %6.1f thr %3d\n",
				float64(s.St.SimMs-t0)/1000, s.St.IAS, s.St.Alt, s.St.Roll,
				s.St.NavRoll, s.St.Throttle)
		}
	}
	fn := fmt.Sprintf("loiter-%s-%s.csv", tag, name)
	if err := writeCSV(fn, rows); err != nil {
		return err
	}
	// stats over the last 120 s (post-capture)
	var ias, roll, dem, lat, lon []float64
	for _, r := range rows {
		if r.simMs > s.St.SimMs-120000 {
			ias = append(ias, r.ias)
			roll = append(roll, r.roll)
			dem = append(dem, r.navRoll)
			lat = append(lat, r.lat)
			lon = append(lon, r.lon)
		}
	}
	im, isd := meanSD(ias)
	rm, rsd := meanSD(roll)
	dm, dsd := meanSD(dem)
	cr, crsd := circleFit(lat, lon)
	fmt.Printf("  LAST 120 s: ias %.1f±%.1f (cmd %.0f), roll %.1f±%.1f, demand %.1f±%.1f, R %.0f±%.0f m (cmd %.0f)  [%s]\n",
		im, isd, speed, rm, rsd, dm, dsd, cr, crsd, rad, fn)
	return nil
}

func cmdLoiter(s *Session, tag string, doTakeoff bool, dur time.Duration) error {
	s.waitEKF()
	if doTakeoff {
		if err := s.takeoff(300); err != nil {
			return err
		}
		fmt.Println("airborne at working altitude — streams up")
	}
	s.MsgInterval(MsgAttitude, 10)
	s.MsgInterval(MsgNavControllerOutput, 5)
	s.MsgInterval(MsgVFRHUD, 5)
	s.MsgInterval(MsgGlobalPositionInt, 2)
	s.MsgInterval(MsgServoOutputRaw, 5)
	if err := s.leg(tag, "std", 29, 553, dur); err != nil {
		return err
	}
	if err := s.leg(tag, "fast", 38, 363, dur); err != nil {
		return err
	}
	fmt.Println("legs done — left in LOITER")
	return nil
}

// ---- fly: departure + hands-off check --------------------------------------

func cmdFly(s *Session, alt float64) error {
	s.waitEKF()
	if err := s.takeoff(alt); err != nil {
		return err
	}
	fmt.Printf("reached %.0f m — FBWA hands-off 60 s at cruise throttle\n", s.St.Alt)
	s.SetMode(ModeFBWA)
	t0 := s.St.SimMs
	worstRoll, worstPitch := 0.0, 0.0
	for s.St.SimMs-t0 < 60000 {
		s.RCOverride(1500, 1500, 1450, 1500) // FBWA throttle is manual: ~cruise
		s.Pump(200 * time.Millisecond)
		worstRoll = math.Max(worstRoll, math.Abs(s.St.Roll))
		worstPitch = math.Max(worstPitch, math.Abs(s.St.Pitch))
	}
	fmt.Printf("hands-off: worst |roll| %.1f, worst |pitch| %.1f, ias %.1f alt %.1f\n",
		worstRoll, worstPitch, s.St.IAS, s.St.Alt)
	if worstRoll < 25 && worstPitch < 25 {
		fmt.Println("FLY CHECK PASS")
		return nil
	}
	return fmt.Errorf("FLY CHECK MARGINAL")
}

// ---- probe: straight-line consistency --------------------------------------

func angDiff(a, b float64) float64 {
	d := math.Mod(a-b+540, 360)
	return d - 180
}

func cmdProbe(s *Session, dur time.Duration) error {
	s.MsgInterval(MsgGPSRawInt, 5)
	s.MsgInterval(MsgAttitude, 10)
	s.SetMode(ModeFBWA)
	fmt.Println("FBWA straight, settling 10 s...")
	s.holdSim(10000, 1500, 1500, 1450)
	fmt.Printf("recording %v...\n", dur)
	t0 := s.St.SimMs
	durMs := uint32(dur.Milliseconds())
	var dcog, dekf, dv []float64
	var last uint32
	for s.St.SimMs-t0 < durMs {
		s.RCOverride(1500, 1500, 1450, 1500)
		s.Pump(200 * time.Millisecond)
		if s.St.GPSVel < 5 {
			continue // not moving: nothing to compare
		}
		ekfCourse := math.Atan2(s.St.Vy, s.St.Vx) * 180 / math.Pi
		if ekfCourse < 0 {
			ekfCourse += 360
		}
		dcog = append(dcog, angDiff(s.St.GPSCog, s.St.Yaw))
		dekf = append(dekf, angDiff(ekfCourse, s.St.Yaw))
		dv = append(dv, s.St.GPSVel-s.St.IAS)
		if s.St.SimMs-last > 20000 {
			last = s.St.SimMs
			fmt.Printf("  yaw %6.1f cog %6.1f ekfv %6.1f |gps| %5.1f ias %5.1f roll %5.1f\n",
				s.St.Yaw, s.St.GPSCog, ekfCourse, s.St.GPSVel, s.St.IAS, s.St.Roll)
		}
	}
	cm, cs := meanSD(dcog)
	em, es := meanSD(dekf)
	vm, vs := meanSD(dv)
	fmt.Printf("cog-yaw %.1f±%.1f deg | ekfvel-yaw %.1f±%.1f deg | gpsV-ias %.1f±%.1f m/s | n=%d\n",
		cm, cs, em, es, vm, vs, len(dcog))
	return nil
}

// ---- tune: pitch autotune ----------------------------------------------------

// cmdTune runs the pitch AUTOTUNE campaign. Lessons encoded: AUTOTUNE_AXES=2
// leaves the validated roll gains alone; events need stick REVERSALS crossing
// trim; stop() RESTORES gains on mode exit unless BOTH D and P limits were
// found, so gains are read while still in the mode; the altitude floor is
// recovered IN-MODE (AUTOTUNE flies like FBWA).
func cmdTune(s *Session, maxRev int) error {
	const workAlt, floorAlt, recoveredAlt = 400.0, 280.0, 380.0
	for _, p := range [][2]interface{}{{"AUTOTUNE_AXES", 2.0}, {"TKOFF_THR_MINSPD", 0.0}} {
		if err := s.SetParam(p[0].(string), p[1].(float64)); err != nil {
			return err
		}
	}
	s.waitEKF()
	if err := s.takeoff(workAlt); err != nil {
		return err
	}
	fmt.Printf("at %.0f m — entering AUTOTUNE (pitch only)\n", s.St.Alt)
	s.MsgInterval(MsgVFRHUD, 5)
	s.MsgInterval(MsgAttitude, 10)
	s.SetMode(ModeAutotune)
	s.holdSim(1000, 1500, 1500, 1450)

	finished := false
	s.textHook = func(txt string) {
		if strings.Contains(txt, "Pitch: Finished") || strings.Contains(txt, "AUTOTUNE: Success") {
			finished = true
		}
	}
	defer func() { s.textHook = nil }()

	nRev := 0
	for !finished && nRev < maxRev {
		if s.St.Alt < floorAlt {
			fmt.Printf("  floor! alt %.0f — in-mode recovery\n", s.St.Alt)
			for s.St.Alt < recoveredAlt {
				s.RCOverride(1500, 1350, 2000, 1500)
				s.Pump(200 * time.Millisecond)
			}
			fmt.Printf("  recovered to %.0f\n", s.St.Alt)
		}
		thr := uint16(1500)
		if s.St.IAS < 30 {
			thr = 1700
		} else if s.St.IAS > 42 {
			thr = 1300
		}
		s.holdSim(1600, 1500, 1200, thr) // full up
		s.holdSim(1600, 1500, 1800, thr) // full down
		nRev++
		s.holdSim(800, 1500, 1500, thr) // settle so the error re-arms
		if nRev%10 == 0 {
			fmt.Printf("  %d reversals, alt %.0f ias %.1f\n", nRev, s.St.Alt, s.St.IAS)
		}
	}
	fmt.Printf("reversals: %d, finished: %v\n", nRev, finished)

	// read gains IN-MODE
	gains := []string{"PTCH_RATE_FF", "PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D",
		"PTCH2SRV_TCONST", "PTCH_RATE_FLTT"}
	sort.Strings(gains)
	fmt.Print("IN-MODE GAINS:")
	for _, g := range gains {
		v, err := s.GetParam(g)
		if err != nil {
			fmt.Printf(" %s=?", g)
			continue
		}
		fmt.Printf(" %s=%.4f", g, v)
	}
	fmt.Println()
	if finished {
		s.SetMode(ModeFBWA)
		s.holdSim(2000, 1500, 1500, 1450)
		fmt.Println("finished tune — gains latched, back to FBWA")
		fmt.Println("REMEMBER: param_set the learned gains explicitly — a DUT reboot reverts RAM-held autotune gains")
	}
	return nil
}
