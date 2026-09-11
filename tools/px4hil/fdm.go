// px4hil — PX4 SITL simulator bridge around the SAME fdm.c the harness
// flies (the sitljson pattern, MAVLink-skinned): PX4 connects to :4560,
// sends HIL_ACTUATOR_CONTROLS, we lockstep the Kitfox and answer with
// HIL_SENSOR + HIL_GPS.
package main

/*
#cgo CFLAGS: -I../../fdm -I../../src/nlib -O2 -fno-builtin
#include "fdm.h"
#include <stdlib.h>
*/
import "C"
import "fmt"

type Fdm struct{ f C.struct_Fdm }

type Controls struct{ Da, De, Dr, Thr float32 }

type Truth struct {
	H          float32
	PosCm      [3]int32
	VNed       [3]float32
	Quat       [4]float32
	SForce     [3]float32
	Rate       [3]float32
	Mag        [3]float32
	PPa, TDegc float32
	Qbar, Va   float32
}

func NewFdm() *Fdm {
	f := &Fdm{}
	C.fdm_defaults(&f.f)
	return f
}

func (f *Fdm) Trim(altM, ias, hdg float32) (Controls, error) {
	var out C.struct_FdmControls
	if C.fdm_trim(&f.f, C.float(altM), C.float(ias), C.float(hdg), &out) != 0 {
		return Controls{}, fmt.Errorf("fdm_trim failed (alt %.0f ias %.0f)", altM, ias)
	}
	return Controls{float32(out.da), float32(out.de), float32(out.dr), float32(out.dt)}, nil
}

func (f *Fdm) Step(c Controls, dt float32) {
	cc := C.struct_FdmControls{da: C.float(c.Da), de: C.float(c.De), dr: C.float(c.Dr), dt: C.float(c.Thr)}
	C.fdm_step(&f.f, &cc, C.float(dt))
}

func (f *Fdm) Truth() Truth {
	t := &f.f.truth
	var o Truth
	o.H = float32(t.h)
	for i := 0; i < 3; i++ {
		o.PosCm[i] = int32(t.pos_cm[i])
		o.VNed[i] = float32(t.v_ned[i])
		o.SForce[i] = float32(t.sforce[i])
		o.Rate[i] = float32(t.rate[i])
		o.Mag[i] = float32(t.mag[i])
	}
	for i := 0; i < 4; i++ {
		o.Quat[i] = float32(t.quat[i])
	}
	o.PPa = float32(t.p_pa)
	o.TDegc = float32(t.t_degc)
	o.Qbar = float32(t.qbar)
	o.Va = float32(t.va)
	return o
}
