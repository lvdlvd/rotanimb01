# host-side golden for the fdm core (fdm-DESIGN.md F0). Links the cordic
# frontend + the bit-exact host emulation, so the math is the target's.

# the cordic frontend comes from the support library in ../src/nlib;
# math_emul.c is the bit-exact host emulation of the STM32G4 CORDIC, so a
# host build runs the same integer arithmetic the target's coprocessor does
NLIB  = ../src/nlib

CORDIC = $(NLIB)/cordic_math.c $(NLIB)/cordic.h $(NLIB)/cordic_port.h math_emul.c
HOSTCC = cc -O2 -std=c2x -Wall -Wextra -Wdouble-promotion -Werror -fno-builtin -I$(NLIB)

golden: fdm.c golden.c fdm.h $(CORDIC) turncheck
	$(HOSTCC) -o $@ fdm.c golden.c $(NLIB)/cordic_math.c math_emul.c
	./golden

# the direction audit the eigenvalue tests can't do: aileron -> bank -> yaw
# -> course must all agree, pulsed and held
turncheck: fdm.c turncheck.c fdm.h $(CORDIC)
	$(HOSTCC) -o $@ fdm.c turncheck.c $(NLIB)/cordic_math.c math_emul.c
	./turncheck

# SITL JSON backend: ArduPilot SITL flying THIS fdm (SIL under the HIL).
# Run it, then: arduplane --model JSON:127.0.0.1 --speedup 10 ...
sitljson: fdm.c sitljson.c fdm.h $(CORDIC)
	$(HOSTCC) -o $@ fdm.c sitljson.c $(NLIB)/cordic_math.c math_emul.c

clean:
	rm -f golden turncheck sitljson

.PHONY: clean
.DELETE_ON_ERROR:
