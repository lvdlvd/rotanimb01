# host-side golden for the fdm core (fdm-DESIGN.md F0). Links the cordic
# frontend + the bit-exact host emulation, so the math is the target's.

# the cordic frontend comes from the shared nlib that ../src owns; nothing
# here reaches an n-array checkout (see ../src/Makefile for the model)
NLIB  = ../src/nlib
CMATH = ../../cordic-math/src

NLIB_FILES = $(addprefix $(NLIB)/, cordic_math.c cordic.h cordic_port.h)

$(NLIB_FILES):
	$(MAKE) -C ../src $(patsubst ../src/%,%,$@)

golden: fdm.c golden.c fdm.h $(NLIB_FILES) turncheck
	cc -O2 -std=c2x -Wall -Wextra -Wdouble-promotion -Werror -fno-builtin \
	   -I$(NLIB) -I$(CMATH) -o $@ fdm.c golden.c $(NLIB)/cordic_math.c $(CMATH)/math_emul.c
	./golden

# the direction audit the eigenvalue tests can't do: aileron -> bank -> yaw
# -> course must all agree, pulsed and held
turncheck: fdm.c turncheck.c fdm.h $(NLIB_FILES)
	cc -O2 -std=c2x -Wall -Wextra -Wdouble-promotion -Werror -fno-builtin \
	   -I$(NLIB) -I$(CMATH) -o $@ fdm.c turncheck.c $(NLIB)/cordic_math.c $(CMATH)/math_emul.c
	./turncheck

# SITL JSON backend: ArduPilot SITL flying THIS fdm (SIL under the HIL).
# Run it, then: arduplane --model JSON:127.0.0.1 --speedup 10 ...
sitljson: fdm.c sitljson.c fdm.h $(NLIB_FILES)
	cc -O2 -std=c2x -Wall -Wextra -Wdouble-promotion -Werror -fno-builtin \
	   -I$(NLIB) -I$(CMATH) -o $@ fdm.c sitljson.c $(NLIB)/cordic_math.c $(CMATH)/math_emul.c

clean:
	rm -f golden turncheck sitljson

.PHONY: clean
.DELETE_ON_ERROR:
