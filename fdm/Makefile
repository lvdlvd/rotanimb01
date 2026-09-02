# host-side golden for the fdm core (fdm-DESIGN.md F0). Links the cordic
# frontend + the bit-exact host emulation, so the math is the target's.

# the cordic frontend comes from the shared nlib that ../src owns; nothing
# here reaches an n-array checkout (see ../src/Makefile for the model)
NLIB  = ../src/nlib

# math_emul.c is the bit-exact host emulation of the CORDIC, copied here from
# the sibling cordic-math project so this golden is self-contained. Same model
# as nlib/: the rule below has no prerequisite on that checkout, so the copy
# is never refreshed behind your back and
#
#	make cordic-math-clean && make
#
# is how you deliberately take the current emulation. Its only include,
# cordic_port.h, is identical to the n-array lib's, so the copy builds against
# $(NLIB) and CMATH_DIR is needed for nothing else.
CMATH_DIR ?= ../../cordic-math/src

NLIB_FILES  = $(addprefix $(NLIB)/, cordic_math.c cordic.h cordic_port.h)
CMATH_FILES = math_emul.c

$(NLIB_FILES):
	$(MAKE) -C ../src $(patsubst ../src/%,%,$@)

$(CMATH_FILES):
	cp $(CMATH_DIR)/$@ $@

golden: fdm.c golden.c fdm.h $(NLIB_FILES) $(CMATH_FILES) turncheck
	cc -O2 -std=c2x -Wall -Wextra -Wdouble-promotion -Werror -fno-builtin \
	   -I$(NLIB) -o $@ fdm.c golden.c $(NLIB)/cordic_math.c math_emul.c
	./golden

# the direction audit the eigenvalue tests can't do: aileron -> bank -> yaw
# -> course must all agree, pulsed and held
turncheck: fdm.c turncheck.c fdm.h $(NLIB_FILES) $(CMATH_FILES)
	cc -O2 -std=c2x -Wall -Wextra -Wdouble-promotion -Werror -fno-builtin \
	   -I$(NLIB) -o $@ fdm.c turncheck.c $(NLIB)/cordic_math.c math_emul.c
	./turncheck

# SITL JSON backend: ArduPilot SITL flying THIS fdm (SIL under the HIL).
# Run it, then: arduplane --model JSON:127.0.0.1 --speedup 10 ...
sitljson: fdm.c sitljson.c fdm.h $(NLIB_FILES) $(CMATH_FILES)
	cc -O2 -std=c2x -Wall -Wextra -Wdouble-promotion -Werror -fno-builtin \
	   -I$(NLIB) -o $@ fdm.c sitljson.c $(NLIB)/cordic_math.c math_emul.c

clean:
	rm -f golden turncheck sitljson

# the deliberate upgrade gesture for the copied host emulation
cordic-math-clean:
	rm -f $(CMATH_FILES)

.PHONY: clean cordic-math-clean
.DELETE_ON_ERROR:
