# host-side golden for the fdm core (fdm-DESIGN.md F0). Links the cordic
# frontend + the bit-exact host emulation, so the math is the target's.

NLIB  = ../../n-array/lib
CMATH = ../../cordic-math/src

golden: fdm.c golden.c fdm.h turncheck
	cc -O2 -std=c2x -Wall -Wextra -Wdouble-promotion -Werror -fno-builtin \
	   -I$(NLIB) -I$(CMATH) -o $@ fdm.c golden.c $(NLIB)/cordic_math.c $(CMATH)/math_emul.c
	./golden

# the direction audit the eigenvalue tests can't do: aileron -> bank -> yaw
# -> course must all agree, pulsed and held
turncheck: fdm.c turncheck.c fdm.h
	cc -O2 -std=c2x -Wall -Wextra -Wdouble-promotion -Werror -fno-builtin \
	   -I$(NLIB) -I$(CMATH) -o $@ fdm.c turncheck.c $(NLIB)/cordic_math.c $(CMATH)/math_emul.c
	./turncheck

clean:
	rm -f golden turncheck

.PHONY: clean
.DELETE_ON_ERROR:
