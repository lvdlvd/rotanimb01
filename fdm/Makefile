# host-side golden for the fdm core (fdm-DESIGN.md F0). Links the cordic
# frontend + the bit-exact host emulation, so the math is the target's.

NLIB  = ../../n-array/lib
CMATH = ../../cordic-math/src

golden: fdm.c golden.c fdm.h
	cc -O2 -std=c2x -Wall -Wextra -Wdouble-promotion -Werror -fno-builtin \
	   -I$(NLIB) -I$(CMATH) -o $@ fdm.c golden.c $(NLIB)/cordic_math.c $(CMATH)/math_emul.c
	./golden

clean:
	rm -f golden

.PHONY: clean
.DELETE_ON_ERROR:
