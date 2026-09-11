# host-side golden for the linearized BMP390 inverse: bmp390lin.c must invert
# the integer compensation the flight code actually runs.
#
# bmp388.c/h is that reference — the datasheet's integer compensation as a
# flight-code driver would run it. The inverse is checked against it here,
# and a fix that belongs in the reference belongs in bmp388.c, not in the
# inverse.

golden: golden.c bmp390lin.c bmp390lin.h bmp388.c bmp388.h
	cc -O2 -Wall -Wextra -I. -o golden golden.c bmp390lin.c bmp388.c -lm

test: golden
	./golden

clean:
	rm -f golden

.PHONY: test clean
