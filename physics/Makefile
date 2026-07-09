# host-side golden test for the physics model (../bmp390inv pattern)

golden: physics.c golden.c physics.h
	cc -O2 -std=c2x -Wall -Wextra -o $@ physics.c golden.c -lm
	./golden

clean:
	rm -f golden

.PHONY: clean
.DELETE_ON_ERROR:
