# host-side golden for the linearized BMP390 inverse: bmp390lin.c must invert
# the integer compensation the flight code actually runs.
#
# bmp388.c is that reference, copied here from the sibling bminator10 project
# so this test is self-contained. Same model as ../src/nlib: the rule below
# has no prerequisite on that checkout, so the copy is never refreshed behind
# your back and
#
#	make bminator10-clean && make test
#
# is how you deliberately take the current reference and find out whether our
# inverse still matches it. A fix that belongs in the reference belongs in
# bminator10, not in the copy.
BM10_DIR ?= ../../bminator10/src
BM10_FILES = bmp388.c bmp388.h

$(BM10_FILES):
	cp $(BM10_DIR)/$@ $@

golden: golden.c bmp390lin.c bmp390lin.h $(BM10_FILES)
	cc -O2 -Wall -Wextra -I. -o golden golden.c bmp390lin.c bmp388.c -lm

test: golden
	./golden

clean:
	rm -f golden

# the deliberate upgrade gesture for the copied reference
bminator10-clean:
	rm -f $(BM10_FILES)

.PHONY: test clean bminator10-clean
