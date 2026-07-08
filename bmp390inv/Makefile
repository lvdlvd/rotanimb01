BM10 = ../../bminator10/src

golden: golden.c bmp390lin.c bmp390lin.h $(BM10)/bmp388.c
	cc -O2 -Wall -Wextra -I. -I$(BM10) -o golden golden.c bmp390lin.c $(BM10)/bmp388.c -lm

test: golden
	./golden

clean:
	rm -f golden

.PHONY: test clean
