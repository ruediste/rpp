CC = gcc
CFLAGS = -Wall -O

all: rpp

rpp: rpp.cpp
	$(CC) $(CFLAGS) rpp.cpp -o rpp -lstdc++

clean:
	rm -f rpp

.PHONY: clean
