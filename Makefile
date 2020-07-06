CC = gcc
CFLAGS = -Wall -O

all: rpp

rpp: rpp.cpp
	$(CC) $(CFLAGS) rpp.cpp -o rpp -lstdc++ -lpigpio -lrt

clean:
	rm -f rpp

.PHONY: clean
