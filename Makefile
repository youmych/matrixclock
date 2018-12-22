# short help
# $@ - means the target
# $^ - means all prerequisites
# $< - means just the first prerequisite

CXX=g++-8.1.0

INCLUDES=-I./src
OPTIMIZE=-O3
RELEASE_OPTS=-DNDEBUG
CFLAGS=$(OPTIMIZE) $(RELEASE_OPTS) -Wall -Wextra -Wimplicit-fallthrough=0 $(INCLUDES)
CXXFLAGS=-std=c++17 $(OPTIMIZE) $(RELEASE_OPTS) -Wall -Wextra -Wno-psabi $(INCLUDES)
LDLIBS=-lpthread -ldl
LDFLAGS=

prefix=/usr/local
PROJECT_DIR=$(shell pwd)

ifeq ($(debug), true)
    OPTIMIZE=-O0 -ggdb
    LDFLAGS+=-ggdb
    RELEASE_OPTS=
endif

EXECUTABLE=bin/matrixclock

CXXSOURCES = src/matrixclock.cpp

OBJS=$(subst .cpp,.o,$(CXXSOURCES))

all: spidev_test matrixclock

spidev_test: bin/spidev_test

matrixclock: bin/matrixclock

bin/spidev_test: src/spidev_test.c
	mkdir -p bin
	gcc -O2 -Wall -o $@ $^

$(EXECUTABLE): $(OBJS) $(C_OBJS)
	mkdir -p bin
	@echo "Linking $(EXECUTABLE)..."
	$(CXX) $(LDFLAGS) -o $@ $(OBJS) $(C_OBJS) $(LDLIBS)

src/.cpp.o:
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f bin/spidev_test bin/matrixclock
	rm -f src/*.o

tests:
	modprobe spicc
	@echo "---==[ spidev 8 bits per word, clock speed = 1Mhz ]==---"
	./spidev_test -D /dev/spidev0.0 -s 1000000 -b 8
	@echo "---==[ spidev 16 bits per word, clock speed = 1Mhz ]==---"
	./spidev_test -D /dev/spidev0.0 -s 1000000 -b 16
	@echo "---==[ spidev 32 bits per word, clock speed = 1Mhz ]==---"
	./spidev_test -D /dev/spidev0.0 -s 1000000 -b 32
	@echo "---==[ spidev 32 bits per word, spi mode = 0 ]==---"
	./spidev_test -D /dev/spidev0.0 -s 1000000 -b 32
	@echo "---==[ spidev 32 bits per word, spi mode = 1 ]==---"
	./spidev_test -D /dev/spidev0.0 -s 1000000 -b 32 -H
	@echo "---==[ spidev 32 bits per word, spi mode = 2 ]==---"
	./spidev_test -D /dev/spidev0.0 -s 1000000 -b 32 -O
	@echo "---==[ spidev 32 bits per word, spi mode = 3 ]==---"
	./spidev_test -D /dev/spidev0.0 -s 1000000 -b 32 -H -O

install: $(EXECUTABLE)
	cp -av etc/systemd/system/matrixclock.service /etc/systemd/system/
	cp -av $(EXECUTABLE) $(prefix)/bin/

uninstall:
	rm -f /etc/systemd/system/matrixclock.service
	rm -f $(prefix)/bin/matrixclock

.PHONY: clean tests install uninstall
