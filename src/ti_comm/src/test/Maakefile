WARNING = -Wall -Wshadow --pedantic
ERROR = -Wvla -Werror
GCC = g++ -std=c++11 -lstdc++ -lm
PKG_CONFIG = pkg-config
CFLAGS = -g $(WARNING) $(ERROR) $(shell $(PKG_CONFIG) --cflags libserialport)
LIBS = $(shell $(PKG_CONFIG) --libs libserialport)

TESTFLAGS = -DDEBUG=1 -DDEBUG_TX=1 -DDEBUG_RX=1 
SOURCES = $(wildcard *.cpp)

BINARIES = $(SOURCES:.cpp=)

%: %.cpp
	$(GCC) $(CFLAGS) $< $(LIBS) -o $@

debug_%: %.cpp
	$(GCC) $(CFLAGS) $(TESTFLAGS) $< $(LIBS) -o $@

all: $(BINARIES)

clean:
	rm $(BINARIES)
