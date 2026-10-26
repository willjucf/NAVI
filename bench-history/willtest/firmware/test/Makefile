# Host build of the NAVI firmware logic tests.
#
# No ESP IDF and no ESP32 required. navi_logic.cpp is deliberately free of
# hardware dependencies so it compiles natively here.
#
#   make run     build and run the suite
#   make clean

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O1 -I../main
BIN      := test_navi_logic
SRCS     := test_navi_logic.cpp ../main/navi_logic.cpp

.PHONY: all run clean

all: $(BIN)

$(BIN): $(SRCS) ../main/navi_logic.h
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(BIN)

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN) $(BIN).exe
