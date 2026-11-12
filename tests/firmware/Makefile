# Host build of the firmware logic tests. No ESP-IDF needed.
#
#   make run      build and run
#   make clean

FW       := ../../firmware/main
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O1 -I$(FW)
BIN      := test_navi_logic
SRCS     := test_navi_logic.cpp $(FW)/navi_logic.cpp

.PHONY: all run clean

all: $(BIN)

$(BIN): $(SRCS) $(FW)/navi_logic.h
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(BIN)

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN) $(BIN).exe
