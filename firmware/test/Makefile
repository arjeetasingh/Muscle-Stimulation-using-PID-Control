# Host tests of the portable firmware core and a syntax check of the sketch (no hardware needed).
#   make test
CXX ?= g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O1 -I ../muscle_stim_pid

.PHONY: test syntax clean
test: test_core controller_trace syntax
	./test_core

test_core: test_core.cpp $(wildcard ../muscle_stim_pid/*.h)
	$(CXX) $(CXXFLAGS) test_core.cpp -o $@

controller_trace: controller_trace.cpp $(wildcard ../muscle_stim_pid/*.h)
	$(CXX) $(CXXFLAGS) controller_trace.cpp -o $@

# compiles the .ino against stub Arduino/ESP32 headers: catches typos and type errors only
syntax:
	$(CXX) -std=c++17 -fsyntax-only -Wall -Wextra -x c++ -I stubs -I ../muscle_stim_pid ../muscle_stim_pid/muscle_stim_pid.ino

clean:
	rm -f test_core controller_trace
