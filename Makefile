CXX=g++
SOA=$(HOME)/dev/satellite-orbit-analysis

INCLUDES=-I/usr/local/include/eigen3
TARGET=./satellite-reflector-beam-solver

all:
	$(CXX) -o $(TARGET) -I$(SOA)/src $(INCLUDES) -DUNITTEST src/*.cpp $(SOA)/src/*.cpp

run-test:
	$(TARGET) \
		35.68 139.77 10.0 \
		"1 28622U 05006A   13126.69863778 -.00000297  00000-0  10000-3 0  8798" \
		"2 28622   0.0267   0.5663 0000597 295.2122 320.4711  1.00271129 29980"

