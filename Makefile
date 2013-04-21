CXX=g++
SOA=$(HOME)/dev/satelite-orbit-analysis

INCLUDES=-I/usr/local/include/eigen3
TARGET=./satelite-reflector-beam-solver

all:
	$(CXX) -o $(TARGET) -I$(SOA)/src $(INCLUDES) -DUNITTEST src/*.cpp $(SOA)/src/*.cpp

go-test:
	$(TARGET) \
		35.68 139.77 10.0 \
		"1 20580U 90037B   13107.72940891  .00001584  00000-0  10118-3 0  2193" \
		"2 20580  28.4702 309.1211 0003456  63.1635  29.3970 15.03482263 60582"

