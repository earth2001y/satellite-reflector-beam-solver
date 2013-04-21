#!/bin/sh

SOA=$PWD/../satelite-orbit-analysis
OPT="-DUNITTEST"

rm -f *.o ./soa-test

g++ -Wall -c $SOA/src/*.cpp $OPT
g++ -Wall -c test/*.cpp src/*.cpp $OPT -I/usr/local/include/eigen3 -I$SOA/src
g++ -Wall -o soa-test *.o -lcppunit
[ $? -eq 0 ] && ./soa-test

