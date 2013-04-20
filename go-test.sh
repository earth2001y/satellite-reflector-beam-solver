#!/bin/sh

g++ -Wall -g -o soa-test test/*.cpp src/*.cpp -DUNITTEST -lcppunit -I/usr/local/include/eigen3
[ $? -eq 0 ] && ./soa-test

