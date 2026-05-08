#!/bin/bash

if [ "$1" = "install" ]
then
	sudo apt update
	sudo apt install -y build-essential pkg-config libopencv-dev
fi

flags="-O3 -std=c++17"

if [ "$1" = "camera-test" ]
then
	g++ camera_open_test.cpp -o camera_open_test $flags `pkg-config --cflags --libs opencv4`
	exit $?
fi

g++ VisualStudio/LaneKeeping/*.cpp -o app $flags `pkg-config --cflags --libs opencv4`
