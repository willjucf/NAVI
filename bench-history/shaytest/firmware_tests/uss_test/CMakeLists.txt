# Bench test for one HC SR04 on the left ultrasonic pins.
#
# Build with:
#   idf.py set-target esp32
#   idf.py build flash monitor

cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(uss_test)
