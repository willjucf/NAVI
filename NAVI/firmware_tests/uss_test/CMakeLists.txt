# Bench test for the ultrasonic on header U1 (IO26 / IO27).
#
# Build with:
#   idf.py set-target esp32
#   idf.py build flash monitor

cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(uss_test)
