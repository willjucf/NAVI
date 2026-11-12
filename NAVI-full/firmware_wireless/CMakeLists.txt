# NAVI ESP32 firmware, ESP IDF project definition.
#
# Build with:
#   idf.py set-target esp32
#   idf.py build flash monitor

cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(navi_firmware)
