# Standalone soundsync firmware: ESP-NOW timing beacons with hardware receive
# stamps, kept beside Wendy Lite (same partition table, so a board's Wendy
# enrollment survives and Wendy Lite can be reflashed at any time).
cmake_minimum_required(VERSION 3.22)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(soundsync_fw)
