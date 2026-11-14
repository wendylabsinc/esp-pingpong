cmake_minimum_required(VERSION 3.16)

# WAMR (a transitive Wendy Lite dependency) declares a CMake minimum that
# CMake 4 rejects; Wendy Lite allows it the same way.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(taploc)
