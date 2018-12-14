cmake_minimum_required(VERSION 2.8.3)
project(SpeckleMatch)

include_directories(
    include
)

add_library(SpeckleMatch
    src/speckle_match.cpp
)