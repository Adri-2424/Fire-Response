cmake_minimum_required(VERSION 3.27)
project(FireResponse)

set(CMAKE_CXX_STANDARD 17)

add_executable(FireResponse main.cpp
        quadTree.cpp)
