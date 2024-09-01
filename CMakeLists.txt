cmake_minimum_required(VERSION 3.29)
project(parafoil)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

#set(CNN_NO_SERIALIZATION ON)

add_executable(parafoil)

target_sources(parafoil
        PRIVATE
            main.cpp dynamics.cpp ode45.cpp
        PUBLIC
            FILE_SET CXX_MODULES
            FILES parafoil.ixx)


