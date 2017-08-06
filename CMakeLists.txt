cmake_minimum_required(VERSION 2.6)

project("stewartPlatformControl" C CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -ggdb")

include_directories(${PROJECT_SOURCE_DIR})
	
add_executable(stewartPlatformControl 
src/Raspberry\ Pi/main.cpp 
src/Raspberry\ Pi/angularcontroller.cpp 
src/Raspberry\ Pi/angularcontroller.h 
src/Raspberry\ Pi/picaminterface.cpp 
src/Raspberry\ Pi/picaminterface.h
src/Arduino\ Uno/stewartplatform.cpp 
src/Arduino\ Uno/stewartplatform.h)
