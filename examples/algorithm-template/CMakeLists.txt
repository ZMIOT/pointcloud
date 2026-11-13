cmake_minimum_required(VERSION 3.20)
project(MyAlgorithm LANGUAGES C CXX)
find_package(FrameworkSDK 1 CONFIG REQUIRED)
framework_add_plugin(MyAlgorithm TYPE algorithm ID custom.identity ALGORITHM_ID custom.identity SOURCES MyAlgorithm.cpp parameters.qrc)
