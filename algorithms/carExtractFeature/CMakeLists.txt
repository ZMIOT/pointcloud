cmake_minimum_required(VERSION 3.20)
project(CarExtractFeature VERSION 1.0.0 LANGUAGES C CXX)
find_package(FrameworkSDK 1 CONFIG REQUIRED)
find_package(PCL 1.11 REQUIRED COMPONENTS common io)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>")
if(MSVC)
  add_compile_options(/utf-8 /bigobj)
  add_compile_definitions(NOMINMAX _USE_MATH_DEFINES)
endif()
add_library(CarLapEdgeCore STATIC core/LapEdge.cpp)
target_include_directories(CarLapEdgeCore PUBLIC core ${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
target_link_libraries(CarLapEdgeCore PUBLIC pcl_common)
framework_add_plugin(CarExtractFeatureAlgorithm TYPE algorithm
  ID custom.carExtractFeature ALGORITHM_ID custom.carExtractFeature
  VERSION 1.0.0 PARAMETER_VERSION 1 SOURCES MyAlgorithm.cpp parameters.qrc)
target_link_libraries(CarExtractFeatureAlgorithm PRIVATE CarLapEdgeCore)
set(CAR_HOST_BIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../install/bin" CACHE PATH
    "Workbench bin directory used by the CMake deploy target")
add_custom_target(CarExtractFeatureDeploy
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CAR_HOST_BIN_DIR}/plugins/algorithm"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "$<TARGET_FILE:CarExtractFeatureAlgorithm>"
    "${CAR_HOST_BIN_DIR}/plugins/algorithm/CarExtractFeatureAlgorithm.dll"
  DEPENDS CarExtractFeatureAlgorithm
  COMMENT "Deploying carExtractFeature to the workbench")
add_executable(CarLapEdgeInspect tools/Inspect.cpp)
target_link_libraries(CarLapEdgeInspect PRIVATE CarLapEdgeCore Framework::SDK pcl_io)
# VS Code 的 cppvsdbg 运行 Release 宿主；保留优化设置与 ABI，同时生成
# 核心、插件和检查器的符号，便于在 LapEdge.cpp 中设置断点。
if(MSVC)
  foreach(target CarLapEdgeCore CarExtractFeatureAlgorithm CarLapEdgeInspect)
    target_compile_options(${target} PRIVATE $<$<CONFIG:Release>:/Z7>)
  endforeach()
  foreach(target CarExtractFeatureAlgorithm CarLapEdgeInspect)
    target_link_options(${target} PRIVATE $<$<CONFIG:Release>:/DEBUG:FULL>)
  endforeach()
endif()
include(CTest)
if(BUILD_TESTING)
  add_executable(CarLapEdgeTests tests/CoreTests.cpp)
  target_link_libraries(CarLapEdgeTests PRIVATE CarLapEdgeCore)
  add_test(NAME car_lap_edge_core COMMAND CarLapEdgeTests)
  add_executable(CarLapEdgePluginTests tests/PluginTests.cpp)
  target_link_libraries(CarLapEdgePluginTests PRIVATE CarLapEdgeCore Framework::SDK pcl_io)
  target_compile_definitions(CarLapEdgePluginTests PRIVATE
    PLUGIN_PATH="$<TARGET_FILE:CarExtractFeatureAlgorithm>"
    PRESET_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../examples/carExtractFeature/parameters.preset.json")
  add_dependencies(CarLapEdgePluginTests CarExtractFeatureAlgorithm)
  add_test(NAME car_lap_edge_plugin COMMAND CarLapEdgePluginTests)
  add_test(NAME car_lap_edge_scan_recovery_3
    COMMAND CarLapEdgeInspect
      "${CMAKE_CURRENT_SOURCE_DIR}/../../examples/carExtractFeature/3.pcd"
      "${CMAKE_CURRENT_BINARY_DIR}/real3/ctest")
  add_executable(CarLapEdgeRealJitterTests tests/RealJitterTests.cpp)
  target_link_libraries(CarLapEdgeRealJitterTests PRIVATE CarLapEdgeCore pcl_io)
  add_test(NAME car_lap_edge_smooth_3
    COMMAND CarLapEdgeRealJitterTests
      "${CMAKE_CURRENT_SOURCE_DIR}/../../examples/carExtractFeature/3.pcd")
endif()
