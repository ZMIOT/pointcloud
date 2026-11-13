cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(CorrugatedPanelFeature VERSION 1.0.0 LANGUAGES C CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTORCC ON)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
  find_package(FrameworkSDK 1 CONFIG REQUIRED)
  find_package(PCL 1.11 REQUIRED COMPONENTS common io)
  if(MSVC)
    add_compile_options(/utf-8 /bigobj)
    add_compile_definitions(NOMINMAX _USE_MATH_DEFINES)
  endif()
endif()

find_package(OpenMP QUIET)

add_library(CorrugatedPanelCore STATIC
  core/CorrugatedPanel.cpp
  core/SurfaceIntersection.cpp
  core/CorrugatedPanel.h)
target_include_directories(CorrugatedPanelCore PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core>
  ${EIGEN_INCLUDE_DIRS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(CorrugatedPanelCore PUBLIC OpenMP::OpenMP_CXX)
  target_compile_definitions(CorrugatedPanelCore PUBLIC BWB_HAS_OPENMP=1)
endif()

framework_add_plugin(CorrugatedPanelAlgorithm TYPE algorithm
  ID custom.corrugatedPanelFeature
  ALGORITHM_ID custom.corrugatedPanelFeature
  VERSION 1.6.1
  PARAMETER_VERSION 4
  SOURCES MyAlgorithm.cpp parameters.qrc)
target_link_libraries(CorrugatedPanelAlgorithm PRIVATE CorrugatedPanelCore)

add_executable(CorrugatedPanelInspect tools/Inspect.cpp)
target_link_libraries(CorrugatedPanelInspect PRIVATE CorrugatedPanelCore)

include(CTest)
if(BUILD_TESTING)
  add_executable(CorrugatedPanelIntersectionTests tests/IntersectionTests.cpp)
  target_link_libraries(CorrugatedPanelIntersectionTests PRIVATE CorrugatedPanelCore)
  add_test(NAME corrugated_panel_intersection COMMAND CorrugatedPanelIntersectionTests)
  add_executable(CorrugatedPanelCoreTests tests/CoreTests.cpp)
  target_link_libraries(CorrugatedPanelCoreTests PRIVATE CorrugatedPanelCore)
  add_test(NAME corrugated_panel_core COMMAND CorrugatedPanelCoreTests)
  add_executable(CorrugatedPanelRealSampleTests tests/RealSampleTests.cpp)
  target_link_libraries(CorrugatedPanelRealSampleTests PRIVATE CorrugatedPanelCore)
  add_test(NAME corrugated_panel_sample6 COMMAND CorrugatedPanelRealSampleTests)
  set_tests_properties(corrugated_panel_sample6 PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME corrugated_panel_sample52 COMMAND CorrugatedPanelRealSampleTests 52)
  set_tests_properties(corrugated_panel_sample52 PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME corrugated_panel_sample1 COMMAND CorrugatedPanelRealSampleTests 1)
  set_tests_properties(corrugated_panel_sample1 PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME corrugated_panel_sample121_fallback COMMAND CorrugatedPanelRealSampleTests 121)
  set_tests_properties(corrugated_panel_sample121_fallback PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(CorrugatedPanelPluginTests tests/PluginTests.cpp)
  target_link_libraries(CorrugatedPanelPluginTests PRIVATE Framework::SDK pcl_common)
  target_compile_definitions(CorrugatedPanelPluginTests PRIVATE
    PLUGIN_PATH="$<TARGET_FILE:CorrugatedPanelAlgorithm>")
  add_dependencies(CorrugatedPanelPluginTests CorrugatedPanelAlgorithm)
  add_test(NAME corrugated_panel_plugin COMMAND CorrugatedPanelPluginTests)
endif()
