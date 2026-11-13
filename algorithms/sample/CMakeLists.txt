framework_add_plugin(SampleAlgorithm TYPE algorithm ID example.sample ALGORITHM_ID cloud.sample_pipeline SOURCES SampleAlgorithm.cpp parameters.qrc)
target_link_libraries(SampleAlgorithm PRIVATE pcl_filters pcl_segmentation)
target_include_directories(SampleAlgorithm PRIVATE ${FLANN_INCLUDE_DIR})
