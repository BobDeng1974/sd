if(USE_ODETECTION_NCNN)
    include_directories(${CMAKE_CURRENT_LIST_DIR}/rcnn)
endif()

if(USE_ODETECTION_TVM)
    include_directories(${CMAKE_CURRENT_LIST_DIR}/tvm)
endif()
