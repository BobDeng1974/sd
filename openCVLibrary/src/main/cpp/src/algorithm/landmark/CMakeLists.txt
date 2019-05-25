if(USE_LANDMARK_DLIB)
    include_directories(${CMAKE_CURRENT_LIST_DIR}/dlib)
endif()

if(USE_LANDMARK_NCNN)
    include_directories(${CMAKE_CURRENT_LIST_DIR}/fast-rcnn)
endif()

if(USE_LANDMARK_TVM)
    include_directories(${CMAKE_CURRENT_LIST_DIR}/tvm)
endif()
