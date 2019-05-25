option(USE_ALIGN            " " ON)
option(USE_FACEID           " " ON)
option(USE_GAZE_TRACKING    " " ON)
option(USE_HEAD_POSE        " " ON)
option(USE_OBJ_DETECTION    " " ON)


if(USE_ALIGN)
    include(${CMAKE_CURRENT_LIST_DIR}/align/CMakeLists.txt)
endif()

if(USE_FACEID)
    include(${CMAKE_CURRENT_LIST_DIR}/faceid/CMakeLists.txt)
endif()

if(USE_GAZE_TRACKING)
    include(${CMAKE_CURRENT_LIST_DIR}/gaze_tracking/CMakeLists.txt)
endif()

if(USE_HEAD_POSE)
    include(${CMAKE_CURRENT_LIST_DIR}/head_pose/CMakeLists.txt)
endif()

if(USE_OBJ_DETECTION)
    include(${CMAKE_CURRENT_LIST_DIR}/object_detection/CMakeLists.txt)
endif()

if(USE_LANDMARK_TVM OR USE_ODETECTION_TVM)
    include(${CMAKE_CURRENT_LIST_DIR}/tvm_runner/CMakeLists.txt)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/utilities/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/landmark/CMakeLists.txt)

if(LIBRARIES)
    list(APPEND LIBRARIES ${ALGORITHM})
endif()
