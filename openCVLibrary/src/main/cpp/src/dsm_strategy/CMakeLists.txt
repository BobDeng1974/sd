include_directories(${CMAKE_SOURCE_DIR}/../)
include_directories(${CMAKE_SOURCE_DIR}/total_flow)
include_directories(${CMAKE_SOURCE_DIR}/total_flow/dsm_strategy)

set(SRCS ../total_flow.cc
        ../algorithm/align/align_method.cpp
        ../algorithm/align/faceAlign.cpp
        ../algorithm/faceid/faceid_ncnn.cpp
        ../algorithm/gaze_tracking/gaze_predict.cc
        ../algorithm/gaze_tracking/Nrmal.cc
        ../algorithm/head_pose/head_pose_estimator.cc
        ../algorithm/head_pose/steady_pose.cc
        ../algorithm/utilities/transform.cpp
        ../algorithm/utilities/utils.cc
        judger.cc
        eye_mouth_plus.cc
        head_pose_states.cc
        config_tracker.cc
        )

if(USE_ODETECTION_NCNN)
    list(APPEND SRCS ../algorithm/object_detection/rcnn/classify.cc)
    list(APPEND SRCS ../algorithm/object_detection/rcnn/object_detection.cc)
endif()
if(USE_ODETECTION_TVM)
    list(APPEND SRCS ../algorithm/object_detection/tvm/classify.cc)
    list(APPEND SRCS ../algorithm/object_detection/tvm/object_detection.cc)
    list(APPEND SRCS ../algorithm/tvm_runner/tvm_data.cc)
    list(APPEND SRCS ../algorithm/tvm_runner/tvm_runner.cc)
endif()

if(USE_LANDMARK_DLIB)
    list(APPEND SRCS ../algorithm/landmark/dlib/shape_predict.cc)
endif()
if(USE_LANDMARK_NCNN)
    list(APPEND SRCS ../algorithm/landmark/fast-rcnn/shape_predict.cc)
endif()
if(USE_LANDMARK_TVM)
    list(APPEND SRCS ../algorithm/landmark/tvm/shape_predict.cc)
    list(APPEND SRCS ../algorithm/tvm_runner/tvm_data.cc)
    list(APPEND SRCS ../algorithm/tvm_runner/tvm_runner.cc)
endif()

add_library(dsm_strategy STATIC ${SRCS})

if(USE_ODETECTION_NCNN OR USE_LANDMARK_NCNN)
    target_link_libraries(dsm_strategy ncnn)
endif()
