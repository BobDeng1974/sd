
# Sets the minimum version of CMake required to build the native
# library. You should either keep the default value or only pass a
# value of 3.4.0 or lower.

cmake_minimum_required(VERSION 3.4.1)

set(CMAKE_VERBOSE_MAKEFILE on)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(src/main/cpp/include/ncnn)

include_directories(src/main/cpp/bridge)
include_directories(src/main/cpp/src/algorithm/align)
include_directories(src/main/cpp/src/algorithm/faceid)
#include_directories(src/main/cpp/algorithm/gaze_tracking)
include_directories(src/main/cpp/src/algorithm/head_pose)
include_directories(src/main/cpp/src/algorithm/object_detection/rcnn)
include_directories(src/main/cpp/src/algorithm/utilities)
include_directories(src/main/cpp/src/dsm_strategy)
include_directories(src/main/cpp/src/dsm_strategy/utils/file)
include_directories(src/main/cpp/src/dsm_strategy/utils/time)

include_directories(src/main/cpp/include)
include_directories(src/main/cpp/include/google)


#opencv
#include_directories(/Users/chris/StudioProjects/include)
add_library( lib_opencv SHARED IMPORTED )
set_target_properties(lib_opencv PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_java3.so)

#dlib
#include_directories(src/main/cpp/algorithm/landmark/dlib/include)
add_library( libdlib STATIC IMPORTED )
set_target_properties(libdlib PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libdlib.a)
#file(GLOB native_srcs7 "src/main/cpp/algorithm/landmark/dlib/src/*.cc")


#gg
add_library( libpro STATIC IMPORTED )
set_target_properties(libpro PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libprotobuf.a)
add_library( libprotol STATIC IMPORTED )
set_target_properties(libprotol PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libprotobuf-lite.a)
add_library( libprotoc STATIC IMPORTED )
set_target_properties(libprotoc PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libprotoc.a)

#ncnn
include_directories(src/main/cpp/src/algorithm/landmark/fast-rcnn)
add_library( lib_ncnn STATIC IMPORTED )
set_target_properties(lib_ncnn PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libncnn.a)
file(GLOB native_srcs7 "src/main/cpp/src/algorithm/landmark/fast-rcnn/*.cc")

#socket
#add_library( libmsg STATIC IMPORTED )
#set_target_properties(libmsg PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libhpsocket.so)


file(GLOB native_srcs "src/main/cpp/*.cpp" "src/main/cpp/*.cc")
file(GLOB native_srcs2 "src/main/cpp/src/algorithm/align/*.cpp")
file(GLOB native_srcs4 "src/main/cpp/src/algorithm/faceid/*.cpp")
#file(GLOB native_srcs5 "src/main/cpp/algorithm/gaze_tracking/*.cpp")
file(GLOB native_srcs6 "src/main/cpp/src/algorithm/head_pose/*.cc")
file(GLOB native_srcs8 "src/main/cpp/src/algorithm/object_detection/rcnn/*.cc")
file(GLOB native_srcs9 "src/main/cpp/src/algorithm/utilities/*.cc" "src/main/cpp/src/algorithm/utilities/*.cpp")

file(GLOB native_srcs10 "src/main/cpp/src/dsm_strategy/*.cpp" "src/main/cpp/src/dsm_strategy/*.cc")
file(GLOB native_srcs11 "src/main/cpp/src/dsm_strategy/utils/file/*.cpp" "src/main/cpp/src/dsm_strategy/utils/file/*.cc")
file(GLOB native_srcs12 "src/main/cpp/src/dsm_strategy/utils/time/*.cpp" "src/main/cpp/src/dsm_strategy/utils/time/*.cc")

file(GLOB native_srcs13 "src/main/cpp/bridge/*.cpp" "src/main/cpp/bridge/*.cc")



add_library( # Sets the name of the library.
        native-lib
        # Sets the library as a shared library.
        SHARED
        ${native_srcs}
        ${native_srcs2}
        ${native_srcs3}
        ${native_srcs4}
#        ${native_srcs5}
        ${native_srcs6}
        ${native_srcs7}
        ${native_srcs8}
        ${native_srcs9}
        ${native_srcs10}
        ${native_srcs11}
        ${native_srcs12}
        ${native_srcs13}
        )

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )
find_library( android-lib android )

target_link_libraries( # Specifies the target library.
                       native-lib
                       lib_ncnn
                       lib_opencv
                       libprotol
                       libprotoc
                       libpro
                        ${log-lib}
                       ${android-lib}
                       )
