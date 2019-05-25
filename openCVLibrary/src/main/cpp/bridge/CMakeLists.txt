include_directories(${CMAKE_CURRENT_LIST_DIR})

set( BRIDGE_SRC
        ${CMAKE_CURRENT_LIST_DIR}/result2u3d.pb.cc
        ${CMAKE_CURRENT_LIST_DIR}/settingsu3d2dms.pb.cc
        ${CMAKE_CURRENT_LIST_DIR}/trans_result.cc
        ${CMAKE_CURRENT_LIST_DIR}/trans_settings.cc)