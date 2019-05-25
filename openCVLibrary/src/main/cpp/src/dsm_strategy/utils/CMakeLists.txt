
file(GLOB SRC "file/*.cc" "time/*.cc")
list(APPEND SOURCES ${SRC})

message(${SOURCES})
add_library(utils STATIC
        ${SOURCES})