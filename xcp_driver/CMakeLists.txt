cmake_minimum_required(VERSION 3.0.2)
project(xcp_driver)

find_package(Boost COMPONENTS chrono thread system program_options REQUIRED)

find_package (glog REQUIRED PATHS "/usr/local/lib/cmake/glog")
# pkg_check_modules(glog REQUIRED PATHS "/usr/lib/x86_64-linux-gnu/pkgconfig")
include_directories(
    ${Boost_INCLUDE_DIRS}
    src/
    )

file(GLOB SRCS
    "src/*.cc"
    "src/cc/*.cc"
    "src/dbc/*.cc"
    )

file(GLOB INCS
    "src/*.h"
    "src/inc/*.h"
    "src/dbc/*.h"
    )

add_executable(${PROJECT_NAME}_node ${SRCS} ${INCS})

#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} )

target_link_libraries(${PROJECT_NAME}_node
    ${Boost_LIBRARIES}
    pthread
    glog::glog
    )

