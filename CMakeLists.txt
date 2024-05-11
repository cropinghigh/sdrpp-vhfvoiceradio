cmake_minimum_required(VERSION 3.13)
project(vhfvoiceradio)

set(CMAKE_INSTALL_PREFIX "/usr/")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.h" "src/*.hpp" "src/*.c" "mbelib/*.c")

find_package(ITPP REQUIRED)

if (NOT SDRPP_MODULE_CMAKE)
    set(SDRPP_MODULE_CMAKE "/usr/share/cmake/Modules/sdrpp_module.cmake")
endif ()

include(${SDRPP_MODULE_CMAKE})

target_link_libraries(vhfvoiceradio PRIVATE fftw3 ${ITPP_LIBRARY})
target_include_directories(vhfvoiceradio PRIVATE "src/" "mbelib/")
