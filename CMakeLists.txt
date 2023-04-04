cmake_minimum_required(VERSION 3.13)
project(vhfvoiceradio)

set(CMAKE_INSTALL_PREFIX "/usr/")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.h" "src/*.hpp" "src/*.c")

find_package(MBE REQUIRED)
find_package(ITPP REQUIRED)

add_library(vhfvoiceradio SHARED ${SRC})
target_link_libraries(vhfvoiceradio PRIVATE sdrpp_core fftw3 ${ITPP_LIBRARY} ${MBE_LIBRARY})
set_target_properties(vhfvoiceradio PROPERTIES PREFIX "")

target_include_directories(vhfvoiceradio PRIVATE "src/" "/usr/include/sdrpp_core/" "/usr/include/sdrpp_core/imgui/" ${MBE_INCLUDE_DIR} ${ITPP_INCLUDE_DIR})

if (MSVC)
    target_compile_options(vhfvoiceradio PRIVATE /O2 /Ob2 /std:c++17 /EHsc)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(vhfvoiceradio PRIVATE -O3 -std=c++17 -Wno-unused-command-line-argument -undefined dynamic_lookup)
else ()
    target_compile_options(vhfvoiceradio PRIVATE -O3 -std=c++17)
endif ()

# Install directives
install(TARGETS vhfvoiceradio DESTINATION lib/sdrpp/plugins)
