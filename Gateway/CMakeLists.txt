cmake_minimum_required(VERSION 3.16)
project(can_ethernet_gateway VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(gateway_protocol src/protocol.cpp)
target_include_directories(gateway_protocol PUBLIC include)
target_compile_options(gateway_protocol PRIVATE -Wall -Wextra -Wpedantic)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(can_gateway src/main.cpp)
    target_link_libraries(can_gateway PRIVATE gateway_protocol)
    target_compile_options(can_gateway PRIVATE -Wall -Wextra -Wpedantic)
else()
    message(STATUS "can_gateway is Linux-only; building protocol tests only")
endif()

enable_testing()
add_executable(protocol_test tests/protocol_test.cpp)
target_link_libraries(protocol_test PRIVATE gateway_protocol)
target_compile_options(protocol_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME protocol_test COMMAND protocol_test)

