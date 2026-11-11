cmake_minimum_required(VERSION 3.20)
project(kuse_can_ethernet_gateway VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)

add_library(kuse_common STATIC
    common/src/can_codec.c
    common/src/checks.c
    common/src/protocol.c
    common/src/torque.c
)

target_include_directories(kuse_common PUBLIC common/include)
set_target_properties(kuse_common PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

if(MSVC)
    target_compile_options(kuse_common PRIVATE /W4 /WX)
else()
    target_compile_options(kuse_common PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if(BUILD_TESTING)
    add_executable(kuse_common_tests tests/test_common.c)
    target_link_libraries(kuse_common_tests PRIVATE kuse_common)
    set_target_properties(kuse_common_tests PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)
    add_test(NAME kuse_common_tests COMMAND kuse_common_tests)

    add_executable(kuse_cpp_header_test tests/test_cpp_compat.cpp)
    target_link_libraries(kuse_cpp_header_test PRIVATE kuse_common)
    set_target_properties(kuse_cpp_header_test PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
    add_test(NAME kuse_cpp_header_test COMMAND kuse_cpp_header_test)
endif()

