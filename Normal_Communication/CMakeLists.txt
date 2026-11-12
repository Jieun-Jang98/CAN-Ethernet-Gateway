cmake_minimum_required(VERSION 3.20)
project(kuse_can_ethernet_gateway VERSION 0.1.0 LANGUAGES CXX)

include(CTest)

add_library(kuse_common STATIC
    common/src/can_codec.cpp
    common/src/checks.cpp
    common/src/protocol.cpp
    common/src/torque.cpp
)

target_include_directories(kuse_common PUBLIC common/include)
set_target_properties(kuse_common PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

if(MSVC)
    target_compile_options(kuse_common PRIVATE /W4 /WX)
else()
    target_compile_options(kuse_common PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if(BUILD_TESTING)
    add_executable(kuse_common_tests tests/test_common.cpp)
    target_link_libraries(kuse_common_tests PRIVATE kuse_common)
    set_target_properties(kuse_common_tests PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
    add_test(NAME kuse_common_tests COMMAND kuse_common_tests)

    add_executable(kuse_cpp_header_test tests/test_cpp_compat.cpp)
    target_link_libraries(kuse_cpp_header_test PRIVATE kuse_common)
    set_target_properties(kuse_cpp_header_test PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
    add_test(NAME kuse_cpp_header_test COMMAND kuse_cpp_header_test)
endif()

add_library(kuse_adas STATIC
    adas/src/adas_controller.cpp
    adas/src/udp_transport.cpp
)

target_include_directories(kuse_adas PUBLIC
    adas/include
)

target_link_libraries(kuse_adas PUBLIC
    kuse_common
)

set_target_properties(kuse_adas PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

if(MSVC)
    target_compile_options(kuse_adas PRIVATE /W4 /WX)
else()
    target_compile_options(kuse_adas PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()

if(BUILD_TESTING)
    add_executable(kuse_adas_tests
        tests/test_adas.cpp
    )

    target_link_libraries(kuse_adas_tests PRIVATE
        kuse_adas
    )

    set_target_properties(kuse_adas_tests PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )

    add_test(
        NAME kuse_adas_tests
        COMMAND kuse_adas_tests
    )
endif()
