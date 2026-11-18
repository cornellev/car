include(FetchContent)
FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG 2d0a9359a60c51affe2a9bebb1be1dca47868151 # v2.5.3
)
FetchContent_MakeAvailable(doctest)

add_executable(chuds-tests
    test_ids.cpp
    test_message.cpp
    test_transport.cpp
    test_bus.cpp
    test_catalog_format.cpp
)
target_link_libraries(chuds-tests PRIVATE chuds chuds_warnings doctest::doctest_with_main)

# address and undefined-behavior sanitizers catch bad reads of bus bytes
# linux only: sanitized binaries hang at startup under the nix clang on macOS
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(CHUDS_SANITIZE -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
    target_compile_options(chuds-tests PRIVATE ${CHUDS_SANITIZE})
    target_link_options(chuds-tests PRIVATE ${CHUDS_SANITIZE})
endif()

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(chuds-tests)

# a rejected wire body is a compile error, so each case builds compile_fail.cpp
# a case passes only if the build fails with the opt-in error, not some unrelated one
foreach(case make_message body_as)
    string(TOUPPER "BAD_${case}" define)
    add_library(chuds-compile-${case} OBJECT EXCLUDE_FROM_ALL compile_fail.cpp)
    target_link_libraries(chuds-compile-${case} PRIVATE chuds)
    target_compile_definitions(chuds-compile-${case} PRIVATE ${define})
    # clang-tidy can't parse a build that is meant to fail
    set_target_properties(chuds-compile-${case} PROPERTIES EXPORT_COMPILE_COMMANDS OFF)
    set(test "wire body compile check: ${case}")
    add_test(NAME ${test}
             COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target chuds-compile-${case})
    # concurrent builds of one build tree corrupt ninja's state
    set_tests_properties(${test} PROPERTIES
        PASS_REGULAR_EXPRESSION "wire body must opt in"
        RESOURCE_LOCK build-tree)
endforeach()

# the socketcan adapter is Linux-only
# test needs a CAN interface (skips without one)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(chuds-socketcan-tests test_socketcan.cpp)
    target_link_libraries(chuds-socketcan-tests PRIVATE chuds_socketcan chuds_warnings doctest::doctest_with_main)
    target_compile_options(chuds-socketcan-tests PRIVATE ${CHUDS_SANITIZE})
    target_link_options(chuds-socketcan-tests PRIVATE ${CHUDS_SANITIZE})
    # the tests share one interface, so they must not run in parallel
    doctest_discover_tests(chuds-socketcan-tests PROPERTIES RESOURCE_LOCK vcan)
endif()
