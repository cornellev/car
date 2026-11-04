# cmake script to check for required build tools and their versions
# we use cmake because its the only guaranteed cross-platform scripting tool
# the build downloads and checks the arm compiler and picotool itself (cmake/pinned_tools.cmake)

set(problems 0)

macro(report label names verarg)
    unset(exe CACHE)
    find_program(exe NAMES ${names})
    if(NOT exe)
        message("  MISSING   ${label}  not installed")
        math(EXPR problems "${problems} + 1")
    else()
        execute_process(
            COMMAND "${exe}" ${verarg}
            RESULT_VARIABLE result
            OUTPUT_VARIABLE out
            ERROR_VARIABLE out
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_STRIP_TRAILING_WHITESPACE)
        string(REGEX MATCH "[0-9]+\\.[0-9]+(\\.[0-9]+)?" ver "${out}")
        if(NOT result EQUAL 0)
            message("  BROKEN    ${label}  ${exe} fails to run")
            math(EXPR problems "${problems} + 1")
        elseif(NOT ver)
            message("  ok        ${label}  installed (version unknown)")
        else()
            message("  ok        ${label}  ${ver}")
        endif()
    endif()
endmacro()

message("checking build tools:")
report("cmake " cmake --version)
report("ninja " ninja --version)
# the same lookup pico-sdk's build uses
find_package(Python3 COMPONENTS Interpreter QUIET)
report("python" "${Python3_EXECUTABLE}" --version)
message("")

if(problems EQUAL 0)
    message("all build tools present")
else()
    message(FATAL_ERROR "${problems} problem(s) above")
endif()
