# download the pinned arm-gcc and picotool into elec/.tools on the first configure
# the build uses these over anything on PATH, so a wrong arm-none-eabi-gcc can't be picked up
# a non-empty ELEC_SYSTEM_TOOLS in the environment (the nix shell sets it) uses the tools on PATH instead
# the first configure reads it, and the build folder keeps that choice for every later shell or editor

# a bump also changes the sha256 values below
set(arm_release 15.3.rel1)
# must match picotool_VERSION_REQUIRED in vendor/pico-sdk/tools/CMakeLists.txt
set(picotool_release 2.3.0)
# the pico-sdk-tools release that ships this picotool
set(picotool_tag v2.3.0-1)
# the version arm-gcc reports, such as 15.3 for 15.3.rel1
string(REGEX MATCH "^[0-9]+\\.[0-9]+" arm_version ${arm_release})

get_filename_component(tools_dir ${CMAKE_CURRENT_LIST_DIR}/../.tools ABSOLUTE)

if("$ENV{ELEC_SYSTEM_TOOLS}" STREQUAL "")
    set(system_tools OFF)
else()
    set(system_tools ON)
endif()
set(ELEC_SYSTEM_TOOLS ${system_tools} CACHE BOOL "use the arm-gcc and picotool on PATH instead of the pinned downloads")

if(ELEC_SYSTEM_TOOLS)
    message(STATUS "using the arm-gcc and picotool on PATH (ELEC_SYSTEM_TOOLS)")
    return()
endif()

set(arm_url https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/packages/generic/gnu-toolchain/${arm_release}/arm-gnu-toolchain-${arm_release})
set(picotool_url https://github.com/raspberrypi/pico-sdk-tools/releases/download/${picotool_tag}/picotool-${picotool_release})

cmake_host_system_information(RESULT cpu QUERY OS_PLATFORM)
if(CMAKE_HOST_APPLE AND cpu STREQUAL "arm64")
    set(arm_file darwin-arm64-arm-none-eabi.tar.xz)
    set(arm_sha 376808a59ca209c1413236f1c6a509e33da4b29857ab28642b9927cf3048af55)
    set(picotool_file mac.zip)
    set(picotool_sha e439a9fdc9dc44cf971d4a40fd11a19bfcd9d920be27bce0c9691f5852db521a)
elseif(CMAKE_HOST_WIN32 AND cpu MATCHES "^(AMD64|ARM64)$")
    # arm windows runs the x64 build under emulation
    set(arm_file mingw-w64-x86_64-arm-none-eabi.zip)
    set(arm_sha b85669d3408e2ae713b17b0cc59bc4ea26369a7f2bd19108fd11df7095f159e6)
    set(picotool_file x64-win.zip)
    set(picotool_sha 52d63c5fb4cc19f95b62bb6c945d2bcb772ec426c7676abf585437849c81c870)
elseif(CMAKE_HOST_UNIX AND NOT CMAKE_HOST_APPLE AND cpu MATCHES "^(x86_64|aarch64)$")
    set(arm_file ${cpu}-arm-none-eabi.tar.xz)
    set(picotool_file ${cpu}-lin.tar.gz)
    if(cpu STREQUAL "x86_64")
        set(arm_sha 563bebb2b97d53382b956d6ee1fe61e2cae26699901417234a37df505ef9b5fa)
        set(picotool_sha 5e92e819a473e3cefe0c06041efb611f43afde70a02f8fd76c7990e205caa493)
    else()
        set(arm_sha 06979e0c8171de58e5dc2a2b2019330a290f30930f27728af98a83e1a7369b3a)
        set(picotool_sha aca1ff3ffa6b50d814b7dee5cef86d7c90140cd21e33c961e7d769771586715b)
    endif()
elseif(CMAKE_HOST_APPLE)
    message(FATAL_ERROR "this terminal runs as ${cpu}, so it is an Intel Mac or a Rosetta terminal. "
        "Intel Macs aren't supported. On Apple Silicon, open a terminal that isn't set to use Rosetta")
else()
    message(FATAL_ERROR "no pinned toolchain for this computer (${cpu})")
endif()

# install_tool(<url> <sha256> <dest> <file>): download, check, and unpack an archive into dest
# dest/.pinned records the url and hash, and file must exist, so a damaged or older install gets replaced
function(install_tool url sha dest file)
    if(EXISTS ${dest}/.pinned AND EXISTS ${dest}/${file})
        file(READ ${dest}/.pinned installed)
        if(installed STREQUAL "${url} ${sha}")
            return()
        endif()
    endif()
    message(STATUS "downloading ${url}")
    set(archive ${dest}-archive)
    file(DOWNLOAD ${url} ${archive} EXPECTED_HASH SHA256=${sha} SHOW_PROGRESS INACTIVITY_TIMEOUT 60 STATUS status)
    list(GET status 0 code)
    if(NOT code EQUAL 0)
        file(REMOVE ${archive})
        message(FATAL_ERROR "download failed: ${url}\n${status}")
    endif()
    set(tmp ${dest}-extract)
    file(REMOVE_RECURSE ${tmp})
    file(ARCHIVE_EXTRACT INPUT ${archive} DESTINATION ${tmp})
    file(REMOVE ${archive})
    # unwrap a single top folder, ignoring the stray .keep file the picotool archives hold
    # the windows arm zip has no top folder
    file(GLOB entries ${tmp}/*)
    set(top "")
    foreach(entry ${entries})
        if(IS_DIRECTORY ${entry})
            list(APPEND top ${entry})
        endif()
    endforeach()
    list(LENGTH top count)
    if(NOT count EQUAL 1)
        set(top ${tmp})
    endif()
    # the old copy goes only once the new one is fully unpacked
    file(REMOVE_RECURSE ${dest})
    file(RENAME ${top} ${dest})
    file(REMOVE_RECURSE ${tmp})
    file(WRITE ${dest}/.pinned "${url} ${sha}")
endfunction()

if(CMAKE_HOST_WIN32)
    set(exe .exe)
endif()
install_tool(${arm_url}-${arm_file} ${arm_sha} ${tools_dir}/arm-gnu-toolchain bin/arm-none-eabi-gcc${exe})
install_tool(${picotool_url}-${picotool_file} ${picotool_sha} ${tools_dir}/picotool picotoolConfig.cmake)

set(PICO_TOOLCHAIN_PATH ${tools_dir}/arm-gnu-toolchain)
set(picotool_DIR ${tools_dir}/picotool)
