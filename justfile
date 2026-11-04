# use powershell on windows
set windows-shell := ["powershell.exe", "-NoProfile", "-Command"]

mod elec

mod telem

mod cev-lib

[private]
default:
    @just --list

# linux-only chuds targets (socketcan tests, chuds_demo) are skipped off linux
# ci also checks formatting with treefmt (nix fmt) and runs lint as separate jobs
# unix only, because it runs test
# build and test like ci (telem/shm needs a pi, excluded)
[unix]
check:
    just elec build
    just test

# unix only, because CMakePresets.json disables the host preset on windows
# test every module
[unix]
test:
    just cev-lib chuds test

# clang-tidy can't find the arm headers itself, so pass the ones the firmware compiler uses
# nix's clang-tidy wrapper adds host headers, so the firmware lint uses the unwrapped binary when there is one
# lint every file the builds compile, except vendored and generated code
[unix]
lint:
    #!/usr/bin/env bash
    set -euo pipefail
    tidy=$(command -v clang-tidy-unwrapped || command -v clang-tidy) || { echo "clang-tidy not found" >&2; exit 1; }
    cmake --preset firmware >/dev/null
    cmake --preset host >/dev/null
    cxx=$(sed -n 's/^CMAKE_CXX_COMPILER:[A-Z]*=//p' build/firmware/CMakeCache.txt)
    includes=$("$cxx" -xc++ -E -v /dev/null 2>&1 | sed -n '/search starts here/,/End of search/s|^ |-extra-arg=-isystem|p')
    # run-clang-tidy prints each file's full command and a "N warnings generated" tally; drop both
    quiet() { "$@" 2>&1 | { grep -vE '^\[ *[0-9]+/[0-9]+\]\[|[0-9]+ warnings?( and [0-9]+ errors?)? generated\.$' || true; } | cat -s; }
    ours="^$PWD/(?!vendor/|build/)"
    quiet run-clang-tidy -p build/firmware -quiet -clang-tidy-binary "$tidy" $includes "$ours"
    quiet run-clang-tidy -p build/host -quiet "$ours"
