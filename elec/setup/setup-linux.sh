#!/usr/bin/env bash
# set up the linux build tools for rp2040 firmware on apt (debian, ubuntu) or pacman (arch)
# the host g++ is for building and testing cev-lib/chuds
# the first build downloads the pinned arm compiler and picotool into elec/.tools
# after this, build from elec/ with: just build
set -euo pipefail

# a root shell, like a container, has no sudo
if [ "$(id -u)" -eq 0 ]; then
    sudo() { "$@"; }
fi

if command -v apt-get >/dev/null; then
    sudo apt-get update
    sudo apt-get install -y cmake ninja-build python3 curl ca-certificates libusb-1.0-0 g++
    # debian 12 has no just package, and ubuntu 24.04's is 1.21, older than the 1.33 the justfiles need
    curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | sudo bash -s -- --tag 1.58.0 --to /usr/local/bin --force
elif command -v pacman >/dev/null; then
    sudo pacman -Syu --needed --noconfirm cmake ninja python just libusb gcc
else
    echo "no apt-get or pacman found. install the tools listed in elec/README.md by hand"
    exit 1
fi

# let picotool reach a board in BOOTSEL mode without root
# containers and wsl have no udev
if [ -d /etc/udev/rules.d ]; then
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-picotool.rules >/dev/null
fi

# fetch the pico-sdk and the one nested submodule the build needs
repo="$(cd "$(dirname "$0")/../.." && pwd)"
git -C "$repo" submodule update --init
git -C "$repo/vendor/pico-sdk" submodule update --init lib/tinyusb

echo "setup complete. build from elec/ with: just build"
