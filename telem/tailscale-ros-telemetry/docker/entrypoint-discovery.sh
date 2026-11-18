#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

echo "Starting Fast DDS discovery server on port 11811.."
exec fastdds discovery --server-id 0
