#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ros-telemetry/install/setup.bash

echo "ROS_DOMAIN_ID is set to: ${ROS_DOMAIN_ID:-<unset>}"

# ensure ros logs show up in container logs reliably
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1

# Generate SUPER_CLIENT profile from template
# SUPER_CLIENT is required for ros2 bag record to discover topic types
TEMPLATE="/workspace/config/super_client.example.xml"
PROFILE="/tmp/super_client.xml"
sed 's/DISCOVERY_SERVER_IP/127.0.0.1/' "$TEMPLATE" > "$PROFILE"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"
echo "Using SUPER_CLIENT profile: $PROFILE"

TOPIC_NAME="/spi_data"
TIMESTAMP=$(date +'%Y%m%d-%H%M%S')
BAG_DIR="/workspace/bags/bag_${TIMESTAMP}"

echo "Recording topic '${TOPIC_NAME}' to '${BAG_DIR}'..."
exec ros2 bag record -s mcap "${TOPIC_NAME}" -o "${BAG_DIR}"
