#!/bin/bash

# Search for the HD USB Camera
#CAMERA_NAME="HD USB Camera"
CAMERA_NAME="Arducam"
DEVICE=$(v4l2-ctl --list-devices | grep -A 1 "$CAMERA_NAME" | tail -n 1 | awk '{print $1}')

# Check if the device was found
if [ -z "$DEVICE" ]; then
    echo "Error: $CAMERA_NAME not found" >&2
    exit 1
fi

# Extract the video index (e.g., /dev/video2 -> 2)
DEVICE_INDEX=$(echo "$DEVICE" | grep -o '[0-9]*$')

# Return the device index
echo "$DEVICE_INDEX"