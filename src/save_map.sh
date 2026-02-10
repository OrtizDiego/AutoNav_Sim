#!/bin/bash

# Script to save SLAM Toolbox map to the correct location
# Usage: ./save_map.sh <map_name>

MAP_NAME=${1:-my_map}
PACKAGE_PATH=$(ros2 pkg prefix my_bot)/share/my_bot

echo "Saving map as: $MAP_NAME"

# Call the SLAM Toolbox service to save the map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$MAP_NAME'}}"

# Wait a moment for the files to be saved
sleep 2

# Copy the map files to the package maps directory
if [ -f "$HOME/dev_ws/$MAP_NAME.yaml" ] && [ -f "$HOME/dev_ws/$MAP_NAME.pgm" ]; then
    echo "Copying map files to package directory..."
    cp "$HOME/dev_ws/$MAP_NAME.yaml" "$PACKAGE_PATH/maps/"
    cp "$HOME/dev_ws/$MAP_NAME.pgm" "$PACKAGE_PATH/maps/"
    echo "Map saved successfully to: $PACKAGE_PATH/maps/"
else
    echo "Error: Map files not found in $HOME/dev_ws/"
    echo "Make sure SLAM Toolbox saved the map correctly"
fi