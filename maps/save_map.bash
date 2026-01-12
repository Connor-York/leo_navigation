#!/bin/bash

# Usage: ./save_map.sh map_name

if [ -z "$1" ]; then
    echo "Usage: ./save_map.sh <map_name>"
    exit 1
fi

MAP_NAME="$1"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FOLDER_NAME="${MAP_NAME^^}"
mkdir -p "$DIR/$FOLDER_NAME"

echo "Saving $MAP_NAME to $DIR/$FOLDER_NAME/"
# Posegraph for localization
rosservice call /slam_toolbox/serialize_map "filename: '$DIR/$FOLDER_NAME/$MAP_NAME'"
# PGM and YAML for keepout editing
rosrun map_server map_saver -f "$DIR/$FOLDER_NAME/$MAP_NAME" map:=/map
echo "Done"
