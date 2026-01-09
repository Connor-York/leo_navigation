#!/bin/bash

# Usage: ./save_map.sh map_name

if [ -z "$1" ]; then
    echo "Usage: ./save_map.sh <map_name>"
    exit 1
fi

MAP_NAME="$1"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Saving $MAP_NAME to $DIR"

# Posegraph for localization
rosservice call /slam_toolbox/serialize_map "filename: '$DIR/$MAP_NAME'"

# PGM and YAML for keepout editing
rosrun map_server map_saver -f "$DIR/$MAP_NAME" map:=/map

echo "Done"
