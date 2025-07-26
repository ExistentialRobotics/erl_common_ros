#!/usr/bin/bash

set -e

# check usage convert_rosbag_1to2.bash <input_bagfile> <output_bag_dir> <topics>
if [ "$#" -gt 1 ]; then
    # Set default values
    INPUT_BAGFILE=${1:-"rosbag.bag"}
    if [ "$#" -gt 2 ]; then
        OUTPUT_BAG_DIR=${2:-"ros2_bag"}
        if [ "$#" -gt 3 ]; then
            TOPICS="${@:3}"
        fi
    fi
fi

INPUT_BAGFILE=${INPUT_BAGFILE:-"rosbag.bag"}
read -p "Enter the input bag file name (default: ${INPUT_BAGFILE}): " user_input
if [ -n "${user_input}" ]; then
    INPUT_BAGFILE=${user_input}
fi
if [ ! -f "${INPUT_BAGFILE}" ]; then
    echo "Input bag file does not exist: ${INPUT_BAGFILE}"
    exit 1
fi

OUTPUT_BAG_DIR=${OUTPUT_BAG_DIR:-"ros2_bag"}
read -p "Enter the output directory (default: ${OUTPUT_BAG_DIR}): " user_input
if [ -n "${user_input}" ]; then
    OUTPUT_BAG_DIR=${user_input}
fi
if [ -z "${OUTPUT_BAG_DIR}" ]; then
    echo "Output bag directory cannot be empty."
    exit 1
fi
if [ -d "${OUTPUT_BAG_DIR}" ]; then
    read -p "Output bag directory exists, remove it? (y/n): " remove_dir
    if [ "${remove_dir}" == "y" ]; then
        sudo rm -rf "${OUTPUT_BAG_DIR}"
        echo "Removed existing output bag directory: ${OUTPUT_BAG_DIR}"
    else
        echo "Abort"
        exit 1
    fi
fi
OUTPUT_BAG_DIR_DIR=$(dirname "${OUTPUT_BAG_DIR}")
OUTPUT_BAG_DIR_NAME=$(basename "${OUTPUT_BAG_DIR}")

read -p "Enter ROS topics to convert, separated by spaces (default: ${TOPICS}): " user_input
if [ -n "${user_input}" ]; then
    TOPICS=${user_input}
fi

read -p "Do you want to use compression? (Y/n, default: y): " user_input
if [[ -z "${user_input}" || "${user_input,,}" == "y" ]]; then
    COMPRESSION_MODE="message"
    echo "Using compression mode: ${COMPRESSION_MODE}"
else
    COMPRESSION_MODE="none"
    echo "No compression will be used."
fi

SCRIPT_DIR=$(cd "$(dirname ${BASH_SOURCE[0]})" && pwd)
cd ${SCRIPT_DIR}

echo "INPUT_BAG_FILE_NAME=${INPUT_BAGFILE}" > .env
echo "OUTPUT_BAG_DIR_DIR=${OUTPUT_BAG_DIR_DIR}" >> .env
echo "OUTPUT_BAG_DIR_NAME=${OUTPUT_BAG_DIR_NAME}" >> .env
echo "TOPICS=${TOPICS}" >> .env
echo "COMPRESSION_MODE=${COMPRESSION_MODE}" >> .env

echo "Starting conversion from ROS 1 to ROS 2 bag format..."
echo ">> Press Ctrl+C to stop the conversion at any time. <<"
docker compose -f rosbag_converter.yml up
docker compose -f rosbag_converter.yml down

sudo chown -R $(whoami):$(whoami) "${OUTPUT_BAG_DIR}"
echo "Conversion completed. Output bag files are in: ${OUTPUT_BAG_DIR}"
