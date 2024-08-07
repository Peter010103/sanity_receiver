#!/bin/bash

PROJECT_DIR="."
SERIAL_PORT="/dev/tty.usbmodem1101"
NUM_ITERATIONS=25

# Path to the config.h file containing MY_ID
CONFIG_FILE="$PROJECT_DIR/main/config.h"

current_id=1

# Iterate and update MY_ID, build, and flash
for ((i=1; i<=NUM_ITERATIONS; i++))
do
  # Prompt for a new MY_ID value or use the incremented value
  read -p "Enter MY_ID (current: $current_id): " input_id
  if [ -n "$input_id" ]; then
    current_id=$input_id
  fi

  # Print the current MY_ID value
  echo "Setting MY_ID to $current_id"

  # Update MY_ID in the config.h file using a temporary file
  temp_file=$(mktemp)
  sed "s/^uint8_t MY_ID = .*/uint8_t MY_ID = $current_id;/" $CONFIG_FILE > "$temp_file"
  mv "$temp_file" $CONFIG_FILE

  # Change to the project directory (absolute path)
  cd $PROJECT_DIR || { echo "Failed to change directory to $PROJECT_DIR"; exit 1; }

  # Print a message before flashing
  echo "Building and flashing ESP32 with MY_ID $current_id..."

  # Build the project
  if ! idf.py build; then
    echo "idf.py build failed. Please check your setup."
    exit 1
  fi

  # Flash the project
  if ! idf.py -p $SERIAL_PORT flash; then
    echo "idf.py flash failed. Please check your setup."
    exit 1
  fi

  # Wait for user to plug in the next ESP32
  echo "Flashing completed for MY_ID $current_id. Please plug in the next ESP32 and press any key to continue..."
  read -n 1 -s

  # Increment MY_ID for the next iteration
  current_id=$((current_id + 1))
done
