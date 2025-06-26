#!/bin/bash

# Script to remove udev rules for the LiDAR device

UDEV_RULES_DIR="/etc/udev/rules.d"
RULE_FILE="99-laser.rules"

# Check if script is run with sudo
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run as root (use sudo)" >&2
    exit 1
fi

# Check if the udev rule file exists
if [ -f "$UDEV_RULES_DIR/$RULE_FILE" ]; then
    echo "Removing $RULE_FILE from $UDEV_RULES_DIR..."
    rm "$UDEV_RULES_DIR/$RULE_FILE"

    # Reload udev rules and trigger
    echo "Reloading udev rules..."
    udevadm control --reload-rules
    udevadm trigger

    echo "Udev rule removed successfully."
    exit 0
else
    echo "Error: Udev rule file $RULE_FILE not found in $UDEV_RULES_DIR."
    exit 1
fi
