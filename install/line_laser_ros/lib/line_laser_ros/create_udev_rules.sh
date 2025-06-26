#!/bin/bash

# Script to install udev rules for the LiDAR device

UDEV_RULES_DIR="/etc/udev/rules.d"
RULE_FILE="99-laser.rules"
PACKAGE_NAME="line_laser_ros"

# Check if script is run with sudo
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run as root (use sudo)" >&2
    exit 1
fi

# Find the package share directory
PACKAGE_SHARE_DIR=$(ros2 pkg prefix --share $PACKAGE_NAME 2>/dev/null)
if [ -z "$PACKAGE_SHARE_DIR" ]; then
    echo "Error: Could not find package $PACKAGE_NAME. Ensure it is built and sourced." >&2
    exit 1
fi

# Check if the udev rule file exists in the package
UDEV_RULE_PATH="$PACKAGE_SHARE_DIR/udev/$RULE_FILE"
if [ ! -f "$UDEV_RULE_PATH" ]; then
    echo "Error: Udev rule file $UDEV_RULE_PATH not found in package $PACKAGE_NAME." >&2
    exit 1
fi

# Copy the udev rule to the system directory
echo "Installing $RULE_FILE to $UDEV_RULES_DIR..."
cp "$UDEV_RULE_PATH" "$UDEV_RULES_DIR/$RULE_FILE"

# Set appropriate permissions
chmod 644 "$UDEV_RULES_DIR/$RULE_FILE"
chown root:root "$UDEV_RULES_DIR/$RULE_FILE"

# Reload udev rules and trigger
echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "Udev rule installed successfully. The LiDAR should be accessible at /dev/laser."
exit 0
