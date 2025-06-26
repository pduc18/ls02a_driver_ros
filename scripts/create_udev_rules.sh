# Script to install udev rules for the LiDAR device

UDEV_RULES_DIR="/etc/udev/rules.d"
RULE_FILE="99-laser.rules"
PACKAGE_NAME="line_laser_ros"
HOME_DIR="/home/$SUDO_USER"

# Check if script is run with sudo
if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root (use sudo)" >&2
    exit 1
fi

# Use absolute path to installed udev rules
UDEV_RULE_SRC="$HOME_DIR/ros2_ws/install/line_laser_ros/share/line_laser_ros/udev/$RULE_FILE"
UDEV_RULE_DEST="$UDEV_RULES_DIR/$RULE_FILE"

# Check if the udev rule file exists
if [ ! -f "$UDEV_RULE_SRC" ]; then
    echo "Error: Udev rule file $UDEV_RULE_SRC not found. Ensure package is built." >&2
    exit 1
fi

# Copy the udev rule to the system directory
echo "Installing $RULE_FILE to $UDEV_RULES_DIR..."
cp "$UDEV_RULE_SRC" "$UDEV_RULE_DEST"

# Set appropriate permissions
chmod 644 "$UDEV_RULE_DEST"
chown root:root "$UDEV_RULE_DEST"

# Reload udev rules and trigger
echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "Udev rule installed successfully. The LiDAR should be accessible at /dev/ttyUSB0 and /dev/ttyUSB1."
exit 0
