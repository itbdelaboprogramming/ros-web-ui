#!/bin/bash
set -euo pipefail

# Verify script is executed from the correct directory
SCRIPT_DIR=$(pwd)
if [ "$(basename "${SCRIPT_DIR}")" != "ros-web-ui" ]; then
    echo "ERROR: Script must be executed from 'ros-web-ui' directory" >&2
    exit 1
fi

# Initialize variables
FORCE=false
CP_FLAGS="-v"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --force)
        FORCE=true
        shift
        ;;
        *)
        echo "ERROR: Unknown option: $1" >&2
        echo "Usage: $0 [--force]" >&2
        exit 1
        ;;
    esac
done

# Set copy flags based on force mode
if [ "$FORCE" = false ]; then
    CP_FLAGS="-n -v"
    echo "Running in safe mode (will not overwrite existing files)"
else
    echo "Running in force mode (will overwrite existing files)"
fi

# Define source paths
MQTT_SRC="./Certificates/mqtt"
SQL_SRC="./Certificates/sql"

# Define destination paths
AWS_MQTT_CERTS="./source/dependencies/aws_mqtt/certs"
MYSQL_DEST1="./source/dependencies/mysql-folder-monitor/certs"
MYSQL_DEST2="./source/dependencies/ROS-dashboard-backend/scripts/certs"

# Create destination directories if they don't exist
mkdir -p "${AWS_MQTT_CERTS}"
mkdir -p "${MYSQL_DEST1}"
mkdir -p "${MYSQL_DEST2}"

# Copy MQTT certificates with error handling
echo "Copying MQTT certificates..."
if ! cp $CP_FLAGS -- "${MQTT_SRC}/"* "${AWS_MQTT_CERTS}/"; then
    echo "ERROR: Failed to copy MQTT certificates" >&2
    exit 1
fi

# Copy SQL certificates to both destinations
echo "Copying SQL certificates..."
if ! cp $CP_FLAGS -- "${SQL_SRC}/"* "${MYSQL_DEST1}/"; then
    echo "ERROR: Failed to copy SQL certificates to first destination" >&2
    exit 1
fi

if ! cp $CP_FLAGS -- "${SQL_SRC}/"* "${MYSQL_DEST2}/"; then
    echo "ERROR: Failed to copy SQL certificates to second destination" >&2
    exit 1
fi

echo "Certificate deployment completed successfully"
echo "Note: Existing files were ${FORCE:-not} overwritten"