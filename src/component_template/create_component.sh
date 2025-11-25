#!/bin/bash

# ROS2 Component Template Generator Script
# This script creates a new ROS2 component package from the component_template

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== ROS2 Component Template Generator ===${NC}\n"

# Get template directory (script is inside component_template/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE_DIR="${SCRIPT_DIR}"
TARGET_BASE_DIR="$(dirname "${SCRIPT_DIR}")"

if [ ! -d "$TEMPLATE_DIR" ]; then
    echo -e "${RED}Error: Template directory not found at ${TEMPLATE_DIR}${NC}"
    exit 1
fi

# Interactive input
echo -e "${GREEN}Please enter the following information:${NC}\n"

# Package name
while true; do
    read -p "Package name (snake_case, e.g., my_robot_controller): " PACKAGE_NAME
    if [[ -z "$PACKAGE_NAME" ]]; then
        echo -e "${YELLOW}Package name cannot be empty${NC}"
        continue
    fi
    if [[ ! "$PACKAGE_NAME" =~ ^[a-z][a-z0-9_]*$ ]]; then
        echo -e "${YELLOW}Package name should start with a lowercase letter and contain only lowercase letters, numbers, and underscores${NC}"
        continue
    fi
    if [ -d "${TARGET_BASE_DIR}/${PACKAGE_NAME}" ]; then
        echo -e "${YELLOW}Directory ${TARGET_BASE_DIR}/${PACKAGE_NAME} already exists${NC}"
        read -p "Overwrite? (y/n): " OVERWRITE
        if [[ "$OVERWRITE" == "y" || "$OVERWRITE" == "Y" ]]; then
            break
        fi
        continue
    fi
    break
done

# Class name
while true; do
    read -p "Class name (PascalCase, e.g., MyRobotController): " CLASS_NAME
    if [[ -z "$CLASS_NAME" ]]; then
        echo -e "${YELLOW}Class name cannot be empty${NC}"
        continue
    fi
    if [[ ! "$CLASS_NAME" =~ ^[A-Z][a-zA-Z0-9]*$ ]]; then
        echo -e "${YELLOW}Class name should start with an uppercase letter and use PascalCase${NC}"
        continue
    fi
    break
done

# Description
read -p "Description (e.g., Robot controller for navigation): " DESCRIPTION
if [[ -z "$DESCRIPTION" ]]; then
    DESCRIPTION="ROS2 component package"
fi

# Node name
read -p "Node name (default: ${PACKAGE_NAME}): " NODE_NAME
if [[ -z "$NODE_NAME" ]]; then
    NODE_NAME="$PACKAGE_NAME"
fi

# Executable name
DEFAULT_EXECUTABLE="${PACKAGE_NAME}_node"
read -p "Executable name (default: ${DEFAULT_EXECUTABLE}): " EXECUTABLE_NAME
if [[ -z "$EXECUTABLE_NAME" ]]; then
    EXECUTABLE_NAME="$DEFAULT_EXECUTABLE"
fi

# Namespace (default to package name)
read -p "Namespace (default: ${PACKAGE_NAME}): " NAMESPACE
if [[ -z "$NAMESPACE" ]]; then
    NAMESPACE="$PACKAGE_NAME"
fi

# Confirmation
echo -e "\n${BLUE}=== Configuration Summary ===${NC}"
echo "Package name:    ${PACKAGE_NAME}"
echo "Class name:      ${CLASS_NAME}"
echo "Description:     ${DESCRIPTION}"
echo "Node name:       ${NODE_NAME}"
echo "Executable name: ${EXECUTABLE_NAME}"
echo "Namespace:       ${NAMESPACE}"
echo -e "\n"

read -p "Continue with these settings? (y/n): " CONFIRM
if [[ "$CONFIRM" != "y" && "$CONFIRM" != "Y" ]]; then
    echo -e "${YELLOW}Aborted${NC}"
    exit 0
fi

# Create target directory
TARGET_DIR="${TARGET_BASE_DIR}/${PACKAGE_NAME}"
echo -e "\n${GREEN}Creating package at ${TARGET_DIR}${NC}"

# Copy template
cp -r "$TEMPLATE_DIR" "$TARGET_DIR"

# Generate uppercase versions for include guards
PACKAGE_NAME_UPPER=$(echo "$PACKAGE_NAME" | tr '[:lower:]' '[:upper:]')
CLASS_NAME_UPPER=$(echo "$CLASS_NAME" | tr '[:lower:]' '[:upper:]')

# Function to perform replacements in a file
replace_in_file() {
    local file=$1

    # Package name replacements
    sed -i "s/component_template/${PACKAGE_NAME}/g" "$file"
    sed -i "s/COMPONENT_TEMPLATE/${PACKAGE_NAME_UPPER}/g" "$file"

    # Class name replacements
    sed -i "s/ComponentTemplate/${CLASS_NAME}/g" "$file"

    # Node name replacement (only in constructor)
    sed -i "s/Node(\"component_template\"/Node(\"${NODE_NAME}\"/g" "$file"

    # Namespace replacement
    sed -i "s/namespace component_template/namespace ${NAMESPACE}/g" "$file"
    sed -i "s/component_template::/\"${NAMESPACE}::/g" "$file"

    # Description replacement
    sed -i "s/Sample component template that publishes and subscribes to int messages\./${DESCRIPTION}/g" "$file"
}

# Replace in CMakeLists.txt
echo "Processing CMakeLists.txt..."
replace_in_file "${TARGET_DIR}/CMakeLists.txt"
# Fix executable name
sed -i "s/EXECUTABLE ${PACKAGE_NAME}_node/EXECUTABLE ${EXECUTABLE_NAME}/g" "${TARGET_DIR}/CMakeLists.txt"

# Replace in package.xml
echo "Processing package.xml..."
replace_in_file "${TARGET_DIR}/package.xml"

# Rename and process header file
echo "Processing header file..."
OLD_HPP="${TARGET_DIR}/include/component_template/component_template.hpp"
NEW_INCLUDE_DIR="${TARGET_DIR}/include/${PACKAGE_NAME}"
NEW_HPP="${NEW_INCLUDE_DIR}/${PACKAGE_NAME}.hpp"

mkdir -p "$NEW_INCLUDE_DIR"
if [ -f "$OLD_HPP" ]; then
    mv "$OLD_HPP" "$NEW_HPP"
    rmdir "${TARGET_DIR}/include/component_template" 2>/dev/null || true
fi
replace_in_file "$NEW_HPP"

# Rename and process source file
echo "Processing source file..."
OLD_CPP="${TARGET_DIR}/src/component_template.cpp"
NEW_CPP="${TARGET_DIR}/src/${PACKAGE_NAME}.cpp"

if [ -f "$OLD_CPP" ]; then
    mv "$OLD_CPP" "$NEW_CPP"
fi
replace_in_file "$NEW_CPP"

echo -e "\n${GREEN}=== Package created successfully! ===${NC}\n"
echo -e "Next steps:"
echo -e "  1. cd $(dirname "${TARGET_BASE_DIR}")"
echo -e "  2. colcon build --packages-select ${PACKAGE_NAME}"
echo -e "  3. source install/setup.bash"
echo -e "  4. ros2 run ${PACKAGE_NAME} ${EXECUTABLE_NAME}"
echo -e "\n${BLUE}Happy coding!${NC}"
