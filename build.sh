#!/bin/sh

# Check if git is installed
echo "Scanning for git..."
if [ "$GIT_VERSION" != "command not found" ]; then
    echo "Found Git installation!"
else
    echo "Error: Git is not installed on this system!"
    exit 126
fi

scriptDir=$(dirname "$(realpath $0)")

# Update submodules
echo "Fetching submodules..."
git submodule update --init --recursive

# Copy, paste, and execute patch file
PATCH_PATH=$scriptDir/src/ti_comm/src/libserialport/libserialport.patch
echo "Applying libserialport patch..."

if test -f "$PATCH_PATH"; then
    echo "Error: Patch has already been applied! Skipping..."
else
    cp $scriptDir/src/ti_comm/src/libserialport.patch $scriptDir/src/ti_comm/src/libserialport/
    cd $scriptDir/src/ti_comm/src/libserialport/
    git apply --ignore-space-change libserialport.patch
fi

# Run build scripts
echo "Running ./autogen..."
cd $scriptDir/src/ti_comm/src/libserialport/
./autogen.sh

echo "Running ./configure..."
cd $scriptDir/src/ti_comm/src/libserialport/
./configure

echo "Running make..."
cd $scriptDir
make

echo "Running catkin_make..."
cd $scriptDir
catkin_make
source devel/setup.bash

echo "Build complete."
cd $scriptDir
