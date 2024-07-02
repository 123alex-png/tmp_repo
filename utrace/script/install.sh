#! /usr/bin/env bash

# Test if the script is running with `sudo` permission

if [ "$(id -u)" != "0" ]; then
    echo "This script requires root permissions."
    echo "Please run this script with sudo."
    exit 1
fi

# Build the directory structure

echo "Building the directory structure"
mkdir -p /opt/utrace

# Install the required file
## Currently, symlink is used to install the file

echo "Installing the required files"
ln -s $(pwd)/build/utrace /opt/utrace/utrace
ln -s $(pwd)/build/libutrace-dbg.so /opt/utrace/libutrace-dbg.so
ln -s $(pwd)/build/libutrace-nodbg.a /opt/utrace/libutrace-nodbg.a
ln -s $(pwd)/library/without-debugger/utrace-library.h /opt/utrace/utrace-library.h

# Add the environment variable

echo "Adding the environment variable"
echo "LD_PRELOAD=/opt/utrace/libutrace-dbg.so" >> /etc/environment

# Install and configure the service

echo "Installing and configuring the service"
ln -s $(pwd)/service/utrace.service /etc/systemd/system/utrace.service
systemctl enable utrace.service

# Done

echo "Installation completed successfully"
echo "Please reboot the system to apply the changes"
