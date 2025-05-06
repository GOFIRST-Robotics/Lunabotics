#!/bin/bash

# Create a user with the same UID and GID as the host user
# This helps with permissions when mounting volumes
if [ ! -z "$USER_UID" ] && [ ! -z "$USER_GID" ]; then
    groupadd -g $USER_GID streamdeckuser
    useradd -m -u $USER_UID -g $USER_GID -s /bin/bash streamdeckuser
    
    # Give the user access to USB devices
    usermod -a -G plugdev streamdeckuser || true
    
    # Switch to the user
    su streamdeckuser -c "streamdeck -m"
else
    # If no UID/GID provided, just run as root
    streamdeck -m
fi
