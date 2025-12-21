#!/bin/bash
# Script to run the standalone client with VNC so you can see the UI
# This script sets up a virtual X display (Xvfb) and VNC server so you can
# view the Qt UI and GStreamer video streams remotely.

set -e  # Exit on error

DISPLAY_NUM=99
VNC_PORT=5901
SCREEN_SIZE="1024x768x24"

# Check if required packages are installed
echo "Checking for required packages..."
if ! command -v Xvfb &> /dev/null; then
    echo "Installing Xvfb..."
    sudo apt-get update && sudo apt-get install -y xvfb
fi

if ! command -v x11vnc &> /dev/null; then
    echo "Installing x11vnc..."
    sudo apt-get update && sudo apt-get install -y x11vnc
fi

# Clean up any existing Xvfb processes and lock files
echo "Cleaning up any existing virtual displays..."
pkill -f "Xvfb :${DISPLAY_NUM}" 2>/dev/null || true
pkill -f "x11vnc.*:${DISPLAY_NUM}" 2>/dev/null || true
rm -f /tmp/.X${DISPLAY_NUM}-lock /tmp/.X11-unix/X${DISPLAY_NUM} 2>/dev/null || true
sleep 1

# Start Xvfb in the background
echo "Starting virtual display (Xvfb) on :${DISPLAY_NUM}..."
Xvfb :${DISPLAY_NUM} -screen 0 ${SCREEN_SIZE} -ac +extension GLX +render -noreset &
XVFB_PID=$!

# Wait for Xvfb to start and verify it's running
sleep 2
if ! kill -0 $XVFB_PID 2>/dev/null; then
    echo "ERROR: Xvfb failed to start!"
    exit 1
fi

# Verify the display is accessible
if ! DISPLAY=:${DISPLAY_NUM} xdpyinfo &>/dev/null; then
    echo "ERROR: Cannot access display :${DISPLAY_NUM}"
    kill $XVFB_PID 2>/dev/null || true
    exit 1
fi

echo "Xvfb is running on display :${DISPLAY_NUM}"

# Check if port is already in use (might be another VNC server)
if ss -tlnp 2>/dev/null | grep -q ":${VNC_PORT}[^0-9]" || netstat -tlnp 2>/dev/null | grep -q ":${VNC_PORT}[^0-9]"; then
    echo "WARNING: Port ${VNC_PORT} is already in use!"
    echo "This might be why you're seeing your local screen."
    echo "Checking what's using the port..."
    ss -tlnp 2>/dev/null | grep ":${VNC_PORT}" || netstat -tlnp 2>/dev/null | grep ":${VNC_PORT}" || true
    echo ""
    echo "Trying to use a different port..."
    VNC_PORT=$((VNC_PORT + 1))
    echo "Using port ${VNC_PORT} instead"
fi

# Start VNC server pointing to the virtual display
# Xvfb doesn't need authentication, so we don't use -auth
echo "Starting VNC server on port ${VNC_PORT}..."
x11vnc -display :${DISPLAY_NUM} -nopw -listen 0.0.0.0 -xkb -forever -shared -bg -rfbport ${VNC_PORT} -noxdamage
sleep 3

# Verify VNC server is running
if ! pgrep -f "x11vnc.*:${DISPLAY_NUM}" > /dev/null; then
    echo "ERROR: VNC server failed to start!"
    kill $XVFB_PID 2>/dev/null || true
    exit 1
fi

# Verify VNC is actually listening and show process info
echo "VNC server process details:"
ps aux | grep "x11vnc.*:${DISPLAY_NUM}" | grep -v grep
echo ""
echo "Verifying VNC is listening on port ${VNC_PORT}..."
if ss -tlnp 2>/dev/null | grep -q ":${VNC_PORT}[^0-9]"; then
    echo "✓ VNC is confirmed listening on port ${VNC_PORT}"
    ss -tlnp 2>/dev/null | grep ":${VNC_PORT}"
elif netstat -tlnp 2>/dev/null | grep -q ":${VNC_PORT}[^0-9]"; then
    echo "✓ VNC is confirmed listening on port ${VNC_PORT}"
    netstat -tlnp 2>/dev/null | grep ":${VNC_PORT}"
else
    echo "⚠ WARNING: Could not verify VNC is listening, but process is running"
fi
echo ""

# Get the actual port (try multiple methods for portability)
VNC_PORT_ACTUAL="${VNC_PORT}"
if command -v ss &> /dev/null; then
    # Try using ss (more modern)
    DETECTED_PORT=$(ss -tlnp 2>/dev/null | grep ":${VNC_PORT}" | head -1 | awk '{print $4}' | cut -d: -f2)
    if [ -n "$DETECTED_PORT" ]; then
        VNC_PORT_ACTUAL="$DETECTED_PORT"
    fi
elif command -v netstat &> /dev/null; then
    # Try using netstat with awk (more portable than grep -oP)
    DETECTED_PORT=$(netstat -tlnp 2>/dev/null | grep "x11vnc\|:${VNC_PORT}" | grep "LISTEN" | head -1 | awk '{print $4}' | awk -F: '{print $NF}')
    if [ -n "$DETECTED_PORT" ]; then
        VNC_PORT_ACTUAL="$DETECTED_PORT"
    fi
fi

# Verify port is actually listening (more flexible check)
if command -v ss &> /dev/null; then
    if ss -tlnp 2>/dev/null | grep -q ":${VNC_PORT_ACTUAL}[^0-9]"; then
        echo "✓ VNC port ${VNC_PORT_ACTUAL} is confirmed listening"
    fi
elif command -v netstat &> /dev/null; then
    if netstat -tlnp 2>/dev/null | grep -q ":${VNC_PORT_ACTUAL}[^0-9]"; then
        echo "✓ VNC port ${VNC_PORT_ACTUAL} is confirmed listening"
    fi
fi

echo ""
echo "=========================================="
echo "VNC Server is running!"
echo "=========================================="
echo "Display: :${DISPLAY_NUM}"
echo "VNC Port: ${VNC_PORT_ACTUAL}"
echo ""
echo "To connect from your host machine:"
if [ -f /.dockerenv ]; then
    echo "  You are in a Docker container"
    echo "  Connect to: localhost:${VNC_PORT_ACTUAL}"
    echo ""
    echo "  If connection fails, you may need to:"
    echo "  1. Forward port ${VNC_PORT_ACTUAL} in VS Code (Ctrl+Shift+P -> 'Forward a Port')"
    echo "  2. Or use Docker with: -p ${VNC_PORT_ACTUAL}:${VNC_PORT_ACTUAL}"
    echo "  3. Or use host networking mode"
elif [ -n "${WSL_DISTRO_NAME}" ] || grep -q microsoft /proc/version 2>/dev/null; then
    echo "  You are in WSL"
    echo "  Connect to: localhost:${VNC_PORT_ACTUAL}"
else
    echo "  Connect to: localhost:${VNC_PORT_ACTUAL}"
fi
echo ""
echo "Recommended VNC viewers:"
echo "  - Windows: TightVNC Viewer, RealVNC Viewer, or TigerVNC"
echo "  - Download from: https://www.tightvnc.com/download.php"
echo "  - Mac: Built-in Screen Sharing (vnc://localhost:${VNC_PORT_ACTUAL})"
echo "  - Linux: Remmina, TigerVNC, or vncviewer"
echo ""
echo "IMPORTANT: Connect with a VNC viewer (NOT a web browser)"
echo "  In TightVNC Viewer, enter: localhost::${VNC_PORT_ACTUAL}"
echo "  (Note the double colon - it means port ${VNC_PORT_ACTUAL})"
echo "=========================================="
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Cleaning up..."
    pkill -f "x11vnc.*:${DISPLAY_NUM}" 2>/dev/null || true
    kill $XVFB_PID 2>/dev/null || true
    rm -f /tmp/.X${DISPLAY_NUM}-lock /tmp/.X11-unix/X${DISPLAY_NUM} 2>/dev/null || true
    echo "Cleanup complete."
}
trap cleanup EXIT INT TERM

# Set display to the virtual display
export DISPLAY=:${DISPLAY_NUM}
export QT_QPA_PLATFORM=xcb  # Use XCB platform for Qt

# Verify DISPLAY is set correctly
echo "DISPLAY environment variable set to: $DISPLAY"
echo ""

# Test that we can create a window on the virtual display
echo "Testing virtual display..."
if DISPLAY=:${DISPLAY_NUM} xwininfo -root &>/dev/null; then
    echo "✓ Virtual display is working"
    # Show what's on the display
    echo "Current windows on display :${DISPLAY_NUM}:"
    DISPLAY=:${DISPLAY_NUM} xwininfo -root -tree 2>/dev/null | head -5 || echo "  (no windows yet)"
else
    echo "✗ WARNING: Could not verify virtual display"
fi
echo ""

# Run the standalone client
echo "Starting standalone client..."
echo "The UI should appear in your VNC viewer!"
echo ""
echo ""
echo "=========================================="
echo "DIAGNOSTIC INFO:"
echo "=========================================="
echo "To verify VNC is working, in a NEW terminal run:"
echo "  export DISPLAY=:99"
echo "  bash ./scripts/test_vnc.sh"
echo ""
echo "This will create a test window you should see in VNC."
echo ""
echo "If you're still seeing your local screen:"
echo "  1. Make sure you're connecting to PORT ${VNC_PORT_ACTUAL}, not display :0 or :1"
echo "  2. In TightVNC Viewer:"
echo "     - Remote Host: localhost::${VNC_PORT_ACTUAL} (DOUBLE colon)"
echo "     - NOT localhost:${VNC_PORT_ACTUAL} (single colon)"
echo "     - NOT localhost:0 or localhost:1"
echo "  3. Check for other VNC servers: ps aux | grep vnc"
echo "  4. Verify port forwarding in VS Code (Ctrl+Shift+P -> Forward a Port -> 5900)"
echo "=========================================="
echo ""
ros2 run gstreamer standalone_client