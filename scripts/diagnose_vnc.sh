#!/bin/bash
# Diagnostic script to figure out why VNC shows local screen

echo "=========================================="
echo "VNC Connection Diagnostic"
echo "=========================================="
echo ""

# Check for Xvfb
echo "1. Checking for Xvfb on display :99..."
if pgrep -f "Xvfb :99" > /dev/null; then
    echo "   ✓ Xvfb is running on :99"
    ps aux | grep "Xvfb :99" | grep -v grep
else
    echo "   ✗ Xvfb is NOT running on :99"
    echo "   Run ./scripts/run_with_vnc.sh first!"
    exit 1
fi
echo ""

# Check for x11vnc
echo "2. Checking for x11vnc..."
if pgrep -f "x11vnc.*:99" > /dev/null; then
    echo "   ✓ x11vnc is running"
    ps aux | grep "x11vnc.*:99" | grep -v grep
    VNC_PID=$(pgrep -f "x11vnc.*:99" | head -1)
    echo "   Process ID: $VNC_PID"
else
    echo "   ✗ x11vnc is NOT running"
    exit 1
fi
echo ""

# Check what ports are listening
echo "3. Checking what's listening on VNC ports..."
echo "   Port 5900:"
ss -tlnp 2>/dev/null | grep ":5900" || netstat -tlnp 2>/dev/null | grep ":5900" || echo "   (nothing on 5900)"
echo ""
echo "   Port 5901:"
ss -tlnp 2>/dev/null | grep ":5901" || netstat -tlnp 2>/dev/null | grep ":5901" || echo "   (nothing on 5901)"
echo ""

# Check what's on display :99
echo "4. Checking what windows are on display :99..."
export DISPLAY=:99
if xwininfo -root -tree &>/dev/null; then
    echo "   Windows on display :99:"
    xwininfo -root -tree 2>/dev/null | head -10
    WINDOW_COUNT=$(xwininfo -root -tree 2>/dev/null | grep -c "0x" || echo "0")
    echo "   Total windows found: $WINDOW_COUNT"
    if [ "$WINDOW_COUNT" -eq "0" ] || [ "$WINDOW_COUNT" -eq "1" ]; then
        echo "   ⚠ WARNING: No application windows found on display :99"
        echo "   The UI might not have started yet, or DISPLAY is wrong"
    fi
else
    echo "   ✗ Cannot access display :99"
fi
echo ""

# Check DISPLAY variable
echo "5. Current DISPLAY environment:"
echo "   DISPLAY=$DISPLAY"
if [ "$DISPLAY" != ":99" ]; then
    echo "   ⚠ WARNING: DISPLAY is not :99"
    echo "   Set it with: export DISPLAY=:99"
fi
echo ""

# Test if we can create a window
echo "6. Testing if we can create a window on :99..."
export DISPLAY=:99
if command -v xdpyinfo &> /dev/null; then
    if xdpyinfo &>/dev/null; then
        echo "   ✓ Display :99 is accessible"
        SCREEN_INFO=$(xdpyinfo 2>/dev/null | grep "dimensions:" | head -1)
        echo "   $SCREEN_INFO"
    else
        echo "   ✗ Cannot access display :99"
    fi
fi
echo ""

# Check for other VNC servers
echo "7. Checking for other VNC servers that might interfere..."
ALL_VNC=$(ps aux | grep -i vnc | grep -v grep | grep -v "x11vnc.*:99")
if [ -n "$ALL_VNC" ]; then
    echo "   ⚠ Found other VNC processes:"
    echo "$ALL_VNC"
    echo "   These might be intercepting your connection!"
else
    echo "   ✓ No other VNC servers found"
fi
echo ""

# Final recommendations
echo "=========================================="
echo "Diagnostic Summary"
echo "=========================================="
echo ""
echo "If you're still seeing your local screen:"
echo ""
echo "1. Make sure you're connecting to the CORRECT port"
echo "   - Check the script output for 'VNC Port: XXXX'"
echo "   - Use that exact port in TightVNC: 127.0.0.1::XXXX"
echo ""
echo "2. Try a different port to avoid conflicts:"
echo "   - Edit run_with_vnc.sh: Change VNC_PORT=5900 to VNC_PORT=5901"
echo "   - Restart the script"
echo "   - Connect to 127.0.0.1::5901"
echo ""
echo "3. Check if Windows has a VNC server running:"
echo "   - Windows Remote Desktop might be interfering"
echo "   - Check Windows Services for 'Remote Desktop'"
echo ""
echo "4. Verify port forwarding in VS Code:"
echo "   - Look for 'PORTS' in the bottom panel"
echo "   - Port 5900 (or your port) should be listed"
echo "   - If not, forward it: Ctrl+Shift+P -> 'Forward a Port'"
echo ""

