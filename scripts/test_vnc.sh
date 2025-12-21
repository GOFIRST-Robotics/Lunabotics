#!/bin/bash
# Simple test script to verify VNC is showing the virtual display
# Run this while run_with_vnc.sh is running in another terminal

export DISPLAY=:99

echo "Testing VNC connection to display :99..."
echo ""

# Check if display is accessible
if ! xdpyinfo &>/dev/null; then
    echo "ERROR: Cannot access display :99"
    echo "Make sure run_with_vnc.sh is running!"
    exit 1
fi

echo "✓ Display :99 is accessible"
echo ""

# List windows on the display
echo "Windows currently on display :99:"
xwininfo -root -tree 2>/dev/null | head -20
echo ""

# Try to create a simple colored window using ImageMagick or similar
if command -v convert &> /dev/null; then
    echo "Creating a test image on display :99..."
    convert -size 400x300 xc:red -pointsize 72 -fill white -gravity center -annotate +0+0 "DISPLAY :99" /tmp/test_vnc.png
    if command -v display &> /dev/null; then
        display -display :99 /tmp/test_vnc.png &
        echo "✓ Test image window opened - you should see a red window with text in VNC"
        echo "  Close it with: pkill display"
    fi
elif command -v python3 &> /dev/null; then
    echo "Creating a test window using Python..."
    python3 << 'EOF'
import sys
try:
    from tkinter import Tk, Label
    root = Tk()
    root.title("VNC Test - Display :99")
    root.geometry("400x300")
    root.configure(bg='red')
    label = Label(root, text="If you see this in VNC,\nVNC is working!\nDisplay :99", 
                  bg='red', fg='white', font=('Arial', 24))
    label.pack(expand=True)
    root.after(30000, root.destroy)  # Close after 30 seconds
    root.mainloop()
except ImportError:
    print("tkinter not available, trying alternative...")
    sys.exit(1)
EOF
    if [ $? -eq 0 ]; then
        echo "✓ Test window opened - you should see a red window in VNC"
    fi
else
    echo "No suitable tool found to create test window"
    echo "Try installing: sudo apt-get install imagemagick python3-tk"
fi

echo ""
echo "If you see the test window in your VNC viewer, VNC is working correctly!"
echo "If you still see your local screen, you're connecting to the wrong display."

