#!/bin/bash
# HWS Driver Streaming Test Script
# Tests v4l2-compliance and basic streaming functionality

set -e

DEVICE="/dev/video0"
LOG_FILE="/tmp/hws_test.log"

echo "HWS Driver Streaming Test" | tee $LOG_FILE
echo "=========================" | tee -a $LOG_FILE
echo "Date: $(date)" | tee -a $LOG_FILE
echo "" | tee -a $LOG_FILE

# Check if device exists
if [ ! -e "$DEVICE" ]; then
    echo "ERROR: Video device $DEVICE not found!" | tee -a $LOG_FILE
    echo "Make sure the HWS driver is loaded: sudo modprobe HwsCapture" | tee -a $LOG_FILE
    exit 1
fi

echo "✓ Video device $DEVICE found" | tee -a $LOG_FILE

# Check device capabilities
echo "" | tee -a $LOG_FILE
echo "Device Capabilities:" | tee -a $LOG_FILE
echo "-------------------" | tee -a $LOG_FILE
v4l2-ctl -d $DEVICE --all 2>&1 | tee -a $LOG_FILE

# Run v4l2-compliance tests
echo "" | tee -a $LOG_FILE
echo "Running v4l2-compliance tests..." | tee -a $LOG_FILE
echo "=================================" | tee -a $LOG_FILE

# Basic compliance
echo "1. Basic compliance test:" | tee -a $LOG_FILE
if v4l2-compliance -d $DEVICE -v 2>&1 | tee -a $LOG_FILE; then
    echo "✓ Basic compliance: PASSED" | tee -a $LOG_FILE
else
    echo "✗ Basic compliance: FAILED" | tee -a $LOG_FILE
fi

# Streaming compliance  
echo "" | tee -a $LOG_FILE
echo "2. Streaming compliance test:" | tee -a $LOG_FILE
if v4l2-compliance -d $DEVICE -s -v 2>&1 | tee -a $LOG_FILE; then
    echo "✓ Streaming compliance: PASSED" | tee -a $LOG_FILE
else
    echo "✗ Streaming compliance: FAILED" | tee -a $LOG_FILE
fi

# Test basic streaming
echo "" | tee -a $LOG_FILE
echo "3. Basic streaming test (5 frames):" | tee -a $LOG_FILE
echo "====================================" | tee -a $LOG_FILE

# Set format
v4l2-ctl -d $DEVICE --set-fmt-video=width=1920,height=1080,pixelformat=YUYV 2>&1 | tee -a $LOG_FILE

# Capture 5 frames
if timeout 10s v4l2-ctl -d $DEVICE --stream-mmap=5 --stream-count=5 --stream-to=/tmp/hws_test.raw 2>&1 | tee -a $LOG_FILE; then
    echo "✓ Basic streaming: PASSED" | tee -a $LOG_FILE
    if [ -f /tmp/hws_test.raw ]; then
        SIZE=$(stat -f%z /tmp/hws_test.raw 2>/dev/null || stat -c%s /tmp/hws_test.raw 2>/dev/null)
        echo "  Captured file size: $SIZE bytes" | tee -a $LOG_FILE
        rm -f /tmp/hws_test.raw
    fi
else
    echo "✗ Basic streaming: FAILED (timeout or error)" | tee -a $LOG_FILE
fi

# Check kernel logs for errors
echo "" | tee -a $LOG_FILE
echo "Recent kernel messages (last 50 lines):" | tee -a $LOG_FILE
echo "=======================================" | tee -a $LOG_FILE
dmesg | tail -50 | grep -i hws | tee -a $LOG_FILE || echo "No HWS messages in dmesg" | tee -a $LOG_FILE

echo "" | tee -a $LOG_FILE
echo "Test completed. Results saved to $LOG_FILE" | tee -a $LOG_FILE
echo "" | tee -a $LOG_FILE
echo "COMPREHENSIVE STREAMING FIXES APPLIED:" | tee -a $LOG_FILE
echo "1. Upstream CREATE_BUFS format validation fix" | tee -a $LOG_FILE
echo "2. Two-buffer approach with ring buffer management" | tee -a $LOG_FILE  
echo "3. Enhanced DMA synchronization and timeout handling" | tee -a $LOG_FILE
echo "4. Comprehensive error handling and diagnostics" | tee -a $LOG_FILE
echo "" | tee -a $LOG_FILE
echo "If streaming fails, check:" | tee -a $LOG_FILE
echo "1. Hardware is connected and powered" | tee -a $LOG_FILE
echo "2. Video signal is present on input" | tee -a $LOG_FILE  
echo "3. Driver logs: dmesg | grep -i hws" | tee -a $LOG_FILE
echo "4. Register dump: echo 'dump' > /sys/kernel/debug/hws/debug" | tee -a $LOG_FILE
echo "5. Ring buffer status in channel diagnostics" | tee -a $LOG_FILE
