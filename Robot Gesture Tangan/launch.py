#!/usr/bin/env python3
"""Launcher script for direct_control_webcamFIX.py with proper protobuf configuration."""
import os
import sys
import subprocess

# Set environment variable for protobuf compatibility
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

# Set UTF-8 encoding for better Unicode support
os.environ['PYTHONIOENCODING'] = 'utf-8'

# Run the main script
script_path = r"D:\SOFTWARE\Demo\Robot Gesture Tangan\direct_control_webcamFIX.py"
if os.path.exists(script_path):
    # Run with sys.executable to ensure we use the current Python
    subprocess.run([sys.executable, script_path])
else:
    print(f"Script not found: {script_path}")
    sys.exit(1)
