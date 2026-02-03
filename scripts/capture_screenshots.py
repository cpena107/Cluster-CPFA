#!/usr/bin/env python3
"""
ARGoS Simulation Screenshot Capture Script

This script captures screenshots of the ARGoS simulation whenever it creates:
- Yellow dots (individual visited locations)
- Cyan dots (site fidelity locations)
- Magenta dots (merged super-clusters)

The script monitors the simulation output and captures screenshots at appropriate times.
"""

import subprocess
import time
import os
import sys
from datetime import datetime
import argparse
import re
import mss
import mss.tools
import psutil


class ARGoSScreenshotCapture:
    def __init__(self, xml_config, output_dir="screenshots", delay=0.5, monitor_num=None):
        """
        Initialize the screenshot capture system.
        
        Args:
            xml_config: Path to ARGoS XML configuration file
            output_dir: Directory to save screenshots
            delay: Delay in seconds between screenshot checks
            monitor_num: Monitor number to capture (None for primary)
        """
        self.xml_config = xml_config
        self.output_dir = output_dir
        self.delay = delay
        self.monitor_num = monitor_num
        self.screenshot_count = 0
        self.last_capture_time = 0
        
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Initialize screen capture
        self.sct = mss.mss()
        
        # Determine which monitor to capture
        if monitor_num is None:
            # Capture primary monitor (usually index 1, as 0 is all monitors)
            self.monitor = self.sct.monitors[1]
        else:
            self.monitor = self.sct.monitors[monitor_num]
        
        print(f"Capturing from monitor: {self.monitor}")
        
        # Statistics
        self.stats = {
            'yellow_dots': 0,  # Visited locations
            'cyan_dots': 0,    # Fidelity locations
            'magenta_dots': 0  # Merged clusters
        }
    
    def find_argos_window(self):
        """
        Find the ARGoS window and return its position.
        This helps to capture only the ARGoS window instead of the entire screen.
        """
        try:
            # Try to find ARGoS window using wmctrl
            result = subprocess.run(['wmctrl', '-l'], capture_output=True, text=True)
            for line in result.stdout.split('\n'):
                if 'argos' in line.lower():
                    # Parse window geometry using xwininfo
                    window_id = line.split()[0]
                    geom_result = subprocess.run(
                        ['xwininfo', '-id', window_id],
                        capture_output=True,
                        text=True
                    )
                    
                    # Extract position and size
                    x = y = width = height = None
                    for geom_line in geom_result.stdout.split('\n'):
                        if 'Absolute upper-left X:' in geom_line:
                            x = int(geom_line.split(':')[1].strip())
                        elif 'Absolute upper-left Y:' in geom_line:
                            y = int(geom_line.split(':')[1].strip())
                        elif 'Width:' in geom_line:
                            width = int(geom_line.split(':')[1].strip())
                        elif 'Height:' in geom_line:
                            height = int(geom_line.split(':')[1].strip())
                    
                    if all([x, y, width, height]):
                        return {"top": y, "left": x, "width": width, "height": height}
        except (subprocess.SubprocessError, FileNotFoundError):
            pass
        
        return None
    
    def capture_screenshot(self, reason=""):
        """
        Capture a screenshot of the ARGoS simulation.
        
        Args:
            reason: Reason for capture (e.g., "yellow_dot", "cyan_dot", "magenta_dot")
        """
        current_time = time.time()
        
        # Prevent too frequent captures (minimum 0.1 seconds between captures)
        if current_time - self.last_capture_time < 0.1:
            return
        
        self.last_capture_time = current_time
        self.screenshot_count += 1
        
        # Try to capture just the ARGoS window, fall back to full monitor
        window_region = self.find_argos_window()
        capture_region = window_region if window_region else self.monitor
        
        # Capture the screenshot
        sct_img = self.sct.grab(capture_region)
        
        # Generate filename with timestamp and reason
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"screenshot_{self.screenshot_count:04d}_{timestamp}_{reason}.png"
        filepath = os.path.join(self.output_dir, filename)
        
        # Save the screenshot
        mss.tools.to_png(sct_img.rgb, sct_img.size, output=filepath)
        
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Captured: {filename}")
        
        # Update statistics
        if reason in self.stats:
            self.stats[reason] += 1
    
    def monitor_simulation(self, capture_interval=1.0):
        """
        Monitor the ARGoS simulation and capture screenshots periodically.
        
        This method captures screenshots at regular intervals during the simulation
        since we can't directly hook into when dots are created without modifying C++ code.
        
        Args:
            capture_interval: Time in seconds between captures
        """
        print(f"\nStarting screenshot capture for: {self.xml_config}")
        print(f"Output directory: {self.output_dir}")
        print(f"Capture interval: {capture_interval} seconds")
        print("\nPress Ctrl+C to stop capture\n")
        
        try:
            frame_count = 0
            while True:
                # Check if ARGoS is still running
                argos_running = False
                for proc in psutil.process_iter(['name']):
                    if 'argos' in proc.info['name'].lower():
                        argos_running = True
                        break
                
                if not argos_running:
                    print("\nARGoS simulation not detected. Waiting...")
                    time.sleep(2)
                    continue
                
                # Capture screenshot
                # We capture periodically and later the user can filter interesting frames
                reason = "periodic"
                if frame_count % 10 == 0:  # Mark some frames for potential analysis
                    reason = "checkpoint"
                
                self.capture_screenshot(reason)
                frame_count += 1
                
                # Wait for next capture
                time.sleep(capture_interval)
                
        except KeyboardInterrupt:
            print("\n\nCapture stopped by user")
            self.print_statistics()
    
    def run_argos_and_capture(self, capture_interval=1.0):
        """
        Launch ARGoS simulation and capture screenshots simultaneously.
        
        Args:
            capture_interval: Time in seconds between captures
        """
        print(f"\nLaunching ARGoS with config: {self.xml_config}")
        print(f"Output directory: {self.output_dir}")
        print(f"Capture interval: {capture_interval} seconds\n")
        
        # Start ARGoS in a subprocess
        try:
            argos_process = subprocess.Popen(
                ['argos3', '-c', self.xml_config],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            print("ARGoS started, waiting for window to appear...")
            time.sleep(3)  # Give ARGoS time to start up
            
            print("Starting screenshot capture...\n")
            
            frame_count = 0
            while argos_process.poll() is None:  # While ARGoS is running
                self.capture_screenshot(f"frame_{frame_count:06d}")
                frame_count += 1
                time.sleep(capture_interval)
            
            # Get final output
            stdout, stderr = argos_process.communicate()
            
            print(f"\n\nARGoS simulation completed")
            print(f"Return code: {argos_process.returncode}")
            
            if stdout:
                print(f"\nOutput:\n{stdout}")
            
            self.print_statistics()
            
        except FileNotFoundError:
            print("Error: argos3 command not found. Make sure ARGoS is installed and in your PATH.")
            sys.exit(1)
        except KeyboardInterrupt:
            print("\n\nCapture stopped by user")
            if argos_process:
                argos_process.terminate()
                argos_process.wait()
            self.print_statistics()
    
    def print_statistics(self):
        """Print capture statistics."""
        print("\n" + "="*60)
        print("Capture Statistics")
        print("="*60)
        print(f"Total screenshots: {self.screenshot_count}")
        print(f"Output directory: {self.output_dir}")
        print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Capture screenshots of ARGoS simulation when colored dots appear"
    )
    parser.add_argument(
        'xml_config',
        help='Path to ARGoS XML configuration file'
    )
    parser.add_argument(
        '-o', '--output-dir',
        default='screenshots',
        help='Output directory for screenshots (default: screenshots)'
    )
    parser.add_argument(
        '-i', '--interval',
        type=float,
        default=1.0,
        help='Capture interval in seconds (default: 1.0)'
    )
    parser.add_argument(
        '-m', '--monitor',
        type=int,
        default=None,
        help='Monitor number to capture (default: primary monitor)'
    )
    parser.add_argument(
        '--monitor-only',
        action='store_true',
        help='Only monitor running ARGoS (don\'t launch it)'
    )
    parser.add_argument(
        '--launch',
        action='store_true',
        help='Launch ARGoS and capture simultaneously'
    )
    
    args = parser.parse_args()
    
    # Validate XML config file exists
    if not os.path.exists(args.xml_config):
        print(f"Error: Configuration file not found: {args.xml_config}")
        sys.exit(1)
    
    # Create capture instance
    capturer = ARGoSScreenshotCapture(
        xml_config=args.xml_config,
        output_dir=args.output_dir,
        monitor_num=args.monitor
    )
    
    # Run based on mode
    if args.launch:
        capturer.run_argos_and_capture(capture_interval=args.interval)
    else:
        # Default: monitor mode
        print("\nMonitoring mode: Make sure ARGoS is already running!")
        print("Or use --launch to start ARGoS automatically\n")
        capturer.monitor_simulation(capture_interval=args.interval)


if __name__ == "__main__":
    main()
