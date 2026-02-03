#!/usr/bin/env python3
"""
Advanced ARGoS Simulation Screenshot Capture Script

This script captures screenshots specifically when the simulation creates:
- Yellow dots (individual visited locations)
- Cyan dots (site fidelity locations)  
- Magenta dots (merged super-clusters)

It uses computer vision to detect when new dots appear on screen.
"""

import subprocess
import time
import os
import sys
from datetime import datetime
import argparse
import numpy as np
import cv2
import mss
import mss.tools
import psutil
from collections import defaultdict


class ColorDotDetector:
    """Detects colored dots in ARGoS simulation screenshots."""
    
    # Color ranges in HSV for detection
    COLOR_RANGES = {
        'yellow': {
            'lower': np.array([20, 100, 100]),
            'upper': np.array([30, 255, 255]),
            'name': 'yellow_dot'
        },
        'cyan': {
            'lower': np.array([80, 100, 100]),
            'upper': np.array([100, 255, 255]),
            'name': 'cyan_dot'
        },
        'magenta': {
            'lower': np.array([140, 100, 100]),
            'upper': np.array([170, 255, 255]),
            'name': 'magenta_dot'
        }
    }
    
    def __init__(self, min_dot_size=5, max_dot_size=200):
        """
        Initialize the color dot detector.
        
        Args:
            min_dot_size: Minimum area in pixels for a dot
            max_dot_size: Maximum area in pixels for a dot
        """
        self.min_dot_size = min_dot_size
        self.max_dot_size = max_dot_size
        self.previous_counts = defaultdict(int)
    
    def detect_dots(self, image):
        """
        Detect colored dots in the image.
        
        Args:
            image: BGR image from screenshot
            
        Returns:
            Dictionary with counts of each color dot
        """
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        dot_counts = {}
        
        for color_name, color_info in self.COLOR_RANGES.items():
            # Create mask for this color
            mask = cv2.inRange(hsv, color_info['lower'], color_info['upper'])
            
            # Apply morphological operations to clean up the mask
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Count dots of appropriate size
            dot_count = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if self.min_dot_size <= area <= self.max_dot_size:
                    dot_count += 1
            
            dot_counts[color_name] = dot_count
        
        return dot_counts
    
    def has_new_dots(self, current_counts):
        """
        Check if new dots have appeared since last check.
        
        Args:
            current_counts: Dictionary of current dot counts
            
        Returns:
            Tuple of (has_new_dots, list of colors with new dots)
        """
        new_colors = []
        
        for color, count in current_counts.items():
            if count > self.previous_counts[color]:
                new_colors.append(color)
        
        # Update previous counts
        self.previous_counts = current_counts.copy()
        
        return (len(new_colors) > 0, new_colors)


class ARGoSScreenshotCaptureAdvanced:
    def __init__(self, xml_config, output_dir="screenshots", check_interval=0.5, monitor_num=None):
        """
        Initialize the advanced screenshot capture system.
        
        Args:
            xml_config: Path to ARGoS XML configuration file
            output_dir: Directory to save screenshots
            check_interval: Time between checks for new dots
            monitor_num: Monitor number to capture (None for primary)
        """
        self.xml_config = xml_config
        self.output_dir = output_dir
        self.check_interval = check_interval
        self.monitor_num = monitor_num
        self.screenshot_count = 0
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        # Initialize screen capture
        self.sct = mss.mss()
        
        # Determine which monitor to capture
        if monitor_num is None:
            self.monitor = self.sct.monitors[1]
        else:
            self.monitor = self.sct.monitors[monitor_num]
        
        print(f"Capturing from monitor: {self.monitor}")
        
        # Initialize color detector
        self.detector = ColorDotDetector()
        
        # Statistics
        self.stats = {
            'yellow_dots': 0,
            'cyan_dots': 0,
            'magenta_dots': 0,
            'total_captures': 0
        }
    
    def find_argos_window(self):
        """Find the ARGoS window and return its position."""
        try:
            result = subprocess.run(['wmctrl', '-l'], capture_output=True, text=True)
            for line in result.stdout.split('\n'):
                if 'argos' in line.lower():
                    window_id = line.split()[0]
                    geom_result = subprocess.run(
                        ['xwininfo', '-id', window_id],
                        capture_output=True,
                        text=True
                    )
                    
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
    
    def capture_and_save(self, reason="unknown", dot_counts=None):
        """
        Capture and save a screenshot.
        
        Args:
            reason: Reason for capture
            dot_counts: Dictionary of dot counts (optional)
        """
        self.screenshot_count += 1
        
        # Try to capture just the ARGoS window
        window_region = self.find_argos_window()
        capture_region = window_region if window_region else self.monitor
        
        # Capture screenshot
        sct_img = self.sct.grab(capture_region)
        
        # Generate filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        
        # Add dot counts to filename if available
        counts_str = ""
        if dot_counts:
            counts_str = f"_y{dot_counts.get('yellow', 0)}_c{dot_counts.get('cyan', 0)}_m{dot_counts.get('magenta', 0)}"
        
        filename = f"shot_{self.screenshot_count:04d}_{timestamp}_{reason}{counts_str}.png"
        filepath = os.path.join(self.output_dir, filename)
        
        # Save the screenshot
        mss.tools.to_png(sct_img.rgb, sct_img.size, output=filepath)
        
        print(f"[{datetime.now().strftime('%H:%M:%S')}] 📸 {filename}")
        
        # Update statistics
        self.stats['total_captures'] += 1
        if reason in self.stats:
            self.stats[reason] += 1
        
        return filepath
    
    def monitor_and_capture(self):
        """
        Monitor the ARGoS simulation and capture screenshots when new dots appear.
        """
        print(f"\n{'='*70}")
        print(f"ARGoS Screenshot Capture - Dot Detection Mode")
        print(f"{'='*70}")
        print(f"Config: {self.xml_config}")
        print(f"Output: {self.output_dir}")
        print(f"Check interval: {self.check_interval}s")
        print(f"\nWatching for:")
        print("  🟡 Yellow dots (visited locations)")
        print("  🔵 Cyan dots (fidelity sites)")
        print("  🟣 Magenta dots (merged clusters)")
        print(f"\nPress Ctrl+C to stop\n{'='*70}\n")
        
        try:
            while True:
                # Check if ARGoS is running
                argos_running = False
                for proc in psutil.process_iter(['name']):
                    if 'argos' in proc.info['name'].lower():
                        argos_running = True
                        break
                
                if not argos_running:
                    print("⏳ Waiting for ARGoS to start...")
                    time.sleep(2)
                    continue
                
                # Capture current screen
                window_region = self.find_argos_window()
                capture_region = window_region if window_region else self.monitor
                sct_img = self.sct.grab(capture_region)
                
                # Convert to numpy array for OpenCV
                img = np.array(sct_img)
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                
                # Detect dots
                dot_counts = self.detector.detect_dots(img)
                
                # Check for new dots
                has_new, new_colors = self.detector.has_new_dots(dot_counts)
                
                if has_new:
                    # Capture screenshot because new dots appeared
                    reason = "_".join([color for color in new_colors])
                    self.capture_and_save(reason, dot_counts)
                    
                    # Print detection info
                    print(f"  └─ Detected: {', '.join(new_colors)}")
                    print(f"     Current counts - Yellow: {dot_counts['yellow']}, "
                          f"Cyan: {dot_counts['cyan']}, Magenta: {dot_counts['magenta']}")
                
                # Wait before next check
                time.sleep(self.check_interval)
                
        except KeyboardInterrupt:
            print("\n\n✋ Capture stopped by user")
            self.print_statistics()
    
    def run_argos_and_capture(self):
        """Launch ARGoS and monitor simultaneously."""
        print(f"\n🚀 Launching ARGoS with config: {self.xml_config}\n")
        
        try:
            # Start ARGoS
            argos_process = subprocess.Popen(
                ['argos3', '-c', self.xml_config],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            print("⏳ Waiting for ARGoS window to appear...")
            time.sleep(3)
            
            print("✅ Starting capture...\n")
            
            # Monitor loop
            while argos_process.poll() is None:
                # Capture current screen
                window_region = self.find_argos_window()
                capture_region = window_region if window_region else self.monitor
                sct_img = self.sct.grab(capture_region)
                
                # Convert to numpy array
                img = np.array(sct_img)
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                
                # Detect dots
                dot_counts = self.detector.detect_dots(img)
                
                # Check for new dots
                has_new, new_colors = self.detector.has_new_dots(dot_counts)
                
                if has_new:
                    reason = "_".join([color for color in new_colors])
                    self.capture_and_save(reason, dot_counts)
                    print(f"  └─ New dots: {', '.join(new_colors)}")
                
                time.sleep(self.check_interval)
            
            # Get final output
            stdout, stderr = argos_process.communicate()
            
            print(f"\n✅ ARGoS simulation completed")
            print(f"Return code: {argos_process.returncode}\n")
            
            self.print_statistics()
            
        except FileNotFoundError:
            print("❌ Error: argos3 command not found.")
            print("   Make sure ARGoS is installed and in your PATH.")
            sys.exit(1)
        except KeyboardInterrupt:
            print("\n\n✋ Stopped by user")
            if argos_process:
                argos_process.terminate()
                argos_process.wait()
            self.print_statistics()
    
    def print_statistics(self):
        """Print capture statistics."""
        print(f"\n{'='*70}")
        print("📊 Capture Statistics")
        print(f"{'='*70}")
        print(f"Total screenshots:    {self.stats['total_captures']}")
        print(f"Yellow dot captures:  {self.stats['yellow_dots']}")
        print(f"Cyan dot captures:    {self.stats['cyan_dots']}")
        print(f"Magenta dot captures: {self.stats['magenta_dots']}")
        print(f"Output directory:     {self.output_dir}")
        print(f"{'='*70}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Advanced ARGoS screenshot capture with color dot detection",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Monitor a running ARGoS simulation
  python capture_screenshots_advanced.py experiments/CPFA_ClusterMap_clustered_16.xml
  
  # Launch ARGoS and capture automatically
  python capture_screenshots_advanced.py experiments/CPFA_ClusterMap_clustered_16.xml --launch
  
  # Custom output directory and check interval
  python capture_screenshots_advanced.py config.xml -o my_captures -i 0.2
        """
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
        default=0.5,
        help='Check interval in seconds (default: 0.5)'
    )
    parser.add_argument(
        '-m', '--monitor',
        type=int,
        default=None,
        help='Monitor number to capture (default: primary)'
    )
    parser.add_argument(
        '--launch',
        action='store_true',
        help='Launch ARGoS and capture simultaneously'
    )
    
    args = parser.parse_args()
    
    # Validate config file
    if not os.path.exists(args.xml_config):
        print(f"❌ Error: Configuration file not found: {args.xml_config}")
        sys.exit(1)
    
    # Check for required packages
    try:
        import cv2
        import numpy as np
        import mss
        import psutil
    except ImportError as e:
        print(f"❌ Error: Missing required package: {e}")
        print("\nInstall required packages with:")
        print("  pip install opencv-python numpy mss psutil")
        sys.exit(1)
    
    # Create capturer
    capturer = ARGoSScreenshotCaptureAdvanced(
        xml_config=args.xml_config,
        output_dir=args.output_dir,
        check_interval=args.interval,
        monitor_num=args.monitor
    )
    
    # Run
    if args.launch:
        capturer.run_argos_and_capture()
    else:
        print("\n⚠️  Monitoring mode: Make sure ARGoS is already running!")
        print("   Or use --launch to start ARGoS automatically\n")
        capturer.monitor_and_capture()


if __name__ == "__main__":
    main()
