# ARGoS Screenshot Capture Scripts

These Python scripts automatically capture screenshots of the ARGoS simulation when colored dots appear:
- **Yellow dots**: Individual visited locations
- **Cyan dots**: Site fidelity locations
- **Magenta dots**: Merged super-clusters

## Installation

Install the required Python packages:

```bash
pip install opencv-python numpy mss psutil
```

For the basic version (no color detection):
```bash
pip install mss psutil
```

## Scripts

### 1. `capture_screenshots.py` (Basic Version)

Captures screenshots at regular intervals while ARGoS is running.

**Usage:**

```bash
# Monitor an already running ARGoS simulation
python capture_screenshots.py experiments/CPFA_ClusterMap_clustered_16.xml

# Launch ARGoS and capture simultaneously
python capture_screenshots.py experiments/CPFA_ClusterMap_clustered_16.xml --launch

# Custom output directory and interval
python capture_screenshots.py experiments/CPFA_ClusterMap_clustered_16.xml -o my_screenshots -i 0.5
```

**Options:**
- `-o, --output-dir`: Output directory (default: `screenshots`)
- `-i, --interval`: Capture interval in seconds (default: 1.0)
- `-m, --monitor`: Monitor number to capture (default: primary)
- `--launch`: Launch ARGoS and capture together
- `--monitor-only`: Only monitor (don't launch ARGoS)

### 2. `capture_screenshots_advanced.py` (Advanced Version with Color Detection)

Uses computer vision to detect when new colored dots appear and captures screenshots only when changes occur.

**Usage:**

```bash
# Monitor an already running ARGoS simulation
python capture_screenshots_advanced.py experiments/CPFA_ClusterMap_clustered_16.xml

# Launch ARGoS and capture simultaneously
python capture_screenshots_advanced.py experiments/CPFA_ClusterMap_clustered_16.xml --launch

# Custom settings
python capture_screenshots_advanced.py experiments/CPFA_ClusterMap_clustered_16.xml -o captures -i 0.2
```

**Options:**
- `-o, --output-dir`: Output directory (default: `screenshots`)
- `-i, --interval`: Check interval in seconds (default: 0.5)
- `-m, --monitor`: Monitor number to capture (default: primary)
- `--launch`: Launch ARGoS and capture together

**Features:**
- Real-time color detection for yellow, cyan, and magenta dots
- Only captures when new dots appear (saves space)
- Includes dot counts in filenames
- Shows statistics at the end

## Examples

### Example 1: Monitor Running Simulation

Start ARGoS manually:
```bash
argos3 -c experiments/CPFA_ClusterMap_clustered_16.xml
```

In another terminal, run the capture script:
```bash
python capture_screenshots_advanced.py experiments/CPFA_ClusterMap_clustered_16.xml
```

### Example 2: Automated Capture

Launch everything automatically:
```bash
python capture_screenshots_advanced.py experiments/CPFA_ClusterMap_clustered_16.xml --launch -o my_experiment_screenshots -i 0.3
```

### Example 3: High-Frequency Capture

Capture very frequently for detailed analysis:
```bash
python capture_screenshots.py experiments/CPFA_ClusterMap_clustered_16.xml --launch -i 0.1
```

## Output

Screenshots are saved with descriptive filenames:

**Basic version:**
```
screenshot_0001_20260119_143052_123456_frame_000000.png
screenshot_0002_20260119_143053_234567_frame_000001.png
```

**Advanced version (with color detection):**
```
shot_0001_20260119_143052_123456_yellow_y5_c2_m0.png
shot_0002_20260119_143055_345678_cyan_y8_c3_m0.png
shot_0003_20260119_143058_456789_magenta_y15_c3_m2.png
```

Where:
- `y5` = 5 yellow dots detected
- `c2` = 2 cyan dots detected
- `m0` = 0 magenta dots detected

## Tips

1. **Window Focus**: The scripts try to capture only the ARGoS window. If that fails, they capture the entire monitor.

2. **Performance**: The advanced version uses more CPU due to image processing. If performance is an issue, increase the interval (-i) or use the basic version.

3. **Multiple Monitors**: Use `-m` to specify which monitor to capture if you have multiple displays.

4. **Stopping**: Press `Ctrl+C` to stop capture at any time. Statistics will be displayed.

5. **Storage**: Screenshots can take up space. Use the advanced version if you only want captures when dots appear.

## Troubleshooting

**"argos3 command not found"**
- Make sure ARGoS is installed and in your PATH

**"Missing required package"**
- Run: `pip install opencv-python numpy mss psutil`

**"ARGoS simulation not detected"**
- Make sure ARGoS is running with GUI (not headless mode)
- Check that the ARGoS window is visible

**No screenshots captured (advanced version)**
- The color thresholds may need adjustment for your display
- Try the basic version first to verify the setup works

## Color Detection Details

The advanced script detects colors using HSV color space:

- **Yellow**: HSV range [20, 100, 100] to [30, 255, 255]
- **Cyan**: HSV range [80, 100, 100] to [100, 255, 255]
- **Magenta**: HSV range [140, 100, 100] to [170, 255, 255]

These ranges can be adjusted in the `ColorDotDetector` class if needed for your specific display settings.
