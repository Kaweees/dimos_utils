# Force-Torque Sensor Calibration and Visualization Guide

This guide explains how to calibrate and visualize data from the OpenFT force-torque sensor using the xARM6 robot.

## Overview

The calibration system consists of three main components:
1. **drake_openft_test.py** - Robot control and calibration data collection
2. **calc_calibration_matrix.py** - Calibration matrix calculation and real-time application
3. **force_torque_visualizer.py** - Real-time visualization dashboard

## System Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Raw Sensor    │────▶│   Calibration    │────▶│  Visualization  │
│   (Port 5555)   │     │  (Ports 5555→5556)│     │   (Port 5556)   │
└─────────────────┘     └──────────────────┘     └─────────────────┘
```

## Prerequisites

### Required Python Packages
```bash
pip install numpy pandas pydrake zmq dash plotly lcm
```

### Required Hardware/Software
- xARM6 robot with OpenFT gripper
- Sensor driver publishing raw data to ZMQ port 5555
- LCM for robot communication (if using --real flag)

## Step-by-Step Calibration Process

### Step 1: Collect Calibration Data

Run the Drake simulation with calibration sequence to collect sensor data paired with known forces/torques:

```bash
# Basic calibration (simulation only)
python drake_openft_test.py --calibrate

# With real robot control
python drake_openft_test.py --calibrate --real

# With sensor data collection (RECOMMENDED)
python drake_openft_test.py --calibrate --sensor

# Full setup with real robot and sensor
python drake_openft_test.py --calibrate --real --sensor
```

**What this does:**
- Moves joint5 and joint6 through a predefined calibration pattern
- Records joint positions, calculated forces/torques, and raw sensor data
- Saves data to CSV file: `openft_calibration_YYYYMMDD_HHMMSS.csv`
- Takes approximately 2-3 minutes to complete

**Calibration sequence:**
1. All joints to zero position
2. Joint5 to -π/2
3. Joint6 from 0 to -π (with joint5 at -π/2)
4. Joint6 from -π to +π
5. Joint6 back to 0
6. Joint5 back to 0

### Step 2: Calculate Calibration Matrix

Process the collected CSV data to compute the calibration matrix:

```bash
# Basic calibration calculation
python calc_calibration_matrix.py --csv openft_calibration_20250813_104335.csv --output calibration.json

# Without bias compensation (if sensor is pre-zeroed)
python calc_calibration_matrix.py --csv openft_calibration_20250813_104335.csv --output calibration.json --no-bias
```

**Output includes:**
- 6×16 calibration matrix mapping 16 sensors to 6 DOF (Fx, Fy, Fz, Mx, My, Mz)
- Bias vector for offset compensation
- Quality metrics (RMSE per axis)
- Sensor contribution analysis

**Expected output:**
```
Calibration quality metrics:
  Force RMSE:  0.0234 N
  Torque RMSE: 0.000456 N⋅m
  Fx RMSE: 0.021345 N
  Fy RMSE: 0.019234 N
  ...
```

### Step 3: Apply Calibration in Real-Time

Run the calibration in live mode to process sensor data:

```bash
# Default ports (input: 5555, output: 5556)
python calc_calibration_matrix.py --live --calibration calibration.json

# Custom ports
python calc_calibration_matrix.py --live --calibration calibration.json --in-port 5555 --out-port 5556
```

**Live mode features:**
- Subscribes to raw sensor data (16 channels)
- Applies calibration matrix in real-time
- Publishes calibrated forces/torques as JSON
- Prints formatted output to console

### Step 4: Visualize Calibrated Data

Launch the visualization dashboard:

```bash
python force_torque_visualizer.py
```

Then open your browser to: **http://localhost:8052**

**Dashboard features:**
- Real-time force components (Fx, Fy, Fz)
- Real-time torque components (Mx, My, Mz)
- Force and torque magnitude plots
- Current values display
- Statistics panel (mean, std, max)
- 500-point rolling history

## Usage Examples

### Example 1: Complete Calibration Workflow

```bash
# Terminal 1: Collect calibration data with sensor
python drake_openft_test.py --calibrate --sensor --real

# Terminal 2: Calculate calibration matrix
python calc_calibration_matrix.py --csv openft_calibration_20250813_104335.csv --output my_calibration.json

# Terminal 3: Run live calibration
python calc_calibration_matrix.py --live --calibration my_calibration.json

# Terminal 4: Visualize results
python force_torque_visualizer.py
```

### Example 2: Testing Without Real Robot

```bash
# Simulation-only calibration
python drake_openft_test.py --calibrate

# Process the data
python calc_calibration_matrix.py --csv openft_calibration_*.csv --output test_cal.json
```

### Example 3: Manual Robot Control Modes

```bash
# Joint control mode (default)
python drake_openft_test.py

# Inverse kinematics mode
python drake_openft_test.py --ik_control

# With real robot
python drake_openft_test.py --real
```

## Command Reference

### drake_openft_test.py

| Flag | Description |
|------|-------------|
| `--ik_control` | Use inverse kinematics control mode (default: joint control) |
| `--calibrate` | Run calibration sequence |
| `--real` | Enable real robot control via LCM |
| `--sensor` | Enable sensor data collection via ZMQ |

### calc_calibration_matrix.py

| Flag | Description |
|------|-------------|
| `--csv FILE` | Input CSV file with calibration data |
| `--output FILE` | Output calibration file (.json or .npz) |
| `--live` | Run live calibration mode |
| `--calibration FILE` | Calibration file for live mode |
| `--in-port PORT` | Input ZMQ port (default: 5555) |
| `--out-port PORT` | Output ZMQ port (default: 5556) |
| `--no-bias` | Calculate without bias term |

### force_torque_visualizer.py

No command-line arguments. Listens on port 5556 by default.

## Data Formats

### Raw Sensor Data (Port 5555)
```json
{
  "sensor_moving_averages": [val1, val2, ..., val16],
  "timestamp": 1234567890.123
}
```

### Calibrated Force-Torque Data (Port 5556)
```json
{
  "timestamp": 1234567890.123,
  "forces": [Fx, Fy, Fz],        // Newtons
  "torques": [Mx, My, Mz],        // Newton-meters
  "force_magnitude": 8.234,       // Newtons
  "torque_magnitude": 0.0234,     // Newton-meters
  "raw_sensors": [...]             // Original sensor values
}
```

### Calibration File Format
```json
{
  "calibration_matrix": [[...], ...],  // 6x16 matrix
  "bias_vector": [...],                 // 6x1 vector
  "sensor_channels": 16,
  "output_channels": 6,
  "timestamp": 1234567890.123,
  "metadata": {
    "source_csv": "openft_calibration_20250813_104335.csv",
    "num_samples": 245,
    "use_bias": true
  }
}
```

## Troubleshooting

### Common Issues

1. **"No sensor data received"**
   - Ensure sensor driver is running and publishing to port 5555
   - Check ZMQ connection with `netstat -an | grep 5555`

2. **"Missing sensor columns in CSV"**
   - Make sure to use `--sensor` flag during calibration data collection
   - Verify sensor driver was running during calibration

3. **High calibration RMSE values**
   - Collect more diverse calibration poses
   - Check for sensor saturation or noise
   - Ensure robot is not near singularities during calibration

4. **Drake/Meshcat connection issues**
   - Open the Meshcat URL shown in console
   - Check firewall settings for port 7001
   - Restart the script if Meshcat becomes unresponsive

5. **LCM communication errors**
   - Verify LCM is installed: `lcm-spy`
   - Check network settings for multicast
   - Ensure robot is powered on and connected

## Tips for Good Calibration

1. **Sensor Warm-up**: Let the sensor run for 5-10 minutes before calibration
2. **Movement Speed**: Use slow, smooth movements (500ms delay between steps)
3. **Data Variety**: Include various orientations to excite all sensor channels
4. **Multiple Runs**: Collect 2-3 calibration datasets and compare results
5. **Validation**: Test calibration with known weights/forces
6. **Environment**: Minimize vibrations and temperature changes during calibration

## Sensor Configuration

The OpenFT sensor uses:
- 4 magnets embedded in the gripper
- 4 hall effect sensors per magnet (plus pattern)
- 16 total measurement channels
- Magnetic field changes indicate forces/torques

Channel mapping:
- Sensors 1-4: Magnet 1 (Middle)
- Sensors 5-8: Magnet 2 (Top)
- Sensors 9-12: Magnet 3 (Bottom Right)
- Sensors 13-16: Magnet 4 (Bottom Left)

## Safety Notes

⚠️ **When using `--real` flag:**
- Ensure emergency stop is accessible
- Clear the robot workspace
- Start with slow movements
- Monitor joint limits
- Never exceed sensor ratings

## Contact and Support

For issues or questions about:
- Drake simulation: Check Drake documentation
- LCM setup: See LCM user guide
- Sensor hardware: Contact OpenFT support
- Scripts: Review code comments and docstrings

## License

These scripts are provided as-is for research and development purposes.