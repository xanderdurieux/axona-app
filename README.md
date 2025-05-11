# Axona App - Cycling Impact Detection System

A real-time impact detection and analysis system for cyclists, designed to monitor and analyze impacts during cycling activities. The system uses IMU (Inertial Measurement Unit) data to detect impacts, calculate various impact metrics, and provide real-time feedback.

## Features

- **Real-time Impact Detection**: Monitors acceleration and angular velocity to detect impacts
- **Impact Level Classification**: Categorizes impacts into 5 levels (0-4) based on severity
- **Advanced Impact Metrics**:
  - HIC (Head Injury Criterion)
  - Peak Linear Acceleration
  - Riding Velocity before Impact
  - Head Velocity on Impact
- **Visual Feedback**: LED indicators for impact severity
- **BLE Connectivity**: Wireless communication with IMU sensors

## Hardware Requirements

- Arduino-compatible board
- IMU sensor with BLE connectivity
- 5 LEDs for impact level indication
- Power supply

## Pin Configuration

- LED_PIN_1: Pin 11
- LED_PIN_2: Pin 9
- LED_PIN_3: Pin 7
- LED_PIN_4: Pin 5
- LED_PIN_5: Pin 3

## Impact Thresholds

The system uses the following acceleration thresholds (in g):
- Low Impact: 2.5g
- Medium Impact: 5.0g
- High Impact: 7.5g
- Severe Impact: 10.0g

## Velocity Calculation

The system calculates two types of velocities:

1. **Riding Velocity**:
   - Calculated over a 5-second window before impact
   - Uses high-pass filtering to remove drift
   - Returns velocity in km/h

2. **Head Impact Velocity**:
   - Calculated over a 100ms window before impact
   - Uses direct integration for precise impact measurement
   - Returns velocity in km/h

## Installation

1. Clone the repository:
```bash
git clone https://github.com/xanderdurieux/axona-app.git
```

2. Open the project in Arduino IDE

3. Install required libraries:
   - ArduinoBLE

4. Upload the code to your Arduino board

## Usage

1. Power on the system
2. The system will automatically:
   - Initialize BLE
   - Scan for IMU sensors
   - Connect to the configured sensor
   - Begin monitoring for impacts

3. When an impact is detected:
   - LEDs will indicate the impact level
   - Impact metrics will be calculated and displayed
   - Data can be monitored through Serial output

## Impact Metrics

### HIC (Head Injury Criterion)
- Calculated over a 15ms window
- Risk levels:
  - Low: < 500
  - Medium: 500-1000
  - High: > 1000

## Acknowledgments

- Developed as part of the Excellence Program
- Special thanks to all contributors and testers 