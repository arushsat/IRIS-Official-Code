# IRIS Robot - Integrated Robotic Irrigation System

## Overview

The IRIS robot is a fully autonomous, solar-powered, mobile water-harvesting unit engineered to operate in extreme environments where traditional infrastructure fails. This repository contains the complete, implementable code for all four core systems of the IRIS robot.

## Features

- **Mobile Water Harvesting**: Atmospheric water generation with intelligent humidity and temperature monitoring
- **Self-Sustained Energy System**: Solar power management with smart battery optimization
- **Intelligent Decision-Making**: Satellite image analysis using TensorFlow for water source detection
- **Obstacle Avoidance & Safe Navigation**: LIDAR-based autonomous navigation with obstacle detection

## Code Structure

### 1. Water Extraction System (`codesnip1.cpp`)
**Arduino-based atmospheric water generator control system**

- **Sensors**: DHT22 (humidity/temperature), water level sensor
- **Actuators**: Cooling system (PWM control), water pump, status LED
- **Features**:
  - Real-time environmental monitoring
  - Adaptive cooling power based on humidity/temperature
  - Water collection statistics and EEPROM storage
  - LCD display with real-time status
  - Button control for system activation

**Hardware Requirements**:
- Arduino Uno/Mega
- DHT22 sensor
- I2C LCD display (16x2)
- Relay module for cooling system
- Water level sensor
- 12V water pump

### 2. Solar Energy Management (`codesnip2.cpp`)
**Arduino-based solar power and battery management system**

- **Sensors**: Voltage dividers, ACS712 current sensors
- **Actuators**: Charge controller, load controller, status indicators
- **Features**:
  - Real-time solar power monitoring
  - LiFePO4 battery management (48V system)
  - Intelligent energy saving modes
  - Power statistics and efficiency tracking
  - Automatic system state management

**Hardware Requirements**:
- Arduino Uno/Mega
- Voltage dividers for 0-50V range
- ACS712 30A current sensors
- I2C LCD display
- Relay modules for charge/load control
- Buzzer for alarms

### 3. Satellite Image Analysis (`codesnip3.py`)
**Python-based machine learning system for water source detection**

- **Technology**: TensorFlow/Keras with ResNet50V2 transfer learning
- **Features**:
  - 8-class land classification (Desert, Water Body, Forest, etc.)
  - Water harvesting potential scoring
  - Intelligent recommendations system
  - Batch processing capabilities
  - Visualization and reporting
  - Model training and fine-tuning

**Dependencies**: See `requirements.txt`

### 4. Navigation System (`codesnip4.py`)
**Python-based autonomous navigation with LIDAR obstacle detection**

- **Sensors**: LIDAR (RPLIDAR A2 compatible)
- **Features**:
  - Real-time obstacle detection and clustering
  - Path planning with obstacle avoidance
  - Autonomous navigation to goals
  - Performance metrics and logging
  - Simulation mode for testing
  - Visualization tools

**Dependencies**: See `requirements.txt`

## Installation & Setup

### Prerequisites

1. **Python Environment** (for codesnip3.py and codesnip4.py):
   ```bash
   python --version  # Python 3.8+
   pip --version     # pip 20.0+
   ```

2. **Arduino IDE** (for codesnip1.cpp and codesnip2.cpp):
   - Download from [arduino.cc](https://www.arduino.cc/en/software)
   - Install required libraries (see Hardware Setup)

### Python Setup

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd IRIS-NYAS-Robot-Code-main
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Verify installation**:
   ```bash
   python -c "import tensorflow, cv2, numpy; print('All dependencies installed successfully!')"
   ```

### Arduino Setup

1. **Install required libraries**:
   - DHT sensor library
   - LiquidCrystal I2C
   - Wire (built-in)
   - EEPROM (built-in)

2. **Hardware connections**: See individual code files for pin definitions

## Usage Examples

### Water Extraction System

```cpp
// Upload codesnip1.cpp to Arduino
// System automatically starts water extraction cycles
// Monitor via Serial Monitor (9600 baud)
// Use button to toggle system on/off
```

### Solar Energy Management

```cpp
// Upload codesnip2.cpp to Arduino
// System automatically manages solar charging
// Monitor power statistics via Serial Monitor
// Use button to toggle energy saving mode
```

### Satellite Image Analysis

```bash
# Single image analysis
python codesnip3.py --image satellite_image.jpg --output results.json --visualize

# Batch analysis
python codesnip3.py --directory ./satellite_images/ --output batch_results.json

# Train new model
python codesnip3.py --train ./training_data/ --epochs 20
```

### Navigation System

```bash
# Basic navigation test
python codesnip4.py --goal-x 5.0 --goal-y 3.0 --simulation

# With visualization
python codesnip4.py --goal-x 5.0 --goal-y 3.0 --simulation --visualize
```

## Configuration

### Water Extraction System
- Adjust `HUMIDITY_THRESHOLD` and `TEMP_THRESHOLD` for different climates
- Modify `CYCLE_INTERVAL` for extraction frequency
- Configure pin assignments as needed

### Solar Energy Management
- Update `BATTERY_FULL_VOLTAGE` and `BATTERY_EMPTY_VOLTAGE` for your battery
- Adjust `SOLAR_MIN_POWER` for charging threshold
- Configure voltage divider ratios for your solar panel

### Satellite Image Analysis
- Modify `class_names` for different land classifications
- Adjust `confidence_threshold` for detection sensitivity
- Update model architecture in `create_model()` method

### Navigation System
- Configure `robot_radius` and `safety_margin` for your robot
- Adjust `look_ahead_distance` for path planning
- Modify velocity limits in `NavigationCommand`

## Performance & Monitoring

### Water Extraction
- Water collection efficiency tracking
- Environmental condition monitoring
- System uptime statistics

### Solar Energy
- Power efficiency calculations
- Battery health monitoring
- Charge cycle tracking

### Satellite Analysis
- Classification confidence scores
- Water harvesting potential ratings
- Batch processing performance

### Navigation
- Obstacle detection accuracy
- Path planning efficiency
- Collision avoidance metrics

## Testing

### Python Code Testing
```bash
# Run basic tests
python -m pytest tests/

# Test individual modules
python codesnip3.py --image test_image.jpg
python codesnip4.py --goal-x 2.0 --goal-y 2.0 --simulation
```

### Arduino Code Testing
- Use Serial Monitor for real-time feedback
- Test individual sensors before full system integration
- Verify pin connections with simple test sketches

## Safety Features

- **Water System**: Storage capacity monitoring, sensor validation
- **Solar System**: Battery protection, overcharge prevention
- **Navigation**: Collision avoidance, emergency stop capabilities
- **General**: Error handling, logging, system state monitoring

## Future Enhancements

- **Water System**: Advanced filtration, water quality monitoring
- **Solar System**: MPPT optimization, weather prediction integration
- **Satellite Analysis**: Real-time satellite data integration, seasonal analysis
- **Navigation**: Advanced path planning algorithms, multi-robot coordination

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes with proper testing
4. Submit a pull request with detailed description

## License

This project is part of the IRIS robot development for the NYAS Climate Change Challenge.

## Support

For technical support or questions:
- Check the code comments for detailed explanations
- Review the logging output for debugging information
- Ensure all hardware connections are correct
- Verify sensor calibrations and configurations

## Related Resources

- [IRIS Robot Specifications](link-to-specs)
- [Hardware Assembly Guide](link-to-assembly)
- [API Documentation](link-to-api)
- [Troubleshooting Guide](link-to-troubleshooting)

---

**Note**: This code is designed for educational and research purposes. Always test thoroughly in controlled environments before deployment in real-world applications.
