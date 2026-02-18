# Introduction to FTC Robot Control System

## Overview

The FTC (FIRST Tech Challenge) Robot Controller is the core software that manages your robot's behavior during matches. It runs on the Control Hub (an Android-based device) and communicates with motors, servos, sensors, and other hardware components.

## System Architecture

```
┌─────────────────────────────────────┐
│     Driver Station (Wireless)       │
│  (Operated by driver via gamepad)   │
└────────────────┬────────────────────┘
                 │ WiFi
                 │
┌────────────────▼────────────────────┐
│       Control Hub                   │
│  - Main Processor                   │
│  - Motor/Servo Controllers          │
│  - Sensor Interfaces                │
│  - Navigation Systems               │
└────────────┬──────────────┬─────────┘
             │              │
    ┌────────▼──┐    ┌─────▼────────┐
    │  Motors   │    │   Sensors    │
    │  Servos   │    │   (Camera,   │
    │           │    │    IMU, etc) │
    └───────────┘    └──────────────┘
```

## Key Components

### Control Hub
Your robot's "brain" - runs the FTC Robot Controller app and executes your programs.

### Motors & Servos
Actuators controlled by the Control Hub to move your robot and manipulate game elements.

### Sensors
Input devices (cameras, distance sensors, gyros) that provide information about your robot's state and environment.

### Driver Station
The operator interface where drivers control the robot using gamepads during matches.

## How Tuning Helps

Tuning is the process of adjusting parameters to get optimal performance:

- **Robot Tuning** - Adjusts how hardware responds (motor speed, servo positioning)
- **General Tuning** - Adjusts how software controls movement (acceleration, turning, autonomy behavior)

Both types work together to make your robot responsive, accurate, and reliable.

## Next Steps

- **For Hardware Setup**: See [Getting Started Guide](../guides/robot-tuning-guide.md)
- **For Software Configuration**: See [General Tuning Guide](../guides/general-tuning-guide.md)
- **For Detailed Reference**: Visit [Official FTC Documentation](https://ftc-docs.firstinspires.org/)
