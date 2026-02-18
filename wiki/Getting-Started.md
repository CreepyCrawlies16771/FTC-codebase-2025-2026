# Getting Started - Setup Guide

This guide walks you through getting your FTC Robot Controller up and running.

## Prerequisites

Before you begin, ensure you have:

- Android Studio Ladybug (2024.2) or later installed
- Java Development Kit (JDK) 11 or higher
- Git installed (for cloning the repository)
- A valid FTC Control Hub or compatible Android device
- USB cable for device connection

## Installation Steps

### Step 1: Clone or Download the Repository

**Option A: Using Git (Recommended)**
```bash
git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController.git
cd FtcRobotController
```

**Option B: Download ZIP**
1. Visit the [GitHub repository](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
2. Click "Code" → "Download ZIP"
3. Extract the ZIP file to your preferred location

### Step 2: Open Project in Android Studio

1. Launch Android Studio
2. Select "File" → "Open"
3. Navigate to the FtcRobotController folder
4. Click "OK" and wait for project sync to complete

### Step 3: Configure Your Build

1. Update `local.properties` with your Android SDK path (if needed)
2. Allow Gradle to sync and download dependencies
3. This may take several minutes on first setup

### Step 4: Connect Your Control Hub

1. Connect your Control Hub via USB cable to your development PC
2. Enable developer mode on the Control Hub
3. In Android Studio, select your device from the device dropdown

### Step 5: Run the App

1. Select "Run" → "Run 'FtcRobotController'" or press Shift+F10
2. Android Studio will build and deploy the app to your Control Hub
3. Wait for the build to complete

## Project Structure Overview

```
FtcRobotController/
├── FtcRobotController/        # Core Control Hub app
├── TeamCode/                   # Your team's code goes here
├── docs/                       # Documentation files
├── build.gradle               # Build configuration
└── settings.gradle            # Project settings
```

### FtcRobotController Folder
Contains the main control system that manages motors, servos, and sensors.

### TeamCode Folder
This is where you write your team's OpMode programs and custom code.

## Creating Your First Program

Programs in FTC are called "OpModes" (Operational Modes). Start in the `TeamCode/src/main/java` folder:

1. Create a new Java class
2. Extend either `LinearOpMode` (sequential) or `OpMode` (iterative)
3. Add your robot logic
4. Register the class with the `@TeleOp` or `@Autonomous` annotation

## Testing Your Setup

Once you've deployed the app:

1. Open the FTC Robot Controller app on your Control Hub
2. Connect a Driver Station device
3. You should see the team's configured OpModes
4. Select an OpMode and verify it runs

## Troubleshooting Initial Setup

| Problem | Solution |
|---------|----------|
| Project won't sync | Update Android SDK: Tools → SDK Manager → Install latest |
| Device not recognized | Install/update USB drivers; check USB cable |
| Build fails | Clean project: Build → Clean Project, then rebuild |
| Gradle issues | Delete `.gradle` folder and resync project |

## Next Steps

- **Build Your First Robot**: Start with teleoperated (TeleOp) programs first
- **Hardware Setup**: See [Robot Tuning Guide](Robot-Tuning-Guide) for motor and sensor configuration
- **Software Optimization**: Use [General Tuning Guide](General-Tuning-Guide) to improve performance
- **Official Docs**: Visit [ftc-docs.firstinspires.org](https://ftc-docs.firstinspires.org/) for detailed API reference

## Need Help?

- Check your build.gradle for dependency issues
- Verify your Java version with `java -version`
- Review the FTC official documentation for specific API questions
- Post on the [FIRST Community Forum](https://www.firstinspires.org/community/technical-support)
