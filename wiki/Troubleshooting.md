# Troubleshooting Guide

## Build & Deployment Issues

### Project Won't Sync
- **Solution**: Go to Tools → SDK Manager and update Android SDK platforms
- Check that your Android SDK is properly configured in project settings

### Device Not Recognized
- **Solution**: Check USB cable connection; try different USB port
- Install/update USB drivers for your OS
- Enable USB debugging on your Control Hub

### Build Fails with Gradle Errors
- **Solution**: 
  1. Delete the `.gradle` folder in your project
  2. Delete `build/` directories
  3. Click "Sync Now" to rebuild from scratch

### Out of Memory Errors
- **Solution**: Increase Gradle heap size in `gradle.properties`
  ```
  org.gradle.jvmargs=-Xmx2048m
  ```

## Runtime Issues

### Robot Doesn't Respond to Commands
1. Check Control Hub is running the FTC app
2. Verify Driver Station is connected (check WiFi connection)
3. Review program logs for errors
4. Restart both Control Hub and Driver Station

### Unexpected Motor Behavior
- Verify motor directions in code
- Check power connections to Control Hub
- Ensure motor encoder cables are secure

### Sensor Readings are Incorrect
- Verify sensor physical connections
- Test sensor in isolation with simple code
- Check sensor orientation/calibration
- Review telemetry for actual values vs. expected

### Autonomous Doesn't Complete
- Add telemetry output to debug which step fails
- Verify sensor thresholds are achievable
- Check timing for action sequences
- Test movement distances independently

## Hardware Issues

### Motors Overheating
- **Cause**: Running at excessive power for too long
- **Solution**: Reduce motor power levels; add cooling time between actions

### Battery Drains Too Quickly
- **Cause**: High current draw (usually from motors)
- **Solution**: Reduce motor power; check for mechanical friction in robot

### Servo Grinding or Stalling
- **Cause**: Servo at mechanical limit or overloaded
- **Solution**: Reduce servo speed; check load is within servo specs

### Sensor Not Responding
- **Cause**: Loose wiring or power issue
- **Solution**: Reseat connectors; verify Control Hub power is stable

## Performance Issues

### Inconsistent Autonomous Results
- Environmental variation (field surface, battery state)
- Add telemetry to monitor actual sensor values
- Use sensor feedback instead of relying on timing alone
- Standardize practice conditions (same field, battery charge)

### Delayed Response to Driver Input
- Check WiFi signal strength
- Reduce program complexity
- Close unnecessary telemetry windows
- Review code for blocking operations

### Program Crashes During Execution
- Add try-catch blocks to catch exceptions
- Use telemetry to monitor before crash
- Check for null pointer exceptions
- Verify array access is within bounds

## Getting Help

### Debugging Steps
1. Check telemetry output for error messages
2. Add strategic telemetry prints to track execution
3. Test components individually before combining
4. Review logs from previous successful runs

### Resources
- [Official FTC Documentation](https://ftc-docs.firstinspires.org/)
- [GitHub Project Issues](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues)
- [FIRST Community Forum](https://www.firstinspires.org/community/technical-support)

## Quick Reference

### USB Connection Troubleshooting
- Verify USB cable is data cable (not charge-only)
- Try different USB port
- Restart Control Hub
- Clear Android Studio device cache

### WiFi Connection Issues
- Ensure Control Hub and Driver Station are on same network
- Check for "Adhoc" or "Auto-Hotspot" setting
- Verify firewall isn't blocking FTC app
- Restart both devices

### Code Compilation Errors
- Check for typos in class/method names
- Verify imports are correct
- Check that methods match the FTC SDK API
- Update FTC SDK library version if needed
