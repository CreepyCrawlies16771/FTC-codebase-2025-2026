# Robot Tuning Guide

*For detailed information on hardware optimization, see [Robot Tuning Guide in GitBook](../docs/guides/robot-tuning-guide.md)*

This is a quick reference for robot hardware tuning. The comprehensive guide includes:

- Motor direction verification
- Motor speed calibration
- Weight distribution checks
- Servo range mapping
- Sensor verification
- Power management

## Quick Start

1. **Verify Motor Directions** - Ensure all motors spin the correct direction
2. **Calibrate Motor Speed** - Confirm all motors have similar response to power commands
3. **Test Straight Movement** - Drive in a line to check for drift
4. **Map Servo Ranges** - Find safe operating positions for all servos
5. **Verify Sensors** - Confirm sensors provide accurate readings

## Common Motor Issues

- **Robot drifts left/right**: Check motor speeds and weight distribution
- **Jerky movement**: Motors may have different speeds; recalibrate
- **No movement**: Check power connections and motor orientation

## Common Servo Issues

- **Servo grinding**: Servo may be at its limit; adjust command range
- **Slow response**: Check servo power connections; may be underpowered

See the complete [Robot Tuning Guide](../docs/guides/robot-tuning-guide.md) for detailed steps and troubleshooting.
