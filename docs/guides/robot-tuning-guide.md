# Robot Tuning Guide - Hardware Optimization

This guide helps you optimize your robot's hardware behavior for better performance and reliability.

## Overview

Robot tuning focuses on adjusting how your physical components (motors, servos, and sensors) respond to commands from the Control Hub. Proper tuning ensures consistent, predictable robot behavior.

## Motor Tuning

### Step 1: Motor Direction Verification
Before any complex movements, ensure all motors spin in the correct direction.

**What to do:**
1. Set all motors to the same power level (start with 50%)
2. Verify each motor spins the expected direction
3. Use the `setDirection()` method if a motor needs reversal
4. Test both forward and reverse directions

**Why it matters:** Incorrect motor direction causes unpredictable movement and failed autonomous paths.

### Step 2: Motor Speed Calibration
Calibrate motors to respond consistently to the same power commands.

**What to do:**
1. Set each motor to 100% power individually
2. Measure actual motor RPM or wheel speed
3. Document the results for comparison
4. Identify any motors that are significantly slower (possible mechanical issues)
5. If all motors perform similarly, your calibration is good

**Why it matters:** Consistent motor response makes predictable movement possible.

### Step 3: Weight Distribution Check
Verify that your robot moves in a straight line when both drive motors receive equal power.

**What to do:**
1. Place robot on a smooth, clear surface
2. Run both drive motors at the same power for 2-3 seconds
3. Observe if the robot drifts left or right
4. Minor adjustments to motor power can compensate for weight distribution

**Why it matters:** Straight-line movement is essential for autonomous navigation.

## Servo Tuning

### Step 1: Servo Range Mapping
Identify the safe operational range for each servo to prevent damage.

**What to do:**
1. Move each servo slowly from 0.0 (fully closed) to 1.0 (fully open)
2. Listen for grinding sounds or resistance
3. Note any positions where the servo stalls or makes unusual sounds
4. Adjust servo commands to stay within safe ranges
5. Document the safe min/max values for your servos

**Why it matters:** Staying within safe ranges prevents mechanical damage and extends servo lifespan.

### Step 2: Servo Response Timing
Test how quickly servos respond to ensure smooth gameplay.

**What to do:**
1. Command a servo to a new position
2. Observe the time it takes to reach that position
3. Test at different power levels if your servo supports variable speed
4. Account for servo response time in your program logic

**Why it matters:** Accurate timing prevents jamming and ensures game element handling works reliably.

## Sensor Verification

### Step 1: Sensor Calibration
Verify that sensors provide accurate readings.

**What to do:**
1. For distance sensors: test at known distances and verify readings
2. For color sensors: test with game elements and verify color detection
3. For gyros/IMU: verify orientation sensors read zero when robot is stationary and oriented correctly
4. Document sensor accuracy ranges

**Why it matters:** Accurate sensor data is critical for autonomous programs and gameplay decisions.

### Step 2: Sensor Noise Testing
Check if sensors have excessive noise that could cause false readings.

**What to do:**
1. Place sensor in stable position
2. Read sensor value 10-20 times in quick succession
3. If values vary significantly, investigate:
   - Wiring connections
   - Sensor alignment
   - Environmental interference (lighting, interference)
4. Add noise filters if needed

**Why it matters:** Noise-free sensor readings prevent erratic autonomous behavior.

## Power Management

### Step 1: Power Supply Check
Verify your battery is providing adequate power.

**What to do:**
1. Monitor voltage while running all motors at full power
2. Voltage should stay above 11V under load
3. If voltage drops significantly, the battery may need replacement
4. Check all connections are tight

**Why it matters:** Low power causes unpredictable motor behavior and sensor failures.

### Step 2: Current Draw Assessment
Monitor how much current your system is drawing.

**What to do:**
1. Check the Control Hub's current display
2. Normal idle: < 1A
3. With all motors at 50%: 5-10A typically
4. If unexpectedly high, investigate for mechanical friction or short circuits

**Why it matters:** Excessive current draw indicates problems that could damage hardware.

## Testing & Validation

### Quick Test Procedure
After tuning changes, run this sequence:

1. **Idle Test**: Start robot, let it sit for 10 seconds - ensure no unexpected movement
2. **Manual Test**: Use driver station to command each subsystem independently
3. **Movement Test**: Drive the robot forward, backward, and rotate - ensure smooth, straight movement
4. **Sensor Test**: Verify all sensors return reasonable values
5. **Full System Test**: Run a complete autonomous routine if available

### Documentation Template

Create a tuning log for your team:

```
Date: [Date]
Hardware: [Motors, servos, sensors used]
Changes Made: [What was adjusted]
Testing Results: [Pass/Fail observations]
Next Steps: [What needs attention]
```

## Common Issues & Quick Fixes

| Issue | Likely Cause | Quick Fix |
|-------|-------------|----------|
| Robot drifts left/right | Weight imbalance or motor speed difference | Adjust motor power or inspect for mechanical issues |
| Servo grinds or stutters | Servo at physical limit or overloaded | Check servo range; reduce load or speed |
| Jerky/unpredictable movement | Inconsistent motor speeds | Recalibrate motors; check for friction |
| Sensor reading errors | Wiring or sensor misalignment | Verify connections; reseat sensor |
| Battery drains quickly | High current draw | Check for mechanical friction; reduce motor power |

## Next Steps

- **For Software Tuning**: See [General Tuning Guide](general-tuning-guide.md)
- **For Autonomous Programming**: Refer to official FTC documentation
- **For Advanced Optimization**: Use telemetry to monitor performance during matches
