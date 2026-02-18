# General Tuning Guide - Software Optimization

This guide helps you optimize your robot's software behavior for better responsiveness, accuracy, and autonomous performance.

## Overview

General tuning focuses on adjusting software parameters that control how your robot responds to commands. Unlike robot tuning (which adjusts hardware), general tuning improves the control algorithms and system behavior.

## Motor Control Parameters

### Step 1: Power Scaling
Adjust how the driver station gamepad values map to motor power.

**What to do:**
1. Test how the robot responds to trigger values on the gamepad
2. If the robot feels "dead" at low power (not responsive), increase power scaling
3. If the robot accelerates too aggressively from a slight trigger pull, decrease power scaling
4. Aim for smooth, predictable power delivery across the full range

**Why it matters:** Proper power scaling makes driving intuitive and responsive.

### Step 2: Acceleration Control
Set how quickly motors accelerate from one speed to another.

**What to do:**
1. Observe how the robot responds when driver quickly changes power levels
2. If acceleration feels jerky or causes the robot to tip, slow it down slightly
3. If acceleration feels sluggish, increase it slightly
4. Test on the actual competition surface if possible

**Why it matters:** Controlled acceleration prevents mechanical stress and improves handling stability.

### Step 3: Deadzone Configuration
Set the minimum gamepad input required to register movement.

**What to do:**
1. With the gamepad at rest, check if the robot drifts
2. If drift occurs, increase the deadzone to require more input before moving
3. If you need very fine control and increase deadzone too much, the robot becomes unresponsive
4. Typical deadzone: 5-10% of full range

**Why it matters:** Proper deadzones prevent unintended drift from controller stick noise.

## Navigation Tuning

### Step 1: Turning Speed
Optimize how quickly the robot rotates in place.

**What to do:**
1. Command the robot to turn 90 degrees
2. If turning is too slow, increase turn command power
3. If turning overshoots or feels unstable, decrease power
4. Test multiple rotations to ensure consistency

**Why it matters:** Consistent turning improves autonomous accuracy and navigation reliability.

### Step 2: Driving Speed Profile
Adjust maximum speed for different conditions.

**What to do:**
1. Establish a maximum safe speed on your practice field
2. For autonomous: faster isn't always better; prioritize accuracy over speed
3. For teleop: find the speed that drivers feel most comfortable with
4. Consider game element handling constraints

**Why it matters:** Finding the right speed balance improves success rates in matches.

### Step 3: Stopping Distance
Calibrate how quickly the robot comes to a complete stop.

**What to do:**
1. Command robot to maximum speed, then immediately to zero power
2. Measure how far the robot travels before stopping (stopping distance)
3. If stopping distance is too large, you may need to:
   - Apply braking (negative power) briefly
   - Reduce max speed
   - Check wheel friction/surface
4. Aim for predictable, consistent stopping distance

**Why it matters:** Accurate stopping distance is essential for positioning game elements and parking.

## Sensor-Based Tuning

### Step 1: Sensor Response Time
Adjust timing to account for sensor lag.

**What to do:**
1. Read a sensor value repeatedly in a tight loop and measure latency
2. Account for this delay in autonomous routines
3. For fast movements, add small delays before acting on sensor data
4. For autonomous navigation, factor this into heading corrections

**Why it matters:** Accounting for sensor lag prevents overshoot and improves accuracy.

### Step 2: Sensor Thresholds
Calibrate when sensors trigger autonomous actions.

**What to do:**
1. Test your sensor at the exact game situation positions
2. Set threshold values based on real conditions
3. Add small margins of error (e.g., ±5% on distance)
4. Document threshold values for consistency

**Why it matters:** Accurate thresholds ensure reliable autonomous decision-making.

## Autonomous Routine Tuning

### Step 1: Movement Timing
Adjust how long movements take to account for real-world conditions.

**What to do:**
1. Command robot to drive forward for a set time
2. Measure actual distance traveled
3. If distance doesn't match expectations, adjust:
   - Motor power levels
   - Time duration
   - Hall effect encoder values (if available)
4. Repeat until predictable

**Why it matters:** Predictable movement timing is essential for reliable autonomous programs.

### Step 2: Action Sequencing
Optimize the order and timing of autonomous actions.

**What to do:**
1. Map out all autonomous actions your robot must perform
2. Test sequences individually first
3. Combine sequences and verify timing works correctly
4. Add small safety delays between critical actions
5. Use telemetry to monitor timing during execution

**Why it matters:** Proper sequencing prevents collisions and missed game element pickups.

### Step 3: Error Recovery
Add simple robustness to handle variations.

**What to do:**
1. Identify critical points where the robot might fail
2. Add basic recovery actions (e.g., retry movements, small corrections)
3. Keep recovery simple - complexity causes more problems
4. Test recovery paths separately

**Why it matters:** Simple error handling makes autonomous more reliable.

## Testing & Validation

### Tuning Test Checklist

- [ ] Power scaling feels responsive and smooth
- [ ] No unintended drift with neutral controls
- [ ] Turning is consistent and accurate
- [ ] Stopping distance is predictable
- [ ] Autonomous movements have consistent timing
- [ ] Sensor readings trigger reliably
- [ ] All test runs behave similarly (no random failures)

### Performance Metrics to Track

Create a simple test log:

```
Test Session: [Date/Time]
Field Condition: [Practice/Competition/Indoor/Outdoor]
Motor Power Settings: [As configured]
Tuning Changes: [What was adjusted]

Test Results:
  Forward Drive (10 feet): _____ feet/seconds
  Turning Accuracy (90°): _____ degrees variance
  Autonomous Run Time: _____ seconds
  Success Rate: _____ / _____ attempts

Notes: [Observations and next adjustments]
```

## Common Issues & Solutions

| Issue | Likely Cause | Quick Solution |
|-------|-------------|----------------|
| Robot unresponsive to light stick input | Deadzone too large | Decrease deadzone or increase power scaling |
| Constant drift even with stick centered | Stick noise or deadzone too small | Increase deadzone or check gamepad calibration |
| Autonomous overshoots target position | Movement timing too aggressive | Reduce motor power or increase brake time |
| Inconsistent autonomous results | Environmental variation (surface, battery) | Standardize practices; use more frequent sensor corrections |
| Servo follows wrong sequence | Timing issues between actions | Add explicit delays between servo commands |
| Autonomous turns wrong angle | Sensor drift or calculation error | Re-verify sensor calibration; simplify turning logic |

## Simple Optimization Tips

1. **Start Conservative**: Begin with reduced speeds and power; increase gradually
2. **One Change at a Time**: Only adjust one parameter per test session
3. **Document Everything**: Keep notes on what works and what doesn't
4. **Use Telemetry**: Print sensor values and timing data to debug issues
5. **Practice Consistently**: Tune on the same field/surface when possible
6. **Test Before Matches**: Never introduce new tuning changes the day of competition

## Next Steps

- **Back to Hardware**: See [Robot Tuning Guide](robot-tuning-guide.md) if hardware issues arise
- **Advanced Programming**: Visit [FIRST Tech Challenge Docs](https://ftc-docs.firstinspires.org/) for detailed API reference
- **Performance Monitoring**: Use telemetry during matches to identify remaining issues
