# Quintic Motion Profiling for LemLib

This implementation adds quintic (5th-order polynomial) motion profiling to your robot's movement functions. Quintic profiles provide smoother motion with continuous velocity, acceleration, and jerk compared to traditional PID control.

## What is Quintic Motion Profiling?

A quintic trajectory uses a 5th-order polynomial to generate smooth motion paths. The main benefits are:

- **Continuous jerk**: Smoother acceleration changes reduce mechanical stress and vibration
- **Zero terminal velocity**: Robot naturally comes to a smooth stop at the target
- **Zero terminal acceleration**: No sudden deceleration at the end of motion
- **Predictable timing**: Motion duration is precisely controlled

The mathematical form is:
```
s(t) = 10τ³ - 15τ⁴ + 6τ⁵
```
where τ = t/T (normalized time from 0 to 1)

## Files Added

- `include/quintic_profile.hpp` - Core quintic profile math
- `src/quintic_profile.cpp` - Profile implementation
- `include/quintic_motion.hpp` - Robot motion functions
- `src/quintic_motion.cpp` - Motion control implementation

## Usage

### Basic moveToPoint

Replace lemlib's `moveToPoint` with the quintic version:

```cpp
// Old lemlib way:
chassis.moveToPoint(20, 15, 4000);

// New quintic way:
quintic::moveToPoint(chassis, 20, 15, 4000);
```

### Basic moveToPose

Replace lemlib's `moveToPose` with the quintic version:

```cpp
// Old lemlib way:
chassis.moveToPose(20, 15, 90, 4000);

// New quintic way:
quintic::moveToPose(chassis, 20, 15, 90, 4000);
```

**Note:** By default, the quintic functions automatically use your chassis's existing lateral and angular PID gains from `lateral_controller` and `angular_controller` (defined in [drive.cpp](src/drive.cpp)). This means they start pre-tuned to your robot! You only need to specify custom gains if you want different behavior than your standard chassis tuning.

### Advanced Examples

#### Move backwards (uses chassis default gains):
```cpp
quintic::moveToPoint(chassis, -30, 40, 3000, {
    .forwards = false
    // lateralKP, lateralKD, angularKP, angularKD automatically use chassis defaults
});
```

#### Move with custom PID gains (override defaults):
```cpp
quintic::moveToPoint(chassis, 10, 10, 2000, {
    .lateralKP = 8.0f,   // Override chassis lateral kP
    .angularKP = 4.0f    // Override chassis angular kP
    // lateralKD and angularKD still use chassis defaults
});
```

#### Move to pose with aggressive turning:
```cpp
quintic::moveToPose(chassis, 10, 10, 45, 2000, {
    .lead = 0.8f  // Higher = more aggressive turning during motion
    // All PID gains use chassis defaults
});
```

#### Synchronous (blocking) movement:
```cpp
// Wait for movement to complete before continuing
quintic::moveToPoint(chassis, 15, 20, 3000, {
    .async = false
});
// This line runs after movement completes
```

#### Custom settling parameters:
```cpp
quintic::moveToPose(chassis, 20, 15, 90, 4000, {
    .settleRange = 0.5f,      // Must be within 0.5 inches
    .settleAngle = 1.0f,      // Must be within 1 degree
    .settleTimeout = 500      // Must settle for 500ms
});
```

## Parameters

### QuinticMoveToPointParams

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `forwards` | bool | true | Move forwards (true) or backwards (false) |
| `async` | bool | true | Run asynchronously in background task |
| `lateralKP` | float | chassis.lateral.kP | Proportional gain for position tracking (uses chassis settings) |
| `lateralKD` | float | chassis.lateral.kD | Derivative gain for position tracking (uses chassis settings) |
| `angularKP` | float | chassis.angular.kP | Proportional gain for heading correction (uses chassis settings) |
| `angularKD` | float | chassis.angular.kD | Derivative gain for heading correction (uses chassis settings) |
| `settleRange` | float | 1.0 | Distance tolerance (inches) for completion |
| `settleTimeout` | int | 250 | Time (ms) to remain settled before completion |

### QuinticMoveToPoseParams

All parameters from `QuinticMoveToPointParams` plus:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `settleAngle` | float | 2.0 | Angle tolerance (degrees) for completion |
| `lead` | float | 0.6 | Turn aggressiveness (0.0 = gradual, 1.0 = aggressive) |

## Tuning Guide

### Understanding Default Gains

The quintic functions automatically inherit your chassis PID gains:
- **lateralKP/KD** use your `lateral_controller` gains (kP and kD)
- **angularKP/KD** use your `angular_controller` gains (kP and kD)

This means if your chassis is already well-tuned for standard movements, the quintic functions should work reasonably well out of the box!

**To override defaults**, simply specify the parameter:
```cpp
// Use chassis defaults except override lateralKP
quintic::moveToPoint(chassis, 20, 15, 4000, {
    .lateralKP = 8.0f  // Override default
    // lateralKD, angularKP, angularKD still use chassis defaults
});
```

### Common Tuning Scenarios

### If robot overshoots target:
- Decrease `lateralKP` (try 3.0-4.0)
- Increase `lateralKD` (try 1.0-3.0)
- Increase `settleRange` tolerance

### If robot oscillates/wobbles:
- Decrease `angularKP` (try 2.0-2.5)
- Increase `angularKD` (try 0.5-2.0)
- Decrease `lead` factor for moveToPose

### If robot is too slow:
- Increase `lateralKP` (try 7.0-10.0)
- Increase `angularKP` (try 4.0-6.0)
- Increase timeout value

### If robot doesn't reach target:
- Increase timeout
- Decrease `settleRange` and `settleAngle`
- Decrease `settleTimeout`

## Example Autonomous Routine

```cpp
void autonomous() {
    // Set starting position
    chassis.setPose(0, 0, 0);
    
    // Move to first position with quintic profiling
    quintic::moveToPoint(chassis, 24, 24, 2000, {
        .lateralKP = 6.0f,
        .angularKP = 4.0f
    });
    
    // Turn to face next target
    chassis.turnToHeading(90, 1000);
    
    // Move to pose with smooth turning
    quintic::moveToPose(chassis, 48, 24, 180, 3000, {
        .lead = 0.7f,
        .lateralKP = 5.5f
    });
    
    // Move backwards to goal
    quintic::moveToPoint(chassis, 12, 12, 2000, {
        .forwards = false,
        .lateralKP = 7.0f
    });
}
```

## Comparison with LemLib

### LemLib (PID + Slew):
- Uses PID control with slew rate limiting
- Can have acceleration discontinuities
- Requires tuning of multiple parameters (kP, kI, kD, slew, etc.)
- Speed varies based on error

### Quintic Profiling:
- Generates smooth velocity/acceleration curves
- Continuous jerk for minimal vibration
- Simpler parameter tuning (mainly just feedback gains)
- Predictable motion timing
- Better for precise movements

## Technical Details

The quintic profile ensures:
1. **Position**: s(0) = 0, s(T) = L
2. **Velocity**: v(0) = 0, v(T) = 0
3. **Acceleration**: a(0) = 0, a(T) = 0

Where T is duration and L is distance. This creates the smoothest possible motion profile with these constraints.

The implementation uses:
- **Feedforward control**: Based on desired velocity from the profile
- **Feedback control**: PD control to correct for disturbances
- **Trajectory following**: Robot tracks desired position over timeFor `moveToPose`, the heading smoothly transitions from path-following to the target heading using the `lead` parameter.

## Notes

- Functions are in the `quintic::` namespace to avoid conflicts with lemlib
- Async mode creates a background task that doesn't block your code
- The profile uses 80% of the timeout for motion, 20% for settling
- Motor commands are clamped to ±127 for safety

## Future Improvements

Potential enhancements:
- Trapezoidal velocity limits for constraint satisfaction
- Dynamic obstacle avoidance
- Velocity-based feedforward tuning
- Multi-segment trajectories
- S-curve profiling options
