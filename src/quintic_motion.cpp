#include "quintic_motion.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <algorithm>

namespace quintic {

// Helper function to normalize angle to [-180, 180]
static float normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// Helper function to calculate angular error
static float angleError(float current, float target) {
    return normalizeAngle(target - current);
}

// Convert motor voltage (-127 to 127) to ensure it's in valid range
static int clampMotorVoltage(float voltage) {
    return std::max(-127, std::min(127, static_cast<int>(voltage)));
}

// Helper function to apply chassis PID defaults to moveToPoint params
static void applyChassisDefaults(QuinticMoveToPointParams& params) {
    // Replace sentinel values (-1.0f) with chassis PID gains
    if (params.lateralKP < 0.0f) params.lateralKP = lateral_controller.kP;
    if (params.lateralKD < 0.0f) params.lateralKD = lateral_controller.kD;
    if (params.angularKP < 0.0f) params.angularKP = angular_controller.kP;
    if (params.angularKD < 0.0f) params.angularKD = angular_controller.kD;
}

// Helper function to apply chassis PID defaults to moveToPose params
static void applyChassisDefaults(QuinticMoveToPoseParams& params) {
    // Replace sentinel values (-1.0f) with chassis PID gains
    if (params.lateralKP < 0.0f) params.lateralKP = lateral_controller.kP;
    if (params.lateralKD < 0.0f) params.lateralKD = lateral_controller.kD;
    if (params.angularKP < 0.0f) params.angularKP = angular_controller.kP;
    if (params.angularKD < 0.0f) params.angularKD = angular_controller.kD;
}

// Task function for async moveToPoint
static void moveToPointTask(void* param);

// Task function for async moveToPose  
static void moveToPoseTask(void* param);

// Struct to pass parameters to async tasks
struct MoveToPointTaskParams {
    lemlib::Chassis* chassis;
    float x;
    float y;
    int timeout;
    QuinticMoveToPointParams params;
};

struct MoveToPoseTaskParams {
    lemlib::Chassis* chassis;
    float x;
    float y;
    float theta;
    int timeout;
    QuinticMoveToPoseParams params;
};

void moveToPoint(lemlib::Chassis& chassis, float x, float y, int timeout,
                 QuinticMoveToPointParams params) {
    if (params.async) {
        // Create task parameters on heap
        MoveToPointTaskParams* taskParams = new MoveToPointTaskParams{
            &chassis, x, y, timeout, params
        };
        pros::Task task(moveToPointTask, taskParams, "Quintic MoveToPoint");
    } else {
        // Execute synchronously
        MoveToPointTaskParams taskParams = {&chassis, x, y, timeout, params};
        moveToPointTask(&taskParams);
    }
}

void moveToPose(lemlib::Chassis& chassis, float x, float y, float theta, int timeout,
                QuinticMoveToPoseParams params) {
    if (params.async) {
        // Create task parameters on heap
        MoveToPoseTaskParams* taskParams = new MoveToPoseTaskParams{
            &chassis, x, y, theta, timeout, params
        };
        pros::Task task(moveToPoseTask, taskParams, "Quintic MoveToPose");
    } else {
        // Execute synchronously
        MoveToPoseTaskParams taskParams = {&chassis, x, y, theta, timeout, params};
        moveToPoseTask(&taskParams);
    }
}

static void moveToPointTask(void* param) {
    MoveToPointTaskParams* p = static_cast<MoveToPointTaskParams*>(param);
    lemlib::Chassis& chassis = *(p->chassis);
    
    // Apply chassis PID defaults if sentinel values are present
    applyChassisDefaults(p->params);
    
    // Get starting position
    lemlib::Pose startPose = chassis.getPose();
    float startX = startPose.x;
    float startY = startPose.y;
    
    // Calculate distance and trajectory
    float dx = p->x - startX;
    float dy = p->y - startY;
    float distance = std::sqrt(dx * dx + dy * dy);
    
    // If already at target, exit early
    if (distance < 0.5f) {
        if (p->params.async) delete p;
        return;
    }
    
    // Calculate target heading based on direction
    // Convert from standard math convention to VEX convention:
    // Standard: atan2(dy,dx) gives angle from +X axis, counter-clockwise
    // VEX: 0° is North (+Y), increases clockwise
    // Conversion: VEX_heading = 90° - standard_angle
    float targetHeading = 90.0f - (std::atan2(dy, dx) * 180.0f / M_PI);
    targetHeading = normalizeAngle(targetHeading);
    
    // Adjust for backwards motion
    float exitVelocity = p->params.minExitVelocity;
    if (!p->params.forwards) {
        targetHeading = normalizeAngle(targetHeading + 180.0f);
        distance = -distance; // Negative for backwards motion
        exitVelocity = -exitVelocity; // Negative velocity for backwards motion
    }
    
    // Create quintic trajectory
    // Use 80% of timeout for motion, 20% for settling (unless chaining with exit velocity)
    float motionTime = (p->params.minExitVelocity > 0) ? p->timeout * 0.95f : p->timeout * 0.8f;
    QuinticTrajectory2D trajectory(startX, startY, p->x, p->y, motionTime, 0.0f, exitVelocity);
    
    // Control loop variables
    uint32_t startTime = pros::millis();
    float prevLateralError = 0;
    float prevAngularError = 0;
    
    uint32_t settleStartTime = 0;
    bool settling = false;
    
    const int loopDelay = 10; // 10ms loop time
    
    while (true) {
        uint32_t currentTime = pros::millis();
        uint32_t elapsedTime = currentTime - startTime;
        
        // Check timeout
        if (elapsedTime >= p->timeout) break;
        
        // Get current pose
        lemlib::Pose currentPose = chassis.getPose();
        
        // Calculate desired position from trajectory
        float t = std::min(static_cast<float>(elapsedTime), motionTime);
        float desiredX = trajectory.getX(t);
        float desiredY = trajectory.getY(t);
        
        // Calculate error to trajectory point for control
        float errorX = desiredX - currentPose.x;
        float errorY = desiredY - currentPose.y;
        
        // Calculate signed lateral error by projecting along CURRENT heading direction
        // (not target heading - we need error relative to where robot is actually facing)
        // Convert heading from VEX convention (0°=North, clockwise) to standard math (0°=East, CCW)
        float currentHeading = currentPose.theta;
        float headingRad = (90.0f - currentHeading) * M_PI / 180.0f;
        float headingCos = std::cos(headingRad);
        float headingSin = std::sin(headingRad);
        
        // Project error vector onto robot's current heading direction (forward is positive)
        // Note: targetHeading is already flipped 180° when forwards=false, so once robot
        // is facing targetHeading, projection naturally gives correct sign for backward motion
        float signedLateralError = errorX * headingCos + errorY * headingSin;
        
        // Also calculate magnitude for debugging/monitoring
        float trajectoryError = std::sqrt(errorX * errorX + errorY * errorY);
        
        // Calculate error to final target for settling check
        float finalErrorX = p->x - currentPose.x;
        float finalErrorY = p->y - currentPose.y;
        float finalError = std::sqrt(finalErrorX * finalErrorX + finalErrorY * finalErrorY);
        
        // Calculate angular error
        float angularError = angleError(currentHeading, targetHeading);
        
        // Check if we're in settling range (use distance to final target)
        // Skip settling if using exit velocity for motion chaining
        if (p->params.minExitVelocity <= 0 && finalError < p->params.settleRange && std::abs(angularError) < 5.0f) {
            if (!settling) {
                settling = true;
                settleStartTime = currentTime;
            } else if (currentTime - settleStartTime >= p->params.settleTimeout) {
                break; // Successfully settled
            }
        } else {
            settling = false;
        }
        
        // Calculate feedforward from trajectory velocity
        float feedforward = 0;
        if (t < motionTime) {
            // Get desired velocity from trajectory
            float vx = trajectory.getVx(t);
            float vy = trajectory.getVy(t);
            float desiredVelocity = std::sqrt(vx * vx + vy * vy);
            
            // Convert from inches/ms to inches/sec
            desiredVelocity *= 1000.0f;
            
            // Scale based on robot's max velocity to avoid over-powering slow movements
            feedforward = (desiredVelocity / p->params.maxVelocity) * 127.0f;
            
            // Apply minimum clamp to overcome static friction
            if (p->params.kS > 0.0f && std::abs(feedforward) > 0.1f && std::abs(feedforward) < p->params.kS) {
                feedforward = feedforward > 0 ? p->params.kS : -p->params.kS;
            }
            
            if (!p->params.forwards) feedforward = -feedforward;
        }
        
        // PID corrections (use signed lateral error for control)
        float lateralCorrection = p->params.lateralKP * signedLateralError;
        float lateralDerivative = p->params.lateralKD * (signedLateralError - prevLateralError);
        
        float angularCorrection = p->params.angularKP * angularError;
        float angularDerivative = p->params.angularKD * (angularError - prevAngularError);
        
        // Calculate motor voltages
        float forwardPower = feedforward + lateralCorrection + lateralDerivative;
        float turnPower = angularCorrection + angularDerivative;
        
        int leftVoltage = clampMotorVoltage(forwardPower + turnPower);
        int rightVoltage = clampMotorVoltage(forwardPower - turnPower);
        
        // Apply motor voltages using tank control
        chassis.tank(leftVoltage, rightVoltage, true);
        
        // Update previous errors
        prevLateralError = signedLateralError;
        prevAngularError = angularError;
        
        pros::delay(loopDelay);
    }
    
    // Stop motors only if not chaining (exit velocity is 0)
    if (p->params.minExitVelocity <= 0) {
        chassis.tank(0, 0, true);
    }
    
    // Clean up if async
    if (p->params.async) delete p;
}

static void moveToPoseTask(void* param) {
    MoveToPoseTaskParams* p = static_cast<MoveToPoseTaskParams*>(param);
    lemlib::Chassis& chassis = *(p->chassis);
    
    // Apply chassis PID defaults if sentinel values are present
    applyChassisDefaults(p->params);
    
    // Get starting position
    lemlib::Pose startPose = chassis.getPose();
    float startX = startPose.x;
    float startY = startPose.y;
    float startTheta = startPose.theta;
    
    // Calculate distance and trajectory
    float dx = p->x - startX;
    float dy = p->y - startY;
    float distance = std::sqrt(dx * dx + dy * dy);
    
    // Calculate angular distance
    float angularDistance = angleError(startTheta, p->theta);
    
    // If already at target, exit early
    if (distance < 0.5f && std::abs(angularDistance) < 1.0f) {
        if (p->params.async) delete p;
        return;
    }
    
    // Create quintic trajectory for translation
    // Use 80% of timeout for motion, 20% for settling (unless chaining with exit velocity)
    float motionTime = (p->params.minExitVelocity > 0) ? p->timeout * 0.95f : p->timeout * 0.8f;
    QuinticTrajectory2D trajectory(startX, startY, p->x, p->y, motionTime, 0.0f, p->params.minExitVelocity);
    
    // Create quintic profile for rotation (always start and end at 0 angular velocity)
    QuinticProfile rotationProfile(angularDistance, motionTime);
    
    // Calculate path direction
    // Convert from standard math convention to VEX convention:
    // Standard: atan2(dy,dx) gives angle from +X axis, counter-clockwise
    // VEX: 0° is North (+Y), increases clockwise
    // Conversion: VEX_heading = 90° - standard_angle
    float pathAngle = 90.0f - (std::atan2(dy, dx) * 180.0f / M_PI);
    pathAngle = normalizeAngle(pathAngle);
    
    // Adjust for backwards motion
    float exitVelocity = p->params.minExitVelocity;
    if (!p->params.forwards) {
        distance = -distance;
        pathAngle = normalizeAngle(pathAngle + 180.0f);
        exitVelocity = -exitVelocity; // Negative velocity for backwards motion
    }
    
    // Control loop variables
    uint32_t startTime = pros::millis();
    float prevLateralError = 0;
    float prevAngularError = 0;
    
    uint32_t settleStartTime = 0;
    bool settling = false;
    
    const int loopDelay = 10; // 10ms loop time
    
    while (true) {
        uint32_t currentTime = pros::millis();
        uint32_t elapsedTime = currentTime - startTime;
        
        // Check timeout
        if (elapsedTime >= p->timeout) break;
        
        // Get current pose
        lemlib::Pose currentPose = chassis.getPose();
        
        // Calculate desired position from trajectory
        float t = std::min(static_cast<float>(elapsedTime), motionTime);
        float desiredX = trajectory.getX(t);
        float desiredY = trajectory.getY(t);
        
        // Calculate desired heading: blend between path following and target heading
        float pathFollowHeading = pathAngle;
        float targetHeading = startTheta + rotationProfile.getPosition(t);
        float progress = t / motionTime;
        
        // Use lead factor to blend between path following and target heading
        float blendFactor = progress * (1.0f + p->params.lead);
        blendFactor = std::min(1.0f, std::max(0.0f, blendFactor));
        
        float desiredHeading = pathFollowHeading * (1.0f - blendFactor) + targetHeading * blendFactor;
        desiredHeading = normalizeAngle(desiredHeading);
        
        // Calculate error to trajectory point for control
        float errorX = desiredX - currentPose.x;
        float errorY = desiredY - currentPose.y;
        
        // Transform error to robot's local coordinate frame using CURRENT heading
        // Convert heading from VEX convention (0°=North, clockwise) to standard math (0°=East, CCW)
        float currentHeading = currentPose.theta;
        float headingRad = (90.0f - currentHeading) * M_PI / 180.0f;
        float headingCos = std::cos(headingRad);
        float headingSin = std::sin(headingRad);
        
        // Forward error: projection along robot's current heading direction
        // Note: pathAngle is already flipped 180° when forwards=false, which affects
        // desiredHeading and thus currentHeading. So projection naturally gives correct sign.
        float forwardError = errorX * headingCos + errorY * headingSin;
        
        // Lateral error: projection perpendicular to robot's current heading
        float lateralError = -errorX * headingSin + errorY * headingCos;
        
        // Also calculate magnitude for debugging/monitoring
        float trajectoryError = std::sqrt(errorX * errorX + errorY * errorY);
        
        // Calculate error to final target for settling check
        float finalErrorX = p->x - currentPose.x;
        float finalErrorY = p->y - currentPose.y;
        float finalError = std::sqrt(finalErrorX * finalErrorX + finalErrorY * finalErrorY);
        
        // Calculate angular error (currentHeading already defined above)
        float angularError = angleError(currentHeading, desiredHeading);
        
        // Check if we're in settling range (use distance to final target and final heading)
        // Skip settling if using exit velocity for motion chaining
        if (p->params.minExitVelocity <= 0 && finalError < p->params.settleRange && std::abs(angleError(currentHeading, p->theta)) < p->params.settleAngle) {
            if (!settling) {
                settling = true;
                settleStartTime = currentTime;
            } else if (currentTime - settleStartTime >= p->params.settleTimeout) {
                break; // Successfully settled
            }
        } else {
            settling = false;
        }
        
        // Calculate feedforward from trajectory velocity
        float feedforward = 0;
        if (t < motionTime) {
            // Get desired velocity from trajectory
            float vx = trajectory.getVx(t);
            float vy = trajectory.getVy(t);
            float desiredVelocity = std::sqrt(vx * vx + vy * vy);

            desiredVelocity *= 1000;
            
            // Scale based on robot's max velocity to avoid over-powering slow movements
            feedforward = (desiredVelocity / p->params.maxVelocity) * 127.0f;
            
            // Apply minimum clamp to overcome static friction
            if (p->params.kS > 0.0f && std::abs(feedforward) > 0.1f && std::abs(feedforward) < p->params.kS) {
                feedforward = feedforward > 0 ? p->params.kS : -p->params.kS;
            }
            
            if (!p->params.forwards) feedforward = -feedforward;
        }
        
        // PID corrections
        // Forward error drives forward/back motion along robot's heading
        float forwardCorrection = p->params.lateralKP * forwardError;
        float forwardDerivative = p->params.lateralKD * (forwardError - prevLateralError);
        
        // Angular error for heading control, plus lateral error to help steer
        float combinedAngularError = angularError + lateralError * 0.1f;
        float angularCorrection = p->params.angularKP * combinedAngularError;
        float angularDerivative = p->params.angularKD * (angularError - prevAngularError);
        
        // Calculate motor voltages
        float forwardPower = feedforward + forwardCorrection + forwardDerivative;
        float turnPower = angularCorrection + angularDerivative;
        
        int leftVoltage = clampMotorVoltage(forwardPower + turnPower);
        int rightVoltage = clampMotorVoltage(forwardPower - turnPower);
        
        // Apply motor voltages using tank control
        chassis.tank(leftVoltage, rightVoltage, true);
        
        // Update previous errors
        prevLateralError = forwardError;
        prevAngularError = angularError;
        
        pros::delay(loopDelay);
    }
    
    // Stop motors only if not chaining (exit velocity is 0)
    if (p->params.minExitVelocity <= 0) {
        chassis.tank(0, 0, true);
    }
    
    // Clean up if async
    if (p->params.async) delete p;
}

} // namespace quintic
