#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "quintic_profile.hpp"
#include "drive.hpp"

namespace quintic {

/**
 * @brief Parameters for quintic moveToPoint
 * 
 * Similar to lemlib's MoveToPointParams but for quintic profiling
 */
struct QuinticMoveToPointParams {
    /** whether the robot should move forwards or backwards. True by default */
    bool forwards = true;
    /** async execution. True by default */
    bool async = true;
    /** PID gains for lateral correction during motion. -1.0f = use chassis lateral PID gains (default) */
    float lateralKP = -1.0f;
    float lateralKD = -1.0f;
    /** PID gains for angular correction during motion. -1.0f = use chassis angular PID gains (default) */
    float angularKP = -1.0f;
    float angularKD = -1.0f;
    /** settling tolerance in inches */
    float settleRange = 1.0f;
    /** settling timeout in milliseconds */
    int settleTimeout = 250;
    /** minimum exit velocity in inches/sec for motion chaining. 0.0f = come to complete stop (default) */
    float minExitVelocity = 0.0f;
    /** maximum robot velocity in inches/sec (used for feedforward scaling). 60.0 = default */
    float maxVelocity = 60.0f;
    /** static friction feedforward constant (motor voltage). Minimum feedforward clamp to overcome friction. 0.0 = disabled (default) */
    float kS = 10.0f;
};

/**
 * @brief Parameters for quintic moveToPose
 * 
 * Similar to lemlib's MoveToPoseParams but for quintic profiling
 */
struct QuinticMoveToPoseParams {
    /** whether the robot should move forwards or backwards. True by default */
    bool forwards = true;
    /** async execution. True by default */
    bool async = true;
    /** PID gains for lateral correction during motion. -1.0f = use chassis lateral PID gains (default) */
    float lateralKP = -1.0f;
    float lateralKD = -1.0f;
    /** PID gains for angular correction during motion. -1.0f = use chassis angular PID gains (default) */
    float angularKP = -1.0f;
    float angularKD = -1.0f;
    /** settling tolerance in inches */
    float settleRange = 1.0f;
    /** settling tolerance in degrees */
    float settleAngle = 2.0f;
    /** settling timeout in milliseconds */
    int settleTimeout = 250;
    /** lead factor for turn smoothing during motion (0.0 to 1.0) */
    float lead = 0.6f;
    /** minimum exit velocity in inches/sec for motion chaining. 0.0f = come to complete stop (default) */
    float minExitVelocity = 0.0f;
    /** maximum robot velocity in inches/sec (used for feedforward scaling). 60.0 = default */
    float maxVelocity = 60.0f;
    /** static friction feedforward constant (motor voltage). Minimum feedforward clamp to overcome friction. 0.0 = disabled (default) */
    float kS = 10.0f;
};

/**
 * @brief Move the chassis to a target point using quintic motion profiling
 * 
 * This function uses a quintic (5th order polynomial) trajectory to provide
 * smooth motion with continuous velocity, acceleration, and jerk.
 * 
 * @param chassis Reference to the lemlib chassis object
 * @param x Target x position in inches
 * @param y Target y position in inches  
 * @param timeout Maximum time for movement in milliseconds
 * @param params Optional parameters for the movement
 * 
 * @note This function will calculate the optimal duration based on distance
 *       and use the quintic profile to generate smooth motion.
 * 
 * @b Example
 * @code {.cpp}
 * // Move to point (20, 15) with a 4 second timeout
 * quintic::moveToPoint(chassis, 20, 15, 4000);
 * 
 * // Move backwards to point (-30, 40) with custom parameters
 * quintic::moveToPoint(chassis, -30, 40, 3000, {.forwards = false});
 * 
 * // Move with custom PID gains
 * quintic::moveToPoint(chassis, 10, 10, 2000, {
 *     .lateralKP = 8.0f,
 *     .angularKP = 4.0f
 * });
 * 
 * // Motion chaining: maintain exit velocity for smooth transitions
 * quintic::moveToPoint(chassis, 20, 20, 2000, {.minExitVelocity = 12.0f, .async = false});
 * quintic::moveToPoint(chassis, 40, 20, 2000, {});  // Smoothly continues from previous motion
 * @endcode
 */
void moveToPoint(lemlib::Chassis& chassis, float x, float y, int timeout, 
                 QuinticMoveToPointParams params = {});

/**
 * @brief Move the chassis to a target pose using quintic motion profiling
 * 
 * This function uses quintic trajectories for both translation and rotation
 * to provide smooth motion to a target pose (position + heading).
 * 
 * @param chassis Reference to the lemlib chassis object
 * @param x Target x position in inches
 * @param y Target y position in inches
 * @param theta Target heading in degrees
 * @param timeout Maximum time for movement in milliseconds
 * @param params Optional parameters for the movement
 * 
 * @note This function smoothly blends translation and rotation using
 *       quintic profiles for both components.
 * 
 * @b Example
 * @code {.cpp}
 * // Move to pose (20, 15, 90°) with a 4 second timeout
 * quintic::moveToPose(chassis, 20, 15, 90, 4000);
 * 
 * // Move backwards to pose
 * quintic::moveToPose(chassis, -30, 40, 180, 3000, {.forwards = false});
 * 
 * // Move with custom lead factor for turn smoothing
 * quintic::moveToPose(chassis, 10, 10, 45, 2000, {
 *     .lead = 0.8f  // Higher values = more aggressive turning
 * });
 * 
 * // Motion chaining: maintain exit velocity for smooth path following
 * quintic::moveToPose(chassis, 20, 20, 90, 2000, {.minExitVelocity = 15.0f, .async = false});
 * quintic::moveToPose(chassis, 40, 40, 180, 2000, {});  // Continues smoothly from previous
 * @endcode
 */
void moveToPose(lemlib::Chassis& chassis, float x, float y, float theta, int timeout,
                QuinticMoveToPoseParams params = {});

} // namespace quintic
