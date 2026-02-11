#pragma once

#include <cmath>

namespace quintic {

/**
 * @brief Quintic motion profile calculator
 * 
 * Generates a 5th-order polynomial trajectory that provides smooth
 * motion with continuous velocity, acceleration, and jerk.
 * The profile ensures zero velocity and acceleration at start and end.
 */
class QuinticProfile {
public:
    /**
     * @brief Construct a quintic profile
     * 
     * @param distance Total distance to travel
     * @param duration Total time for the movement (milliseconds)
     */
    QuinticProfile(float distance, float duration);
    
    /**
     * @brief Get position at a specific time
     * 
     * @param t Time in milliseconds (0 to duration)
     * @return Position value (0 to distance)
     */
    float getPosition(float t) const;
    
    /**
     * @brief Get velocity at a specific time
     * 
     * @param t Time in milliseconds
     * @return Velocity value
     */
    float getVelocity(float t) const;
    
    /**
     * @brief Get acceleration at a specific time
     * 
     * @param t Time in milliseconds
     * @return Acceleration value
     */
    float getAcceleration(float t) const;
    
    /**
     * @brief Get the total duration of the profile
     * 
     * @return Duration in milliseconds
     */
    float getDuration() const { return duration; }
    
    /**
     * @brief Get the total distance of the profile
     * 
     * @return Distance
     */
    float getDistance() const { return distance; }

private:
    float distance; // Total distance
    float duration; // Total duration in milliseconds
    
    // Quintic polynomial coefficients for normalized time [0, 1]
    // s(τ) = a3*τ³ + a4*τ⁴ + a5*τ⁵
    // where τ = t/T (normalized time)
    float a3, a4, a5;
};

/**
 * @brief 2D quintic trajectory planner
 * 
 * Plans smooth motion in 2D space using quintic profiles
 */
class QuinticTrajectory2D {
public:
    /**
     * @brief Construct a 2D quintic trajectory
     * 
     * @param startX Starting X position
     * @param startY Starting Y position
     * @param endX Target X position
     * @param endY Target Y position
     * @param duration Total time for the movement (milliseconds)
     */
    QuinticTrajectory2D(float startX, float startY, float endX, float endY, float duration);
    
    /**
     * @brief Get X position at a specific time
     * 
     * @param t Time in milliseconds
     * @return X position
     */
    float getX(float t) const;
    
    /**
     * @brief Get Y position at a specific time
     * 
     * @param t Time in milliseconds
     * @return Y position
     */
    float getY(float t) const;
    
    /**
     * @brief Get X velocity at a specific time
     * 
     * @param t Time in milliseconds
     * @return X velocity
     */
    float getVx(float t) const;
    
    /**
     * @brief Get Y velocity at a specific time
     * 
     * @param t Time in milliseconds
     * @return Y velocity
     */
    float getVy(float t) const;
    
    /**
     * @brief Get total distance of trajectory
     * 
     * @return Euclidean distance
     */
    float getDistance() const { return distance; }
    
    /**
     * @brief Get duration of trajectory
     * 
     * @return Duration in milliseconds
     */
    float getDuration() const { return duration; }

private:
    float startX, startY;
    float endX, endY;
    float distance;
    float duration;
    
    // Direction angle
    float angle;
    
    // Single quintic profile for distance
    QuinticProfile profile;
};

} // namespace quintic
