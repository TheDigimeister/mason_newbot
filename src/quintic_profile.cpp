#include "quintic_profile.hpp"
#include <cmath>

namespace quintic {

QuinticProfile::QuinticProfile(float distance, float duration) 
    : distance(distance), duration(duration) {
    // For a quintic profile with boundary conditions:
    // s(0) = 0, s'(0) = 0, s''(0) = 0
    // s(T) = L, s'(T) = 0, s''(T) = 0
    // where T is duration and L is total distance
    //
    // Using normalized time τ = t/T ∈ [0, 1]
    // s(τ) = L * (10τ³ - 15τ⁴ + 6τ⁵)
    //
    // Therefore:
    // a0 = 0, a1 = 0, a2 = 0
    // a3 = 10L, a4 = -15L, a5 = 6L
    
    a3 = 10.0f * distance;
    a4 = -15.0f * distance;
    a5 = 6.0f * distance;
}

float QuinticProfile::getPosition(float t) const {
    // Clamp time to valid range
    if (t <= 0) return 0;
    if (t >= duration) return distance;
    
    // Normalize time to [0, 1]
    float tau = t / duration;
    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    float tau4 = tau3 * tau;
    float tau5 = tau4 * tau;
    
    // s(τ) = a3*τ³ + a4*τ⁴ + a5*τ⁵
    return a3 * tau3 + a4 * tau4 + a5 * tau5;
}

float QuinticProfile::getVelocity(float t) const {
    // Clamp time to valid range
    if (t <= 0 || t >= duration) return 0;
    
    // Normalize time to [0, 1]
    float tau = t / duration;
    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    float tau4 = tau3 * tau;
    
    // v(τ) = ds/dt = (ds/dτ) * (dτ/dt)
    // ds/dτ = 3*a3*τ² + 4*a4*τ³ + 5*a5*τ⁴
    // dτ/dt = 1/T
    float dsdtau = 3.0f * a3 * tau2 + 4.0f * a4 * tau3 + 5.0f * a5 * tau4;
    return dsdtau / duration;
}

float QuinticProfile::getAcceleration(float t) const {
    // Clamp time to valid range
    if (t <= 0 || t >= duration) return 0;
    
    // Normalize time to [0, 1]
    float tau = t / duration;
    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    
    // a(τ) = d²s/dt² = (d²s/dτ²) * (dτ/dt)²
    // d²s/dτ² = 6*a3*τ + 12*a4*τ² + 20*a5*τ³
    // (dτ/dt)² = 1/T²
    float d2sdtau2 = 6.0f * a3 * tau + 12.0f * a4 * tau2 + 20.0f * a5 * tau3;
    return d2sdtau2 / (duration * duration);
}

// QuinticTrajectory2D implementation

QuinticTrajectory2D::QuinticTrajectory2D(float startX, float startY, float endX, float endY, float duration)
    : startX(startX), startY(startY), endX(endX), endY(endY), duration(duration),
      profile(0, duration) {
    
    // Calculate total distance and direction
    float dx = endX - startX;
    float dy = endY - startY;
    distance = std::sqrt(dx * dx + dy * dy);
    
    // Calculate direction angle
    angle = std::atan2(dy, dx);
    
    // Create profile with actual distance
    profile = QuinticProfile(distance, duration);
}

float QuinticTrajectory2D::getX(float t) const {
    float s = profile.getPosition(t);
    return startX + s * std::cos(angle);
}

float QuinticTrajectory2D::getY(float t) const {
    float s = profile.getPosition(t);
    return startY + s * std::sin(angle);
}

float QuinticTrajectory2D::getVx(float t) const {
    float v = profile.getVelocity(t);
    return v * std::cos(angle);
}

float QuinticTrajectory2D::getVy(float t) const {
    float v = profile.getVelocity(t);
    return v * std::sin(angle);
}

} // namespace quintic
