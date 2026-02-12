#include "quintic_profile.hpp"
#include <cmath>

namespace quintic {

QuinticProfile::QuinticProfile(float distance, float duration, float initialVelocity, float finalVelocity) 
    : distance(distance), duration(duration), v0(initialVelocity), vf(finalVelocity) {
    // For a quintic profile with general boundary conditions:
    // s(0) = 0, s'(0) = v0, s''(0) = 0
    // s(T) = L, s'(T) = vf, s''(T) = 0
    // where T is duration, L is total distance, v0 is initial velocity, vf is final velocity
    //
    // Using normalized time τ = t/T ∈ [0, 1] and normalized velocities:
    // v0_norm = v0 * T / L (initial velocity normalized)
    // vf_norm = vf * T / L (final velocity normalized)
    //
    // Solving the system of equations yields:
    // a0 = 0
    // a1 = v0_norm
    // a2 = 0
    // a3 = 10 - 6*v0_norm - 4*vf_norm
    // a4 = -15 + 8*v0_norm + 7*vf_norm
    // a5 = 6 - 3*v0_norm - 3*vf_norm
    //
    // Then multiply by L to get actual distance
    
    float v0_norm = (duration > 0 && distance != 0) ? v0 * duration / distance : 0;
    float vf_norm = (duration > 0 && distance != 0) ? vf * duration / distance : 0;
    
    a0 = 0;
    a1 = v0_norm * distance;
    a2 = 0;
    a3 = (10.0f - 6.0f * v0_norm - 4.0f * vf_norm) * distance;
    a4 = (-15.0f + 8.0f * v0_norm + 7.0f * vf_norm) * distance;
    a5 = (6.0f - 3.0f * v0_norm - 3.0f * vf_norm) * distance;
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
    
    // s(τ) = a0 + a1*τ + a2*τ² + a3*τ³ + a4*τ⁴ + a5*τ⁵
    return a0 + a1 * tau + a2 * tau2 + a3 * tau3 + a4 * tau4 + a5 * tau5;
}

float QuinticProfile::getVelocity(float t) const {
    // Handle boundary conditions
    if (t <= 0) return v0;
    if (t >= duration) return vf;
    
    // Normalize time to [0, 1]
    float tau = t / duration;
    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    float tau4 = tau3 * tau;
    
    // v(τ) = ds/dt = (ds/dτ) * (dτ/dt)
    // ds/dτ = a1 + 2*a2*τ + 3*a3*τ² + 4*a4*τ³ + 5*a5*τ⁴
    // dτ/dt = 1/T
    float dsdtau = a1 + 2.0f * a2 * tau + 3.0f * a3 * tau2 + 4.0f * a4 * tau3 + 5.0f * a5 * tau4;
    return dsdtau / duration;
}

float QuinticProfile::getAcceleration(float t) const {
    // Clamp time to valid range (acceleration boundary conditions are 0)
    if (t <= 0 || t >= duration) return 0;
    
    // Normalize time to [0, 1]
    float tau = t / duration;
    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    
    // a(τ) = d²s/dt² = (d²s/dτ²) * (dτ/dt)²
    // d²s/dτ² = 2*a2 + 6*a3*τ + 12*a4*τ² + 20*a5*τ³
    // (dτ/dt)² = 1/T²
    float d2sdtau2 = 2.0f * a2 + 6.0f * a3 * tau + 12.0f * a4 * tau2 + 20.0f * a5 * tau3;
    return d2sdtau2 / (duration * duration);
}

// QuinticTrajectory2D implementation

QuinticTrajectory2D::QuinticTrajectory2D(float startX, float startY, float endX, float endY, float duration,
                                         float initialVelocity, float finalVelocity)
    : startX(startX), startY(startY), endX(endX), endY(endY), duration(duration),
      profile(0, duration) {
    
    // Calculate total distance and direction
    float dx = endX - startX;
    float dy = endY - startY;
    distance = std::sqrt(dx * dx + dy * dy);
    
    // Calculate direction angle
    angle = std::atan2(dy, dx);
    
    // Create profile with actual distance and velocities
    profile = QuinticProfile(distance, duration, initialVelocity, finalVelocity);
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
