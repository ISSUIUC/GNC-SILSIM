# Rotational Extended Kalman Filter (EKF) for Rocket Attitude Estimation

## Overview

This implementation provides an improved rotational Extended Kalman Filter for estimating rocket attitude (roll, pitch, yaw angles) and angular rates using gyroscope and magnetometer measurements. The implementation is based on the state-space modeling principles from the paper "State-Space Modeling of a Rocket for Optimal Control System Design" by Aliyu Bhar Kisabo and Aliyu Funmilayo Adebimpe.

## Paper Analysis Summary

### Key Concepts from the Paper

1. **6-Degree of Freedom (6DoF) Model**: The paper presents a complete mathematical model for rocket dynamics:
   - **Translational motion** (3 DoF): Position and velocity in x, y, z directions
   - **Rotational motion** (3 DoF): Angular position (Euler angles) and angular velocity

2. **State Variables**: The complete state vector includes:
   - `u, v, w`: Linear velocities in body frame
   - `p, q, r`: Angular velocities (roll, pitch, yaw rates)
   - `φ, θ, ψ`: Euler angles (roll, pitch, yaw)

3. **Forces and Moments**: The model accounts for:
   - **Aerodynamic forces**: Drag, lift, side forces
   - **Propulsive forces**: Thrust vector
   - **Gravitational forces**: Gravity in body coordinates
   - **Aerodynamic moments**: Roll, pitch, yaw moments
   - **Propulsive moments**: Thrust-induced moments

4. **Decoupling**: The paper separates the 6DoF equations into:
   - **Longitudinal dynamics**: Pitch plane (u, w, q, θ)
   - **Lateral dynamics**: Roll/yaw plane (v, p, r, φ, ψ)

### Rotational Dynamics Equations

The paper provides the following rotational dynamics equations:

**Euler Angle Rates (Equation 16):**
```
φ̇ = p + (q*sin(φ) + r*cos(φ))*tan(θ)
θ̇ = q*cos(φ) - r*sin(φ)  
ψ̇ = (q*sin(φ) + r*cos(φ))/cos(θ)
```

**Angular Rate Derivatives (Equation 11):**
```
ṗ = (LA + Lp - q*r*(Iz-Iy))/Ix
q̇ = (MA + Mp - r*p*(Ix-Iz))/Iy
ṙ = (NA + Np - p*q*(Iy-Ix))/Iz
```

Where:
- `LA, MA, NA`: Aerodynamic moments
- `Lp, Mp, Np`: Propulsive moments
- `Ix, Iy, Iz`: Moments of inertia

## Implementation Improvements

### Issues with Original Implementation

1. **State Vector Mismatch**: The original EKF tracked 9 states (3 angles + 3 angular velocities + 3 angular accelerations), but the paper's rotational dynamics only need 6 states (3 angles + 3 angular velocities).

2. **Measurement Model Issues**: 
   - Using accelerometer data to estimate angles was incorrect
   - The `H` matrix didn't properly map to the state vector
   - Missing proper gyroscope measurements

3. **Process Model Issues**: 
   - The Jacobian matrix `F` was overly complex for a rotational system
   - Angular acceleration states were unnecessary and caused instability

4. **Aerodynamic Modeling**: The aerodynamic moment calculations needed refinement

### Improved Implementation Features

1. **Correct State Vector**: `[φ, p, θ, q, ψ, r]^T` (6 states)
2. **Proper Measurement Model**: Direct measurement of all states using gyroscope and magnetometer
3. **Accurate Process Model**: Based on the paper's rotational dynamics equations
4. **Proper Jacobian**: Computed analytically from the nonlinear dynamics
5. **Aerodynamic Moments**: Implemented according to the paper's equations
6. **Thrust Moments**: Proper calculation of thrust-induced moments

## Usage

### Basic Usage

```python
from r_ekf import KalmanFilter_R
import numpy as np

# Initialize EKF
dt = 0.01  # seconds
initial_roll = 0.0    # radians
initial_pitch = 0.1   # radians
initial_yaw = 0.0     # radians
initial_p = 0.0       # rad/s
initial_q = 0.0       # rad/s
initial_r = 0.0       # rad/s

ekf = KalmanFilter_R(dt, initial_roll, initial_pitch, initial_yaw, 
                    initial_p, initial_q, initial_r)

# Rocket parameters
rho = 1.225  # kg/m³ (air density)
vel_mag = 50.0  # m/s (velocity magnitude)
alpha = 0.0  # rad (angle of attack)
beta = 0.0  # rad (sideslip angle)
thrust = np.array([1000.0, 0.0, 0.0])  # N (thrust vector)
cp = np.array([0.0, 0.0, 0.0])  # m (center of pressure)
cm = np.array([0.0, 0.0, 0.0])  # m (center of mass)

# Prediction step
ekf.predict(rho, vel_mag, alpha, beta, thrust, cp, cm)

# Update step (with sensor measurements)
phi_meas = 0.02   # rad (measured roll angle)
p_meas = 0.01     # rad/s (measured roll rate)
theta_meas = 0.15 # rad (measured pitch angle)
q_meas = 0.005    # rad/s (measured pitch rate)
psi_meas = 0.0    # rad (measured yaw angle)
r_meas = 0.002    # rad/s (measured yaw rate)

ekf.update(phi_meas, p_meas, theta_meas, q_meas, psi_meas, r_meas)

# Get estimates
roll, pitch, yaw = ekf.get_angles()
p_rate, q_rate, r_rate = ekf.get_rates()
state = ekf.get_state()
covariance = ekf.get_covariance()
```

### Integration with Rocket Simulation

```python
# In your main simulation loop
for timestep in simulation:
    # Get current rocket state and parameters
    rho = get_air_density(altitude)
    vel_mag = get_velocity_magnitude()
    alpha = get_angle_of_attack()
    beta = get_sideslip_angle()
    thrust = get_thrust_vector()
    cp = get_center_of_pressure()
    cm = get_center_of_mass()
    
    # EKF prediction
    ekf.predict(rho, vel_mag, alpha, beta, thrust, cp, cm)
    
    # Get sensor measurements
    phi_meas = gyroscope.get_roll_angle()
    p_meas = gyroscope.get_roll_rate()
    theta_meas = gyroscope.get_pitch_angle()
    q_meas = gyroscope.get_pitch_rate()
    psi_meas = magnetometer.get_yaw_angle()
    r_meas = gyroscope.get_yaw_rate()
    
    # EKF update
    ekf.update(phi_meas, p_meas, theta_meas, q_meas, psi_meas, r_meas)
    
    # Use estimates for control
    estimated_roll, estimated_pitch, estimated_yaw = ekf.get_angles()
    estimated_p, estimated_q, estimated_r = ekf.get_rates()
```

## Tuning Parameters

### Process Noise Covariance (Q)

The process noise covariance matrix represents uncertainty in the system dynamics:

```python
# Default values (tune based on your rocket)
self.Q = np.eye(6) * 0.01
self.Q[1,1] = 0.1  # roll rate noise (higher uncertainty)
self.Q[3,3] = 0.1  # pitch rate noise
self.Q[5,5] = 0.1  # yaw rate noise
```

### Measurement Noise Covariance (R)

The measurement noise covariance matrix represents sensor uncertainty:

```python
# Default values (tune based on your sensors)
self.R = np.eye(6) * 0.1
self.R[0,0] = 0.05  # roll angle measurement noise
self.R[2,2] = 0.05  # pitch angle measurement noise
self.R[4,4] = 0.05  # yaw angle measurement noise
self.R[1,1] = 0.01  # roll rate measurement noise (gyro more accurate)
self.R[3,3] = 0.01  # pitch rate measurement noise
self.R[5,5] = 0.01  # yaw rate measurement noise
```

### Tuning Guidelines

1. **Increase Q** if the filter is too slow to track changes
2. **Decrease Q** if the filter is too noisy
3. **Increase R** if sensors are noisy
4. **Decrease R** if sensors are accurate

## Aerodynamic Coefficients

The EKF uses simplified aerodynamic coefficients. For accurate results, replace the default values with actual coefficients for your rocket:

```python
# In compute_aerodynamic_moments method
Cl_delta = 0.1  # Roll control effectiveness - get from wind tunnel/CFD
Cm_alpha = -0.5 # Pitch stability derivative - get from wind tunnel/CFD
Cm_delta = 0.2  # Pitch control effectiveness - get from wind tunnel/CFD
Cn_beta = 0.3   # Yaw stability derivative - get from wind tunnel/CFD
Cn_delta = 0.2  # Yaw control effectiveness - get from wind tunnel/CFD
```

## Testing

Run the test script to verify the implementation:

```bash
python test_rotational_ekf.py
```

This will:
1. Simulate a rocket flight scenario
2. Generate noisy sensor measurements
3. Run the EKF
4. Plot the results comparing true states, measurements, and estimates
5. Print RMS error statistics

## Key Differences from Original Implementation

| Aspect | Original | Improved |
|--------|----------|----------|
| State Vector | 9 states (angles + rates + accelerations) | 6 states (angles + rates) |
| Measurement Model | Incorrect use of accelerometer data | Direct measurement of all states |
| Process Model | Overly complex Jacobian | Based on paper's dynamics |
| Aerodynamic Modeling | Simplified force calculations | Proper moment calculations |
| Stability | Unstable due to acceleration states | Stable with proper dynamics |

## References

1. Kisabo, A. B., & Adebimpe, A. F. (2019). State-Space Modeling of a Rocket for Optimal Control System Design. In Ballistics. IntechOpen.
2. Anderson, B. D., & Moore, J. B. (1979). Optimal filtering. Prentice-Hall.
3. Welch, G., & Bishop, G. (2006). An introduction to the Kalman filter. University of North Carolina at Chapel Hill.

## Notes

- The implementation assumes small angles for linearization validity
- For large angle maneuvers, consider using quaternions instead of Euler angles
- The aerodynamic coefficients should be obtained from wind tunnel tests or CFD analysis
- Tune the noise parameters based on your specific sensors and rocket characteristics 