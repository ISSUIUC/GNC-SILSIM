#!/usr/bin/env python3
"""
Test script for the improved rotational EKF implementation
Demonstrates how to use the EKF with realistic rocket parameters and sensor measurements
"""

import numpy as np
import matplotlib.pyplot as plt
from r_ekf import KalmanFilter_R

def simulate_rocket_flight(duration=10.0, dt=0.01):
    """
    Simulate a simple rocket flight scenario for testing the EKF
    
    Args:
        duration (float): simulation duration in seconds
        dt (float): time step in seconds
        
    Returns:
        tuple: (time_array, true_states, measurements)
    """
    # Time array
    t = np.arange(0, duration, dt)
    n_steps = len(t)
    
    # Initialize arrays for true states and measurements
    true_states = np.zeros((6, n_steps))
    measurements = np.zeros((6, n_steps))
    
    # Initial conditions
    true_states[:, 0] = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]  # Small initial pitch
    
    # Simulate true dynamics (simplified)
    for i in range(1, n_steps):
        # Simple dynamics: constant pitch rate with some noise
        dt_sim = t[i] - t[i-1]
        
        # True state derivatives (simplified)
        phi, p, theta, q, psi, r = true_states[:, i-1]
        
        # Simple dynamics
        phi_dot = p
        p_dot = 0.1 * np.sin(t[i])  # Small oscillating roll moment
        theta_dot = q
        q_dot = -0.05 * theta  # Pitch stability
        psi_dot = r
        r_dot = 0.05 * np.cos(t[i])  # Small oscillating yaw moment
        
        # Integrate
        true_states[0, i] = phi + phi_dot * dt_sim
        true_states[1, i] = p + p_dot * dt_sim
        true_states[2, i] = theta + theta_dot * dt_sim
        true_states[3, i] = q + q_dot * dt_sim
        true_states[4, i] = psi + psi_dot * dt_sim
        true_states[5, i] = r + r_dot * dt_sim
    
    # Generate noisy measurements
    measurement_noise_std = np.array([0.02, 0.01, 0.02, 0.01, 0.02, 0.01])  # rad, rad/s
    
    for i in range(n_steps):
        measurements[:, i] = true_states[:, i] + np.random.normal(0, measurement_noise_std)
    
    return t, true_states, measurements

def test_rotational_ekf():
    """Test the rotational EKF with simulated data"""
    
    # Simulation parameters
    duration = 10.0  # seconds
    dt = 0.01  # seconds
    
    # Generate test data
    t, true_states, measurements = simulate_rocket_flight(duration, dt)
    
    # Initialize EKF
    initial_roll = measurements[0, 0]
    initial_pitch = measurements[2, 0]
    initial_yaw = measurements[4, 0]
    initial_p = measurements[1, 0]
    initial_q = measurements[3, 0]
    initial_r = measurements[5, 0]
    
    ekf = KalmanFilter_R(dt, initial_roll, initial_pitch, initial_yaw, 
                        initial_p, initial_q, initial_r)
    
    # Arrays to store EKF estimates
    ekf_states = np.zeros((6, len(t)))
    ekf_covariances = np.zeros((6, len(t)))
    
    # Rocket parameters (example values)
    rho = 1.225  # kg/m³ (sea level air density)
    vel_mag = 50.0  # m/s (example velocity)
    alpha = 0.0  # rad (angle of attack)
    beta = 0.0  # rad (sideslip angle)
    thrust = np.array([1000.0, 0.0, 0.0])  # N (thrust vector)
    cp = np.array([0.0, 0.0, 0.0])  # m (center of pressure)
    cm = np.array([0.0, 0.0, 0.0])  # m (center of mass)
    
    # Run EKF
    for i in range(len(t)):
        # Prediction step
        ekf.predict(rho, vel_mag, alpha, beta, thrust, cp, cm)
        
        # Update step
        phi_meas = measurements[0, i]
        p_meas = measurements[1, i]
        theta_meas = measurements[2, i]
        q_meas = measurements[3, i]
        psi_meas = measurements[4, i]
        r_meas = measurements[5, i]
        
        ekf.update(phi_meas, p_meas, theta_meas, q_meas, psi_meas, r_meas)
        
        # Store results
        ekf_states[:, i] = ekf.get_state().flatten()
        ekf_covariances[:, i] = np.diag(ekf.get_covariance())
    
    # Plot results
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle('Rotational EKF Test Results', fontsize=16)
    
    # Roll
    axes[0, 0].plot(t, np.rad2deg(true_states[0, :]), 'b-', label='True', linewidth=2)
    axes[0, 0].plot(t, np.rad2deg(measurements[0, :]), 'r.', label='Measurements', markersize=2)
    axes[0, 0].plot(t, np.rad2deg(ekf_states[0, :]), 'g-', label='EKF Estimate', linewidth=2)
    axes[0, 0].set_ylabel('Roll Angle (deg)')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    # Roll rate
    axes[0, 1].plot(t, np.rad2deg(true_states[1, :]), 'b-', label='True', linewidth=2)
    axes[0, 1].plot(t, np.rad2deg(measurements[1, :]), 'r.', label='Measurements', markersize=2)
    axes[0, 1].plot(t, np.rad2deg(ekf_states[1, :]), 'g-', label='EKF Estimate', linewidth=2)
    axes[0, 1].set_ylabel('Roll Rate (deg/s)')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # Pitch
    axes[1, 0].plot(t, np.rad2deg(true_states[2, :]), 'b-', label='True', linewidth=2)
    axes[1, 0].plot(t, np.rad2deg(measurements[2, :]), 'r.', label='Measurements', markersize=2)
    axes[1, 0].plot(t, np.rad2deg(ekf_states[2, :]), 'g-', label='EKF Estimate', linewidth=2)
    axes[1, 0].set_ylabel('Pitch Angle (deg)')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    # Pitch rate
    axes[1, 1].plot(t, np.rad2deg(true_states[3, :]), 'b-', label='True', linewidth=2)
    axes[1, 1].plot(t, np.rad2deg(measurements[3, :]), 'r.', label='Measurements', markersize=2)
    axes[1, 1].plot(t, np.rad2deg(ekf_states[3, :]), 'g-', label='EKF Estimate', linewidth=2)
    axes[1, 1].set_ylabel('Pitch Rate (deg/s)')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    # Yaw
    axes[2, 0].plot(t, np.rad2deg(true_states[4, :]), 'b-', label='True', linewidth=2)
    axes[2, 0].plot(t, np.rad2deg(measurements[4, :]), 'r.', label='Measurements', markersize=2)
    axes[2, 0].plot(t, np.rad2deg(ekf_states[4, :]), 'g-', label='EKF Estimate', linewidth=2)
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('Yaw Angle (deg)')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    
    # Yaw rate
    axes[2, 1].plot(t, np.rad2deg(true_states[5, :]), 'b-', label='True', linewidth=2)
    axes[2, 1].plot(t, np.rad2deg(measurements[5, :]), 'r.', label='Measurements', markersize=2)
    axes[2, 1].plot(t, np.rad2deg(ekf_states[5, :]), 'g-', label='EKF Estimate', linewidth=2)
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_ylabel('Yaw Rate (deg/s)')
    axes[2, 1].legend()
    axes[2, 1].grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Print final statistics
    print("EKF Test Results:")
    print(f"Final roll angle: {np.rad2deg(ekf_states[0, -1]):.2f}° (true: {np.rad2deg(true_states[0, -1]):.2f}°)")
    print(f"Final pitch angle: {np.rad2deg(ekf_states[2, -1]):.2f}° (true: {np.rad2deg(true_states[2, -1]):.2f}°)")
    print(f"Final yaw angle: {np.rad2deg(ekf_states[4, -1]):.2f}° (true: {np.rad2deg(true_states[4, -1]):.2f}°)")
    
    # Calculate RMS errors
    roll_rms = np.sqrt(np.mean((ekf_states[0, :] - true_states[0, :])**2))
    pitch_rms = np.sqrt(np.mean((ekf_states[2, :] - true_states[2, :])**2))
    yaw_rms = np.sqrt(np.mean((ekf_states[4, :] - true_states[4, :])**2))
    
    print(f"RMS errors - Roll: {np.rad2deg(roll_rms):.3f}°, Pitch: {np.rad2deg(pitch_rms):.3f}°, Yaw: {np.rad2deg(yaw_rms):.3f}°")

if __name__ == "__main__":
    test_rotational_ekf() 