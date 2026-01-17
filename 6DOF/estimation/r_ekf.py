# Rotational Extended Kalman Filter for Rocket Attitude Estimation
# Based on state-space modeling from "State-Space Modeling of a Rocket for Optimal Control System Design"
from filterpy.common import Q_continuous_white_noise
import numpy as np
import util.vectors as vct

"""
Improved Rotational EKF Implementation

State Vector (6x1): [φ, p, θ, q, ψ, r]^T
- φ: roll angle (rad)
- p: roll rate (rad/s) 
- θ: pitch angle (rad)
- q: pitch rate (rad/s)
- ψ: yaw angle (rad)
- r: yaw rate (rad/s)

Measurements (6x1): [φ_meas, p_meas, θ_meas, q_meas, ψ_meas, r_meas]^T
- From gyroscope: p_meas, q_meas, r_meas
- From magnetometer/IMU: φ_meas, θ_meas, ψ_meas

Based on the paper's rotational dynamics equations:
φ̇ = p + (q*sin(φ) + r*cos(φ))*tan(θ)
θ̇ = q*cos(φ) - r*sin(φ)  
ψ̇ = (q*sin(φ) + r*cos(φ))/cos(θ)
ṗ = (LA + Lp - q*r*(Iz-Iy))/Ix
q̇ = (MA + Mp - r*p*(Ix-Iz))/Iy
ṙ = (NA + Np - p*q*(Iy-Ix))/Iz
"""

class KalmanFilter_R:
    """Extended Kalman Filter for 3D rotational estimation
    
    Implements the rotational dynamics from the rocket state-space model paper.
    Estimates roll, pitch, yaw angles and their rates using gyroscope and magnetometer data.
    """

    def __init__(self, dt, roll, pitch, yaw, w_x, w_y, w_z):
        """
        Initialize the rotational EKF
        
        Args:
            dt (float): time step (seconds)
            roll (float): initial roll angle (radians)
            pitch (float): initial pitch angle (radians) 
            yaw (float): initial yaw angle (radians)
            w_x (float): initial roll rate (rad/s)
            w_y (float): initial pitch rate (rad/s)
            w_z (float): initial yaw rate (rad/s)
        """
        self.dt = dt
        
        # State vector: [φ, p, θ, q, ψ, r]^T
        self.x_k = np.array([[roll, w_x, pitch, w_y, yaw, w_z]]).T
        
        # State dimension
        self.n = 6
        
        # Initialize covariance matrices
        self.P_k = np.eye(self.n) * 0.1  # Initial state uncertainty
        
        # Process noise covariance - tune based on system dynamics
        self.Q = np.eye(self.n) * 0.01
        # Increase noise for angular rates (more uncertain)
        self.Q[1,1] = 0.1  # roll rate noise
        self.Q[3,3] = 0.1  # pitch rate noise  
        self.Q[5,5] = 0.1  # yaw rate noise
        
        # Measurement noise covariance - tune based on sensor characteristics
        self.R = np.eye(self.n) * 0.1
        # Gyroscope typically more accurate than magnetometer
        self.R[0,0] = 0.05  # roll angle measurement noise
        self.R[2,2] = 0.05  # pitch angle measurement noise
        self.R[4,4] = 0.05  # yaw angle measurement noise
        self.R[1,1] = 0.01  # roll rate measurement noise
        self.R[3,3] = 0.01  # pitch rate measurement noise
        self.R[5,5] = 0.01  # yaw rate measurement noise
        
        # Priori state and covariance
        self.x_priori = np.zeros((self.n, 1))
        self.P_priori = np.zeros((self.n, self.n))
        
        # Measurement matrix - maps state to measurements
        self.H = np.eye(self.n)  # Direct measurement of all states
        
        # Rocket physical parameters (initialize with reasonable defaults)
        self.mass = 10.0  # kg
        self.radius = 0.1  # m
        self.height = 1.0  # m
        
        # Moments of inertia (simplified cylindrical rocket model)
        self.Ix = 0.5 * self.mass * self.radius**2  # Roll inertia
        self.Iy = (1/3) * self.mass * self.height**2 + (1/4) * self.mass * self.radius**2  # Pitch inertia
        self.Iz = self.Iy  # Yaw inertia (assume symmetry)
        
        self.current_time = 0.0

    def compute_aerodynamic_moments(self, rho, vel_mag, alpha, beta, delta_roll, delta_pitch, delta_yaw):
        """
        Compute aerodynamic moments based on the paper's equations
        
        Args:
            rho (float): air density (kg/m³)
            vel_mag (float): velocity magnitude (m/s)
            alpha (float): angle of attack (rad)
            beta (float): sideslip angle (rad)
            delta_roll (float): roll control surface deflection (rad)
            delta_pitch (float): pitch control surface deflection (rad)
            delta_yaw (float): yaw control surface deflection (rad)
            
        Returns:
            tuple: (LA, MA, NA) aerodynamic moments in body frame
        """
        # Reference area and length
        S = np.pi * self.radius**2  # Cross-sectional area
        d = 2 * self.radius  # Reference length (diameter)
        
        # Dynamic pressure
        q_dyn = 0.5 * rho * vel_mag**2
        
        # Aerodynamic coefficients (simplified - should come from lookup table)
        # These are example values - replace with actual coefficients for your rocket
        Cl_delta = 0.1  # Roll control effectiveness
        Cm_alpha = -0.5  # Pitch stability derivative
        Cm_delta = 0.2   # Pitch control effectiveness
        Cn_beta = 0.3    # Yaw stability derivative
        Cn_delta = 0.2   # Yaw control effectiveness
        
        # Aerodynamic moments (simplified model)
        LA = q_dyn * S * d * Cl_delta * delta_roll  # Roll moment
        MA = q_dyn * S * d * (Cm_alpha * alpha + Cm_delta * delta_pitch)  # Pitch moment
        NA = q_dyn * S * d * (Cn_beta * beta + Cn_delta * delta_yaw)  # Yaw moment
        
        return LA, MA, NA

    def compute_thrust_moments(self, thrust, cp, cm):
        """
        Compute thrust-induced moments
        
        Args:
            thrust (np.array): thrust vector in body frame [N]
            cp (np.array): center of pressure [m]
            cm (np.array): center of mass [m]
            
        Returns:
            tuple: (Lp, Mp, Np) thrust moments in body frame
        """
        # Moment arm from thrust to center of mass
        moment_arm = cp - cm
        
        # Thrust moment
        thrust_moment = np.cross(moment_arm, thrust)
        
        return thrust_moment[0], thrust_moment[1], thrust_moment[2]  # Lp, Mp, Np

    def compute_state_derivatives(self, p, q, r, LA, Lp, MA, Mp, NA, Np):
        """
        Compute state derivatives based on the paper's rotational dynamics
        
        Args:
            p, q, r: angular rates
            LA, Lp: aerodynamic and thrust roll moments
            MA, Mp: aerodynamic and thrust pitch moments  
            NA, Np: aerodynamic and thrust yaw moments
            
        Returns:
            np.array: state derivatives [φ̇, ṗ, θ̇, q̇, ψ̇, ṙ]^T
        """
        # Extract current angles
        phi = self.x_k[0, 0]  # roll
        theta = self.x_k[2, 0]  # pitch
        psi = self.x_k[4, 0]  # yaw
        
        # Euler angle derivatives (from paper equation 16)
        phi_dot = p + (q * np.sin(phi) + r * np.cos(phi)) * np.tan(theta)
        theta_dot = q * np.cos(phi) - r * np.sin(phi)
        psi_dot = (q * np.sin(phi) + r * np.cos(phi)) / np.cos(theta)
        
        # Angular rate derivatives (from paper equation 11)
        p_dot = (LA + Lp - q * r * (self.Iz - self.Iy)) / self.Ix
        q_dot = (MA + Mp - r * p * (self.Ix - self.Iz)) / self.Iy
        r_dot = (NA + Np - p * q * (self.Iy - self.Ix)) / self.Iz
        
        return np.array([[phi_dot, p_dot, theta_dot, q_dot, psi_dot, r_dot]]).T

    def compute_jacobian(self, p, q, r, LA, Lp, MA, Mp, NA, Np):
        """
        Compute the Jacobian matrix F for the EKF
        
        Args:
            p, q, r: angular rates
            LA, Lp, MA, Mp, NA, Np: moments
            
        Returns:
            np.array: Jacobian matrix F (6x6)
        """
        phi = self.x_k[0, 0]
        theta = self.x_k[2, 0]
        
        # Partial derivatives of state derivatives with respect to states
        # This is the linearized system matrix around the current state
        
        F = np.zeros((self.n, self.n))
        
        # ∂φ̇/∂φ = (q*cos(φ) - r*sin(φ))*tan(θ)
        F[0, 0] = (q * np.cos(phi) - r * np.sin(phi)) * np.tan(theta)
        # ∂φ̇/∂p = 1
        F[0, 1] = 1.0
        # ∂φ̇/∂θ = (q*sin(φ) + r*cos(φ))/cos²(θ)
        F[0, 2] = (q * np.sin(phi) + r * np.cos(phi)) / (np.cos(theta)**2)
        # ∂φ̇/∂q = sin(φ)*tan(θ)
        F[0, 3] = np.sin(phi) * np.tan(theta)
        # ∂φ̇/∂r = cos(φ)*tan(θ)
        F[0, 5] = np.cos(phi) * np.tan(theta)
        
        # ∂θ̇/∂φ = -q*sin(φ) - r*cos(φ)
        F[2, 0] = -q * np.sin(phi) - r * np.cos(phi)
        # ∂θ̇/∂q = cos(φ)
        F[2, 3] = np.cos(phi)
        # ∂θ̇/∂r = -sin(φ)
        F[2, 5] = -np.sin(phi)
        
        # ∂ψ̇/∂φ = (q*cos(φ) - r*sin(φ))/cos(θ)
        F[4, 0] = (q * np.cos(phi) - r * np.sin(phi)) / np.cos(theta)
        # ∂ψ̇/∂θ = (q*sin(φ) + r*cos(φ))*sin(θ)/cos²(θ)
        F[4, 2] = (q * np.sin(phi) + r * np.cos(phi)) * np.sin(theta) / (np.cos(theta)**2)
        # ∂ψ̇/∂q = sin(φ)/cos(θ)
        F[4, 3] = np.sin(phi) / np.cos(theta)
        # ∂ψ̇/∂r = cos(φ)/cos(θ)
        F[4, 5] = np.cos(phi) / np.cos(theta)
        
        # Angular rate derivatives are linear in rates
        # ∂ṗ/∂q = -r*(Iz-Iy)/Ix
        F[1, 3] = -r * (self.Iz - self.Iy) / self.Ix
        # ∂ṗ/∂r = -q*(Iz-Iy)/Ix
        F[1, 5] = -q * (self.Iz - self.Iy) / self.Ix
        
        # ∂q̇/∂p = -r*(Ix-Iz)/Iy
        F[3, 1] = -r * (self.Ix - self.Iz) / self.Iy
        # ∂q̇/∂r = -p*(Ix-Iz)/Iy
        F[3, 5] = -p * (self.Ix - self.Iz) / self.Iy
        
        # ∂ṙ/∂p = -q*(Iy-Ix)/Iz
        F[5, 1] = -q * (self.Iy - self.Ix) / self.Iz
        # ∂ṙ/∂q = -p*(Iy-Ix)/Iz
        F[5, 3] = -p * (self.Iy - self.Ix) / self.Iz
        
        return F

    def predict(self, rho, vel_mag, alpha, beta, thrust, cp, cm, 
                delta_roll=0, delta_pitch=0, delta_yaw=0):
        """
        Prediction step of the EKF
        
        Args:
            rho (float): air density
            vel_mag (float): velocity magnitude
            alpha (float): angle of attack
            beta (float): sideslip angle
            thrust (np.array): thrust vector
            cp (np.array): center of pressure
            cm (np.array): center of mass
            delta_roll, delta_pitch, delta_yaw: control surface deflections
        """
        # Extract current angular rates
        p = self.x_k[1, 0]
        q = self.x_k[3, 0]
        r = self.x_k[5, 0]
        
        # Compute moments
        LA, MA, NA = self.compute_aerodynamic_moments(rho, vel_mag, alpha, beta, 
                                                     delta_roll, delta_pitch, delta_yaw)
        Lp, Mp, Np = self.compute_thrust_moments(thrust, cp, cm)
        
        # Compute state derivatives
        x_dot = self.compute_state_derivatives(p, q, r, LA, Lp, MA, Mp, NA, Np)
        
        # Predict next state (Euler integration)
        self.x_priori = self.x_k + x_dot * self.dt
        
        # Compute Jacobian for covariance propagation
        F = self.compute_jacobian(p, q, r, LA, Lp, MA, Mp, NA, Np)
        
        # Predict covariance
        self.P_priori = F @ self.P_k @ F.T + self.Q

    def update(self, phi_meas, p_meas, theta_meas, q_meas, psi_meas, r_meas):
        """
        Update step of the EKF
        
        Args:
            phi_meas (float): measured roll angle
            p_meas (float): measured roll rate
            theta_meas (float): measured pitch angle
            q_meas (float): measured pitch rate
            psi_meas (float): measured yaw angle
            r_meas (float): measured yaw rate
        """
        # Measurement vector
        y_k = np.array([[phi_meas, p_meas, theta_meas, q_meas, psi_meas, r_meas]]).T
        
        # Kalman gain
        S = self.H @ self.P_priori @ self.H.T + self.R
        K = self.P_priori @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.x_k = self.x_priori + K @ (y_k - self.H @ self.x_priori)
        
        # Update covariance
        I_KH = np.eye(self.n) - K @ self.H
        self.P_k = I_KH @ self.P_priori @ I_KH.T + K @ self.R @ K.T
        
        self.current_time += self.dt

    def get_state(self):
        """Get current state estimate"""
        return self.x_k.copy()
    
    def get_angles(self):
        """Get current angle estimates (roll, pitch, yaw)"""
        return self.x_k[0, 0], self.x_k[2, 0], self.x_k[4, 0]
    
    def get_rates(self):
        """Get current angular rate estimates (p, q, r)"""
        return self.x_k[1, 0], self.x_k[3, 0], self.x_k[5, 0]
    
    def get_covariance(self):
        """Get current state covariance"""
        return self.P_k.copy()


