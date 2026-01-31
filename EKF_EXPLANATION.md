# Extended Kalman Filter (EKF) Explanation

## State Model

### State Vector (x_k)
Your state vector has **6 states**:
```
x_k = [x, vx, y, vy, z, vz]ᵀ
```

Where:
- **x** = position in X direction (altitude/up)
- **vx** = velocity in X direction
- **y** = position in Y direction (east)
- **vy** = velocity in Y direction
- **z** = position in Z direction (north)
- **vz** = velocity in Z direction

### State Transition Model (Prediction/Propagation)

The state evolves according to:
```
x_{k+1} = F * x_k + B * u + w_k
```

Where:
- **F** = State transition matrix (6×6)
- **B** = Control input matrix (6×3)
- **u** = Control input vector [ax, ay, az]ᵀ (acceleration from accelerometer)
- **w_k** = Process noise (Gaussian, zero mean, covariance Q)

### F Matrix (State Transition Matrix) - 6×6

The F matrix describes how states evolve over time:

```
F = [1  dt  0   0   0   0 ]   x_{k+1} = x_k + vx_k * dt
    [0  1   0   0   0   0 ]   vx_{k+1} = vx_k  (accel via B*u)
    [0  0   1   dt  0   0 ]   y_{k+1} = y_k + vy_k * dt
    [0  0   0   1   0   0 ]   vy_{k+1} = vy_k  (accel via B*u)
    [0  0   0   0   1   dt]   z_{k+1} = z_k + vz_k * dt
    [0  0   0   0   0   1 ]   vz_{k+1} = vz_k  (accel via B*u)
```

**Key points:**
- Position states integrate velocity: `pos_{k+1} = pos_k + vel_k * dt`
- Velocity states remain constant (acceleration applied via B matrix)
- Each axis (x, y, z) has independent dynamics

### B Matrix (Control Input Matrix) - 6×3

The B matrix maps acceleration control inputs to velocity changes:

```
B = [0   0   0 ]   x doesn't change from acceleration
    [dt  0   0 ]   vx += ax * dt
    [0   0   0 ]   y doesn't change from acceleration
    [0   dt  0 ]   vy += ay * dt
    [0   0   0 ]   z doesn't change from acceleration
    [0   0   dt]   vz += az * dt
```

**Control input u = [ax, ay, az]ᵀ** comes from:
- Accelerometer readings transformed to global frame
- Converted from g's to m/s²
- Accounts for gravity and sensor bias

### Q Matrix (Process Noise Covariance) - 6×6

Q models uncertainty in the state transition. Uses Singer model for position-velocity:

```
Q = [Q_x  0    0  ]
    [0    Q_y  0  ]
    [0    0    Q_z]

Where each Q_i (2×2) for axis i:
Q_i = [dt⁵/20  dt⁴/8 ] * spectral_density
      [dt⁴/8   dt³/3 ]
```

**What it means:**
- Position uncertainty grows as dt⁵ (very fast)
- Velocity uncertainty grows as dt³ (moderate)
- Cross-correlation between position and velocity
- Scaled by `spectral_density` (currently 13.0)

---

## Measurement Model (Update)

### Measurement Equation

```
y_k = H * x_k + v_k
```

Where:
- **y_k** = Measurement vector
- **H** = Measurement matrix (maps state to measurements)
- **v_k** = Measurement noise (Gaussian, zero mean, covariance R)

### H Matrix (Measurement Matrix)

You have **two measurement sources**:

#### 1. Barometer (H_baro) - 1×6
```
H_baro = [1  0  0  0  0  0]
```
Measures: **x position (altitude) only**

#### 2. GPS (H_gps) - 3×6
```
H_gps = [1  0  0  0  0  0]   Measures x position (altitude)
        [0  0  1  0  0  0]   Measures y position (east)
        [0  0  0  0  1  0]   Measures z position (north)
```
Measures: **x, y, z positions**

### R Matrix (Measurement Noise Covariance)

#### R_baro (Barometer) - 1×1
```
R_baro = [1.9]
```
Barometer noise: **1.9 m²** (moderate trust)

#### R_gps (GPS) - 3×3
```
R_gps = [5.0   0     0   ]   GPS altitude noise (lower trust)
        [0    0.01   0   ]   GPS east noise (very high trust)
        [0     0   0.01 ]   GPS north noise (very high trust)
```
GPS noise: **5.0 m²** for altitude, **0.01 m²** for horizontal (very high trust)

---

## Kalman Filter Algorithm

### 1. Prediction Step (Priori)

```
x_priori = F * x_k + B * u
P_priori = F * P_k * Fᵀ + Q
```

**What happens:**
- Predict next state using dynamics model
- Predict next covariance (uncertainty grows)
- Apply acceleration control input

### 2. Update Step (Posteriori)

#### For Barometer:
```
innovation = y_baro - H_baro * x_priori
S = H_baro * P_priori * H_baroᵀ + R_baro
K = P_priori * H_baroᵀ * S⁻¹
x_k = x_priori + K * innovation
P_k = (I - K * H_baro) * P_priori
```

#### For GPS:
```
innovation = y_gps - H_gps * x_priori
S = H_gps * P_priori * H_gpsᵀ + R_gps
K = P_priori * H_gpsᵀ * S⁻¹
x_k = x_priori + K * innovation
P_k = (I - K * H_gps) * P_priori
```

**Key matrices:**
- **innovation** = Measurement residual (difference between measurement and prediction)
- **S** = Innovation covariance (uncertainty in the innovation)
- **K** = Kalman gain (how much to trust the measurement vs prediction)
- **P_k** = State covariance (uncertainty in state estimate)

---

## About Jacobians

**Note:** Your filter is actually a **Linear Kalman Filter**, not an Extended Kalman Filter!

### Why No Jacobian?

You're using **linear** models:
- **F matrix** is constant (doesn't depend on state)
- **H matrix** is constant (linear measurement model)
- **B matrix** is constant

### When Would You Need Jacobians?

You'd need Jacobians (F = ∂f/∂x, H = ∂h/∂x) if:
- State transition was nonlinear: `x_{k+1} = f(x_k, u_k)`
- Measurement model was nonlinear: `y_k = h(x_k)`

**Example:** If you had:
- Rotational dynamics (quaternions, Euler angles)
- Range/bearing measurements
- GPS coordinates directly (lat/lon) instead of converted to ENU

### Your Current Model

Since everything is linear, you're using:
- **F** = constant state transition matrix
- **H** = constant measurement matrix
- No Jacobian computation needed!

---

## Summary

**State Model:**
- 6 states: [x, vx, y, vy, z, vz]
- Linear dynamics: position integrates velocity, velocity changes via acceleration
- Acceleration is a **control input**, not a state

**Measurement Model:**
- Barometer: measures altitude (x)
- GPS: measures altitude (x), east (y), north (z)
- Both are linear measurements

**Filter Type:**
- **Linear Kalman Filter** (not Extended)
- No Jacobians needed (all models are linear)
- Uses standard KF equations

**Key Insight:**
Acceleration is treated as a **known control input** (from accelerometer) rather than an **unknown state**, which is why your filter works well and avoids the velocity divergence issues you had before!
