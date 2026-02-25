This is the rundown of how the EKF is structured for our flights.

Sensor inputs: Accelerometer, GPS, Barometer, Gyroscope data

States tracked: $x_k = [p_x,v_x,p_y,v_y,p_z,v_z]$

Global Frame: NED Coordinate System. We define $x$ as UP, $y$ as East, $z$ as North.
Body Frame: $x$ is out of the antenna, $z$ is out of the board, and $y$ is their cross product.


## Matrices
### $H$ Measurement Matrix (4x6):  
This matrix is responsible for taking sensor inputs and mapping them to our states.

$$H= \begin{bmatrix}
1 & 0 & 0 & 0&0&0\\ 
1 & 0 & 0 & 0&0&0\\
0 & 0 & 1 & 0&0&0\\
0 & 0 & 0 & 0&1&0\\
\end{bmatrix}$$

- H(0, 0) = 1; // barometer measures x position (altitude)
- H(1, 0) = 1; // GPS measures x position (altitude)
- H(2, 2) = 1; // GPS measures y position (east)
- H(3, 4) = 1; // GPS measures z position (north)


### $R$ Measurement Noise Matrix (4x4)
This matrix represents the RMS noise witin each sensor, pulled from the sensor data sheets.
$$R = \begin{bmatrix}
0.1 & 0 & 0 & 0\\ 
0 & 1.5 & 0 & 0\\ 
0 & 0 & 0.3 & 0\\ 
0 & 0 & 0 & 0.3\\ 
\end{bmatrix}$$

- R(0, 0) = 0.1f;  // barometer noise (m)
- R(1, 1) = 1.5f; // GPS altitude noise (m)
- R(2, 2) = 0.3f; // GPS east noise (deg)
- R(3, 3) = 0.3f; // GPS north noise (deg)

### $F$ Transition matrix (6 x 6)
This matrix transitions the Kalman state in between timesteps using the kinematics equations, assuming constant acceleration since we're on a small timestep. This is represented by, where $i$ is the value at the ith iteration. This matrix uses only the state terms-> accel is modeled and used by the $B$ matrix.
$$x_{i+1} = x_i + v_i*dt +\frac{1}{2}a_i(dt)^2 $$
$$v_{i+1} = v_i +a_i*dt $$
$$a_{i+1} = a_i$$

$$F = \begin{bmatrix}
1 & dt & 0 & 0 & 0 & 0\\ 
0 & 1 & 0 & 0 & 0 & 0\\ 
0 & 0 & 1 & dt & 0 & 0\\ 
0 & 0 & 0 & 1 & 0 & 0\\ 
0 & 0 & 0 & 0 & 1 & dt\\ 
0 & 0 & 0 & 0 & 0 & 1\\ 
\end{bmatrix}$$

- F_mat.setIdentity();
- F_mat(0, 1) = dt; // x += vx * dt
- F_mat(2, 3) = dt; // y += vy * dt
- F_mat(4, 5) = dt; // z += vz * dt



### $B$ Control matrix (6 x 3)
This matrix transitions the Kalman state in between timesteps using the kinematics equations, focusing on control input acceleration.

$$x_{i+1} = x_i + v_i*dt +\frac{1}{2}a_i(dt)^2 $$
$$v_{i+1} = v_i +a_i*dt $$
$$a_{i+1} = a_i$$

$$B = \begin{bmatrix}
\frac{1}{2}a_i(dt)^2  & 0 & 0\\ 
dt & 0 & 0\\ 
0 & \frac{1}{2}a_i(dt)^2 & 0\\ 
0 & dt & 0\\ 
0 & 0 & \frac{1}{2}a_i(dt)^2\\ 
0 & 0 & dt\\ 
\end{bmatrix}$$

- F_mat.setIdentity();
- F_mat(0, 1) = dt; // x += vx * dt
- F_mat(2, 3) = dt; // y += vy * dt
- F_mat(4, 5) = dt; // z += vz * dt