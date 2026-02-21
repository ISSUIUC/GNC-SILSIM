This is the README for the Extended Kalman Filter.

The filter is stored in ekf.cpp and uses ekf.h as its header file. Both are implementations of the kalman_filter.h.

States tracked: $[p_x,p_y,p_z,v_x,v_y,v_z]$. Position is meters, velocity is m/s. Acceleration is treated as a control input.

**MIDAS Sensors**
GPS (MAX-M10S):
https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf

Units: Altitude (m), Latitude/Longitude (deg)
Noise (Altitude): 1.5m
Noise (Latitude/Longitude): 0.3deg

IMU (KX134-1211):
https://www.rohm.com/products/sensors-mems/accelerometer-ics/kx134-1211-product

- Reference Frame: IMU reference frame is NOT rotated. It matches exactly with MIDAS's reference frame so we keep it as such.
  ![IMU Reference](image.png)
- Accel ($\frac{m}{s^2}$): RMS Noise is 1.6mg. Noise density is 300 $\frac{ug}{Hz}$
- Noise varies with ODR, power mode, and the Average Filter Control (AVC) settings. Measured with RES = 1, ODR = 50Hz, LPRO = 1, GSEL = 0 settings

Magnetometer (BNO086):
https://www.oakchina.cn/wp-content/uploads/2022/05/BNO080_085-Datasheet.pdf

- Units: µTesla
- Reference frame: ![alt text](image-1.png). We rotate this to match with our rocket's frame. X -> -X, Y -> Y, Z -> -Z (NOT IMPLEMENTED YET)
- Noise: 1.4uT

Barometer (MS5611-01BA03):
https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5611-01BA03&DocType=Data+Sheet&DocLang=English

- Units: converted to meters with altitude function
- Reference frame: Not considered
- Noise (Resolution): 0.1m

Gyroscope (BNO086):
https://www.oakchina.cn/wp-content/uploads/2022/05/BNO080_085-Datasheet.pdf

- Units: Angular velocity (rad/s), Roll Pitch Yaw (deg)
- Yaw: The yaw is a measure of the rotation around the Z-axis since reset. The yaw has a range of +/- 180˚ and is
provided in 0.01˚ increments, i.e. a report of 8734 is equivalent to 87.34˚.
- Pitch: The pitch is a measure of the rotation around the Y-axis. The pitch has a range of +/- 90˚ and is provided in
0.01˚ increments, i.e. a report of 1072 is equivalent to 10.72˚.
- Roll: The roll is a measure of the rotation around the X-axis. The roll has a range of +/- 180˚ and is provided in
0.01˚ increments, i.e. a report of 1072 is equivalent to 10.72˚.
- Reference frame: ![alt text](image-1.png). We rotate this to match with our rocket's frame. X -> -X, Y -> Y, Z -> -Z (NOT IMPLEMENTED YET)
- To determine the actual orientation of the module, the rotations should be applied in the order yaw, pitch then roll.
- Noise: 1.4uT

**MIDAS Mini Sensors**
GPS (name of sensor):

- Altitude (m): Noise is 
- Latitude (deg): Noise is
- Longitude (deg): Noise is

Accelerometer (LSM6DSV320X):

-

Magnetometer (MMC5983MA):

-

Barometer (MS5611):
Temp barom docs //TO DELETE//

- OSR is highest mode 4096 -> sensor rms .012 mbar
