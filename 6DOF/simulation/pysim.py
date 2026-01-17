# ______      _____ _              __________ ___________ 
# | ___ \    /  ___(_)            / ___|  _  \  _  |  ___|
# | |_/ /   _\ `--. _ _ __ ___   / /___| | | | | | | |_   
# |  __/ | | |`--. \ | '_ ` _ \  | ___ \ | | | | | |  _|  
# | |  | |_| /\__/ / | | | | | | | \_/ | |/ /\ \_/ / |    
# \_|   \__, \____/|_|_| |_| |_| \_____/___/  \___/\_|    
#        __/ |                                            
#       |___/                                             

# A 6DOF RK-4 Based simulation that uses RASAero aerodynamic data and known motor thrust data to simulate motion of the rocket
# with simulated sensor data as well as a implementation of the Extended Kalman Filter and active drag PID controller for the ISS
# Spaceshot entry for the 2023 IREC competition. This simulation was used to quantify the effects of the airbrakes, test 
# different system design methodlogies, and provide preliminary tuning for the EKF and controller prior to implementation in 
# SILSIM and flight software.
# 
# 2022-2023 Guidance, Navigation, and Control Main Contributors #
# Sub-Team Lead: Parth Shrotri (2024)
# Colin Kinsey (2024)
# Evan Yu (2025)
# Rithvik Bhogavilli (2025)
# Kabir Cheema (2025)
# Freya Bansal (2025)
# Ishaan Bansal (2025)
# Ethan Pereira (2026)

# 2024-2025 Guidance, Navigation, and Control Main Contributors #
# Sub-Team Lead: Shishir Bhatta (2026)
# Ishaan Kandamuri (2026)
# Keshav Balaji (2026)
# William Yeh (2026)
# Divij Garg (2026)
# Adi Srikanth (2026)
# Aneesh Ganti (2028)

import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import shutil

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..')))

# import estimation.ekf as ekf  # Not needed - EKF will be run by test_ekf.cpp
# import estimation.r_ekf as r_ekf  # Not needed - EKF will be run by test_ekf.cpp
import properties.properties as prop
import properties.data_loader as dataloader
import simulator as sim_class
import dynamics.sensors as sensors
import time
import dynamics.rocket as rocket_model
import environment.atmosphere as atmosphere

# Load desired config file
config = dataloader.config

# Runs simulation for the specific component of the rocket
class Simulation:
    # dt can be dynamic in the future, so we need to 
    def __init__(self, rocket, motor, dt, x0, time_stamp=0, stages=[]):
        self.rocket = rocket
        self.stages = stages
        self.motor = motor
        self.dt = dt
        self.x = x0.copy()
        self.baro = 0
        self.time_stamp = time_stamp
        self.sensor_config = self.rocket.stage_config['sensors']
 
    # Call on Navigation class?

    def time_step(self):
        self.time_stamp += self.dt

    def idle_stage(self):
        while self.time_stamp < self.rocket.delay:
            baro_alt, accel, gyro, bno_ang_pos = self.get_sensor_data()
            # EKF not run here - will be run by test_ekf.cpp
            # Create dummy kalman data (zeros) since EKF will compute it
            current_state = np.zeros(9)
            current_covariance = np.zeros(9)
            current_state_r = np.zeros(9)
            fsm_state = "STATE_IDLE"

            self.rocket.add_to_dict(self.x, baro_alt, accel, bno_ang_pos, gyro, current_state, current_covariance, current_state_r, 0, self.rocket.get_rocket_dry_mass(), self.rocket.get_total_motor_mass(self.time_stamp), 0, dt, fsm_state)
            self.time_step()

    def execute_stage(self):
        # Run the stages
        stage_separation_delay = 1
        self.rocket.get_motor().ignite(self.time_stamp)

        ignition_time = self.time_stamp 
        start = True
        print(f"Staged at {self.time_stamp}")
        burn_time = self.rocket.get_motor().get_burn_time()
        
        while self.time_stamp < ignition_time + burn_time + stage_separation_delay:
            # Get sensor data
            baro_alt, accel, gyro, bno_ang_pos = self.get_sensor_data()
            
            # EKF not run here - will be run by test_ekf.cpp
            # Create dummy kalman data (zeros) since EKF will compute it
            current_state = np.zeros(9)
            current_covariance = np.zeros(9)
            current_state_r = np.zeros(9)
            
            # Determine FSM state based on flight phase
            time_since_ignition = self.time_stamp - ignition_time
            if self.rocket.current_stage == -1:
                fsm_state = "STATE_FIRST_BOOST" if time_since_ignition < burn_time else "STATE_BURNOUT"
            elif self.rocket.current_stage == 0:
                fsm_state = "STATE_SECOND_BOOST" if time_since_ignition < burn_time else "STATE_BURNOUT"
            else:
                fsm_state = "STATE_FIRST_BOOST" if time_since_ignition < burn_time else "STATE_BURNOUT"

            self.rocket.set_motor_mass(self.time_stamp)

            is_staging = start and self.rocket.current_stage != -1
            self.x, alpha = sim.RK4(self.x, dt, self.time_stamp, is_staging, 0)

            self.rocket.add_to_dict(self.x, baro_alt, accel, bno_ang_pos, gyro, current_state, current_covariance, current_state_r, alpha, self.rocket.get_rocket_dry_mass(), self.rocket.get_total_motor_mass(self.time_stamp), 0, dt, fsm_state)
            self.time_step()
            if start:
                start = False

    def run_stages(self):
        has_more_stages = True
        while has_more_stages:
            self.execute_stage()
            has_more_stages = self.rocket.separate_stage(self.time_stamp)

    # Function to retrive all sensor data
    def get_sensor_data(self):
        return (self.rocket.get_barometer_data(self.x, self.sensor_config),
                self.rocket.get_accelerometer_data(self.x, self.sensor_config),
                self.rocket.get_gyro_data(self.x, self.sensor_config), 
                self.rocket.get_bno_orientation(self.x, self.sensor_config))

    def coast(self):
        max_altitude = 0.0
        apogee_reached = False
        
        while self.x[0, 0] >= 0:  # While altitude >= 0
            # Get sensor data
            baro_alt, accel, gyro, bno_ang_pos = self.get_sensor_data()
            
            # EKF not run here - will be run by test_ekf.cpp
            # Create dummy kalman data (zeros) since EKF will compute it
            current_state = np.zeros(9)
            current_covariance = np.zeros(9)
            current_state_r = np.zeros(9)
            
            # Track apogee and determine FSM state
            current_altitude = self.x[0, 0]
            if current_altitude > max_altitude:
                max_altitude = current_altitude
                fsm_state = "STATE_COAST"  # Still ascending, in coast phase
            elif not apogee_reached and current_altitude < max_altitude - 1.0:  # 1m tolerance
                apogee_reached = True
                fsm_state = "STATE_APOGEE"
            elif apogee_reached:
                fsm_state = "STATE_COAST"  # Past apogee, descending
            else:
                fsm_state = "STATE_COAST"  # Default to coast

            self.x, alpha = sim.RK4(self.x, dt, self.time_stamp, False, 0)

            self.rocket.add_to_dict(self.x, baro_alt, accel, bno_ang_pos, gyro, current_state, current_covariance, current_state_r, alpha, self.rocket.get_rocket_dry_mass(), self.rocket.get_total_motor_mass(self.time_stamp), 0, dt, fsm_state)
            self.time_step()
    
def simulator(x0, rocket, motor, dt) -> None:
    '''Method which handles running the simulation and logging sim data to dict

    Args:
        x0 (np.array): state vector initialized to 0s [6x3]
             x:         y:         z:
           [[pos,       pos,       pos],
            [vel,       vel,       vel],
            [accel,     accel,     accel],
            [ang_pos,   ang_pos,   ang_pos],
            [ang_vel,   ang_vel,   ang_vel],
            [ang_accel, ang_accel, ang_accel]]
        dt (float): time step between each iteration in simulation
    '''
    simulator = Simulation(rocket, motor, dt, x0, stages=rocket.stages)
    simulator.idle_stage()
    t_start = time.time()
    simulator.run_stages()
    simulator.coast()
    t_end = time.time() - t_start
    print(f"Runtime: {t_end:.2f} seconds")


if __name__ == '__main__':
    x0 = np.zeros((6, 3))
    x0[3] = [0, 0.05, 0]
    dt = 0.01

    atm = atmosphere.Atmosphere(enable_direction_variance=True, enable_magnitude_variance=True)

    stages = []
    for stage in config['rocket']['stages'][1:]:
        stages.append(rocket_model.Rocket(dt, x0, stage, atm=atm))
    rocket = rocket_model.Rocket(dt, x0, config['rocket']['stages'][0], atm=atm, stages=stages)

    motor = rocket.motor
    sim = sim_class.Simulator(atm=atm, rocket=rocket)

    simulator(x0, rocket, motor, dt)

    print("Writing to MIDAS format CSV file...")

    record = rocket.to_midas_csv()
    # Save to data folder with MIDAS Trimmed naming convention
    data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'data')
    os.makedirs(data_dir, exist_ok=True)
    output_file = os.path.join(data_dir, "6DOF_RK4_SIMULATED.csv")
    
    # MIDAS Trimmed CSV header
    midas_header = "sensor,file number,timestamp,lowg.ax,lowg.ay,lowg.az,highg.ax,highg.ay,highg.az,barometer.temperature,barometer.pressure,barometer.altitude,continuity.pins[0],continuity.pins[1],continuity.pins[2],continuity.pins[3],voltage.voltage,voltage.current,gps.latitude,gps.longitude,gps.altitude,gps.speed,gps.fix_type,gps.sats_in_view,gps.time,magnetometer.mx,magnetometer.my,magnetometer.mz,orientation.has_data,orientation.reading_type,orientation.yaw,orientation.pitch,orientation.roll,orientation.orientation_velocity.vx,orientation.orientation_velocity.vy,orientation.orientation_velocity.vz,orientation.angular_velocity.vx,orientation.angular_velocity.vy,orientation.angular_velocity.vz,orientation.orientation_acceleration.ax,orientation.orientation_acceleration.ay,orientation.orientation_acceleration.az,orientation.linear_acceleration.ax,orientation.linear_acceleration.ay,orientation.linear_acceleration.az,orientation.gx,orientation.gy,orientation.gz,orientation.magnetometer.mx,orientation.magnetometer.my,orientation.magnetometer.mz,orientation.temperature,orientation.pressure,orientation.tilt,orientation.orientation_quaternion.w,orientation.orientation_quaternion.x,orientation.orientation_quaternion.y,orientation.orientation_quaternion.z,lowglsm.gx,lowglsm.gy,lowglsm.gz,lowglsm.ax,lowglsm.ay,lowglsm.az,fsm,kalman.position.px,kalman.position.py,kalman.position.pz,kalman.velocity.vx,kalman.velocity.vy,kalman.velocity.vz,kalman.acceleration.ax,kalman.acceleration.ay,kalman.acceleration.az,kalman.altitude,pyro.is_global_armed,pyro.channel_firing[0],pyro.channel_firing[1],pyro.channel_firing[2],pyro.channel_firing[3],cameradata.camera_state,cameradata.camera_voltage\n"
    
    with open(output_file, 'w') as f:
        f.write(midas_header)
        for point in record:
            f.write(f"{','.join(point)}\n")
    
    print(f"MIDAS format CSV written to: {output_file}")
    print(f"Total data points: {len(record)}")
    print("This file can now be used with test_ekf.cpp")