#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include "ekf.h"
#include "sensor_data.h"
#include "systems.h"

struct FlightData
{
    std::string sensor;
    int file_number;
    float timestamp;
    LowGData lowg;
    HighGData highg;
    Barometer barometer;
    Continuity continuity;
    Voltage voltage;
    GPS gps;
    Magnetometer magnetometer;
    Orientation orientation;
    LowGLSMData lowglsm;
    FSMState fsm;
    KalmanData kalman;
    Pyro pyro;

    float barometer_altitude;
    float highg_ax, highg_ay, highg_az;
};

class CSVReader
{
private:
    std::vector<std::string> headers;
    std::map<std::string, int> header_index;

    static bool isValidNumber(const std::string &s)
    {
        if (s.empty())
            return false;
        if (s == "NaN" || s == "nan" || s == "NAN")
            return false;
        return true;
    }

    static float toFloatOr(const std::string &s, float fallback)
    {
        if (!isValidNumber(s))
            return fallback;
        try
        {
            return std::stof(s);
        }
        catch (...)
        {
            return fallback;
        }
    }

public:
    std::vector<FlightData> readCSV(const std::string &filename)
    {
        std::vector<FlightData> data;
        std::ifstream file(filename);
        std::string line;

        if (!file.is_open())
        {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return data;
        }

        if (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string header;
            int index = 0;
            while (std::getline(ss, header, ','))
            {
                headers.push_back(header);
                header_index[header] = index++;
            }
        }

        FlightData last_valid{};
        last_valid.sensor = "";
        last_valid.file_number = 0;
        last_valid.timestamp = 0.0f;
        while (std::getline(file, line))
        {
            FlightData row;
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> values;

            while (std::getline(ss, cell, ','))
            {
                values.push_back(cell);
            }

            if (values.size() < headers.size())
            {
                continue;
            }

            try
            {
                row = last_valid;

                {
                    std::string v = getValue(values, "sensor");
                    if (!v.empty() && v != "NaN")
                        row.sensor = v;
                }
                {
                    std::string v = getValue(values, "file number");
                    if (!v.empty() && v != "NaN")
                    {
                        try
                        {
                            row.file_number = std::stoi(v);
                        }
                        catch (...)
                        {
                        }
                    }
                }
                row.timestamp = toFloatOr(getValue(values, "timestamp"), row.timestamp);

                row.gps.latitude = toFloatOr(getValue(values, "gps.latitude"), row.gps.latitude);
                row.gps.longitude = toFloatOr(getValue(values, "gps.longitude"), row.gps.longitude);
                row.gps.altitude = toFloatOr(getValue(values, "gps.altitude"), row.gps.altitude);
                row.gps.fix_type = toFloatOr(getValue(values, "gps.fix_type"), row.gps.fix_type);
                row.gps.time = toFloatOr(getValue(values, "gps.time"), row.gps.time);

                row.lowg.ax = toFloatOr(getValue(values, "lowg.ax"), row.lowg.ax);
                row.lowg.ay = toFloatOr(getValue(values, "lowg.ay"), row.lowg.ay);
                row.lowg.az = toFloatOr(getValue(values, "lowg.az"), row.lowg.az);

                row.highg.ax = toFloatOr(getValue(values, "highg.ax"), row.highg.ax);
                row.highg.ay = toFloatOr(getValue(values, "highg.ay"), row.highg.ay);
                row.highg.az = toFloatOr(getValue(values, "highg.az"), row.highg.az);

                row.barometer.temperature = toFloatOr(getValue(values, "barometer.temperature"), row.barometer.temperature);
                row.barometer.pressure = toFloatOr(getValue(values, "barometer.pressure"), row.barometer.pressure);
                row.barometer.altitude = toFloatOr(getValue(values, "barometer.altitude"), row.barometer.altitude);

                {
                    std::string v = getValue(values, "orientation.has_data");
                    if (!v.empty() && v != "NaN")
                    {
                        try
                        {
                            row.orientation.has_data = std::stoi(v) != 0;
                        }
                        catch (...)
                        {
                        }
                    }
                }
                {
                    std::string v = getValue(values, "orientation.reading_type");
                    if (!v.empty() && v != "NaN")
                        row.orientation.reading_type = v;
                }
                row.orientation.yaw = toFloatOr(getValue(values, "orientation.yaw"), row.orientation.yaw);
                row.orientation.pitch = toFloatOr(getValue(values, "orientation.pitch"), row.orientation.pitch);
                row.orientation.roll = toFloatOr(getValue(values, "orientation.roll"), row.orientation.roll);

                row.orientation.orientation_velocity.vx = toFloatOr(getValue(values, "orientation.orientation_velocity.vx"), row.orientation.orientation_velocity.vx);
                row.orientation.orientation_velocity.vy = toFloatOr(getValue(values, "orientation.orientation_velocity.vy"), row.orientation.orientation_velocity.vy);
                row.orientation.orientation_velocity.vz = toFloatOr(getValue(values, "orientation.orientation_velocity.vz"), row.orientation.orientation_velocity.vz);

                row.orientation.angular_velocity.vx = toFloatOr(getValue(values, "orientation.angular_velocity.vx"), row.orientation.angular_velocity.vx);
                row.orientation.angular_velocity.vy = toFloatOr(getValue(values, "orientation.angular_velocity.vy"), row.orientation.angular_velocity.vy);
                row.orientation.angular_velocity.vz = toFloatOr(getValue(values, "orientation.angular_velocity.vz"), row.orientation.angular_velocity.vz);

                row.orientation.quaternion.w = toFloatOr(getValue(values, "orientation.orientation_quaternion.w"), row.orientation.quaternion.w);
                row.orientation.quaternion.x = toFloatOr(getValue(values, "orientation.orientation_quaternion.x"), row.orientation.quaternion.x);
                row.orientation.quaternion.y = toFloatOr(getValue(values, "orientation.orientation_quaternion.y"), row.orientation.quaternion.y);
                row.orientation.quaternion.z = toFloatOr(getValue(values, "orientation.orientation_quaternion.z"), row.orientation.quaternion.z);

                {
                    std::string fsm_str = getValue(values, "fsm");
                    if (!fsm_str.empty() && fsm_str != "NaN")
                    {
                        if (fsm_str == "STATE_SAFE")
                            row.fsm = FSMState::STATE_SAFE;
                        else if (fsm_str == "STATE_PYRO_TEST")
                            row.fsm = FSMState::STATE_PYRO_TEST;
                        else if (fsm_str == "STATE_IDLE")
                            row.fsm = FSMState::STATE_IDLE;
                        else if (fsm_str == "STATE_FIRST_BOOST")
                            row.fsm = FSMState::STATE_FIRST_BOOST;
                        else if (fsm_str == "STATE_BURNOUT")
                            row.fsm = FSMState::STATE_BURNOUT;
                        else if (fsm_str == "STATE_COAST")
                            row.fsm = FSMState::STATE_COAST;
                        else if (fsm_str == "STATE_APOGEE")
                            row.fsm = FSMState::STATE_APOGEE;
                        else if (fsm_str == "STATE_DROGUE_DEPLOY")
                            row.fsm = FSMState::STATE_DROGUE_DEPLOY;
                        else if (fsm_str == "STATE_DROGUE")
                            row.fsm = FSMState::STATE_DROGUE;
                        else if (fsm_str == "STATE_MAIN_DEPLOY")
                            row.fsm = FSMState::STATE_MAIN_DEPLOY;
                        else if (fsm_str == "STATE_MAIN")
                            row.fsm = FSMState::STATE_MAIN;
                        else if (fsm_str == "STATE_LANDED")
                            row.fsm = FSMState::STATE_LANDED;
                        else if (fsm_str == "STATE_SUSTAINER_IGNITION")
                            row.fsm = FSMState::STATE_SUSTAINER_IGNITION;
                        else if (fsm_str == "STATE_SECOND_BOOST")
                            row.fsm = FSMState::STATE_SECOND_BOOST;
                        else if (fsm_str == "STATE_FIRST_SEPARATION")
                            row.fsm = FSMState::STATE_FIRST_SEPARATION;
                    }
                }

                row.barometer_altitude = row.barometer.altitude;
                row.highg_ax = row.highg.ax;
                row.highg_ay = row.highg.ay;
                row.highg_az = row.highg.az;

                last_valid = row;

                data.push_back(row);
            }
            catch (const std::exception &e)
            {
                continue;
            }
        }

        file.close();
        return data;
    }

private:
    std::string getValue(const std::vector<std::string> &values, const std::string &header)
    {
        auto it = header_index.find(header);
        if (it != header_index.end() && it->second < values.size())
        {
            return values[it->second];
        }
        return "0";
    }
};

class EKFSimulator
{
private:
    EKF ekf;
    RocketSystems rocket_systems;
    std::vector<FlightData> flight_data;
    std::vector<KalmanData> results;
    float start_time;
    FSMState stop_state;

    static const char *fsmToString(FSMState s)
    {
        switch (s)
        {
        case FSMState::STATE_SAFE:
            return "STATE_SAFE";
        case FSMState::STATE_PYRO_TEST:
            return "STATE_PYRO_TEST";
        case FSMState::STATE_IDLE:
            return "STATE_IDLE";
        case FSMState::STATE_FIRST_BOOST:
            return "STATE_FIRST_BOOST";
        case FSMState::STATE_BURNOUT:
            return "STATE_BURNOUT";
        case FSMState::STATE_COAST:
            return "STATE_COAST";
        case FSMState::STATE_APOGEE:
            return "STATE_APOGEE";
        case FSMState::STATE_DROGUE_DEPLOY:
            return "STATE_DROGUE_DEPLOY";
        case FSMState::STATE_DROGUE:
            return "STATE_DROGUE";
        case FSMState::STATE_MAIN_DEPLOY:
            return "STATE_MAIN_DEPLOY";
        case FSMState::STATE_MAIN:
            return "STATE_MAIN";
        case FSMState::STATE_LANDED:
            return "STATE_LANDED";
        case FSMState::STATE_SUSTAINER_IGNITION:
            return "STATE_SUSTAINER_IGNITION";
        case FSMState::STATE_SECOND_BOOST:
            return "STATE_SECOND_BOOST";
        case FSMState::STATE_FIRST_SEPARATION:
            return "STATE_FIRST_SEPARATION";
        default:
            return "UNKNOWN STATE";
        }
        return "STATE_SAFE";
    }

public:
    EKFSimulator(const std::string &csv_filename, FSMState stop_state = FSMState::STATE_LANDED)
        : stop_state(stop_state)
    {
        CSVReader reader;
        flight_data = reader.readCSV(csv_filename);
        std::cout << "Loaded " << flight_data.size() << " data points" << std::endl;

        if (!flight_data.empty())
        {
            start_time = flight_data[0].timestamp;
        }
    }

    void runSimulation()
    {
        if (flight_data.empty())
        {
            std::cerr << "No flight data available" << std::endl;
            return;
        }

        std::cout << "Starting EKF simulation..." << std::endl;

        initializeEKF();

        float last_timestamp = start_time;

        for (size_t i = 0; i < flight_data.size(); i++)
        {
            const FlightData &data = flight_data[i];

            // if (data.fsm == stop_state) {
            //     std::cout << "Reached " << fsmToString(stop_state) << " at time " << (data.timestamp - start_time) / 1000.0f << "s" << std::endl;
            //     std::cout << "Stopping simulation at data point " << i << std::endl;
            //     break;
            // }

            float dt = (data.timestamp - last_timestamp) / 1000.0f;
            if (dt < 0 || dt > 1.0f)
                dt = 0.05f;

            updateSensorBuffers(data);

            Barometer current_barom = data.barometer;
            Acceleration current_accel = {data.highg.ax, data.highg.ay, data.highg.az};
            Orientation current_orientation = data.orientation;
            FSMState current_fsm = data.fsm;
            GPS current_gps = data.gps;

            ekf.tick(dt, 13.0f, current_barom, current_accel, current_orientation, current_fsm, current_gps);

            KalmanData current_state = ekf.getState();
            results.push_back(current_state);

            if (i % 1000 == 0)
            {
                std::cout << "Processed " << i << " data points" << std::endl;
                std::cout << "Time: " << (data.timestamp - start_time) / 1000.0f << "s" << std::endl;
                std::cout << "FSM: " << fsmToString(current_fsm) << std::endl;
                std::cout << "Position: (" << current_state.position.px << ", "
                          << current_state.position.py << ", " << current_state.position.pz << ")" << std::endl;
                std::cout << "Velocity: (" << current_state.velocity.vx << ", "
                          << current_state.velocity.vy << ", " << current_state.velocity.vz << ")" << std::endl;
                std::cout << "Acceleration: (" << current_state.acceleration.ax << ", "
                          << current_state.acceleration.ay << ", " << current_state.acceleration.az << ")" << std::endl;
                std::cout << "---" << std::endl;
            }

            last_timestamp = data.timestamp;
        }

        std::cout << "Simulation completed. Processed " << results.size() << " data points." << std::endl;
    }

    void saveResults(const std::string &filename)
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Error: Could not create output file " << filename << std::endl;
            return;
        }

        file << "timestamp,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,altitude,fsm,raw_gps_latitude,raw_gps_longitude,raw_gps_altitude,raw_baro_alt,raw_highg_ax,raw_highg_ay,raw_highg_az" << std::endl;

        for (size_t i = 0; i < results.size() && i < flight_data.size(); i++)
        {
            const KalmanData &result = results[i];
            const FlightData &data = flight_data[i];

            file << std::fixed << std::setprecision(6)
                 << (data.timestamp - start_time) / 1000.0f << ","
                 << result.position.px << ","
                 << result.position.py << ","
                 << result.position.pz << ","
                 << result.velocity.vx << ","
                 << result.velocity.vy << ","
                 << result.velocity.vz << ","
                 << result.acceleration.ax << ","
                 << result.acceleration.ay << ","
                 << result.acceleration.az << ","
                 << result.altitude << ","
                 << fsmToString(data.fsm) << ","
                 << data.gps.latitude << ","
                 << data.gps.longitude << ","
                 << data.gps.altitude << ","
                 << data.barometer_altitude << ","
                 << data.highg_ax << ","
                 << data.highg_ay << ","
                 << data.highg_az << std::endl;
        }

        file.close();
        std::cout << "Results saved to " << filename << std::endl;
    }

private:
    void initializeEKF()
    {
        for (int i = 0; i < std::min(30, (int)flight_data.size()); i++)
        {
            updateSensorBuffers(flight_data[i]);
        }

        ekf.initialize(&rocket_systems);
        std::cout << "EKF initialized" << std::endl;
    }

    void updateSensorBuffers(const FlightData &data)
    {
        rocket_systems.rocket_data.barometer.push(data.barometer);
        rocket_systems.rocket_data.orientation.push(data.orientation);
        rocket_systems.rocket_data.high_g.push(data.highg);
        rocket_systems.rocket_data.low_g.push(data.lowg);
        rocket_systems.rocket_data.fsm_state.push(data.fsm);
        rocket_systems.rocket_data.gps.push(data.gps);
    }
};

int main(int argc, char *argv[])
{
    std::string csv_filename = "MIDAS Sustainer (Trimmed CSV).csv";
    std::string output_filename = "ekf_results.csv";
    FSMState stop_state = FSMState::STATE_LANDED;

    if (argc > 1)
    {
        csv_filename = argv[1];
    }
    if (argc > 2)
    {
        output_filename = argv[2];
    }
    if (argc > 3)
    {
        std::string stop_state_str = argv[3];
        if (stop_state_str == "STATE_SAFE")
            stop_state = FSMState::STATE_SAFE;
        else if (stop_state_str == "STATE_PYRO_TEST")
            stop_state = FSMState::STATE_PYRO_TEST;
        else if (stop_state_str == "STATE_IDLE")
            stop_state = FSMState::STATE_IDLE;
        else if (stop_state_str == "STATE_FIRST_BOOST")
            stop_state = FSMState::STATE_FIRST_BOOST;
        else if (stop_state_str == "STATE_BURNOUT")
            stop_state = FSMState::STATE_BURNOUT;
        else if (stop_state_str == "STATE_COAST")
            stop_state = FSMState::STATE_COAST;
        else if (stop_state_str == "STATE_APOGEE")
            stop_state = FSMState::STATE_APOGEE;
        else if (stop_state_str == "STATE_DROGUE_DEPLOY")
            stop_state = FSMState::STATE_DROGUE_DEPLOY;
        else if (stop_state_str == "STATE_DROGUE")
            stop_state = FSMState::STATE_DROGUE;
        else if (stop_state_str == "STATE_MAIN_DEPLOY")
            stop_state = FSMState::STATE_MAIN_DEPLOY;
        else if (stop_state_str == "STATE_MAIN")
            stop_state = FSMState::STATE_MAIN;
        else if (stop_state_str == "STATE_LANDED")
            stop_state = FSMState::STATE_LANDED;
        else if (stop_state_str == "STATE_SUSTAINER_IGNITION")
            stop_state = FSMState::STATE_SUSTAINER_IGNITION;
        else if (stop_state_str == "STATE_SECOND_BOOST")
            stop_state = FSMState::STATE_SECOND_BOOST;
        else if (stop_state_str == "STATE_FIRST_SEPARATION")
            stop_state = FSMState::STATE_FIRST_SEPARATION;
        else
        {
            std::cerr << "Unknown FSM state: " << stop_state_str << std::endl;
            std::cerr << "Valid states: STATE_SAFE, STATE_PYRO_TEST, STATE_IDLE, STATE_FIRST_BOOST, STATE_BURNOUT, STATE_COAST, STATE_APOGEE, STATE_DROGUE_DEPLOY, STATE_DROGUE, STATE_MAIN_DEPLOY, STATE_MAIN, STATE_LANDED, STATE_SUSTAINER_IGNITION, STATE_SECOND_BOOST, STATE_FIRST_SEPARATION" << std::endl;
            return 1;
        }
    }

    std::cout << "EKF Flight Data Simulator" << std::endl;
    std::cout << "Input file: " << csv_filename << std::endl;
    std::cout << "Output file: " << output_filename << std::endl;
    std::cout << "---" << std::endl;

    try
    {
        EKFSimulator simulator(csv_filename, stop_state);
        simulator.runSimulation();
        simulator.saveResults(output_filename);

        std::cout << "Simulation completed successfully!" << std::endl;
        std::cout << "Results saved to: " << output_filename << std::endl;
        std::cout << "You can now plot the results using the provided Python script." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
