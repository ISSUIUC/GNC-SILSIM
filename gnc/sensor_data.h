#pragma once

#include <string>
#include <cmath>

struct Barometer
{
    float temperature;
    float pressure;
    float altitude;
};

struct Acceleration
{
    float ax;
    float ay;
    float az;
};

struct Position
{
    float px;
    float py;
    float pz;
};

struct Velocity
{
    float vx;
    float vy;
    float vz;
};

struct euler_t
{
    float roll;
    float pitch;
    float yaw;
};

struct Quaternion
{
    float w, x, y, z;

    static float dot(const Quaternion &q1, const Quaternion &q2)
    {
        return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
    }
    float norm() const
    {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    Quaternion normalized() const
    {
        float n = norm();
        if (!(n > 0.0f) || !std::isfinite(n))
        {
            // Return identity quaternion if input is zero/invalid
            return Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
        }
        return Quaternion{w / n, x / n, y / n, z / n};
    }
};

struct Orientation
{
    bool has_data;
    std::string reading_type;
    float yaw;
    float pitch;
    float roll;
    Velocity orientation_velocity;
    Velocity angular_velocity;
    Acceleration orientation_acceleration;
    Acceleration linear_acceleration;
    Quaternion quaternion;

    float gx, gy, gz;
    float mx, my, mz;
    float temperature;
    float pressure;
    float tilt;

    euler_t getEuler() const
    {
        return {roll, pitch, yaw};
    }

    Velocity getVelocity() const
    {
        return orientation_velocity;
    }

    Velocity getAngularVelocity() const
    {
        return angular_velocity;
    }
};

struct KalmanData
{
    Position position;
    Velocity velocity;
    Acceleration acceleration;
    float altitude;
};

enum class FSMState
{
    STATE_SAFE = 0,
    STATE_PYRO_TEST = 1,
    STATE_IDLE = 2,
    STATE_FIRST_BOOST = 3,
    STATE_BURNOUT = 4,
    STATE_COAST = 5,
    STATE_APOGEE = 6,
    STATE_DROGUE_DEPLOY = 7,
    STATE_DROGUE = 8,
    STATE_MAIN_DEPLOY = 9,
    STATE_MAIN = 10,
    STATE_LANDED = 11,
    STATE_SUSTAINER_IGNITION = 12,
    STATE_SECOND_BOOST = 13,
    STATE_FIRST_SEPARATION = 14,
    FSM_STATE_COUNT = 15
};

const FSMState STATE_SAFE = FSMState::STATE_SAFE;
const FSMState STATE_PYRO_TEST = FSMState::STATE_PYRO_TEST;
const FSMState STATE_IDLE = FSMState::STATE_IDLE;
const FSMState STATE_FIRST_BOOST = FSMState::STATE_FIRST_BOOST;
const FSMState STATE_BURNOUT = FSMState::STATE_BURNOUT;
const FSMState STATE_COAST = FSMState::STATE_COAST;
const FSMState STATE_APOGEE = FSMState::STATE_APOGEE;
const FSMState STATE_DROGUE_DEPLOY = FSMState::STATE_DROGUE_DEPLOY;
const FSMState STATE_DROGUE = FSMState::STATE_DROGUE;
const FSMState STATE_MAIN_DEPLOY = FSMState::STATE_MAIN_DEPLOY;
const FSMState STATE_MAIN = FSMState::STATE_MAIN;
const FSMState STATE_LANDED = FSMState::STATE_LANDED;
const FSMState STATE_SUSTAINER_IGNITION = FSMState::STATE_SUSTAINER_IGNITION;
const FSMState STATE_SECOND_BOOST = FSMState::STATE_SECOND_BOOST;
const FSMState STATE_FIRST_SEPARATION = FSMState::STATE_FIRST_SEPARATION;

struct LowGData
{
    float ax, ay, az;
};

struct HighGData
{
    float ax, ay, az;
};

struct LowGLSMData
{
    float gx, gy, gz;
    float ax, ay, az;
};

struct Magnetometer
{
    float mx, my, mz;
};

struct GPS
{
    float latitude;
    float longitude;
    float altitude;
    float speed;
    int fix_type;
    float time;
};

struct Voltage
{
    float voltage;
    float current;
};

struct Continuity
{
    bool pins[4];
};

struct Pyro
{
    bool is_global_armed;
    bool channel_firing[4];
};
