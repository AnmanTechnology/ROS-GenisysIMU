#ifndef _GENISYS_IMU_H_
#define _GENISYS_IMU_H_
#include <iostream>
#include <sstream>
#include <math.h>
#include "serial/serial.h"

#define IMU_DATA_SIZE 52
const std::string col_red("\033[0;31m");
const std::string col_green("\033[0;32m");
const std::string col_yellow("\033[0;33m");
const std::string col_blue("\033[0;34m");
const std::string col_magenta("\033[0;35m");
const std::string col_cyan("\033[0;36m");
const std::string col_reset("\033[0m");

struct EulerAngles
{
    float roll;
    float pitch;
    float yaw;
};

struct Quaternion
{
    float x;
    float y;
    float z;
    float w;
};

struct Vector3
{
    float x;
    float y;
    float z;
};

struct IMU
{
    Quaternion orientation;
    Vector3 angular_velocity;
    Vector3 linear_acceleration;
    Vector3 magnetic_field;

    float orientation_covariance[9];
    float angular_velocity_covariance[9];
    float linear_acceleration_covariance[9];
    float magnetic_field_covariance[9];
};

class genisys_imu
{
private:
    serial::Serial *_comm;

public:
    Vector3 pose;
    Vector3 twist;
    IMU data;
    genisys_imu(/* args */);
    ~genisys_imu();
    uint8_t begin(std::string port, uint32_t baud);
    void update();
};

genisys_imu::genisys_imu()
{
    float covQuat[9] = {0.00000101,
                        0.00000053,
                        0.0000002,
                        0.00000053,
                        0.00002619,
                        -0.00002129,
                        0.0000002,
                        -0.00002129,
                        0.00001793};

    float covAccel[9] = {0.0044, 0.0000, 0.0000,
                         0.0000, 0.0038, 0.0000,
                         0.0000, 0.0000, 0.0040};

    float covGyro[9] = {0.000022462, 0.0000, 0.0000,
                        0.0000, 0.000023671, 0.0000,
                        0.0000, 0.0000, 0.000024383};

    float covMag[9] = {0.000000000000547, 0.0000000000000000, 0.0000000000000000,
                       0.000000000000000, 0.0000000000006777, 0.0000000000000000,
                       0.000000000000000, 0.0000000000000000, 0.0000000000006867};

    for (size_t i = 0; i < 9; i++)
    {
        data.orientation_covariance[i] = covQuat[i];
        data.angular_velocity_covariance[i] = covGyro[i];
        data.linear_acceleration_covariance[i] = covAccel[i];
        data.magnetic_field_covariance[i] = covMag[i];
    }

    pose.x = 0;
    pose.y = 0;
    pose.z = 0;
    twist.x = 0;
    twist.y = 0;
    twist.z = 0;
}

genisys_imu::~genisys_imu()
{
    if (_comm->isOpen())
    {
        _comm->close();
    }
}

uint8_t genisys_imu::begin(std::string port, uint32_t baud)
{
    serial::Timeout TimeOut = serial::Timeout(50, 50, 0, 50, 0);
    _comm = new serial::Serial(port, baud, TimeOut);
    if (_comm->isOpen())
    {
        // _comm->flushInput();
        // _comm->flushOutput();

        std::cout << "Successfully, connected to port." << std::endl;

        return 1;
    }
    else
    {
        // _comm->close();
        std::cerr << "Connection fail." << std::endl;
    }
    return 0;
}

void genisys_imu::update()
{
    uint8_t ret[150];

    // std::cerr << "Cosdf." << std::endl;
    if (_comm->waitReadable())
    {

        if (uint8_t(*_comm->read(1).c_str()) == 0xFF)
        {
            // std::cout << "FF_1, ";
            if (uint8_t(*_comm->read(1).c_str()) == 0xFF)
            {
                // std::cout << "FF_2, ";
                if (uint8_t(*_comm->read(1).c_str()) == 0xFF)
                {
                    // std::cout << "FF_3, ";
                    if (uint8_t(*_comm->read(1).c_str()) == 0xFD)
                    {
                        // std::cout << "FD_4, ";
                        uint8_t size = _comm->read(&ret[0], IMU_DATA_SIZE);
                        if (size == IMU_DATA_SIZE)
                        {
                            memcpy(&data, &ret[0], size);

                            // std::cout << std::dec << (int)size << " >>> ";
                            // for (int i = 0; i < size; i++)
                            //     std::cout
                            //         << std::hex << (int)ret[i] << " ";
                            // std::cout << "\n";

                            // std::cout << std::fixed << std::setw(10) << std::setprecision(5) << std::right;
                            // std::cout << col_cyan << data.orientation.x << " " << data.orientation.y << " " << data.orientation.z << " " << data.orientation.w << " ";
                            // std::cout << col_green << data.linear_acceleration.x << " " << data.linear_acceleration.y << " " << data.linear_acceleration.z << " ";
                            // std::cout << col_red << data.angular_velocity.x << " " << data.angular_velocity.y << " " << data.angular_velocity.z << " ";
                            // std::cout << col_yellow << data.magnetic_field.x << " " << data.magnetic_field.y << " " << data.magnetic_field.z << " ";
                            // std::cout << col_reset << std::endl;
                        }
                    }
                }
            }
        }
    }
    // else
    // {
    //     _comm->flushInput();
    // }

    // EulerAngles angles;
    // Quaternion q = data.orientation;

    // // roll (x-axis rotation)
    // float sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    // float cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    // angles.roll = atan2(sinr_cosp, cosr_cosp);

    // // pitch (y-axis rotation)
    // double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    // if (fabs(sinp) >= 1)
    //     angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    // else
    //     angles.pitch = asin(sinp);

    // // yaw (z-axis rotation)
    // float siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    // float cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    // angles.yaw = atan2(siny_cosp, cosy_cosp);

    // std::cout << std::fixed << std::setw(10) << std::setprecision(5) << std::right;
    // std::cout << col_cyan << angles.roll * 180.0 / M_PI << " " << angles.pitch * 180.0 / M_PI << " " << angles.yaw * 180.0 / M_PI << " ";
    // std::cout << col_reset << std::endl;
}
#endif