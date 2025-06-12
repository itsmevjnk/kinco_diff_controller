#pragma once

#include "kinco_diff_controller/motor_interface.hpp"

class Motor {
public:
    Motor(MotorInterface& interface, uint8_t id, bool profiled = false);
    ~Motor();

    bool SetVelocity(double rpm);
    double GetVelocity() const; // in rpm
    int32_t GetRawPosition() const; // in increments
    double GetPosition() const; // in revolutions

    uint32_t GetResolution() const; // in increments per revolution
private:
    MotorInterface& m_interface;
    const uint8_t kID;
    uint32_t m_resolution; // encoder resolution
    double m_speedFactor; // rpm to dec conversion factor
};