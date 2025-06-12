#include "kinco_diff_controller/motor.hpp"

Motor::Motor(MotorInterface& interface, uint8_t id, bool profiled) : m_interface(interface), kID(id) {
    m_resolution = m_interface.Read(kID, 0x6410, 0x03);
    if (!m_resolution) {
        std::cerr << "Cannot read encoder resolution from ID " << kID << std::endl;
        throw std::runtime_error("Encoder resolution reading failed");
    }
    m_speedFactor = 512 * m_resolution / 1875.0;

    if (!m_interface.Write8(kID, 0x6060, 0x00, (int8_t)((profiled) ? 3 : -3))) {
        std::cerr << "Cannot set motor control mode on ID " << kID << std::endl;
        throw std::runtime_error("Motor control mode setting failed");
    }
    if (!m_interface.Write16(kID, 0x6040, 0x00, 0x002F)) { // put motor into ready
        std::cerr << "Cannot set motor control word on ID " << kID << std::endl;
        throw std::runtime_error("Motor control word setting failed");
    }
    if (!SetVelocity(0)) {
        std::cerr << "Cannot stop motor on ID " << kID << std::endl;
        throw std::runtime_error("Motor stopping in initialisation failed");
    }
}

Motor::~Motor() {
    if (!SetVelocity(0)) {
        std::cerr << "Cannot stop motor on ID " << kID << std::endl;
    }
}

bool Motor::SetVelocity(double rpm) {
    int32_t speed = rpm * m_speedFactor;
    return m_interface.Write32(kID, 0x60FF, 0x00, (uint32_t)speed);
}

double Motor::GetVelocity() const {
    int32_t speed = m_interface.ReadInt32(kID, 0x606C, 0x00);
    return speed / m_speedFactor;
}

int32_t Motor::GetRawPosition() const {
    return m_interface.ReadInt32(kID, 0x6063, 0x00);
}

double Motor::GetPosition() const {
    return GetRawPosition() / (double)m_resolution;
}

uint32_t Motor::GetResolution() const {
    return m_resolution;
}