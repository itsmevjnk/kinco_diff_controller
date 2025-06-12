#include "kinco_diff_controller/motor_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

MotorInterface::MotorInterface(const char* port) {
    m_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (m_fd < 0) {
        std::cerr << "Cannot open port " << port << ": " << strerror(errno) << std::endl;
        throw std::runtime_error("Serial port opening failed");
    }

    struct termios tty;
    if (tcgetattr(m_fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        throw std::runtime_error("tcgetattr() failed");
    }

    cfsetospeed(&tty, B38400);
    cfsetispeed(&tty, B38400);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signalling chars, no echo, no canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN] = 0; // non-blocking read
    tty.c_cc[VTIME] = 5; // 500ms read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off XON/XOFF
    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // turn off parity
    tty.c_cflag &= ~(CSTOPB | CRTSCTS); // 1 stop bit, disable RTS/CTS

    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        throw std::runtime_error("tcsetattr() failed");
    }

    usleep(1000);
    ioctl(m_fd, TCFLSH, 0); // flush receive
}

MotorInterface::~MotorInterface() {
    close(m_fd);
}

uint8_t MotorInterface::CalculateChecksum(const uint8_t* data) {
    uint16_t sum = 0;
    for (size_t i = 0; i < 9; i++) sum += data[i];
    return (uint8_t)-sum;
}

bool MotorInterface::Write(uint8_t id, uint16_t cmd, uint16_t index, uint8_t subIndex, const uint8_t* data, size_t len) {
    uint8_t buf[10];
    buf[0] = id;
    buf[1] = cmd;
    buf[2] = index & 0xFF;
    buf[3] = (index >> 8) & 0xFF;
    buf[4] = subIndex;
    memset(&buf[5], 0, 4);
    memcpy(&buf[5], data, len);
    buf[9] = CalculateChecksum(buf);
    if (write(m_fd, buf, 10) != 10) {
        std::cerr << "Cannot write payload to serial port" << std::endl;
        throw std::runtime_error("write() failed");
    }

    if (read(m_fd, buf, 10) != 10) {
        std::cerr << "Cannot read payload from serial port" << std::endl;
        throw std::runtime_error("read() failed");
    }

    if (buf[1] != 0x60) {
        std::cerr << "Object write failed" << std::endl;
        return false;
    }

    return true;
}

bool MotorInterface::Write32(uint8_t id, uint16_t index, uint8_t subIndex, uint32_t data) {
    uint8_t dataBuf[4] = {
        (uint8_t)(data & 0xFF), (uint8_t)((data >> 8) & 0xFF), (uint8_t)((data >> 16) & 0xFF), (uint8_t)((data >> 24) & 0xFF)
    };
    return Write(id, 0x23, index, subIndex, dataBuf, 4);
}

bool MotorInterface::Write16(uint8_t id, uint16_t index, uint8_t subIndex, uint16_t data) {
    uint8_t dataBuf[2] = {
        (uint8_t)(data & 0xFF), (uint8_t)((data >> 8) & 0xFF)
    };
    return Write(id, 0x2B, index, subIndex, dataBuf, 2);
}

bool MotorInterface::Write8(uint8_t id, uint16_t index, uint8_t subIndex, uint8_t data) {
    return Write(id, 0x2F, index, subIndex, &data, 1);
}

uint32_t MotorInterface::Read(uint8_t id, uint16_t index, uint8_t subIndex) {
    uint8_t buf[10];
    buf[0] = id;
    buf[1] = 0x40;
    buf[2] = index & 0xFF;
    buf[3] = (index >> 8) & 0xFF;
    buf[4] = subIndex;
    memset(&buf[5], 0, 4);
    buf[9] = CalculateChecksum(buf);
    if (write(m_fd, buf, 10) != 10) {
        std::cerr << "Cannot write payload to serial port" << std::endl;
        throw std::runtime_error("write() failed");
    }

    if (read(m_fd, buf, 10) != 10) {
        std::cerr << "Cannot read payload from serial port" << std::endl;
        throw std::runtime_error("read() failed");
    }
    
    switch (buf[1]) {
        case 0x43:
            return buf[5] | ((uint32_t)buf[6] << 8) | ((uint32_t)buf[7] << 16) | ((uint32_t)buf[8] << 24);

        case 0x4B:
            return buf[5] | ((uint32_t)buf[6] << 8);

        case 0x4F:
            return buf[5];
            
        default:
            std::cerr << "Object read failed" << std::endl;
            return 0;
    }
}

int32_t MotorInterface::ReadInt32(uint8_t id, uint16_t index, uint8_t subIndex) {
    return (int32_t)Read(id, index, subIndex);
}

int16_t MotorInterface::ReadInt16(uint8_t id, uint16_t index, uint8_t subIndex) {
    return (int16_t)Read(id, index, subIndex);
}

int8_t MotorInterface::ReadInt8(uint8_t id, uint16_t index, uint8_t subIndex) {
    return (int8_t)Read(id, index, subIndex);
}  
