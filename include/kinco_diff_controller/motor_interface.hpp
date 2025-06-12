#pragma once

#include <bits/stdc++.h>

class MotorInterface {
public:
    MotorInterface(const char* port);
    ~MotorInterface();

    bool Write32(uint8_t id, uint16_t index, uint8_t subIndex, uint32_t data);
    bool Write16(uint8_t id, uint16_t index, uint8_t subIndex, uint16_t data);
    bool Write8(uint8_t id, uint16_t index, uint8_t subIndex, uint8_t data);
    uint32_t Read(uint8_t id, uint16_t index, uint8_t subIndex);
    int32_t ReadInt32(uint8_t id, uint16_t index, uint8_t subIndex);
    int16_t ReadInt16(uint8_t id, uint16_t index, uint8_t subIndex);
    int8_t ReadInt8(uint8_t id, uint16_t index, uint8_t subIndex);    

private:
    bool Write(uint8_t id, uint16_t cmd, uint16_t index, uint8_t subIndex, const uint8_t* data, size_t len);
    uint8_t CalculateChecksum(const uint8_t* data);

    int m_fd;
};