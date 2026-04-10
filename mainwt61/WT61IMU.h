#ifndef WT61IMU_H
#define WT61IMU_H

#include <Arduino.h>

class WT61IMU {
public:
    WT61IMU(HardwareSerial &serialPort);

    // Switches sensor to 100Hz/115200, calibrates all axes.
    // Call AFTER Serial2.begin().
    void begin();

    // Call every loop(). Drains serial buffer, parses angle packets.
    void update();

    // Re-calibrate (sensor must be still).
    void calibrate(uint16_t durationMs = 3000);

    // Euler angles in degrees (calibrated)
    float getRoll()  const;
    float getPitch() const;
    float getYaw()   const;

    // JPL quaternion (scalar last)
    float getQx() const;
    float getQy() const;
    float getQz() const;
    float getQw() const;

    // Calibration offsets
    float getRollOffset()  const;
    float getPitchOffset() const;
    float getYawOffset()   const;

    uint32_t getPacketCount() const;
    uint32_t getErrorCount()  const;
    bool     isCalibrated()   const;

private:
    HardwareSerial &_serial;
    uint8_t _buf[11];
    uint8_t _bufIdx;

    float _roll, _pitch, _yaw;
    float _qx, _qy, _qz, _qw;

    float _rollOffset, _pitchOffset, _yawOffset;
    bool  _calibrated;

    uint32_t _packetCount;
    uint32_t _errorCount;

    bool _readOneAnglePacket();
    void _parseRawAngles(float &roll, float &pitch, float &yaw);
    void _computeQuaternion();
};

#endif