#include "WT61IMU.h"
#include <math.h>

WT61IMU::WT61IMU(HardwareSerial &serialPort)
    : _serial(serialPort),
      _bufIdx(0),
      _roll(0), _pitch(0), _yaw(0),
      _qx(0), _qy(0), _qz(0), _qw(1),
      _rollOffset(0), _pitchOffset(0), _yawOffset(0),
      _calibrated(false),
      _packetCount(0),
      _errorCount(0)
{
}

void WT61IMU::begin() {
    // Step 1: Send the "switch to 115200 / 100Hz" command.
    // The sensor might currently be at 9600 OR 115200 — we don't know.
    // So we send the command at BOTH baud rates to be safe.
    // The sensor saves this setting permanently (survives power-off).
    // Command: 0xFF 0xAA 0x63 = "baud 115200, output rate 100Hz"
    uint8_t cmd[] = {0xFF, 0xAA, 0x63};

    // Try at 9600 first (in case sensor is still at factory-changed 9600)
    _serial.begin(9600, SERIAL_8N1, 16, 17);
    delay(100);
    _serial.write(cmd, 3);
    _serial.flush();
    delay(200);
    _serial.end();

    // Now open at 115200 (which the sensor should now be using)
    _serial.begin(115200, SERIAL_8N1, 16, 17);
    delay(500);

    // Also send the command at 115200 in case it was already there
    _serial.write(cmd, 3);
    _serial.flush();
    delay(200);

    // Flush any garbage from the baud switch
    while (_serial.available()) _serial.read();
    _bufIdx = 0;
    _packetCount = 0;
    _errorCount = 0;

    // Step 2: Calibrate
    calibrate();
}

void WT61IMU::update() {
    while (_serial.available()) {
        uint8_t b = _serial.read();
        if (_bufIdx == 0 && b != 0x55) continue;
        _buf[_bufIdx++] = b;
        if (_bufIdx >= 11) {
            _bufIdx = 0;
            uint8_t sum = 0;
            for (int i = 0; i < 10; i++) sum += _buf[i];
            if (sum != _buf[10]) { _errorCount++; continue; }
            if (_buf[1] == 0x53) {
                float r, p, y;
                _parseRawAngles(r, p, y);
                _roll  = r - _rollOffset;
                _pitch = p - _pitchOffset;
                _yaw   = y - _yawOffset;
                // Normalize to [-180, +180] so offsets don't cause -359° etc.
                if (_roll >  180.0f) _roll -= 360.0f;
                if (_roll < -180.0f) _roll += 360.0f;
                if (_pitch > 180.0f) _pitch -= 360.0f;
                if (_pitch < -180.0f) _pitch += 360.0f;
                if (_yaw >  180.0f) _yaw -= 360.0f;
                if (_yaw < -180.0f) _yaw += 360.0f;
                _computeQuaternion();
                _packetCount++;
            }
        }
    }
}

bool WT61IMU::_readOneAnglePacket() {
    unsigned long timeout = millis() + 500;
    while (millis() < timeout) {
        if (!_serial.available()) { delay(1); continue; }
        uint8_t b = _serial.read();
        if (_bufIdx == 0 && b != 0x55) continue;
        _buf[_bufIdx++] = b;
        if (_bufIdx >= 11) {
            _bufIdx = 0;
            uint8_t sum = 0;
            for (int i = 0; i < 10; i++) sum += _buf[i];
            if (sum != _buf[10]) { _errorCount++; continue; }
            if (_buf[1] == 0x53) {
                _packetCount++;
                return true;
            }
        }
    }
    return false;
}

void WT61IMU::_parseRawAngles(float &roll, float &pitch, float &yaw) {
    int16_t rawRoll  = (int16_t)(_buf[3] << 8 | _buf[2]);
    int16_t rawPitch = (int16_t)(_buf[5] << 8 | _buf[4]);
    int16_t rawYaw   = (int16_t)(_buf[7] << 8 | _buf[6]);
    roll  = rawRoll  / 32768.0f * 180.0f;
    pitch = rawPitch / 32768.0f * 180.0f;
    yaw   = rawYaw   / 32768.0f * 180.0f;
}

void WT61IMU::_computeQuaternion() {
    float hr = _roll  * 0.5f * DEG_TO_RAD;
    float hp = _pitch * 0.5f * DEG_TO_RAD;
    float hy = _yaw   * 0.5f * DEG_TO_RAD;
    float cr = cosf(hr), sr = sinf(hr);
    float cp = cosf(hp), sp = sinf(hp);
    float cy = cosf(hy), sy = sinf(hy);
    _qx = sr*cp*cy - cr*sp*sy;
    _qy = cr*sp*cy + sr*cp*sy;
    _qz = cr*cp*sy - sr*sp*cy;
    _qw = cr*cp*cy + sr*sp*sy;
}

void WT61IMU::calibrate(uint16_t durationMs) {
    _rollOffset = 0;
    _pitchOffset = 0;
    _yawOffset = 0;

    float sinRollSum = 0, cosRollSum = 0;
    float sinPitchSum = 0, cosPitchSum = 0;
    float sinYawSum = 0, cosYawSum = 0;
    int count = 0;
    unsigned long startMs = millis();

    while ((millis() - startMs) < durationMs) {
        if (_readOneAnglePacket()) {
            float r, p, y;
            _parseRawAngles(r, p, y);
            float rRad = r * DEG_TO_RAD;
            float pRad = p * DEG_TO_RAD;
            float yRad = y * DEG_TO_RAD;
            sinRollSum  += sinf(rRad);  cosRollSum  += cosf(rRad);
            sinPitchSum += sinf(pRad);  cosPitchSum += cosf(pRad);
            sinYawSum   += sinf(yRad);  cosYawSum   += cosf(yRad);
            count++;
        }
    }

    if (count > 0) {
        _rollOffset  = atan2f(sinRollSum,  cosRollSum)  * RAD_TO_DEG;
        _pitchOffset = atan2f(sinPitchSum, cosPitchSum) * RAD_TO_DEG;
        _yawOffset   = atan2f(sinYawSum,   cosYawSum)   * RAD_TO_DEG;
        _calibrated = true;
    } else {
        _calibrated = false;
    }
}

float WT61IMU::getRoll()  const { return _roll; }
float WT61IMU::getPitch() const { return _pitch; }
float WT61IMU::getYaw()   const { return _yaw; }
float WT61IMU::getQx() const { return _qx; }
float WT61IMU::getQy() const { return _qy; }
float WT61IMU::getQz() const { return _qz; }
float WT61IMU::getQw() const { return _qw; }
float WT61IMU::getRollOffset()  const { return _rollOffset; }
float WT61IMU::getPitchOffset() const { return _pitchOffset; }
float WT61IMU::getYawOffset()   const { return _yawOffset; }
uint32_t WT61IMU::getPacketCount() const { return _packetCount; }
uint32_t WT61IMU::getErrorCount()  const { return _errorCount; }
bool     WT61IMU::isCalibrated()   const { return _calibrated; }