#pragma once
#include <Arduino.h>
#include "CANfetti.hpp"

class ServoManager {
public:
  struct CMD {
    struct HOME {
      static const uint8_t SET = 0x90;
      static const uint8_t GO = 0x91;
      static const uint8_t ZERO = 0x92;
    };
    struct READ {
      static const uint8_t ENCODER_CARRY = 0x30;
      static const uint8_t ENCODER_ABSOLUTE = 0x31;
      static const uint8_t SPEED = 0x32;
      static const uint8_t PULSES = 0x33;
      static const uint8_t IO = 0x34;
      static const uint8_t ANGLE = 0x39;
      static const uint8_t PINS = 0x3A;
      static const uint8_t POWER_ZERO = 0x3B;
      static const uint8_t PROTECTION = 0x3E;
      static const uint8_t VERSION = 0x40;
    };
    struct SET {
      static const uint8_t CALIBRATION = 0x80;
      static const uint8_t MODE = 0x82;
      static const uint8_t WORKING_CURRENT = 0x83;
      static const uint8_t HOLDING_CURRENT = 0x9B;
      static const uint8_t SUBDIVISION = 0x84;
      static const uint8_t PIN = 0x85;
      static const uint8_t DIRECTION = 0x86;
      static const uint8_t SCREEN = 0x87;
      static const uint8_t PROTECTION = 0x88;
      static const uint8_t INTERPOLATION = 0x89;
      static const uint8_t CAN_RATE = 0x8A;
      static const uint8_t CAN_ID = 0x8B;
      static const uint8_t LOCK = 0x8F;
      static const uint8_t GROUP = 0x8D;
      static const uint8_t REMAP = 0x9E;
      static const uint8_t MODE0 = 0x9A;
    };
    struct MOTOR {
      static const uint8_t ABSOLUTE = 0xFE;
      static const uint8_t RELATIVE_PULSE = 0xFD;
      static const uint8_t RELATIVE_AXIS = 0xF4;
      static const uint8_t ABSOLUTE_AXIS = 0xF5;
      static const uint8_t SPEED = 0xF6;
      static const uint8_t STOP = 0xF7;
    };
    struct CONTROL {
      static const uint8_t QUERY = 0xF1;
      static const uint8_t ENABLE = 0xF3;
      static const uint8_t SAVE = 0xFF;
      static const uint8_t RESET = 0x3F;
      static const uint8_t RELEASE = 0x3D;
    };
  };

  struct SERVO_STATE {
    static const uint8_t COMPLETE = 0x00;
    static const uint8_t IN_PROGRESS = 0x01;
    static const uint8_t FAILED = 0x02;
    static const uint8_t SERVO_ENABLED = 0x01;
    static const uint8_t SERVO_DISABLED = 0x00;
  };

  enum ResponseType {
    TYPE_NONE,
    TYPE_ENCODER,
    TYPE_SPEED,
    TYPE_MOTION,
    TYPE_ZERO,
    TYPE_IO
  };

  struct ResponseData {
    ResponseType type = TYPE_NONE;
    union {
      uint32_t value;
      struct {
        uint8_t status;
        uint8_t command;
        struct {
          uint8_t direction;
          uint16_t speed;
          uint8_t acceleration;
        } params;
      } motion;
      struct {
        uint8_t inputs;
        uint8_t outputs;
      } io;
      struct {
        uint8_t state;
        const char* stateStr;
      } zero;
    };
  };

  ServoManager(uint8_t id = 0x01) : canId(id), lastResponse{} {}

  CANfettiFrame setHomeParams(uint8_t trigger, uint8_t direction, uint8_t speed, uint8_t end) {
    return pack({ CMD::HOME::SET, 
                  servoBit(trigger), 
                  servoBit(direction), 
                  servoByte(speed), 
                  servoBit(end) });
  }

  CANfettiFrame goHome() { return pack({ CMD::HOME::GO }); }
  CANfettiFrame setCurrentPositionZero() { return pack({ CMD::HOME::ZERO }); }
  CANfettiFrame readEncoderCarry() { return pack({ CMD::READ::ENCODER_CARRY }); }
  CANfettiFrame readEncoderAbsolute() { return pack({ CMD::READ::ENCODER_ABSOLUTE }); }
  CANfettiFrame readSpeed() { return pack({ CMD::READ::SPEED }); }
  CANfettiFrame readNumPulsesReceived() { return pack({ CMD::READ::PULSES }); }
  CANfettiFrame readIoPortStatus() { return pack({ CMD::READ::IO }); }
  CANfettiFrame readMotorShaftAngleError() { return pack({ CMD::READ::ANGLE }); }
  CANfettiFrame readEnPinsStatus() { return pack({ CMD::READ::PINS }); }
  CANfettiFrame readGoBackToZeroStatusWhenPowerOn() { return pack({ CMD::READ::POWER_ZERO }); }
  CANfettiFrame readMotorShaftProtectionState() { return pack({ CMD::READ::PROTECTION }); }

  CANfettiFrame setWorkingCurrent(uint16_t current) {
    uint8_t buf[3] = { CMD::SET::WORKING_CURRENT };
    putWord(current, &buf[1]);
    return pack(buf, 3);
  }

  CANfettiFrame setHoldingCurrent(uint8_t percentage) {
    return pack({ CMD::SET::HOLDING_CURRENT, servoByte(percentage) });
  }

  CANfettiFrame setGroupId(uint16_t id) {
    uint8_t buf[3] = { CMD::SET::GROUP };
    putWord(id, &buf[1]);
    return pack(buf, 3);
  }

  CANfettiFrame setCanId(uint16_t id) {
    uint8_t buf[3] = { CMD::SET::CAN_ID };
    putWord(id, &buf[1]);
    return pack(buf, 3);
  }

  CANfettiFrame setSubdivisions(uint16_t subdivisions) {  return pack({ CMD::SET::SUBDIVISION, servoByte(subdivisions) });  }
  CANfettiFrame setEnablePinConfig(uint8_t config) {  return pack({ CMD::SET::PIN, servoByte(config) });  }
  CANfettiFrame setMotorDirection(uint8_t direction) {  return pack({ CMD::SET::DIRECTION, servoBit(direction) });  }
  CANfettiFrame setAutoScreenOff(bool enabled) {  return pack({ CMD::SET::SCREEN, servoBit(enabled) });  }
  CANfettiFrame setShaftProtection(bool enabled) {  return pack({ CMD::SET::PROTECTION, servoBit(enabled) });  }
  CANfettiFrame setSubdivisionInterpolation(bool enabled) {  return pack({ CMD::SET::INTERPOLATION, servoBit(enabled) });  }
  CANfettiFrame setCanBitrate(uint8_t rate) {  return pack({ CMD::SET::CAN_RATE, servoByte(rate) });  }
  CANfettiFrame setKeyLock(bool enabled) {  return pack({ CMD::SET::LOCK, servoBit(enabled) });  }
  CANfettiFrame setLimitPortRemap(bool enabled) {  return pack({ CMD::SET::REMAP, servoBit(enabled) });  }
  CANfettiFrame setMode0(uint8_t mode, bool enabled, uint8_t speed, uint8_t direction) {
    return pack({ CMD::SET::MODE0, servoByte(mode), servoBit(enabled), servoByte(speed), servoBit(direction) });
  }

  CANfettiFrame absoluteMotion(uint16_t speed, uint8_t acceleration, uint32_t position) {
    uint8_t buf[7] = { CMD::MOTOR::ABSOLUTE };
    putWord(speed, &buf[1]);
    buf[3] = servoByte(acceleration);
    putPosition(position, &buf[4]);
    return pack(buf, 7);
  }

  CANfettiFrame runMotorRelativeMotionByPulses(uint8_t dir, uint16_t speed, uint8_t acceleration, uint32_t pulses) {
    uint8_t buf[7] = { CMD::MOTOR::RELATIVE_PULSE };
    buf[1] = servoDirection(dir, speed);
    buf[2] = servoByte(speed);
    buf[3] = servoByte(acceleration);
    putPosition(pulses, &buf[4]);
    return pack(buf, 7);
  }

  CANfettiFrame runMotorRelativeMotionByAxis(uint8_t dir, uint16_t speed, uint8_t acceleration, uint32_t axis) {
    uint8_t buf[7] = { CMD::MOTOR::RELATIVE_AXIS };
    buf[1] = servoDirection(dir, speed);
    buf[2] = servoByte(speed);
    buf[3] = servoByte(acceleration);
    putPosition(axis, &buf[4]);
    return pack(buf, 7);
  }

  CANfettiFrame runMotorAbsoluteMotionByAxis(uint16_t speed, uint8_t acceleration, uint32_t axis) {
    uint8_t buf[7] = { CMD::MOTOR::ABSOLUTE_AXIS };
    putWord(speed, &buf[1]);
    buf[3] = servoByte(acceleration);
    putPosition(axis, &buf[4]);
    return pack(buf, 7);
  }

  CANfettiFrame runMotorSpeedMode(uint8_t dir, uint16_t speed, uint8_t acceleration) {
    return pack({ CMD::MOTOR::SPEED,
                  servoDirection(dir, speed),
                  servoByte(speed),
                  servoByte(acceleration) });
  }

  CANfettiFrame queryStatus() { return pack({ CMD::CONTROL::QUERY }); }
  CANfettiFrame enable(bool state) { return pack({ CMD::CONTROL::ENABLE, servoBit(state) }); }
  CANfettiFrame motorCalibration() { return pack({ CMD::SET::CALIBRATION, 0x00 }); }
  CANfettiFrame restoreDefaultParameters() { return pack({ CMD::CONTROL::RESET }); }
  CANfettiFrame releaseMotorShaftLockedProtectionState() { return pack({ CMD::CONTROL::RELEASE }); }
  CANfettiFrame emergencyStop() { return pack({ CMD::MOTOR::STOP }); }
  CANfettiFrame saveParameters() { return pack({ CMD::CONTROL::SAVE, 0x00 }); }
  CANfettiFrame savePositionOnSpeedMode() { return pack({ CMD::CONTROL::SAVE, 0x01 }); }
  CANfettiFrame saveAndResetOnSpeedMode() { return pack({ CMD::CONTROL::SAVE, 0x02 }); }
  CANfettiFrame saveClearOnSpeedMode() { return pack({ CMD::CONTROL::SAVE, 0x03 }); }

  ResponseData* parseResponse(const CANfettiFrame& frame) {
    if (frame.len < 3) return nullptr;

    static ResponseData response;
    response = ResponseData{};
    const uint8_t cmd = frame.buf[1];

    switch (cmd) {
      case CMD::READ::ENCODER_CARRY:
      case CMD::READ::ENCODER_ABSOLUTE:
        response.type = TYPE_ENCODER;
        response.value = getValue(&frame.buf[2], 3);
        break;

      case CMD::READ::SPEED:
        response.type = TYPE_SPEED;
        response.value = getValue(&frame.buf[2], 2);
        break;

      case CMD::MOTOR::SPEED:
      case CMD::MOTOR::RELATIVE_PULSE:
      case CMD::MOTOR::RELATIVE_AXIS:
      case CMD::MOTOR::ABSOLUTE_AXIS:
        if (!parseMotionResponse(frame, response)) return nullptr;
        break;

      case CMD::READ::POWER_ZERO:
        response.type = TYPE_ZERO;
        response.zero.state = frame.buf[2];
        response.zero.stateStr = getZeroStateString(frame.buf[2]);
        break;

      case CMD::READ::IO:
        response.type = TYPE_IO;
        response.io.inputs = frame.buf[2];
        response.io.outputs = frame.buf[3];
        break;

      default: return nullptr;
    }

    return &response;
  }

  ResponseData* getLastResponse() { return &lastResponse; }
  void resetResponse() { lastResponse = ResponseData{}; }

private:
  uint8_t canId;
  ResponseData lastResponse;

  uint8_t servoBit(uint8_t v) { return v & 0x01; }
  uint8_t servoByte(uint16_t v) { return v & 0xFF; }
  uint8_t servoDirection(uint8_t d, uint16_t s) { return (d << 7) | ((s >> 8) & 0x0F); }
  
  void putWord(uint16_t v, uint8_t* result) {
    result[0] = (v >> 8) & 0xFF;
    result[1] = v & 0xFF;
  }
  
  void putPosition(uint32_t v, uint8_t* result) {
    result[0] = (v >> 16) & 0xFF;
    result[1] = (v >> 8) & 0xFF;
    result[2] = v & 0xFF;
  }
  
  uint8_t checksum(const uint8_t* data, uint8_t len) {
    uint16_t sum = canId;
    for (uint8_t i = 0; i < len; i++) { sum += data[i]; }
    return sum & 0xFF;
  }

  uint32_t getValue(const uint8_t* data, uint8_t length) {
    uint32_t value = 0;
    while (length--) value = (value << 8) | *data++;
    return value;
  }

  const char* getZeroStateString(uint8_t state) {
    switch (state) {
      case SERVO_STATE::IN_PROGRESS: return "zeroing";
      case SERVO_STATE::COMPLETE: return "complete";
      default: return "failed";
    }
  }

  bool parseMotionResponse(const CANfettiFrame& frame, ResponseData& response) {
    const uint8_t status = frame.buf[2];

    if (lastResponse.type == TYPE_MOTION && lastResponse.motion.status == status) {
      return false;
    }

    response.type = TYPE_MOTION;
    response.motion.status = status;
    response.motion.command = frame.buf[1];

    if (frame.buf[1] == CMD::MOTOR::RELATIVE_PULSE && status == SERVO_STATE::IN_PROGRESS) {
      response.motion.params.direction = (frame.buf[3] & 0x80) >> 7;
      response.motion.params.speed = ((frame.buf[3] & 0x7F) << 8) | frame.buf[4];
      response.motion.params.acceleration = frame.buf[5];
    }

    lastResponse = response;
    return true;
  }

  template<size_t N>
  CANfettiFrame pack(const uint8_t (&data)[N]) { return pack(data, N); }
  CANfettiFrame pack(const uint8_t* data, size_t len) {
    CANfetti gen;
    uint8_t fullData[9];
    memcpy(fullData, data, len);
    fullData[len] = checksum(data, len);

    return gen.setId(canId)
      .setDataLength(len + 1)
      .setData(fullData, len + 1)
      .build();
  }
};