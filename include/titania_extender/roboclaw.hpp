// RoboClaw packet-serial protocol driver for the titania_extender package.
//
// Implements the subset of the Basic Micro RoboClaw command set needed to
// drive two motors in position mode over USB (or UART):
//   - velocity PID setup (commands 28/29)
//   - position PID setup (commands 61/62)
//   - buffered speed/accel/deccel/position moves (commands 65/66)
//   - encoder, speed, current, temperature, voltage and error reads
//
// Packet format: [address, command, data..., crc16]. The CRC is CCITT
// (XMODEM, poly 0x1021, init 0) over address+command+data. Write commands
// are acknowledged with a single 0xFF byte. Read commands return data
// followed by a CRC computed over address+command+data.
//
// All public methods are thread-safe (serialized by an internal mutex).

#pragma once

#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace titania_extender {

class RoboClawError : public std::runtime_error {
 public:
  explicit RoboClawError(const std::string &what) : std::runtime_error(what) {}
};

class RoboClaw {
 public:
  enum Motor { kM1 = 0, kM2 = 1 };

  struct EncoderResult {
    int32_t value;
    uint8_t status;
  };

  // Opens the serial device. Throws RoboClawError on failure.
  RoboClaw(const std::string &device_name, uint32_t baud_rate,
           uint8_t address);
  ~RoboClaw();

  RoboClaw(const RoboClaw &) = delete;
  RoboClaw &operator=(const RoboClaw &) = delete;

  std::string getVersion();

  // Velocity PID (commands 28/29). Values are scaled by 65536 on the wire.
  void setVelocityPid(Motor motor, float p, float i, float d, uint32_t qpps);

  // Position PID (commands 61/62). P/I/D are scaled by 1024 on the wire.
  void setPositionPid(Motor motor, float p, float i, float d, uint32_t max_i,
                      uint32_t deadzone, int32_t min_position,
                      int32_t max_position);

  // Buffered move with signed speed, accel, deccel and position
  // (commands 65/66). When immediate is true the move replaces anything in
  // the RoboClaw's command buffer and starts at once.
  void moveToPosition(Motor motor, uint32_t accel, uint32_t speed,
                      uint32_t decel, int32_t position, bool immediate = true);

  EncoderResult readEncoder(Motor motor);        // commands 16/17
  void setEncoder(Motor motor, int32_t value);   // commands 22/23
  void resetEncoders();                          // command 20
  int32_t readSpeed(Motor motor);                // commands 18/19, QPPS

  std::pair<float, float> readCurrents();        // command 49, amps (m1, m2)
  float readTemperature();                       // command 82, degrees C
  float readMainBatteryVoltage();                // command 24, volts
  float readLogicBatteryVoltage();               // command 25, volts
  uint32_t readErrorStatus();                    // command 90
  static std::string errorString(uint32_t status);

  // Command buffer depth for (m1, m2). 0x80 => buffer empty (move done),
  // 0 => last command is executing. (Command 47.)
  std::pair<uint8_t, uint8_t> readBufferLengths();

  // Stop a motor by commanding zero duty cycle (commands 32/33).
  void stop(Motor motor);
  void stopAll();

 private:
  void openPort();
  void closePort();
  void restartPort();

  static void updateCrc(uint16_t &crc, uint8_t data);

  // Writes [address, command, data..., crc] and waits for the 0xFF ack.
  // Retries internally; throws RoboClawError on persistent failure.
  void writeCommand(uint8_t command, const uint8_t *data, size_t length);

  // Writes [address, command], reads expected_length bytes + 2 CRC bytes,
  // validates the CRC. Retries internally; throws on persistent failure.
  void readCommand(uint8_t command, uint8_t *buffer, size_t expected_length);

  void writeBytes(const uint8_t *data, size_t length);
  uint8_t readByteWithTimeout();
  void flushInput();

  std::string device_name_;
  uint32_t baud_rate_;
  uint8_t address_;
  int fd_ = -1;
  std::mutex mutex_;
};

}  // namespace titania_extender
