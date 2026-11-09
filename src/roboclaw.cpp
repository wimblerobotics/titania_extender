// SPDX-License-Identifier: Apache-2.0
#include "titania_extender/roboclaw.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <sstream>
#include <vector>

namespace titania_extender {

namespace {

// RoboClaw packet-serial command numbers (Basic Micro user manual).
enum Command : uint8_t {
  kM1Duty = 32,
  kM2Duty = 33,
  kGetM1Enc = 16,
  kGetM2Enc = 17,
  kGetM1Speed = 18,
  kGetM2Speed = 19,
  kResetEnc = 20,
  kGetVersion = 21,
  kSetM1EncCount = 22,
  kSetM2EncCount = 23,
  kGetMBatt = 24,
  kGetLBatt = 25,
  kSetM1VelocityPid = 28,
  kSetM2VelocityPid = 29,
  kGetBuffers = 47,
  kGetCurrents = 49,
  kSetM1PositionPid = 61,
  kSetM2PositionPid = 62,
  kM1SpeedAccelDeccelPos = 65,
  kM2SpeedAccelDeccelPos = 66,
  kGetTemp = 82,
  kGetError = 90,
};

constexpr int kReadTimeoutMs = 100;
constexpr int kRetries = 3;
constexpr uint8_t kAck = 0xFF;

void packU32(std::vector<uint8_t>& buffer, uint32_t value) {
  buffer.push_back(static_cast<uint8_t>(value >> 24));
  buffer.push_back(static_cast<uint8_t>(value >> 16));
  buffer.push_back(static_cast<uint8_t>(value >> 8));
  buffer.push_back(static_cast<uint8_t>(value));
}

uint32_t unpackU32(const uint8_t* data) {
  return (static_cast<uint32_t>(data[0]) << 24) |
         (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
}

uint16_t unpackU16(const uint8_t* data) {
  return static_cast<uint16_t>((data[0] << 8) | data[1]);
}

}  // namespace

RoboClaw::RoboClaw(const std::string& device_name, uint32_t baud_rate,
                   uint8_t address)
    : device_name_(device_name), baud_rate_(baud_rate), address_(address) {
  openPort();
}

RoboClaw::~RoboClaw() {
  try {
    stopAll();
  } catch (...) {
    // Best effort on shutdown.
  }
  closePort();
}

void RoboClaw::openPort() {
  fd_ = ::open(device_name_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    throw RoboClawError("Unable to open device '" + device_name_ +
                        "': " + std::strerror(errno));
  }

  struct termios options;
  if (tcgetattr(fd_, &options) < 0) {
    closePort();
    throw RoboClawError(std::string("tcgetattr failed: ") +
                        std::strerror(errno));
  }

  speed_t baud;
  switch (baud_rate_) {
    case 9600:
      baud = B9600;
      break;
    case 19200:
      baud = B19200;
      break;
    case 38400:
      baud = B38400;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    case 230400:
      baud = B230400;
      break;
    default:
      closePort();
      throw RoboClawError("Unsupported baud rate: " +
                          std::to_string(baud_rate_));
  }
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  // 8N1, no flow control, raw mode.
  options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  options.c_cflag |= CS8 | CREAD | CLOCAL;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK |
                       ISTRIP | INLCR | IGNCR | ICRNL);
  options.c_oflag &= ~(OPOST | ONLCR);
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &options) < 0) {
    closePort();
    throw RoboClawError(std::string("tcsetattr failed: ") +
                        std::strerror(errno));
  }
  tcflush(fd_, TCIOFLUSH);
}

void RoboClaw::closePort() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void RoboClaw::restartPort() {
  closePort();
  openPort();
}

void RoboClaw::updateCrc(uint16_t& crc, uint8_t data) {
  crc = crc ^ (static_cast<uint16_t>(data) << 8);
  for (int i = 0; i < 8; i++) {
    if (crc & 0x8000) {
      crc = (crc << 1) ^ 0x1021;
    } else {
      crc <<= 1;
    }
  }
}

void RoboClaw::writeBytes(const uint8_t* data, size_t length) {
  size_t written = 0;
  while (written < length) {
    ssize_t result = ::write(fd_, data + written, length - written);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw RoboClawError(std::string("write failed: ") + std::strerror(errno));
    }
    written += static_cast<size_t>(result);
  }
}

uint8_t RoboClaw::readByteWithTimeout() {
  struct pollfd ufd;
  ufd.fd = fd_;
  ufd.events = POLLIN;

  int result = poll(&ufd, 1, kReadTimeoutMs);
  if (result < 0) {
    throw RoboClawError(std::string("poll failed: ") + std::strerror(errno));
  }
  if (result == 0) {
    throw RoboClawError("read timeout");
  }
  if (ufd.revents & (POLLERR | POLLHUP)) {
    restartPort();
    throw RoboClawError("device error on poll");
  }
  uint8_t byte = 0;
  ssize_t bytes_read = ::read(fd_, &byte, 1);
  if (bytes_read != 1) {
    throw RoboClawError("failed to read byte");
  }
  return byte;
}

void RoboClaw::flushInput() { tcflush(fd_, TCIFLUSH); }

void RoboClaw::writeCommand(uint8_t command, const uint8_t* data,
                            size_t length) {
  std::string last_error;
  for (int attempt = 0; attempt < kRetries; attempt++) {
    try {
      flushInput();
      uint16_t crc = 0;
      std::vector<uint8_t> packet;
      packet.reserve(length + 4);
      packet.push_back(address_);
      packet.push_back(command);
      for (size_t i = 0; i < length; i++) {
        packet.push_back(data[i]);
      }
      for (uint8_t byte : packet) {
        updateCrc(crc, byte);
      }
      packet.push_back(static_cast<uint8_t>(crc >> 8));
      packet.push_back(static_cast<uint8_t>(crc & 0xFF));
      writeBytes(packet.data(), packet.size());

      uint8_t ack = readByteWithTimeout();
      if (ack == kAck) {
        return;
      }
      last_error = "bad ack byte: " + std::to_string(ack);
    } catch (const RoboClawError& e) {
      last_error = e.what();
    }
  }
  std::ostringstream message;
  message << "command " << static_cast<int>(command) << " failed after "
          << kRetries << " attempts: " << last_error;
  throw RoboClawError(message.str());
}

void RoboClaw::readCommand(uint8_t command, uint8_t* buffer,
                           size_t expected_length) {
  std::string last_error;
  for (int attempt = 0; attempt < kRetries; attempt++) {
    try {
      flushInput();
      uint8_t header[2] = {address_, command};
      writeBytes(header, 2);

      uint16_t crc = 0;
      updateCrc(crc, address_);
      updateCrc(crc, command);
      for (size_t i = 0; i < expected_length; i++) {
        buffer[i] = readByteWithTimeout();
        updateCrc(crc, buffer[i]);
      }
      uint16_t response_crc = static_cast<uint16_t>(readByteWithTimeout()) << 8;
      response_crc |= readByteWithTimeout();
      if (response_crc == crc) {
        return;
      }
      last_error = "CRC mismatch";
    } catch (const RoboClawError& e) {
      last_error = e.what();
    }
  }
  std::ostringstream message;
  message << "read command " << static_cast<int>(command) << " failed after "
          << kRetries << " attempts: " << last_error;
  throw RoboClawError(message.str());
}

std::string RoboClaw::getVersion() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::string last_error;
  for (int attempt = 0; attempt < kRetries; attempt++) {
    try {
      flushInput();
      uint8_t header[2] = {address_, kGetVersion};
      writeBytes(header, 2);

      uint16_t crc = 0;
      updateCrc(crc, address_);
      updateCrc(crc, kGetVersion);

      std::string version;
      // Version string is null-terminated, at most 48 bytes.
      for (int i = 0; i < 48; i++) {
        uint8_t byte = readByteWithTimeout();
        updateCrc(crc, byte);
        if (byte == 0) {
          break;
        }
        version.push_back(static_cast<char>(byte));
      }
      uint16_t response_crc = static_cast<uint16_t>(readByteWithTimeout()) << 8;
      response_crc |= readByteWithTimeout();
      if (response_crc == crc) {
        // Strip trailing newline.
        while (!version.empty() &&
               (version.back() == '\n' || version.back() == '\r')) {
          version.pop_back();
        }
        return version;
      }
      last_error = "CRC mismatch";
    } catch (const RoboClawError& e) {
      last_error = e.what();
    }
  }
  throw RoboClawError("getVersion failed: " + last_error);
}

void RoboClaw::setVelocityPid(Motor motor, float p, float i, float d,
                              uint32_t qpps) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> data;
  // Wire order is D, P, I, QPPS; PID values scaled by 65536.
  packU32(data, static_cast<uint32_t>(d * 65536.0f));
  packU32(data, static_cast<uint32_t>(p * 65536.0f));
  packU32(data, static_cast<uint32_t>(i * 65536.0f));
  packU32(data, qpps);
  writeCommand(motor == kM1 ? kSetM1VelocityPid : kSetM2VelocityPid,
               data.data(), data.size());
}

void RoboClaw::setPositionPid(Motor motor, float p, float i, float d,
                              uint32_t max_i, uint32_t deadzone,
                              int32_t min_position, int32_t max_position) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> data;
  // Wire order is D, P, I, MaxI, Deadzone, MinPos, MaxPos; PID scaled 1024.
  packU32(data, static_cast<uint32_t>(d * 1024.0f));
  packU32(data, static_cast<uint32_t>(p * 1024.0f));
  packU32(data, static_cast<uint32_t>(i * 1024.0f));
  packU32(data, max_i);
  packU32(data, deadzone);
  packU32(data, static_cast<uint32_t>(min_position));
  packU32(data, static_cast<uint32_t>(max_position));
  writeCommand(motor == kM1 ? kSetM1PositionPid : kSetM2PositionPid,
               data.data(), data.size());
}

void RoboClaw::moveToPosition(Motor motor, uint32_t accel, uint32_t speed,
                              uint32_t decel, int32_t position,
                              bool immediate) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> data;
  packU32(data, accel);
  packU32(data, speed);
  packU32(data, decel);
  packU32(data, static_cast<uint32_t>(position));
  data.push_back(immediate ? 1 : 0);  // 1 => replace buffer, start now.
  writeCommand(motor == kM1 ? kM1SpeedAccelDeccelPos : kM2SpeedAccelDeccelPos,
               data.data(), data.size());
}

RoboClaw::EncoderResult RoboClaw::readEncoder(Motor motor) {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t buffer[5];
  readCommand(motor == kM1 ? kGetM1Enc : kGetM2Enc, buffer, sizeof(buffer));
  EncoderResult result;
  result.value = static_cast<int32_t>(unpackU32(buffer));
  result.status = buffer[4];
  return result;
}

void RoboClaw::setEncoder(Motor motor, int32_t value) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> data;
  packU32(data, static_cast<uint32_t>(value));
  writeCommand(motor == kM1 ? kSetM1EncCount : kSetM2EncCount, data.data(),
               data.size());
}

void RoboClaw::resetEncoders() {
  std::lock_guard<std::mutex> lock(mutex_);
  writeCommand(kResetEnc, nullptr, 0);
}

void RoboClaw::driveSpeed(Motor motor, int32_t speed) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> data;
  packU32(data, static_cast<uint32_t>(speed));
  writeCommand(motor == kM1 ? 35 : 36, data.data(), data.size());
}

int32_t RoboClaw::readSpeed(Motor motor) {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t buffer[5];
  readCommand(motor == kM1 ? kGetM1Speed : kGetM2Speed, buffer, sizeof(buffer));
  // buffer[4] is a direction/status byte; the speed is already signed.
  return static_cast<int32_t>(unpackU32(buffer));
}

std::pair<float, float> RoboClaw::readCurrents() {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t buffer[4];
  readCommand(kGetCurrents, buffer, sizeof(buffer));
  float m1 = static_cast<int16_t>(unpackU16(buffer)) / 100.0f;
  float m2 = static_cast<int16_t>(unpackU16(buffer + 2)) / 100.0f;
  return {m1, m2};
}

float RoboClaw::readTemperature() {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t buffer[2];
  readCommand(kGetTemp, buffer, sizeof(buffer));
  return unpackU16(buffer) / 10.0f;
}

float RoboClaw::readMainBatteryVoltage() {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t buffer[2];
  readCommand(kGetMBatt, buffer, sizeof(buffer));
  return unpackU16(buffer) / 10.0f;
}

float RoboClaw::readLogicBatteryVoltage() {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t buffer[2];
  readCommand(kGetLBatt, buffer, sizeof(buffer));
  return unpackU16(buffer) / 10.0f;
}

uint32_t RoboClaw::readErrorStatus() {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t buffer[4];
  readCommand(kGetError, buffer, sizeof(buffer));
  return unpackU32(buffer);
}

std::string RoboClaw::errorString(uint32_t status) {
  if (status == 0) {
    return "normal";
  }
  static const struct {
    uint32_t bit;
    const char* name;
  } kBits[] = {
      {0x000001, "e-stop"},
      {0x000002, "temperature error"},
      {0x000004, "temperature 2 error"},
      {0x000008, "main voltage high"},
      {0x000010, "logic voltage high"},
      {0x000020, "logic voltage low"},
      {0x000040, "M1 driver fault"},
      {0x000080, "M2 driver fault"},
      {0x000100, "M1 speed error"},
      {0x000200, "M2 speed error"},
      {0x000400, "M1 position error"},
      {0x000800, "M2 position error"},
      {0x001000, "M1 current error"},
      {0x002000, "M2 current error"},
      {0x010000, "M1 over current warning"},
      {0x020000, "M2 over current warning"},
      {0x040000, "main voltage high warning"},
      {0x080000, "main voltage low warning"},
      {0x100000, "temperature warning"},
      {0x200000, "temperature 2 warning"},
      {0x400000, "S4 signal warning"},
      {0x800000, "S5 signal warning"},
      {0x01000000, "M1 speed warning"},
      {0x02000000, "M2 speed warning"},
      {0x04000000, "M1 position warning"},
      {0x08000000, "M2 position warning"},
      {0x10000000, "CAN bus warning"},
      {0x20000000, "boot warning"},
      {0x40000000, "M1 over-regeneration warning"},
      {0x80000000, "M2 over-regeneration warning"},
  };
  std::string result;
  for (const auto& entry : kBits) {
    if (status & entry.bit) {
      if (!result.empty()) {
        result += ", ";
      }
      result += entry.name;
    }
  }
  if (result.empty()) {
    std::ostringstream unknown;
    unknown << "unknown error bits: 0x" << std::hex << status;
    result = unknown.str();
  }
  return result;
}

std::pair<uint8_t, uint8_t> RoboClaw::readBufferLengths() {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t buffer[2];
  readCommand(kGetBuffers, buffer, sizeof(buffer));
  return {buffer[0], buffer[1]};
}

void RoboClaw::stop(Motor motor) {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t data[2] = {0, 0};  // Zero duty.
  writeCommand(motor == kM1 ? kM1Duty : kM2Duty, data, sizeof(data));
}

void RoboClaw::stopAll() {
  stop(kM1);
  stop(kM2);
}

}  // namespace titania_extender
