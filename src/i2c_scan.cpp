// Copyright 2026 Wimblerobotics
// SPDX-License-Identifier: Apache-2.0

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <system_error>

namespace {

bool scanBus(int64_t bus) {
  const std::string path = "/dev/i2c-" + std::to_string(bus);
  const int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    std::cerr << "Cannot open " << path << ": " << std::strerror(errno) << '\n';
    return false;
  }

  unsigned long funcs = 0;  // NOLINT(runtime/int): Required by I2C_FUNCS.
  if (ioctl(fd, I2C_FUNCS, &funcs) < 0) {
    std::cerr << "Cannot query " << path << ": " << std::strerror(errno)
              << '\n';
    close(fd);
    return false;
  }

  unsigned found = 0;
  unsigned incomplete = 0;
  std::cout << "Scanning " << path << '\n';
  for (int address = 0x08; address <= 0x77; ++address) {
    const bool read_byte = (address >= 0x30 && address <= 0x37) ||
                           (address >= 0x50 && address <= 0x5f) ||
                           !(funcs & I2C_FUNC_SMBUS_QUICK);
    const bool probe_supported = read_byte ? (funcs & I2C_FUNC_SMBUS_READ_BYTE)
                                           : (funcs & I2C_FUNC_SMBUS_QUICK);
    if (!probe_supported) {
      ++incomplete;
      continue;
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
      std::cerr << path << ":0x" << std::hex << address << std::dec << ": "
                << std::strerror(errno) << '\n';
      ++incomplete;
      continue;
    }

    union i2c_smbus_data data = {};
    i2c_smbus_ioctl_data probe = {};
    probe.read_write = read_byte ? I2C_SMBUS_READ : I2C_SMBUS_WRITE;
    probe.size = read_byte ? I2C_SMBUS_BYTE : I2C_SMBUS_QUICK;
    probe.data = read_byte ? &data : nullptr;

    const auto started = std::chrono::steady_clock::now();
    const int result = ioctl(fd, I2C_SMBUS, &probe);
    const int probe_errno = errno;
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started);
    if (duration >= std::chrono::milliseconds(20)) {
      std::cerr << path << ":0x" << std::hex << address << std::dec
                << " probe took " << duration.count() << " ms\n";
    }

    if (result == 0) {
      ++found;
      std::cout << "i2c_device_found:" << path << ":0x" << std::hex << address
                << std::dec << '\n';
    } else if (probe_errno != ENXIO && probe_errno != EREMOTEIO) {
      std::cerr << path << ":0x" << std::hex << address << std::dec << ": "
                << std::strerror(probe_errno) << '\n';
      ++incomplete;
    }
  }

  close(fd);
  std::cout << "i2c_scan_complete:" << path << ":devices=" << found
            << ",incomplete=" << incomplete << '\n';
  return incomplete == 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: i2c_scan <bus-number>\n";
    return 2;
  }

  int64_t bus = 0;
  const char* const end = argv[1] + std::strlen(argv[1]);
  const auto parsed = std::from_chars(argv[1], end, bus);
  if (parsed.ec != std::errc{} || parsed.ptr != end || bus < 0) {
    std::cerr << "Invalid I2C bus number: " << argv[1] << '\n';
    return 2;
  }

  return scanBus(bus) ? 0 : 1;
}
