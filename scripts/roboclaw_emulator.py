#!/usr/bin/env python3
"""RoboClaw packet-serial emulator for testing titania_extender without
hardware.

Creates a pseudo-terminal and emulates the subset of the RoboClaw protocol
used by the driver, including a simple motion model: commanded position
moves advance the encoders at the commanded speed.

Usage:
    python3 roboclaw_emulator.py [--link /tmp/roboclaw_emu]

Then run the driver with device_name set to the link path.
"""

import argparse
import os
import struct
import time


def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class Motor:
    def __init__(self):
        self.position = 0.0
        self.target = 0.0
        self.speed = 0.0          # commanded speed magnitude, qpps
        self.moving = False
        self.last_update = time.monotonic()

    def update(self):
        now = time.monotonic()
        dt = now - self.last_update
        self.last_update = now
        if not self.moving:
            return
        remaining = self.target - self.position
        step = self.speed * dt
        if abs(remaining) <= step:
            self.position = self.target
            self.moving = False
        else:
            self.position += step if remaining > 0 else -step

    @property
    def velocity(self) -> int:
        if not self.moving:
            return 0
        return int(self.speed if self.target > self.position else -self.speed)


# Data lengths (bytes) for supported write commands.
WRITE_LENGTHS = {
    20: 0,            # reset encoders
    22: 4, 23: 4,     # set encoder value
    28: 16, 29: 16,   # set velocity PID
    32: 2, 33: 2,     # duty
    61: 28, 62: 28,   # set position PID
    65: 17, 66: 17,   # speed/accel/deccel/position
}

ADDRESS = 0x80
VERSION = b"RoboClaw Emulator v1.0\n"


class Emulator:
    def __init__(self, link: str):
        self.master, slave = os.openpty()
        slave_name = os.ttyname(slave)
        if os.path.lexists(link):
            os.unlink(link)
        os.symlink(slave_name, link)
        print(f"RoboClaw emulator ready on {link} ({slave_name})", flush=True)
        self.motors = [Motor(), Motor()]

    def read_exact(self, count: int) -> bytes:
        data = b""
        while len(data) < count:
            data += os.read(self.master, count - len(data))
        return data

    def send(self, data: bytes):
        os.write(self.master, data)

    def reply_read(self, command: int, payload: bytes):
        crc = crc16(bytes([ADDRESS, command]) + payload)
        self.send(payload + struct.pack(">H", crc))

    def handle_write(self, command: int):
        length = WRITE_LENGTHS[command]
        data = self.read_exact(length + 2)  # data + crc
        payload = data[:length]
        received_crc = struct.unpack(">H", data[length:])[0]
        expected = crc16(bytes([ADDRESS, command]) + payload)
        if received_crc != expected:
            return  # No ack on bad CRC; the driver retries.

        if command == 20:
            for motor in self.motors:
                motor.position = 0.0
                motor.target = 0.0
                motor.moving = False
        elif command in (22, 23):
            motor = self.motors[command - 22]
            motor.position = float(struct.unpack(">i", payload)[0])
            motor.moving = False
        elif command in (32, 33):
            motor = self.motors[command - 32]
            motor.moving = False  # zero duty stop
        elif command in (65, 66):
            motor = self.motors[command - 65]
            _accel, speed, _decel, position = struct.unpack(
                ">IIIi", payload[:16])
            motor.update()
            motor.target = float(position)
            motor.speed = float(max(speed, 1))
            motor.moving = True
            print(f"M{command - 64}: move to {position} at {speed} qpps",
                  flush=True)

        self.send(b"\xff")

    def handle_read(self, command: int):
        if command == 21:
            self.reply_read(command, VERSION + b"\x00")
            return
        if command in (16, 17, 18, 19):
            motor = self.motors[(command - 16) % 2]
            motor.update()
            if command in (16, 17):
                self.reply_read(
                    command, struct.pack(">iB", int(motor.position), 0))
            else:
                self.reply_read(
                    command, struct.pack(">iB", motor.velocity, 0))
            return
        if command == 24:
            self.reply_read(command, struct.pack(">H", 126))   # 12.6 V
        elif command == 25:
            self.reply_read(command, struct.pack(">H", 50))    # 5.0 V
        elif command == 47:
            buffers = bytes(0x80 if not m.moving else 0 for m in self.motors)
            self.reply_read(command, buffers)
        elif command == 49:
            self.reply_read(command, struct.pack(">hh", 42, 17))  # amps*100
        elif command == 82:
            self.reply_read(command, struct.pack(">H", 312))   # 31.2 C
        elif command == 90:
            self.reply_read(command, struct.pack(">I", 0))
        else:
            print(f"Unsupported read command: {command}", flush=True)

    def run(self):
        while True:
            address = self.read_exact(1)[0]
            if address != ADDRESS:
                continue
            command = self.read_exact(1)[0]
            if command in WRITE_LENGTHS:
                self.handle_write(command)
            else:
                self.handle_read(command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--link", default="/tmp/roboclaw_emu",
                        help="Symlink to create for the emulated device.")
    args = parser.parse_args()
    Emulator(args.link).run()


if __name__ == "__main__":
    main()
