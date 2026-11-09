#!/usr/bin/env python3
"""Test-only Feetech protocol emulator. Opens a new pseudo-terminal, never USB."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import sys
import threading
import time

class SerialEmulator:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.stop = threading.Event()
        self.writes = []
        self.torque_events = []
        self.feedback_offsets = {i: 0 for i in range(1, 7)}
        self.registers = {i: bytearray(256) for i in range(1, 7)}
        for i, reg in self.registers.items():
            reg[3:5] = (777).to_bytes(2, 'little')
            reg[5] = i; reg[30] = 1; reg[55] = 1; reg[62] = 124; reg[63] = 33
            reg[11:13] = (4095).to_bytes(2, 'little')
            reg[56:58] = (2048).to_bytes(2, 'little')
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def update(self, servo, address, data):
        self.writes.append((servo, address, bytes(data)))
        if address == 40:
            self.torque_events.append(dict(id=servo, value=data[0], time=time.time()))
        reg = self.registers[servo]
        reg[address:address + len(data)] = data
        if reg[40] and (address == 40 or address <= 42 < address + len(data)):
            reg[56:58] = reg[42:44]

    def run(self):
        buffer = bytearray()
        while not self.stop.is_set():
            if not select.select([self.master], [], [], 0.05)[0]: continue
            buffer.extend(os.read(self.master, 4096))
            while len(buffer) >= 4:
                if buffer[:2] != b'\xff\xff': del buffer[0]; continue
                length = buffer[3] + 4
                if len(buffer) < length: break
                packet = bytes(buffer[:length]); del buffer[:length]
                if sum(packet[2:]) & 255 != 255: continue
                servo, instruction = packet[2], packet[4]
                if servo == 254 and instruction == 0x83:
                    address, count = packet[5:7]
                    for index in range(7, length - 1, count + 1):
                        self.update(packet[index], address, packet[index + 1:index + 1 + count])
                    continue
                if servo not in self.registers: continue
                data = b''
                if instruction == 2:
                    address, count = packet[5:7]
                    registers = bytearray(self.registers[servo])
                    measured = int.from_bytes(registers[56:58], 'little') + self.feedback_offsets[servo]
                    registers[56:58] = max(0, min(4095, measured)).to_bytes(2, 'little')
                    data = bytes(registers[address:address + count])
                elif instruction == 3:
                    self.update(servo, packet[5], packet[6:-1])
                payload = bytes([servo, len(data) + 2, 0]) + data
                os.write(self.master, b'\xff\xff' + payload + bytes([~sum(payload) & 255]))

    def close(self):
        self.stop.set(); self.thread.join(); os.close(self.master); os.close(self.slave)


if __name__ == '__main__':
    bus = SerialEmulator()
    if len(sys.argv) > 3:
        initial = json.loads(Path(sys.argv[3]).read_text())
        for servo, ticks in initial.items():
            bus.registers[int(servo)][56:58] = int(ticks).to_bytes(2, 'little')
    running = True
    def stop(signum, frame):
        global running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    Path(sys.argv[1]).write_text(json.dumps({'port': bus.path}))
    try:
        while running:
            fault_path = Path(sys.argv[1]).parent / 'fault.json'
            if fault_path.exists():
                fault = json.loads(fault_path.read_text())
                if 'feedback_offset' in fault:
                    bus.feedback_offsets[fault['servo']] = fault['feedback_offset']
                else:
                    bus.registers[fault['servo']][fault['address']] = fault['value']
                fault_path.unlink()
            time.sleep(0.05)
    finally:
        bus.close()
        Path(sys.argv[2]).write_text(json.dumps({
            'torque': {str(i): reg[40] for i, reg in bus.registers.items()},
            'eeprom_writes': sum(address < 40 for _, address, _ in bus.writes),
            'positions': {str(i): int.from_bytes(reg[56:58], 'little') for i, reg in bus.registers.items()},
            'torque_events': bus.torque_events,
            'writes': len(bus.writes)}))
