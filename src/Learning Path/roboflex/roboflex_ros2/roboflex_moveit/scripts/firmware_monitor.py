#!/usr/bin/env python3
"""USB status/ARM/HOLD bridge; never arms automatically or claims measured angles."""
import argparse
import json
import re
import time

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from std_srvs.srv import Trigger

DEFAULT_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
HEADER = re.compile(r'rx_count=(\d+) armed=(\d) hold=(\d) driver_fault=(\d) last_rx_ms_ago=(\d+) wifi=(\w+).*target_deg=\[([^]]+)\]')
CURRENT = re.compile(r'\[STATUS\] current_deg=\[([^]]+)\]')


class FirmwareMonitor(Node):
    def __init__(self, port):
        super().__init__('roboflex_firmware_monitor')
        self.port = serial.Serial(port=None, baudrate=115200, timeout=0,
                                  write_timeout=0.5, exclusive=True)
        self.port.dtr = False
        self.port.rts = False
        self.port.port = port
        self.port.open()
        self.buffer = b''
        self.pending = None
        self.status = None
        self.received_at = 0.0
        self.pub = self.create_publisher(String, '/roboflex/firmware_status', 10)
        self.create_service(Trigger, '/roboflex/arm', self.arm)
        self.create_service(Trigger, '/roboflex/hold', self.hold)
        self.create_timer(0.05, self.poll)
        self.get_logger().info('USB connected; no commands sent. Outputs require explicit /roboflex/arm.')

    def send(self, command):
        # A leading newline clears any partial line left by a previous terminal.
        self.port.write(('\n' + command + '\n').encode('ascii'))
        self.port.flush()

    def arm(self, request, response):
        if not self.status or time.monotonic() - self.received_at > 3:
            response.message = 'No fresh firmware status; ARM refused'
        elif self.status['driver_fault'] or self.status['wifi'] != 'connected':
            response.message = 'Driver/network unhealthy; ARM refused'
        else:
            try:
                self.send('ARM')
                response.success = True
                response.message = 'ARM sent; verify armed status before executing'
            except serial.SerialException as exc:
                response.message = str(exc)
        return response

    def hold(self, request, response):
        try:
            self.send('HOLD')
            response.success = True
            response.message = 'HOLD sent; PWM remains powered, this is not a power disconnect'
        except serial.SerialException as exc:
            response.message = str(exc)
        return response

    def line(self, text):
        if 'Starting RoboFlex' in text or 'Brownout' in text or text.startswith('rst:'):
            self.status = None
            self.pending = None
        if not text.startswith('[STATUS]'):
            self.get_logger().info(text)
        match = HEADER.search(text)
        if match:
            values = match.groups()
            self.pending = dict(zip(
                ['rx_count', 'armed', 'hold', 'driver_fault', 'last_rx_ms_ago'],
                map(int, values[:5])))
            self.pending.update(wifi=values[5], target_deg=[float(v) for v in values[6].split(',')])
        match = CURRENT.search(text)
        if match and self.pending:
            self.pending['current_deg'] = [float(v) for v in match[1].split(',')]
            if len(self.pending['current_deg']) == len(self.pending['target_deg']) == 5:
                self.status = self.pending
                self.received_at = time.monotonic()
                self.get_logger().info(json.dumps(self.status))
            self.pending = None

    def poll(self):
        try:
            self.buffer += self.port.read(self.port.in_waiting or 1)
            if len(self.buffer) > 8192:
                self.buffer = b''
                self.status = None
            while b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                self.line(line.decode('utf-8', errors='replace').strip())
        except (OSError, serial.SerialException) as exc:
            self.status = None
            self.get_logger().error('Serial connection lost: ' + str(exc), throttle_duration_sec=5)
        data = dict(self.status or {})
        data['status_age_s'] = time.monotonic() - self.received_at if self.status else None
        self.pub.publish(String(data=json.dumps(data)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=DEFAULT_PORT)
    args = parser.parse_args()
    rclpy.init()
    node = None
    try:
        node = FirmwareMonitor(args.port)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.port.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
