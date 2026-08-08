#!/usr/bin/env python3
"""Pi host health: publishes CPU load, utilization, the serial_bridge process CPU,
memory, and CPU temperature (~1 Hz) so the dashboard can answer "is the Pi the bottleneck?".
"""

import os

import psutil
import rclpy
from rclpy.node import Node

from follow_me_interfaces.msg import PiHealth

TOPIC_PI_HEALTH = "pi_health"
THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"
BRIDGE_MATCH = "serial_bridge"
PUBLISH_PERIOD_S = 1.0


class PiHealthNode(Node):
    """Samples host metrics via psutil + thermal sysfs and publishes them at ~1 Hz."""

    def __init__(self):
        """Set up the publisher and timer, and prime the CPU-percent counter."""
        super().__init__("pi_health")
        self.ncpu = psutil.cpu_count() or 1
        self._bridge = None  # cached serial_bridge psutil.Process (re-found if it restarts)
        psutil.cpu_percent(interval=None)  # prime: the first call always returns 0.0
        self.pub = self.create_publisher(PiHealth, TOPIC_PI_HEALTH, 10)
        self.timer = self.create_timer(PUBLISH_PERIOD_S, self._tick)

    def _bridge_cpu(self):
        """serial_bridge process CPU % (cached, re-found on restart); -1 if the process is absent."""
        if self._bridge is None or not self._bridge.is_running():
            self._bridge = None
            for p in psutil.process_iter(["cmdline"]):
                cmd = " ".join(p.info["cmdline"] or [])
                if BRIDGE_MATCH in cmd and "pi_health" not in cmd:
                    self._bridge = p
                    p.cpu_percent(interval=None)  # prime; the first sample has no delta yet
                    return 0.0
            return -1.0  # serial_bridge not running
        try:
            return float(self._bridge.cpu_percent(interval=None))
        except psutil.Error:
            self._bridge = None
            return -1.0

    def _temp_c(self):
        """CPU temperature in C from thermal sysfs, or NaN if unreadable."""
        try:
            with open(THERMAL_PATH) as f:
                return int(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            return float("nan")

    def _tick(self):
        """Sample all host metrics and publish one PiHealth message."""
        vm = psutil.virtual_memory()
        msg = PiHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.load_percent = float(os.getloadavg()[0]) / self.ncpu * 100.0  # % of total CPU capacity
        msg.cpu_percent = float(psutil.cpu_percent(interval=None))  # % since the last tick (~1 s)
        msg.bridge_cpu_percent = self._bridge_cpu()
        msg.mem_used_mb = int((vm.total - vm.available) // (1024 * 1024))
        msg.mem_total_mb = int(vm.total // (1024 * 1024))
        msg.temp_c = self._temp_c()
        self.pub.publish(msg)


def main(args=None):
    """Init rclpy, spin the PiHealthNode, and shut down cleanly."""
    rclpy.init(args=args)
    node = PiHealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
