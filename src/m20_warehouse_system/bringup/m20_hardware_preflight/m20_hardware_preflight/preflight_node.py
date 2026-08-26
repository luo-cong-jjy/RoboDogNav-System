import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class M20HardwarePreflight(Node):
    """Fail-closed motion interface smoke test; motion is opt-in."""

    def __init__(self):
        super().__init__('m20_hardware_preflight')
        self.declare_parameter('command_topic', '/m20/control/cmd_vel_safe')
        self.declare_parameter('backend_ready_topic', '/m20/locomotion/backend_ready')
        self.declare_parameter('backend_fault_topic', '/m20/locomotion/backend_fault')
        self.declare_parameter('measured_twist_topic', '/m20/locomotion/measured_twist')
        self.declare_parameter('safety_state_topic', '/m20/control/safety_state')
        self.declare_parameter('enable_motion_service', '/m20/hardware/enable_motion')
        self.declare_parameter('feedback_timeout_sec', 1.0)
        self.declare_parameter('run_motion_test', False)
        self.declare_parameter('test_speed', 0.05)
        self.declare_parameter('test_duration_sec', 1.0)
        self._ready = False
        self._fault = ''
        self._safety = 'UNKNOWN'
        self._last_feedback = 0.0
        self._running = False
        self._start = 0.0
        self._pub = self.create_publisher(Twist, self.get_parameter('command_topic').value, 10)
        self.create_subscription(Bool, self.get_parameter('backend_ready_topic').value, self._ready_cb, 10)
        self.create_subscription(String, self.get_parameter('backend_fault_topic').value, self._fault_cb, 10)
        self.create_subscription(String, self.get_parameter('safety_state_topic').value, self._safety_cb, 10)
        self.create_subscription(Twist, self.get_parameter('measured_twist_topic').value, self._feedback_cb, 10)
        self.create_service(Trigger, '~/start_motion_test', self._start_test)
        self.create_timer(0.05, self._tick)
        self.get_logger().info('Preflight ready; motion test is disabled by default')

    def _ready_cb(self, msg):
        self._ready = bool(msg.data)

    def _fault_cb(self, msg):
        self._fault = str(msg.data)

    def _safety_cb(self, msg):
        self._safety = str(msg.data)

    def _feedback_cb(self, _msg):
        self._last_feedback = time.monotonic()

    def _start_test(self, _request, response):
        if not self._ready or self._fault or self._safety not in ('NAVIGATION', 'MANUAL'):
            response.success = False
            response.message = f'blocked: ready={self._ready}, fault={self._fault}, safety={self._safety}'
            return response
        self._running = True
        self._start = time.monotonic()
        response.success = True
        response.message = 'low-speed forward motion test started'
        return response

    def _tick(self):
        msg = Twist()
        feedback_age = time.monotonic() - self._last_feedback if self._last_feedback else float('inf')
        safe = self._ready and not self._fault and self._safety in ('NAVIGATION', 'MANUAL')
        if self._running and safe and feedback_age <= float(self.get_parameter('feedback_timeout_sec').value):
            if time.monotonic() - self._start < float(self.get_parameter('test_duration_sec').value):
                msg.linear.x = float(self.get_parameter('test_speed').value)
            else:
                self._running = False
                self.get_logger().info('motion test complete; command returned to zero')
        elif self._running:
            self._running = False
            self.get_logger().error('motion test aborted by readiness, safety, fault, or feedback timeout')
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = M20HardwarePreflight()
    try:
        rclpy.spin(node)
    finally:
        node._pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
