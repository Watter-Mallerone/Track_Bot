#!/usr/bin/env python3

import sys, termios, tty, select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

HELP = """
↑ : 전진 속도 +
↓ : 후진 속도 -
SPACE : 정지
q : 종료
"""

def getkey(timeout=0.05):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            ch = sys.stdin.read(1)
        else:
            ch = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

class WheelTeleop(Node):
    def __init__(self):
        super().__init__('track_bot_keyboard_controller')
        # 컨트롤러 토픽: <controller_name>/commands
        self.pub = self.create_publisher(Float64MultiArray, '/wheel_control/commands', 10)

        # 파라미터
        self.declare_parameter('step', 1.0)      # 키 1번당 증가 속도(rad/s)
        self.declare_parameter('max_speed', 20.0)
        self.declare_parameter('num_wheels', 4)  # 2 또는 4

        self.step = float(self.get_parameter('step').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.num_wheels = int(self.get_parameter('num_wheels').value)

        self.v = 0.0
        self.get_logger().info(HELP)

        self.timer = self.create_timer(0.05, self.loop)

    def clamp(self, x, lo, hi):
        return max(min(x, hi), lo)

    def publish_velocity(self):
        msg = Float64MultiArray()
        # 모든 구동바퀴 동일 속도
        msg.data = [self.v] * self.num_wheels
        self.pub.publish(msg)

    def loop(self):
        key = getkey()
        if key == '\x1b':  # arrow keys start with ESC
            key2 = getkey()
            key3 = getkey()
            if key2 == '[':
                if key3 == 'A':   # Up
                    self.v = 3.0 # self.clamp(self.v + self.step, -self.max_speed, self.max_speed)
                elif key3 == 'B': # Down
                    self.v = -3.0 # self.clamp(self.v - self.step, -self.max_speed, self.max_speed)
                self.get_logger().info(f'vel = {self.v:.2f} rad/s')
                self.publish_velocity()
        elif key == ' ':
            self.v = 0.0
            self.get_logger().info('STOP')
            self.publish_velocity()
        elif key == 'q':
            self.get_logger().info('quit')
            rclpy.shutdown()

def main():
    rclpy.init()
    node = WheelTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
