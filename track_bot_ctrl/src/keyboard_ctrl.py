#!/usr/bin/env python3

import os, sys, termios, tty, select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

HELP = """
w : 전진
s : 후진

q : 종료
"""
class KeyReader:
    def __init__(self):
        self.stream = None
        self.fd = None
        self.old = None

    def open(self):
        # 1) stdin이 TTY면 그대로 사용
        if sys.stdin and sys.stdin.isatty():
            self.stream = sys.stdin
            self.fd = sys.stdin.fileno()
        else:
            # 2) 아니면 /dev/tty 시도 (launch에서 종종 필요)
            try:
                self.stream = open('/dev/tty', 'rb', buffering=0)
                self.fd = self.stream.fileno()
            except Exception:
                raise RuntimeError("No TTY available. Run in a real terminal or set emulate_tty=True in the launch file.")
        # 현재 터미널 설정 저장
        self.old = termios.tcgetattr(self.fd)

    def close(self):
        # 터미널 설정 복구
        if self.fd is not None and self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        # /dev/tty를 열었으면 닫기
        if self.stream not in (None, sys.stdin):
            self.stream.close()

    def getKey(self, timeout=0.05):
        tty.setraw(self.fd)
        try:
            r, _, _ = select.select([self.fd], [], [], timeout)
            ch = ''
            if r:
                try:
                    b = os.read(self.fd, 1)
                    ch = b.decode(errors='ignore')
                except Exception:
                    data = self.stream.read(1)
                    ch = data if isinstance(data, str) else data.decode(errors='ignore')
            return ch
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

class WheelTeleop(Node):
    def __init__(self, key_reader: KeyReader):
        super().__init__('track_bot_keyboard_controller')
        # 컨트롤러 토픽: <controller_name>/commands
        self.pub = self.create_publisher(Float64MultiArray, '/wheel_control/commands', 10)

        self.v = 3.0
        self.get_logger().info(HELP)

        self.kr = key_reader
        self.timer = self.create_timer(0.01, self.loop)

    def clamp(self, x, lo, hi):
        return max(min(x, hi), lo)

    def loop(self):
        key = self.kr.getKey(0.05)
        msg = Float64MultiArray()
        msg.data = [0.0] * 4

        if key == 'w':
            msg.data = [self.v, self.v, self.v, self.v]
            # self.get_logger().info(f'vel = {self.v:.2f}')
        elif key == 's':
            msg.data = [-self.v, -self.v, -self.v, -self.v]
        elif key == 'a':
            msg.data = [self.v, -self.v, self.v, -self.v]
        elif key == 'd':
            msg.data = [-self.v, self.v, -self.v, self.v]
        elif key == '':
            msg.data = [0.0, 0.0, 0.0, 0.0]
        elif key == 'q':
            msg.data = [0.0, 0.0, 0.0, 0.0]
            self.pub.publish(msg)
            self.get_logger().info('quit')
            rclpy.shutdown()
            return

        self.pub.publish(msg)

def main():
    rclpy.init()
    kr = KeyReader()
    try:
        kr.open()
    except Exception as e:
        # TTY가 없으면 친절히 안내 후 종료
        rclpy.init() if not rclpy.ok() else None
        node = rclpy.create_node('track_bot_keyboard_controller_init_error')
        node.get_logger().error(f"{e} (Hint: launch 파일 Node에 emulate_tty=True 설정 또는 'ros2 run'으로 실행)")
        node.destroy_node()
        rclpy.shutdown()
        return

    node = WheelTeleop(kr)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        kr.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
