"""가짜 작업대: 카메라·비전 없이 planner와 robot_controller(sim)를 테스트하기 위한 노드.

- 무작위 위치에 색상 큐브를 만들어 /detected_cubes로 발행한다.
- robot_controller(sim 모드)가 /sim/picked 로 집은 위치를 알려주면 가장 가까운 큐브를 지운다.
"""
import math
import random

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from picker_interfaces.msg import DetectedCube, DetectedCubeArray


class FakeWorld(Node):
    def __init__(self):
        super().__init__('fake_world')
        self.declare_parameter('counts', ['red:3', 'blue:2', 'green:3', 'yellow:2'])
        self.declare_parameter('area_mm', [300.0, -200.0, 600.0, 200.0])  # x_min, y_min, x_max, y_max
        self.declare_parameter('cube_size_mm', 50.0)
        self.declare_parameter('top_z_mm', 50.0)
        self.declare_parameter('seed', 0)
        self.declare_parameter('unknown_objects', 1)

        size = self.get_parameter('cube_size_mm').value
        rng = random.Random(self.get_parameter('seed').value)
        x0, y0, x1, y1 = self.get_parameter('area_mm').value
        z = self.get_parameter('top_z_mm').value

        colors = []
        for item in self.get_parameter('counts').value:
            name, n = item.split(':')
            colors += [name] * int(n)
        colors += ['unknown'] * self.get_parameter('unknown_objects').value

        self.cubes = []
        for color in colors:
            for _ in range(200):  # 큐브끼리 겹치지 않는 위치 찾기
                x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
                if all(math.hypot(x - c.x_mm, y - c.y_mm) > size * 1.1 for c in self.cubes):
                    break
            self.cubes.append(DetectedCube(color=color, x_mm=x, y_mm=y, z_mm=z,
                                           yaw_deg=rng.uniform(-45, 45),
                                           confidence=1.0 if color != 'unknown' else 0.3))

        self.pub = self.create_publisher(DetectedCubeArray, '/detected_cubes', 10)
        self.create_subscription(Point, '/sim/picked', self._on_picked, 10)
        self.create_timer(0.2, self._publish)
        self.get_logger().info(f'가짜 큐브 {len(self.cubes)}개 생성')

    def _publish(self):
        size = self.get_parameter('cube_size_mm').value
        for a in self.cubes:
            gaps = [math.hypot(a.x_mm - b.x_mm, a.y_mm - b.y_mm) - size for b in self.cubes if b is not a]
            a.min_gap_mm = float(max(0.0, min(gaps))) if gaps else 999.0
        msg = DetectedCubeArray(cubes=self.cubes)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        self.pub.publish(msg)

    def _on_picked(self, p):
        if not self.cubes:
            return
        nearest = min(self.cubes, key=lambda c: math.hypot(c.x_mm - p.x, c.y_mm - p.y))
        if math.hypot(nearest.x_mm - p.x, nearest.y_mm - p.y) < 30:
            self.cubes.remove(nearest)
            self.get_logger().info(f'{nearest.color} 큐브 제거 (남은 {len(self.cubes)}개)')


def main():
    rclpy.init()
    node = FakeWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
