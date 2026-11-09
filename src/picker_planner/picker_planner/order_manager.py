"""Order Manager: orders.yaml을 읽어 /orders로 발행하고, /task_status를 보기 좋게 출력한다.

실행할 때마다 주문 1세트가 들어간다. 다시 실행하려면 이 노드만 재시작하면 된다:
  ros2 run picker_planner order_manager --ros-args -p orders_file:=<경로>
"""
import json

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from picker_interfaces.msg import Order, OrderArray, OrderItem

PRIORITY = {'URGENT': Order.URGENT, 'NORMAL': Order.NORMAL}


class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')
        self.declare_parameter('orders_file', '')
        path = self.get_parameter('orders_file').value
        if not path:
            raise RuntimeError('orders_file 파라미터가 비어 있습니다.')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OrderArray, '/orders', latched)
        self.create_subscription(String, '/task_status', self._on_status, 10)

        msg = self._load(path)
        self.pub.publish(msg)
        for o in msg.orders:
            items = ', '.join(f'{i.color}×{i.quantity}' for i in o.items)
            tag = '🚨긴급' if o.priority == Order.URGENT else '일반'
            self.get_logger().info(f'주문 #{o.order_id} [{tag}] {items} → 박스 {o.box_id}')

    @staticmethod
    def _load(path):
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
        msg = OrderArray()
        for o in data['orders']:
            msg.orders.append(Order(
                order_id=int(o['id']),
                priority=PRIORITY[o.get('priority', 'NORMAL').upper()],
                box_id=str(o['box']),
                items=[OrderItem(color=c, quantity=int(q), picked=0) for c, q in o['items'].items()],
                status='PENDING',
            ))
        return msg

    def _on_status(self, msg):
        data = json.loads(msg.data)
        lines = [f"[{data['event']}]"]
        for o in data['orders']:
            items = ' '.join(f'{c}:{v}' for c, v in o['items'].items())
            short = f" 부족:{o['shortage']}" if o['shortage'] else ''
            lines.append(f"  #{o['order_id']} {o['priority']:<6} {o['status']:<11} {items}{short}")
        self.get_logger().info('\n'.join(lines))


def main():
    rclpy.init()
    node = OrderManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
