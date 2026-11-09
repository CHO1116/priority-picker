"""Task Planner: 주문 + 검출 결과 → 다음 피킹 대상 결정 → Robot Controller에 요청.

흐름 (주문 1세트 처리):
  스캔(최신 검출 대기) → 재고 부족 확인 → 대상 선택 → PickPlace 액션 → 결과 처리 → 다시 스캔
  주문 완료 후 남은 물체(주문에 없는 색, unknown)는 보류함(HOLD)으로 옮긴다.

ROS 콜백은 메인 스레드의 executor가, 순차 작업 흐름은 별도 worker 스레드가 처리한다.
"""
import csv
import json
import os
import threading
import time
from datetime import datetime

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from picker_interfaces.action import PickPlace
from picker_interfaces.msg import DetectedCubeArray, OrderArray

from .priority import (Cube, OrderState, find_shortages, leftover_cubes,
                       needed_colors, position_key, select_next)

RESULT_NAMES = {0: 'SUCCESS', 1: 'GRASP_FAIL', 2: 'MOTION_ERROR'}


class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')
        self.declare_parameter('strategy', 'priority')          # priority / color_order / random
        self.declare_parameter('finger_clearance_mm', 15.0)
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('max_retry', 2)
        self.declare_parameter('color_order', ['red', 'blue', 'green', 'yellow'])
        self.declare_parameter('scan_settle_sec', 0.5)          # 로봇 정지 후 영상 안정화 대기
        self.declare_parameter('detection_timeout_sec', 5.0)
        self.declare_parameter('move_leftovers_to_hold', True)
        self.declare_parameter('poses_file', '')
        self.declare_parameter('results_dir', os.path.expanduser('~/robo_ws/results'))

        poses = self._load_yaml(self.get_parameter('poses_file').value)
        self.box_xy = {k: v[:2] for k, v in poses['boxes'].items()}
        self.scan_xy = poses['scan_xy_mm']

        self._lock = threading.Lock()
        self._latest = None          # 최신 DetectedCubeArray
        self._pending_orders = None  # 아직 시작하지 않은 OrderArray
        self._busy = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DetectedCubeArray, '/detected_cubes', self._on_cubes, 10)
        self.create_subscription(OrderArray, '/orders', self._on_orders, latched)
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        self.pick_client = ActionClient(self, PickPlace, '/pick_place')

        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info(f"Task Planner 시작 (strategy={self.get_parameter('strategy').value})")

    # ------------------------------------------------------------------ 콜백
    def _on_cubes(self, msg):
        with self._lock:
            self._latest = msg

    def _on_orders(self, msg):
        with self._lock:
            if self._busy:
                self.get_logger().warn('작업 중에 들어온 주문은 무시합니다. 완료 후 다시 보내주세요.')
                return
            self._pending_orders = msg
        self.get_logger().info(f'주문 {len(msg.orders)}건 수신')

    # ------------------------------------------------------------------ 작업 스레드
    def _worker(self):
        while rclpy.ok():
            with self._lock:
                orders_msg, self._pending_orders = self._pending_orders, None
                self._busy = orders_msg is not None
            if orders_msg is None:
                time.sleep(0.1)
                continue
            try:
                self._run(orders_msg)
            except Exception as e:  # 작업 스레드가 죽지 않도록
                self.get_logger().error(f'작업 중 예외: {e!r}')
            finally:
                with self._lock:
                    self._busy = False

    def _run(self, orders_msg):
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        strategy = p('strategy')
        orders = [OrderState(order_id=o.order_id, priority=o.priority, box_id=o.box_id,
                             quantity={i.color: i.quantity for i in o.items})
                  for o in orders_msg.orders]
        for o in orders:
            if o.box_id not in self.box_xy:
                self.get_logger().error(f'주문 #{o.order_id}의 박스 {o.box_id}가 poses.yaml에 없습니다.')
                return

        if not self.pick_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/pick_place 액션 서버(robot_controller)가 없습니다.')
            return

        log_rows = []
        failures = {}   # position_key → 실패 횟수
        skipped = set()
        t_start = time.time()
        last_motion_end = self.get_clock().now()

        # ---------- 1단계: 주문 처리 ----------
        while any(not o.is_finished() for o in orders):
            cubes = self._wait_fresh_detection(last_motion_end)
            if cubes is None:
                self.get_logger().error('검출 결과가 들어오지 않아 중단합니다. vision_node를 확인하세요.')
                break
            cubes = [c for c in cubes if position_key(c) not in skipped]

            for color, n in find_shortages(cubes, orders).items():
                self.get_logger().warn(f'⚠️ 재고 부족: {color} {n}개 → 해당 주문 부분 완료 처리')
                self._mark_shortage(orders, color)
            self._publish_status(orders, strategy, 'SCAN')

            sel = select_next(cubes, orders, self.scan_xy, self.box_xy,
                              strategy=strategy,
                              finger_clearance_mm=p('finger_clearance_mm'),
                              min_confidence=p('min_confidence'),
                              color_order=p('color_order'))
            if sel is None:
                # 남은 물량이 있는데 집을 수 있는 큐브가 없음 (신뢰도 낮음 / 재시도 초과로 건너뜀)
                for color in needed_colors(orders):
                    self.get_logger().warn(f'⚠️ 집을 수 있는 {color} 큐브가 없음 → 부분 완료 처리')
                    self._mark_shortage(orders, color)
                break
            if sel.gap_ignored:
                self.get_logger().warn(f'{sel.cube.color} 큐브 주변 간격 부족({sel.cube.min_gap:.0f}mm)이지만 다른 후보가 없어 시도합니다.')

            self.get_logger().info(
                f'▶ 주문 #{sel.order.order_id} ({"긴급" if sel.order.priority == 0 else "일반"}) '
                f'{sel.cube.color} @ ({sel.cube.x:.0f}, {sel.cube.y:.0f}) → 박스 {sel.order.box_id}')
            result, elapsed = self._pick_place(sel.cube, sel.order.box_id)
            last_motion_end = self.get_clock().now()

            key = position_key(sel.cube)
            if result == 0:
                sel.order.picked[sel.cube.color] = sel.order.picked.get(sel.cube.color, 0) + 1
            else:
                failures[key] = failures.get(key, 0) + 1
                if failures[key] > p('max_retry'):
                    self.get_logger().warn(f'재시도 {p("max_retry")}회 초과 → 이 큐브는 건너뜁니다.')
                    skipped.add(key)
            log_rows.append(self._log_row(strategy, sel.order.order_id, sel.cube, result,
                                          failures.get(key, 0), elapsed, sel.cost_mm))
            self._publish_status(orders, strategy, RESULT_NAMES.get(result, '?'))

        # ---------- 2단계: 남은 물체 보류함으로 ----------
        if p('move_leftovers_to_hold') and 'HOLD' in self.box_xy:
            while True:
                cubes = self._wait_fresh_detection(last_motion_end)
                if not cubes:
                    break
                rest = [c for c in leftover_cubes(cubes, orders) if position_key(c) not in skipped]
                if not rest:
                    break
                cube = rest[0]
                self.get_logger().info(f'▶ 보류함으로 이동: {cube.color} @ ({cube.x:.0f}, {cube.y:.0f})')
                result, elapsed = self._pick_place(cube, 'HOLD')
                last_motion_end = self.get_clock().now()
                if result != 0:
                    skipped.add(position_key(cube))  # 보류 물체는 재시도하지 않음
                log_rows.append(self._log_row(strategy, -1, cube, result, 0, elapsed, 0.0))

        total = time.time() - t_start
        self._publish_status(orders, strategy, 'FINISHED')
        self._save_report(strategy, log_rows, total, orders)

    # ------------------------------------------------------------------ 보조 함수
    def _wait_fresh_detection(self, after):
        """after 시각 + 안정화 시간 이후에 촬영된 검출 결과를 기다린다."""
        settle_ns = int(self.get_parameter('scan_settle_sec').value * 1e9)
        deadline = time.time() + self.get_parameter('detection_timeout_sec').value
        threshold = after.nanoseconds + settle_ns
        while time.time() < deadline and rclpy.ok():
            with self._lock:
                msg = self._latest
            if msg is not None:
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                if stamp >= threshold:
                    return [Cube(c.color, c.x_mm, c.y_mm, c.z_mm, c.yaw_deg, c.confidence, c.min_gap_mm)
                            for c in msg.cubes]
            time.sleep(0.05)
        return None

    def _pick_place(self, cube, box_id):
        goal = PickPlace.Goal(x_mm=cube.x, y_mm=cube.y, z_mm=cube.z,
                              yaw_deg=cube.yaw, place_box_id=box_id)
        t0 = time.time()
        gh = self._wait_future(self.pick_client.send_goal_async(goal), 10.0)
        if gh is None or not gh.accepted:
            self.get_logger().error('PickPlace 요청이 거부되었습니다.')
            return 2, time.time() - t0
        res = self._wait_future(gh.get_result_async(), 120.0)
        if res is None:
            self.get_logger().error('PickPlace 결과 대기 시간 초과')
            return 2, time.time() - t0
        r = res.result
        if r.result != 0:
            self.get_logger().warn(f'결과: {RESULT_NAMES.get(r.result)} ({r.message})')
        return r.result, r.elapsed_sec

    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                return None
            time.sleep(0.02)
        return future.result()

    @staticmethod
    def _mark_shortage(orders, color):
        for o in orders:
            n = o.remaining(color)
            if n > 0:
                o.shortage[color] = o.shortage.get(color, 0) + n

    def _publish_status(self, orders, strategy, event):
        data = {
            'event': event,
            'strategy': strategy,
            'orders': [{
                'order_id': o.order_id,
                'priority': 'URGENT' if o.priority == 0 else 'NORMAL',
                'box_id': o.box_id,
                'status': o.status,
                'items': {c: f'{o.picked.get(c, 0)}/{q}' for c, q in o.quantity.items()},
                'shortage': o.shortage,
            } for o in orders],
        }
        self.status_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    @staticmethod
    def _log_row(strategy, order_id, cube, result, retry, elapsed, cost):
        return {
            'time': datetime.now().isoformat(timespec='seconds'),
            'strategy': strategy, 'order_id': order_id, 'color': cube.color,
            'x_mm': round(cube.x, 1), 'y_mm': round(cube.y, 1),
            'result': RESULT_NAMES.get(result, str(result)), 'retry': retry,
            'elapsed_sec': round(elapsed, 2), 'travel_cost_mm': round(cost, 1),
        }

    def _save_report(self, strategy, rows, total, orders):
        out_dir = self.get_parameter('results_dir').value
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        if rows:
            path = os.path.join(out_dir, f'run_{stamp}_{strategy}.csv')
            with open(path, 'w', newline='', encoding='utf-8-sig') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            self.get_logger().info(f'피킹 로그 저장: {path}')

        n_ok = sum(r['result'] == 'SUCCESS' for r in rows)
        summary = {
            'time': stamp, 'strategy': strategy, 'total_sec': round(total, 1),
            'attempts': len(rows), 'success': n_ok,
            'success_rate': round(n_ok / len(rows), 3) if rows else 0.0,
            'travel_cost_mm': round(sum(r['travel_cost_mm'] for r in rows), 1),
            'orders': ' '.join(f'#{o.order_id}:{o.status}' for o in orders),
        }
        spath = os.path.join(out_dir, 'summary.csv')
        new = not os.path.exists(spath)
        with open(spath, 'a', newline='', encoding='utf-8-sig') as f:
            w = csv.DictWriter(f, fieldnames=list(summary.keys()))
            if new:
                w.writeheader()
            w.writerow(summary)
        self.get_logger().info(f'✅ 완료: {summary}')

    @staticmethod
    def _load_yaml(path):
        if not path:
            raise RuntimeError('poses_file 파라미터가 비어 있습니다 (picker_bringup/config/poses.yaml).')
        with open(path, encoding='utf-8') as f:
            return yaml.safe_load(f)


def main():
    rclpy.init()
    node = TaskPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
