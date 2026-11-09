"""Robot Controller: /pick_place 액션 서버.

동작 순서
  APPROACH  큐브 위 (approach_height_mm) 로 이동, 그리퍼 열기
  DESCEND   파지 높이로 하강
  GRASP     그리퍼 닫기 → 벌림 폭이 grasp_fail_width_mm 이하면 파지 실패
  LIFT      위로 상승
  PLACE     박스 위로 이동 → 그리퍼 열기
  RETURN    스캔 자세로 복귀 (다음 촬영을 위해)
파지 실패 / 모션 오류가 나도 항상 안전 높이 → 스캔 자세로 복귀한 뒤 결과를 반환한다.

실행 모드 (파라미터)
  robot_mode:=sim | dsr      gripper_mode:=sim | drl
"""
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from picker_interfaces.action import PickPlace

from .motion import (DrlGripper, DsrRobot, SimGripper, SimRobot, top_down_pose,
                     within_workspace)


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.declare_parameter('robot_mode', 'sim')
        self.declare_parameter('gripper_mode', 'sim')
        self.declare_parameter('sim_grasp_fail_rate', 0.1)
        self.declare_parameter('dsr_namespace', 'dsr01')
        self.declare_parameter('poses_file', '')

        with open(self.get_parameter('poses_file').value, encoding='utf-8') as f:
            self.cfg = yaml.safe_load(f)
        m = self.cfg['motion']

        if self.get_parameter('robot_mode').value == 'dsr':
            self.robot = DsrRobot(self, self.get_parameter('dsr_namespace').value)
        else:
            self.robot = SimRobot(self.get_logger())
        if self.get_parameter('gripper_mode').value == 'drl':
            self.gripper = DrlGripper(self)
        else:
            self.gripper = SimGripper(m['cube_size_mm'], self.get_parameter('sim_grasp_fail_rate').value)

        self.picked_pub = self.create_publisher(Point, '/sim/picked', 10)  # fake_world용
        self._busy = threading.Lock()
        self._action_server = ActionServer(self, PickPlace, '/pick_place', execute_callback=self._execute,
                     goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.REJECT,
                     callback_group=ReentrantCallbackGroup())

        self.get_logger().info('스캔 자세로 이동합니다.')
        threading.Thread(target=self._go_scan, daemon=True).start()
        self.get_logger().info(f"Robot Controller 시작 (robot={self.get_parameter('robot_mode').value}, "
                               f"gripper={self.get_parameter('gripper_mode').value})")

    def _on_goal(self, goal):
        return GoalResponse.REJECT if self._busy.locked() else GoalResponse.ACCEPT

    def _go_scan(self):
        m = self.cfg['motion']
        return self.robot.move_joint(self.cfg['scan_joint_deg'], m['joint_vel'], m['joint_acc'])

    def _execute(self, goal_handle):
        with self._busy:
            g = goal_handle.request
            t0 = time.time()
            result = PickPlace.Result()
            try:
                result.result, result.message = self._pick_place(goal_handle, g)
            except Exception as e:
                result.result, result.message = PickPlace.Result.MOTION_ERROR, f'예외: {e!r}'
            if result.result != PickPlace.Result.SUCCESS:
                self._recover()
            result.elapsed_sec = time.time() - t0
            goal_handle.succeed()   # 액션 자체는 끝까지 수행됨 → 성공 여부는 result 필드로 전달
            return result

    def _pick_place(self, gh, g):
        R = PickPlace.Result
        m, lim = self.cfg['motion'], self.cfg['workspace_mm']
        orient = self.cfg['tool_down_orientation_deg']

        if g.place_box_id not in self.cfg['boxes']:
            return R.MOTION_ERROR, f'알 수 없는 박스: {g.place_box_id}'
        grasp_z = g.z_mm - m['grasp_depth_mm']
        if not within_workspace(g.x_mm, g.y_mm, grasp_z, lim):
            return R.MOTION_ERROR, f'작업 영역 밖 좌표: ({g.x_mm:.0f}, {g.y_mm:.0f}, {grasp_z:.0f})'

        above = top_down_pose(g.x_mm, g.y_mm, g.z_mm + m['approach_height_mm'], g.yaw_deg, orient)
        grasp = top_down_pose(g.x_mm, g.y_mm, grasp_z, g.yaw_deg, orient)
        v, a = m['line_vel'], m['line_acc']

        steps = [
            ('APPROACH', lambda: self.robot.move_line(above, v, a)),
            ('APPROACH', lambda: (self.gripper.open(), True)[1]),
            ('DESCEND', lambda: self.robot.move_line(grasp, m['slow_vel'], a)),
        ]
        for stage, fn in steps:
            self._feedback(gh, stage)
            if not fn():
                return R.MOTION_ERROR, f'{stage} 단계 이동 실패'

        self._feedback(gh, 'GRASP')
        width = self.gripper.close()
        self._feedback(gh, 'LIFT')
        if not self.robot.move_line(above, m['slow_vel'], a):
            return R.MOTION_ERROR, 'LIFT 단계 이동 실패'
        if width <= m['grasp_fail_width_mm']:
            return R.GRASP_FAIL, f'파지 실패 (벌림 폭 {width:.1f}mm)'

        self.picked_pub.publish(Point(x=g.x_mm, y=g.y_mm, z=g.z_mm))

        self._feedback(gh, 'PLACE')
        if not self.robot.move_line(self.cfg['boxes'][g.place_box_id], v, a):
            return R.MOTION_ERROR, 'PLACE 단계 이동 실패'
        self.gripper.open()

        self._feedback(gh, 'RETURN')
        if not self._go_scan():
            return R.MOTION_ERROR, '스캔 자세 복귀 실패'
        return R.SUCCESS, 'OK'

    def _recover(self):
        """실패 후 복구: 그리퍼 열기 → 스캔 자세 (관절 이동이라 현재 위치에서 위로 빠진 뒤 수행 권장)."""
        try:
            self.gripper.open()
            self._go_scan()
        except Exception as e:
            self.get_logger().error(f'복구 동작 실패: {e!r} — 티치펜던트로 확인하세요!')

    @staticmethod
    def _feedback(gh, stage):
        fb = PickPlace.Feedback()
        fb.stage = stage
        gh.publish_feedback(fb)


def main():
    rclpy.init()
    node = RobotController()
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
