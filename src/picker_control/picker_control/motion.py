"""로봇 / 그리퍼 하드웨어 추상화.

- SimRobot / SimGripper: 하드웨어 없이 전체 흐름 테스트 (시간만 흘려보냄)
- DsrRobot: doosan-robot2 서비스 호출
- DrlGripper: RH-P12-RN-A 제어 (⚠️ 실습실 제공 예제 코드로 채워야 함)

⚠️ doosan-robot2 서비스 이름·필드는 설치된 버전에서 반드시 확인:
   ros2 service list | grep motion
   ros2 interface show dsr_msgs2/srv/MoveLine
"""
import time
from typing import List, Sequence


class RobotBase:
    def move_joint(self, joints_deg: Sequence[float], vel: float, acc: float) -> bool:
        raise NotImplementedError

    def move_line(self, pose: Sequence[float], vel: float, acc: float) -> bool:
        """pose = [x, y, z, rx, ry, rz] (mm, deg, base 기준 ZYZ 오일러)."""
        raise NotImplementedError


class GripperBase:
    def open(self) -> None:
        raise NotImplementedError

    def close(self) -> float:
        """닫고 나서 손가락 벌림 폭(mm)을 반환. 큐브를 잡았으면 ≈ 큐브 크기, 놓쳤으면 ≈ 0."""
        raise NotImplementedError


# ---------------------------------------------------------------- 시뮬레이션
class SimRobot(RobotBase):
    def __init__(self, logger, speed_mm_s: float = 250.0):
        self.log = logger
        self.speed = speed_mm_s
        self.pos = None

    def move_joint(self, joints_deg, vel, acc):
        time.sleep(0.8)
        self.pos = None
        return True

    def move_line(self, pose, vel, acc):
        if self.pos is not None:
            dist = sum((a - b) ** 2 for a, b in zip(pose[:3], self.pos[:3])) ** 0.5
            time.sleep(min(dist / self.speed, 3.0))
        else:
            time.sleep(0.5)
        self.pos = list(pose)
        return True


class SimGripper(GripperBase):
    def __init__(self, cube_size_mm: float, fail_rate: float = 0.0, rng=None):
        import random
        self.cube = cube_size_mm
        self.fail_rate = fail_rate
        self.rng = rng or random.Random(0)

    def open(self):
        time.sleep(0.3)

    def close(self):
        time.sleep(0.3)
        return 0.0 if self.rng.random() < self.fail_rate else self.cube


# ---------------------------------------------------------------- 실제 로봇 (doosan-robot2)
class DsrRobot(RobotBase):
    """doosan-robot2의 motion 서비스를 동기 호출한다 (호출 스레드는 executor와 달라야 함)."""

    def __init__(self, node, namespace: str = 'dsr01', timeout: float = 60.0):
        from dsr_msgs2.srv import MoveJoint, MoveLine  # 실제 로봇 PC에서만 import
        self.MoveJoint, self.MoveLine = MoveJoint, MoveLine
        self.node = node
        self.timeout = timeout
        self.cli_j = node.create_client(MoveJoint, f'/{namespace}/motion/move_joint')
        self.cli_l = node.create_client(MoveLine, f'/{namespace}/motion/move_line')
        for cli in (self.cli_j, self.cli_l):
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'{cli.srv_name} 서비스가 없습니다. dsr_bringup2를 먼저 실행하세요.')

    def _call(self, cli, req) -> bool:
        fut = cli.call_async(req)
        deadline = time.time() + self.timeout
        while not fut.done():
            if time.time() > deadline:
                self.node.get_logger().error(f'{cli.srv_name} 응답 시간 초과')
                return False
            time.sleep(0.01)
        res = fut.result()
        return bool(getattr(res, 'success', True))

    def move_joint(self, joints_deg, vel, acc):
        req = self.MoveJoint.Request()
        req.pos = [float(v) for v in joints_deg]
        req.vel = float(vel)
        req.acc = float(acc)
        return self._call(self.cli_j, req)

    def move_line(self, pose, vel, acc):
        req = self.MoveLine.Request()
        req.pos = [float(v) for v in pose]
        req.vel = [float(vel), float(vel)]   # [선속도 mm/s, 각속도 deg/s]
        req.acc = [float(acc), float(acc)]
        return self._call(self.cli_l, req)


class DrlGripper(GripperBase):
    """RH-P12-RN-A 그리퍼.

    TODO(류창표): 실습실에서 제공하는 그리퍼 예제(DRL 스크립트 / Modbus / 전용 노드)를 확인해서
    open(), close(), 현재 위치 읽기를 채운다. 연결 방식에 따라 구현이 완전히 달라지므로
    Day 1에 '열기/닫기/위치 읽기' 세 가지가 되는지부터 확인할 것.
    close()는 반드시 '닫힌 뒤의 벌림 폭(mm)'을 반환해야 파지 실패 판정(F3-5)이 동작한다.
    """

    def __init__(self, node):
        self.node = node
        raise NotImplementedError('DrlGripper를 구현하거나, gripper_mode:=sim 으로 실행하세요.')

    def open(self):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError


def within_workspace(x: float, y: float, z: float, limits: dict) -> bool:
    return (limits['x'][0] <= x <= limits['x'][1]
            and limits['y'][0] <= y <= limits['y'][1]
            and limits['z'][0] <= z <= limits['z'][1])


def top_down_pose(x: float, y: float, z: float, yaw_deg: float, base_orient: List[float]) -> List[float]:
    """위에서 내려다보는 그리퍼 자세. base_orient = 그리퍼가 아래를 볼 때의 [rx, ry, rz].
    큐브 yaw는 마지막 회전(rz)에 더한다. ⚠️ 실제 로봇에서 45° 큐브로 방향(부호)을 꼭 확인할 것."""
    rx, ry, rz = base_orient
    return [x, y, z, rx, ry, rz + yaw_deg]
