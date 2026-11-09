"""캘리브레이션·검출 수학 테스트 (ROS 없이: python -m pytest src/picker_vision/test)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from picker_vision.calibrate import kabsch, residuals  # noqa: E402


def rot_z(deg):
    r = math.radians(deg)
    return np.array([[math.cos(r), -math.sin(r), 0], [math.sin(r), math.cos(r), 0], [0, 0, 1]])


def test_kabsch_recovers_transform():
    rng = np.random.default_rng(0)
    R = rot_z(30) @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # 아래를 보는 카메라
    t = np.array([400.0, -50.0, 600.0])
    P_cam = rng.uniform(-200, 200, size=(10, 3)) + [0, 0, 500]
    P_base = (R @ P_cam.T).T + t + rng.normal(0, 0.5, size=(10, 3))
    T = kabsch(P_cam, P_base)
    assert np.allclose(T[:3, :3], R, atol=1e-2)
    assert np.allclose(T[:3, 3], t, atol=2.0)
    assert residuals(T, P_cam, P_base).max() < 3.0


def test_detection_pure_helpers():
    cv2 = __import__('pytest').importorskip('cv2')  # noqa: F841
    from picker_vision.detection import Detection, Intrinsics, deproject, fill_min_gap, normalize_yaw
    assert normalize_yaw(60) == -30
    assert normalize_yaw(-50) == 40
    k = Intrinsics(600, 600, 320, 240)
    assert np.allclose(deproject(320, 240, 500, k), [0, 0, 500])
    dets = [Detection('red', 0, 0, 0, 0, 1), Detection('blue', 60, 0, 0, 0, 1)]
    fill_min_gap(dets, 50)
    assert math.isclose(dets[0].min_gap, 10)
