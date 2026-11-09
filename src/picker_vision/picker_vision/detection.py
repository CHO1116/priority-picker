"""색상 큐브 검출 (ROS 의존성 없음). OpenCV + NumPy만 사용.

입력: BGR 컬러 영상, 컬러에 정렬된 깊이 영상(mm, uint16), 카메라 내부 파라미터, T_base_camera
출력: 로봇 base 좌표계(mm) 기준 큐브 목록
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class Detection:
    color: str
    x: float          # base 좌표 (mm)
    y: float
    z: float
    yaw: float        # base 좌표계 기준 (deg, -45 ~ 45)
    confidence: float
    min_gap: float = 999.0
    cam_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # 캘리브레이션 데이터 수집용
    pixel: Tuple[int, int] = (0, 0)
    box_points: Optional[np.ndarray] = None                  # 디버그 그리기용


def deproject(u: float, v: float, z_mm: float, k: Intrinsics) -> np.ndarray:
    """픽셀 + 깊이 → 카메라 좌표 (mm). 핀홀 모델 (왜곡 무시)."""
    return np.array([(u - k.cx) * z_mm / k.fx, (v - k.cy) * z_mm / k.fy, z_mm])


def to_base(p_cam: np.ndarray, T_base_cam: np.ndarray) -> np.ndarray:
    return (T_base_cam @ np.append(p_cam, 1.0))[:3]


def normalize_yaw(deg: float) -> float:
    """정육면체는 90도 대칭이므로 -45 ~ 45 범위로."""
    return (deg + 45.0) % 90.0 - 45.0


def color_mask(hsv: np.ndarray, ranges: List[List[List[int]]]) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:   # 빨강처럼 H가 0/180 양쪽에 걸치면 범위 2개
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def median_depth(depth: np.ndarray, u: int, v: int, r: int) -> float:
    """중심 주변 정사각형에서 0(측정 실패)을 뺀 깊이 중앙값."""
    h, w = depth.shape
    patch = depth[max(v - r, 0):min(v + r + 1, h), max(u - r, 0):min(u + r + 1, w)]
    valid = patch[patch > 0]
    return float(np.median(valid)) if valid.size else 0.0


def detect_cubes(bgr: np.ndarray, depth_mm: np.ndarray, k: Intrinsics,
                 T_base_cam: np.ndarray, cfg: Dict) -> List[Detection]:
    x0, y0, x1, y1 = cfg['roi']
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    roi_mask = np.zeros(hsv.shape[:2], np.uint8)
    roi_mask[y0:y1, x0:x1] = 255

    min_area, max_area = cfg['min_area_px'], cfg['max_area_px']
    exp_z, z_tol = cfg['expected_top_z_mm'], cfg['top_z_tolerance_mm']
    out: List[Detection] = []

    for color, ranges in cfg['colors'].items():
        mask = color_mask(hsv, ranges) & roi_mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue  # 노이즈
            (cu, cv_), (w, h), angle = cv2.minAreaRect(cnt)
            u, v = int(round(cu)), int(round(cv_))
            d = median_depth(depth_mm, u, v, max(2, int(min(w, h) * 0.25)))
            if d <= 0:
                continue

            p_cam = deproject(cu, cv_, d, k)
            p_base = to_base(p_cam, T_base_cam)

            # 이미지상의 변 방향을 base 좌표로 바꿔 yaw 계산
            rad = math.radians(angle)
            q_cam = deproject(cu + 20 * math.cos(rad), cv_ + 20 * math.sin(rad), d, k)
            q_base = to_base(q_cam, T_base_cam)
            yaw = normalize_yaw(math.degrees(math.atan2(q_base[1] - p_base[1], q_base[0] - p_base[0])))

            confidence = 1.0
            label = color
            if area > max_area:
                confidence = 0.4          # 같은 색 큐브 여러 개가 붙어 한 덩어리로 보임
            if abs(p_base[2] - exp_z) > z_tol:
                label, confidence = 'unknown', 0.3   # 높이가 큐브와 다름 → 큐브가 아니거나 쌓여 있음

            out.append(Detection(label, *p_base.tolist(), yaw=yaw, confidence=confidence,
                                 cam_xyz=tuple(p_cam.tolist()), pixel=(u, v),
                                 box_points=cv2.boxPoints(((cu, cv_), (w, h), angle)).astype(int)))

    fill_min_gap(out, cfg['cube_size_mm'])
    return out


def fill_min_gap(dets: List[Detection], cube_size_mm: float) -> None:
    """중심 간 거리 - 큐브 한 변 ≈ 두 큐브 사이 틈 (회전 무시한 근사값)."""
    for i, a in enumerate(dets):
        gaps = [math.hypot(a.x - b.x, a.y - b.y) - cube_size_mm
                for j, b in enumerate(dets) if i != j]
        a.min_gap = max(0.0, min(gaps)) if gaps else 999.0


BGR = {'red': (0, 0, 255), 'blue': (255, 0, 0), 'green': (0, 200, 0),
       'yellow': (0, 220, 220), 'unknown': (128, 128, 128)}


def draw_debug(bgr: np.ndarray, dets: List[Detection], roi) -> np.ndarray:
    img = bgr.copy()
    cv2.rectangle(img, tuple(roi[:2]), tuple(roi[2:]), (255, 255, 255), 1)
    for d in dets:
        c = BGR.get(d.color, (255, 255, 255))
        if d.box_points is not None:
            cv2.drawContours(img, [d.box_points], 0, c, 2)
        cv2.putText(img, f'{d.color} ({d.x:.0f},{d.y:.0f},{d.z:.0f}) {d.yaw:.0f}deg',
                    (d.pixel[0] - 60, d.pixel[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, c, 1)
        cv2.putText(img, f'gap {d.min_gap:.0f} conf {d.confidence:.1f}',
                    (d.pixel[0] - 60, d.pixel[1] + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.4, c, 1)
    return img
