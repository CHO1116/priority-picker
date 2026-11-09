"""간이 캘리브레이션: (카메라 좌표, 로봇 좌표) 점 쌍 → T_base_camera (4x4).

데이터 수집 방법
  1. 로봇을 스캔 자세로 이동 (매번 같은 관절값)
  2. vision_node를 log_camera_coords:=true 로 실행 → 로그의 [CAL] cam=(...) 값 기록
  3. 로봇 TCP를 그 큐브 윗면 중심에 직접 교시로 맞추고, 티치펜던트의 x, y, z 기록
  4. 위치를 바꿔 8곳 이상 반복 → CSV 작성 (헤더: cam_x,cam_y,cam_z,base_x,base_y,base_z, 단위 mm)

실행
  ros2 run picker_vision calibrate --input points.csv --output calibration.yaml
"""
import argparse
import csv

import numpy as np
import yaml


def kabsch(P_cam: np.ndarray, P_base: np.ndarray) -> np.ndarray:
    """P_base ≈ R @ P_cam + t 를 만족하는 강체 변환 (최소제곱, SVD)."""
    assert P_cam.shape == P_base.shape and P_cam.shape[0] >= 3
    c_cam, c_base = P_cam.mean(axis=0), P_base.mean(axis=0)
    H = (P_cam - c_cam).T @ (P_base - c_base)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))   # 반사(reflection) 방지
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = c_base - R @ c_cam
    return T


def residuals(T: np.ndarray, P_cam: np.ndarray, P_base: np.ndarray) -> np.ndarray:
    pred = (T @ np.c_[P_cam, np.ones(len(P_cam))].T).T[:, :3]
    return np.linalg.norm(pred - P_base, axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', required=True)
    ap.add_argument('--output', required=True)
    args, _ = ap.parse_known_args()

    with open(args.input, encoding='utf-8-sig') as f:
        rows = list(csv.DictReader(f))
    P_cam = np.array([[float(r['cam_x']), float(r['cam_y']), float(r['cam_z'])] for r in rows])
    P_base = np.array([[float(r['base_x']), float(r['base_y']), float(r['base_z'])] for r in rows])

    T = kabsch(P_cam, P_base)
    err = residuals(T, P_cam, P_base)
    for i, e in enumerate(err):
        print(f'  점 {i + 1:2d}: 오차 {e:6.2f} mm' + ('  ← 큼, 재측정 권장' if e > 10 else ''))
    print(f'평균 오차 {err.mean():.2f} mm / 최대 {err.max():.2f} mm ({len(err)}점)')

    with open(args.output, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'T_base_camera': T.round(6).tolist(),
                        'mean_error_mm': float(err.mean()),
                        'num_points': len(err)}, f, allow_unicode=True)
    print(f'저장: {args.output}')


if __name__ == '__main__':
    main()
