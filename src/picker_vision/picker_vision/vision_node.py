"""Vision Node: RealSense 컬러 + 정렬된 깊이 → /detected_cubes.

실행 전 RealSense를 깊이 정렬 옵션으로 켜야 한다:
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
토픽 이름은 드라이버 버전에 따라 다르므로 vision.yaml의 topics에서 맞춘다
(`ros2 topic list | grep image` 로 확인).
"""
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from picker_interfaces.msg import DetectedCube, DetectedCubeArray

from .detection import Intrinsics, detect_cubes, draw_debug


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('vision_config', '')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('log_camera_coords', False)   # 캘리브레이션 데이터 수집 시 True

        with open(self.get_parameter('vision_config').value, encoding='utf-8') as f:
            self.cfg = yaml.safe_load(f)
        with open(self.get_parameter('calibration_file').value, encoding='utf-8') as f:
            self.T = np.array(yaml.safe_load(f)['T_base_camera'], dtype=float)

        self.bridge = CvBridge()
        self.k = None
        self.frame = None   # (color_msg, depth_msg)

        t = self.cfg['topics']
        self.create_subscription(CameraInfo, t['camera_info'], self._on_info, 10)
        sync = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, t['color']), Subscriber(self, Image, t['depth'])],
            queue_size=10, slop=0.05)
        sync.registerCallback(self._on_frame)

        self.pub = self.create_publisher(DetectedCubeArray, '/detected_cubes', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/debug_image', 1)
        self.create_timer(1.0 / self.cfg['rate_hz'], self._process)
        self.get_logger().info('Vision Node 시작')

    def _on_info(self, msg):
        if self.k is None:
            self.k = Intrinsics(fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5])
            self.get_logger().info(f'카메라 내부 파라미터 수신: {self.k}')

    def _on_frame(self, color_msg, depth_msg):
        self.frame = (color_msg, depth_msg)

    def _process(self):
        if self.frame is None or self.k is None:
            return
        color_msg, depth_msg = self.frame
        self.frame = None
        bgr = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')  # 16UC1, mm

        dets = detect_cubes(bgr, depth, self.k, self.T, self.cfg)

        out = DetectedCubeArray()
        out.header = color_msg.header   # stamp = 촬영 시각 (planner가 '최신 영상'인지 판단)
        for d in dets:
            out.cubes.append(DetectedCube(
                color=d.color, x_mm=d.x, y_mm=d.y, z_mm=d.z, yaw_deg=d.yaw,
                confidence=float(d.confidence), min_gap_mm=float(d.min_gap)))
            if self.get_parameter('log_camera_coords').value:
                cx, cy, cz = d.cam_xyz
                self.get_logger().info(f'[CAL] {d.color} cam=({cx:.1f}, {cy:.1f}, {cz:.1f})')
        self.pub.publish(out)

        if self.debug_pub.get_subscription_count() > 0:
            dbg = draw_debug(bgr, dets, self.cfg['roi'])
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, 'bgr8'))


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
