# 필수 라이브러리 및 패키지 임포트
import numpy as np
import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node
from typing import Any, Callable, Optional, Tuple
# ROS2에서 패키지 경로를 가져오기 위한 유틸리티
from ament_index_python.packages import get_package_share_directory
# 사용자 정의 서비스 메시지 (요청: 객체 이름, 응답: 3D 좌표)
from od_msg.srv import SrvDepthPosition
# 이미지 및 모델 처리를 위한 사용자 정의 모듈
from object_detection.realsense import ImgNode  # Realsense 카메라 프레임 제공
from object_detection.yolo import YoloModel     # YOLO 모델 객체
# 패키지 이름과 경로 (사용하지는 않지만 참고용으로 불러옴)
PACKAGE_NAME = 'pick_and_place_text'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


# 객체 인식 노드를 정의
class ObjectDetectionNode(Node):
    def __init__(self, model_name='yolo'):
        # 노드 이름을 'object_detection_node'로 초기화
        super().__init__('object_detection_node')
        # 이미지 처리용 카메라 노드 초기화 (depth/color 프레임 제공)
        self.img_node = ImgNode()
        # 지정된 이름의 모델을 로드 (기본은 YOLO)
        self.model = self._load_model(model_name)
        # 카메라 내부 파라미터(intrinsic)를 확보 (fx, fy, ppx, ppy)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )

        # 서비스 서버 생성: 클라이언트로부터 "타겟 객체"를 받고 3D 위치를 반환
        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth
        )
        self.get_logger().info("ObjectDetectionNode initialized.")

    # 모델 이름에 따라 객체 인식 모델을 로드
    def _load_model(self, name):
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")
    
    # 서비스 요청을 받아 객체의 3D 좌표를 계산해 응답
    def handle_get_depth(self, request, response):
        self.get_logger().info(f"Received request: {request}")
        coords = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]  # float 배열로 응답
        return response

    # 객체의 중심 픽셀을 찾아 깊이와 함께 3D 좌표 계산
    def _compute_position(self, target):
        # 최신 카메라 프레임을 수신
        rclpy.spin_once(self.img_node)
        # 객체 인식 실행 → 가장 신뢰도 높은 검출 결과(box, score) 획득
        box, score = self.model.get_best_detection(self.img_node, target)
        # 객체가 검출되지 않으면 (None 반환)
        if box is None or score is None:
            self.get_logger().warn("No detection found.")
            return 0.0, 0.0, 0.0  # 실패 시 기본값 반환
        self.get_logger().info(f"Detection: box={box}, score={score}")
        # 바운딩 박스 중심 좌표 계산 (cx, cy)
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        # 해당 중심 픽셀의 깊이(z값) 가져오기
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return 0.0, 0.0, 0.0
        # 픽셀 좌표 + 깊이로 카메라 기준 3D 좌표 변환
        return self._pixel_to_camera_coords(cx, cy, cz)

    def _get_depth(self, x, y):
        """특정 픽셀 위치의 깊이 값을 안전하게 가져오기"""
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return frame[y, x]  # numpy 배열이므로 [y, x] 순서
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        """
        데이터가 유효할 때까지 ROS 이벤트 루프를 돌며 기다림.
        예: 카메라 intrinsic이나 depth frame 등이 None 또는 빈 배열일 경우.
        """
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        """
        픽셀 좌표 (x, y)와 깊이 z를 이용해
        카메라 좌표계 상의 (X, Y, Z)로 변환
        공식: X = (x - ppx) * z / fx
              Y = (y - ppy) * z / fy
              Z = z
        """
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )


# 메인 실행 함수
def main(args=None):
    rclpy.init(args=args)             # ROS 노드 초기화
    node = ObjectDetectionNode()      # 노드 인스턴스 생성
    try:
        rclpy.spin(node)              # 노드가 종료될 때까지 이벤트 루프
    finally:
        node.destroy_node()           # 노드 종료
        rclpy.shutdown()              # ROS 시스템 종료

# Python 단독 실행 시 main 함수 실행
if __name__ == '__main__':
    main()
