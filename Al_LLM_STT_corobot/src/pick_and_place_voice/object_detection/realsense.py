from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class ImgNode(Node):
    def __init__(self):
        # ROS2 노드 초기화
        super().__init__('img_node')

        # OpenCV <-> ROS 이미지 변환용 브릿지 객체 생성
        self.bridge = CvBridge()

        # 각종 프레임 및 정보 초기화
        self.color_frame = None             # RGB 이미지 프레임
        self.color_frame_stamp = None       # 프레임 타임스탬프
        self.depth_frame = None             # Depth 이미지
        self.intrinsics = None              # 카메라 내참수 (fx, fy, cx, cy)

        # RGB 이미지 토픽 구독자 생성
        self.color_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',   # RGB 영상 토픽
            self.color_callback,                # 수신 콜백 함수
            10
        )

        # 정렬된 Depth 이미지 토픽 구독자 생성
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',  # RGB에 맞춰진 Depth 영상
            self.depth_callback,
            10
        )

        # 카메라 파라미터(CameraInfo) 토픽 구독자 생성
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.get_logger().info("Waiting for client's call...")

    def camera_info_callback(self, msg):
        """
        카메라 내부 파라미터를 저장합니다.
        msg.k는 3x3 카메라 행렬을 1차원 배열로 저장한 것입니다.
        """
        self.intrinsics = {
            "fx": msg.k[0],   # focal length x
            "fy": msg.k[4],   # focal length y
            "ppx": msg.k[2],  # principal point x
            "ppy": msg.k[5],  # principal point y
        }

    def color_callback(self, msg):
        """
        수신된 RGB 이미지 메시지를 OpenCV 이미지로 변환하여 저장합니다.
        """
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 타임스탬프는 YOLO 처리 시 중복 방지용으로 사용 가능
        self.color_frame_stamp = str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)

    def depth_callback(self, msg):
        """
        수신된 Depth 이미지를 저장합니다.
        'passthrough'로 원본 Depth 포맷 유지
        """
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # 외부에서 RGB 프레임 요청 시 사용
    def get_color_frame(self):
        return self.color_frame

    # 프레임의 ROS 타임스탬프 문자열 반환
    def get_color_frame_stamp(self):
        return self.color_frame_stamp

    # Depth 프레임 반환
    def get_depth_frame(self):
        return self.depth_frame

    # 카메라 내참수 반환
    def get_camera_intrinsic(self):
        return self.intrinsics

