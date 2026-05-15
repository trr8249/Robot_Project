import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class ArucoPoseFollower(Node):
    def __init__(self): # sub/pub 설정, ArUco마커 세팅, AMR PD제어 초기화
        super().__init__('aruco_pose_follower')

        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/robot9/oakd/rgb/image_raw/compressed',
            self.image_callback,
            10
        )

        self.car_id_sub = self.create_subscription(Int32, '/robot9/recognized_car', self.car_id_callback, 10)
        self.point1_sub = self.create_subscription(Bool, '/robot9/point1', self.point1_callback, 10)
        self.point2_sub = self.create_subscription(Bool, '/robot9/point2', self.point2_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/robot9/cmd_vel', 10)
        self.done1_pub = self.create_publisher(Bool, '/robot9/done1', 10)
        self.done2_pub = self.create_publisher(Bool, '/robot9/done2', 10)

        # ArUco settings
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.camera_matrix = np.array([[620.0, 0.0, 320.0], [0.0, 620.0, 240.0], [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros((5, 1))
        self.marker_length = 0.05  # meters
        self.target_distance = 0.30  # meters

        # PD control
        self.Kp = 0.002
        self.Kd = 0.0008
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()

        # State
        self.mode = None  # 'point1', 'point2'
        self.car_id = 0
        self.target_marker_id = None

        self.get_logger().info("✅ ArUco Pose Follower Node Initialized")

    def point1_callback(self, msg): # point1 명령 수신 시, 마커 AMR1 부각 마커 추적 시작
        if msg.data:
            self.mode = 'point1'
            self.target_marker_id = 8
            self.get_logger().info("🚩 point1 명령 수신: 마커 ID 8 추적 시작")

    def point2_callback(self, msg): # point2 명령 수신 시, 하역장 이동 시작 
        if msg.data:
            self.mode = 'point2'
            self.target_marker_id = self.car_id
            self.get_logger().info(f"🚩 point2 명령 수신: 마커 ID {self.car_id} 추적 시작")

    def car_id_callback(self, msg): # 인식된 차량 ID 저장
        self.car_id = msg.data
        self.get_logger().info(f"🎯 인식된 차량 ID: {self.car_id}")

    def image_callback(self, msg): # 이미지 수신 후 하역장 ArUco 마커 추적 및 이동
        if self.mode is None or self.target_marker_id is None:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        twist = Twist()

        if ids is not None and self.target_marker_id in ids:
            index = np.where(ids == self.target_marker_id)[0][0]
            marker_corners = [corners[index]]

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners,
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs
            )

            tvec = tvecs[0][0]
            distance = tvec[2]
            c = corners[index][0]
            center_x = int(np.mean(c[:, 0]))
            error = center_x - (frame.shape[1] // 2)

            current_time = self.get_clock().now()
            dt = (current_time - self.previous_time).nanoseconds / 1e9
            derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
            angular_z = -(self.Kp * error + self.Kd * derivative)

            twist.angular.z = angular_z
            self.previous_error = error
            self.previous_time = current_time

            if distance > self.target_distance:
                twist.linear.x = min(0.15, 0.5 * (distance - self.target_distance))
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)

                done_msg = Bool()
                done_msg.data = True
                if self.mode == 'point1':
                    self.done1_pub.publish(done_msg)
                    self.get_logger().info("✅ point1 목표 도달. /robot9/done1 = True 발행")
                elif self.mode == 'point2':
                    self.done2_pub.publish(done_msg)
                    self.get_logger().info("✅ point2 목표 도달. /robot9/done2 = True 발행")

                # Reset
                self.mode = None
                self.target_marker_id = None
                return

            self.get_logger().info(f'[ID {self.target_marker_id}] 거리: {distance:.2f} m | 오차: {error} px | 회전속도: {angular_z:.3f}')
            cv2.aruco.drawDetectedMarkers(frame, marker_corners)

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.005
            self.get_logger().info(f'🔍 마커 ID {self.target_marker_id} 탐색 중...')

        self.cmd_pub.publish(twist)

        cv2.imshow('Aruco Pose Tracking', frame)
        cv2.waitKey(1)

    def destroy_node(self): # 노드 종료 시 OpenCV 윈도우 닫기
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = ArucoPoseFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


