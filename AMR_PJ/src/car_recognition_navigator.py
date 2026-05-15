import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class CarRecognitionNavigator(Node):
    def __init__(self): # 위치 설정, 상태변수 초기화, pub/sub 등록
        super().__init__('car_recognition_navigator')
        self.get_logger().info('CarRecognitionNavigator 노드 초기화 시작')

        self.navigator = TurtleBot4Navigator()
        self.goal_pose = []

        # 위치 설정
        self.point1 = self.navigator.getPoseStamped([0.77, -2.8], TurtleBot4Directions.WEST)
        self.point2 = self.navigator.getPoseStamped([0.836, -5.32], TurtleBot4Directions.NORTH)
        self.point3 = self.navigator.getPoseStamped([1.95, -2.74], TurtleBot4Directions.NORTH)

        # 상태 변수 초기화
        self.latest_robot8_value = None
        self.latest_robot9_value = None
        self.is_nothing = False
        self.done1 = False
        self.done2 = False

        # 구독자 등록
        self.create_subscription(Int32, '/robot8/recognized_car', self.robot8_callback, 10)
        self.create_subscription(Int32, '/robot9/recognized_car', self.robot9_callback, 10)
        self.create_subscription(Bool, '/is_nothing', self.is_nothing_callback, 10)
        self.create_subscription(Bool, '/robot9/done1', self.done1_callback, 10)
        self.create_subscription(Bool, '/robot9/done2', self.done2_callback, 10)

        # 발행자 등록
        self.pub_point1 = self.create_publisher(Bool, '/robot9/point1', 10)
        self.pub_point2 = self.create_publisher(Bool, '/robot9/point2', 10)

        self.initialize_navigation()

    def initialize_navigation(self): # 초기 pose 설정 값, Nav2 활성화
        self.get_logger().info('초기 위치 및 Nav2 설정 시작')
        if not self.navigator.getDockedStatus():
            self.get_logger().info('현재 도킹 안됨 → 도킹 수행 중...')
            self.navigator.dock()
        else:
            self.get_logger().info('이미 도킹된 상태')

        initial_pose = self.navigator.getPoseStamped([2.91, -2.59, 0.34], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 활성화 완료 및 초기 위치 설정됨')

    def is_nothing_callback(self, msg):# msg 없을 경우 상태표시
        self.is_nothing = msg.data
        #self.get_logger().info(f'/is_nothing 수신: {self.is_nothing}')

    def done1_callback(self, msg): # msg 있을 경우 상태표시
        self.done1 = msg.data
        self.get_logger().info('✔️ done1 수신 완료')

    def done2_callback(self, msg): # gotomarker가 발행한 done2 수신 상태표시
        self.done2 = msg.data
        self.get_logger().info('✔️ done2 수신 완료')

    def robot8_callback(self, msg): # AMR1 차량 인계 여부와 종류 확인 후 AMR2 인수인계 지점 이동  
        self.latest_robot8_value = msg.data
        self.get_logger().info(f'/robot8/recognized_car 수신: {msg.data}')

        if msg.data in [1, 2, 3]:
            if self.navigator.getDockedStatus():
                self.get_logger().info('도킹된 상태 → 언도킹 수행')
                self.navigator.undock()

            self.get_logger().info('point1으로 이동 시작')
            self.navigator.startFollowWaypoints([self.point1])
            self.pub_point1.publish(Bool(data=True))
            self.get_logger().info('/robot9/point1 토픽 발행 완료 --2초 기다림')
            time.sleep(2)

    def robot9_callback(self, msg): # gotomarker 실행 여부에 따라 AMR2 이동 제어
        self.latest_robot9_value = msg.data
        self.get_logger().info(f'/robot9/recognized_car 수신: {msg.data}')

        # done1 = True and recognized_car in [1,2,3]
        if self.done1 and msg.data in [1, 2, 3]:
            self.get_logger().info('조건: done1=True && recognized_car=[1,2,3] → point2로 이동 --3초 기다림')
            time.sleep(3)
            self.navigator.startFollowWaypoints([self.point2])
            self.pub_point2.publish(Bool(data=True))
            self.get_logger().info('/robot9/point2 토픽 발행 완료')
            self.done1 = False

        # done2 = True and recognized_car == 0
        elif self.done2 and msg.data == 0:
            self.get_logger().info('조건 확인 중: done2=True && recognized_car=0')
            if self.is_nothing and self.latest_robot8_value == 0:
                self.get_logger().info('조건 만족: is_nothing=True && robot8=0 → point3으로 이동 후 도킹')
                self.navigator.startFollowWaypoints([self.point3])
                self.get_logger().info('2초 기다림')
                time.sleep(2)
                self.navigator.dock()
                self.get_logger().info('도킹 완료')
                self.done2 = False
            else:
                self.get_logger().info('조건 불만족 → point1로 재이동')
                self.navigator.startFollowWaypoints([self.point1])

def main():
    rclpy.init()
    try:
        node = CarRecognitionNavigator()
        rclpy.spin(node)
    except Exception as e:
        print(f'[main] 예외 발생: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    