import sys
import cv2 # OpenCV, 영상 재생 및 프레임 처리
import rclpy # ROS2 노드 기능
from rclpy.node import Node
# GUI 구축 (영상 표시, 슬라이더 포함)
from PyQt5.QtWidgets import ( 
    QApplication, QLabel, QWidget, QHBoxLayout, QVBoxLayout,
    QSlider, QDesktopWidget, QMainWindow, QFrame
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt


# ------------------ 영상만 보여주는 창 ------------------ #
class VideoWindow(QWidget):
    def __init__(self, left_path, right_path):
        super().__init__()
        self.setWindowTitle("rviz - 토토로  /  실제 - 토토로")  # 창 제목 설정

        # ▶ 왼쪽과 오른쪽 영상 파일 열기
        self.left_cap = cv2.VideoCapture(left_path)   # 왼쪽 영상 스트림
        self.right_cap = cv2.VideoCapture(right_path) # 오른쪽 영상 스트림

        # ▶ 왼쪽 영상의 원본 해상도 정보 가져오기
        self.original_width = int(self.left_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.original_height = int(self.left_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # ▶ 영상 크기 비율 초기값 설정 (0.5 = 50% 크기로 시작)
        self.scale_ratio = 0.5

        # ▶ QLabel 생성: 각 영상 프레임을 표시할 위젯
        self.left_label = QLabel()
        self.right_label = QLabel()

        # ▶ QLabel에 영상을 꽉 차게 표시할 수 있도록 설정
        self.left_label.setScaledContents(True)
        self.right_label.setScaledContents(True)

        # ▶ 수평 레이아웃에 QLabel 두 개 배치 (좌우로 영상 나란히)
        layout = QHBoxLayout()
        layout.addWidget(self.left_label)
        layout.addWidget(self.right_label)
        self.setLayout(layout)

        # ▶ 타이머 생성: 일정 간격마다 프레임을 업데이트
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frames)  # 프레임 갱신 함수 연결
        self.timer.start(30)  # 30ms 간격으로 프레임 갱신 (~33fps)

        # ▶ 초기 창 크기 설정 (화면 크기 기반, 비율 유지)
        self.init_size()

    def init_size(self):
        # ▶ 현재 화면 해상도 가져오기
        screen = QDesktopWidget().screenGeometry()
        screen_width = screen.width()
        screen_height = screen.height()

        # ▶ 영상의 가로세로 비율 계산
        aspect_ratio = self.original_width / self.original_height

        # ▶ 창의 최대 너비/높이 = 화면의 50% 수준
        max_width = int(screen_width * 0.5)
        max_height = int(screen_height * 0.5)

        # ▶ 가로 기준으로 창 크기 계산 (비율 유지)
        width = max_width
        height = int(width / aspect_ratio)

        # ▶ 계산된 높이가 최대 높이를 넘는다면 높이 기준으로 다시 계산
        if height > max_height:
            height = max_height
            width = int(height * aspect_ratio)

        # ▶ 최종 크기로 창 크기 조정
        self.resize(width, height)

    def set_scale(self, ratio):
        # ▶ 외부 슬라이더로부터 영상 스케일 값을 전달받아 저장
        self.scale_ratio = ratio

    def resize_window_by_ratio(self, ratio):
        # ▶ 외부 슬라이더로부터 창 크기 배율을 전달받아 창 전체 크기 조정
        screen = QDesktopWidget().screenGeometry()
        w = int(screen.width() * ratio)
        h = int(screen.height() * ratio)
        self.resize(w, h)

    def update_frames(self):
        # ▶ 왼쪽/오른쪽 영상에서 각각 프레임 읽기
        ret_left, frame_left = self.left_cap.read()
        ret_right, frame_right = self.right_cap.read()

        # ▶ 프레임을 정상적으로 읽은 경우, 스케일 조정 후 표시
        if ret_left:
            resized = self.scale_frame(frame_left)
            self.left_label.setPixmap(self.convert_frame(resized))

        if ret_right:
            resized = self.scale_frame(frame_right)
            self.right_label.setPixmap(self.convert_frame(resized))

    def scale_frame(self, frame):
        # ▶ 현재 scale_ratio에 따라 프레임을 리사이즈 (비율 유지)
        w = int(self.original_width * self.scale_ratio)
        h = int(self.original_height * self.scale_ratio)
        return cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)

    def convert_frame(self, frame):
        # ▶ OpenCV BGR → RGB 변환 → QImage → QPixmap 변환
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        return QPixmap.fromImage(QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888))



# ------------------ 슬라이더 전용 GUI 창 (영상 없음) ------------------ #
class ControlWindow(QWidget):
    def __init__(self, video_window):
        super().__init__()
        self.setWindowTitle("영상 제어 패널")
        self.video_window = video_window  # 조절할 VideoWindow 인스턴스 연결

        # ▶ 해상도 조절 슬라이더 (10% ~ 200%)
        self.scale_slider = QSlider(Qt.Horizontal)
        self.scale_slider.setMinimum(10)
        self.scale_slider.setMaximum(100)
        self.scale_slider.setValue(50)  # 초기값 100%
        self.scale_slider.valueChanged.connect(self.update_scale)

        # ▶ 창 크기 조절 슬라이더 (50% ~ 150%)
        self.window_slider = QSlider(Qt.Horizontal)
        self.window_slider.setMinimum(10)
        self.window_slider.setMaximum(100)
        self.window_slider.setValue(50)  # 초기값 100%
        self.window_slider.valueChanged.connect(self.update_window_size)

        # 슬라이더 레이아웃 구성
        layout = QVBoxLayout()
        layout.addWidget(QLabel("해상도 조절 (배율%)"))
        layout.addWidget(self.scale_slider)
        layout.addSpacing(20)
        layout.addWidget(QLabel("창 크기 조절 (화면 비율%)"))
        layout.addWidget(self.window_slider)

        self.setLayout(layout)

        # 슬라이더 패널의 창 크기는 고정
        self.setFixedSize(400, 200)


    def update_scale(self, value):
        # 해상도 배율 변경 시 VideoWindow에 전달
        self.video_window.set_scale(value / 100.0)


    def update_window_size(self, value):
        # GUI 창 크기 비율 변경 시 VideoWindow에 전달
        self.video_window.resize_window_by_ratio(value / 100.0)



# ------------------ 메인: 영상창 + 슬라이더창 동시에 실행 ------------------ #
def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    # 1. 영상 표시용 GUI 창 생성 및 실행
    video_window = VideoWindow(
        "/home/shindonghyun/doosan_ws/image/rviz_draw.mp4",
        "/home/shindonghyun/doosan_ws/image/real_draw.mp4"
    )
    video_window.show()

    # 2. 슬라이더만 있는 제어 GUI 창 생성 및 실행 (video_window를 조절)
    control_window = ControlWindow(video_window)
    control_window.show()

    # Qt 이벤트 루프 실행
    sys.exit(app.exec_())



if __name__ == "__main__":
    main()


