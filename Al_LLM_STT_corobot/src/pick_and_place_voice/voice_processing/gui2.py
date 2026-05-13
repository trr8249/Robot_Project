import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
import threading
import tkinter as tk
from PIL import Image, ImageTk
import cv2
from ultralytics import YOLO

class TriggerNode(Node):
    def __init__(self):
        super().__init__('trigger_node')
        self.status_message = "대기 중..."
        self.diesel_message = "대기 중..."
        self.gasoline_message = "대기 중..."
        self.pump_message = "대기 중..."
        self.card_message = "대기 중..."
        self.obstacle_message = "대기 중..."
        self.payment_total = 0

        self.create_subscription(String, '/robot/status', self.status_callback, 10)
        self.create_subscription(Int32, '/oil/Diesel', self.diesel_callback, 10)
        self.create_subscription(Int32, '/oil/Gasoline', self.gasoline_callback, 10)
        self.create_subscription(Bool, '/target/Pump', self.pump_callback, 10)
        self.create_subscription(Bool, '/target/Card', self.card_callback, 10)
        self.create_subscription(Bool, '/target/Obstacle', self.obstacle_callback, 10)
        self.create_subscription(Int32, '/payment/total', self.payment_callback, 10)

    def status_callback(self, msg):
        self.get_logger().info(f"[로봇 상태] {msg.data}")
        self.status_message = msg.data

    def diesel_callback(self, msg):
        if msg.data == 0:
            self.diesel_message = "🚚 경유 감지됨: 경유 주유 시작 중..."

    def gasoline_callback(self, msg):
        if msg.data == 1:
            self.gasoline_message = "⛽ 휘발유 감지됨: 휘발유 주유 시작 중..."

    def pump_callback(self, msg):
        if msg.data:
            self.pump_message = "🛠️ 주유기 감지됨: 주유기 접근 중..."

    def card_callback(self, msg):
        if msg.data:
            self.card_message = "💳 카드 감지됨: 결제 시도 중..."

    def obstacle_callback(self, msg):
        if msg.data:
            self.obstacle_message = "⚠️ 장애물 감지됨: 경로 재계획 중..."

    def payment_callback(self, msg):
        self.payment_total = msg.data

def start_gui(trigger_node: TriggerNode):
    root = tk.Tk()
    root.title("⛽ ROS2 주유소 GUI")
    root.geometry("800x600")  # 기존 GUI 크기

    main_frame = tk.Frame(root, width=800, height=600)
    main_frame.pack(fill="both", expand=True)

    # 배경 이미지 설정
    bg_image = Image.open("/home/shindonghyun/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/gas_station.png").resize((800, 600))
    bg_photo = ImageTk.PhotoImage(bg_image)

    canvas = tk.Canvas(main_frame, width=800, height=600, highlightthickness=0)
    canvas.place(x=0, y=0)
    canvas.create_image(0, 0, image=bg_photo, anchor="nw")

    buttons = {}

    # 왼쪽 영역 메시지
    left_sections = [
        ("로봇 상태", trigger_node.status_message, 20),
        ("경유 상태", trigger_node.diesel_message, 80),
        ("휘발유 상태", trigger_node.gasoline_message, 140),
        ("총 결제액", f"{trigger_node.payment_total} 원", 200)
    ]
    for title, initial_text, y in left_sections:
        btn = tk.Button(main_frame, text=f"{title}: {initial_text}",
                        font=("Arial", 12, "bold"), bg="#444444", fg="white",
                        activebackground="#666666", relief="raised", bd=3, anchor="w")
        btn.place(x=20, y=y, width=360, height=50)
        buttons[title] = btn

    # 오른쪽 영역 메시지
    right_sections = [
        ("주유기 상태", trigger_node.pump_message, 20),
        ("카드 상태", trigger_node.card_message, 80),
        ("장애물 상태", trigger_node.obstacle_message, 140)
    ]
    for title, initial_text, y in right_sections:
        btn = tk.Button(main_frame, text=f"{title}: {initial_text}",
                        font=("Arial", 12, "bold"), bg="#444444", fg="white",
                        activebackground="#666666", relief="raised", bd=3, anchor="w")
        btn.place(x=420, y=y, width=360, height=50)
        buttons[title] = btn

    # ===== 별도 영상 창 생성 =====
    video_window = tk.Toplevel(root)
    video_window.title("카메라 영상")
    video_window.geometry("640x480+850+0")  # 기존 GUI 오른쪽 위치에 띄움

    video_label = tk.Label(video_window)
    video_label.pack(fill="both", expand=True)

    # YOLO 모델 & 카메라 초기화
    model = YOLO("/home/shindonghyun/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/resource/best.pt")
    cap = cv2.VideoCapture(4)

    if not cap.isOpened():
        print("[ERROR] 카메라 열기 실패! 연결 및 인덱스 확인 필요.")

    def update_gui():
        # GUI 버튼 상태 업데이트
        buttons["로봇 상태"].config(text=f"로봇 상태: {trigger_node.status_message}")
        buttons["경유 상태"].config(text=f"경유 상태: {trigger_node.diesel_message}")
        buttons["휘발유 상태"].config(text=f"휘발유 상태: {trigger_node.gasoline_message}")
        buttons["주유기 상태"].config(text=f"주유기 상태: {trigger_node.pump_message}")
        buttons["카드 상태"].config(text=f"카드 상태: {trigger_node.card_message}")
        buttons["장애물 상태"].config(text=f"장애물 상태: {trigger_node.obstacle_message}")
        buttons["총 결제액"].config(text=f"총 결제액: {trigger_node.payment_total} 원")

        # 카메라 feed 처리
        ret, frame = cap.read()
        if ret:
            try:
                results = model.predict(source=frame, conf=0.6, verbose=False)[0]
                for box in results.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    cls_name = model.names[cls_id]
                    label = f'{cls_name} ({cls_id}) {conf:.2f}'
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 2,  # 기존 0.6 -> 1.2로 키움
                                (0, 255, 0), 4)                # 기존 thickness 2 -> 3으로 키움

                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)



                # 현재 video_window 창 크기에 맞춰 동적 리사이즈
                w = video_window.winfo_width()
                h = video_window.winfo_height()
                w = max(100, w)  # 최소 크기 보장
                h = max(100, h)

                img = Image.fromarray(rgb_frame).resize((w, h))
                imgtk = ImageTk.PhotoImage(image=img)
                video_label.imgtk = imgtk
                video_label.config(image=imgtk)
            except Exception as e:
                print(f"[ERROR] YOLO 추론 중 오류: {e}")
        else:
            print("[WARN] 카메라 프레임 읽기 실패.")

        root.after(100, update_gui)

    update_gui()
    root.mainloop()
    cap.release()

def main():
    rclpy.init()
    trigger_node = TriggerNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(trigger_node,), daemon=True)
    ros_thread.start()

    start_gui(trigger_node)

    trigger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
