# ROS2 & Doosan
import rclpy, DR_init

# Python 기본 라이브러리
import time, math
from collections import deque

# 이미지 처리
import numpy as np
import cv2

# Skeleton image processing
from skimage.color import rgb2gray
from skimage.filters import threshold_otsu
from skimage.morphology import skeletonize

# ROS visualization
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Debug visualization
import matplotlib.pyplot as plt

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 200, 200
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
OFF, ON = 0, 1

right_eyes_x = 15
right_eyes_y = -172

left_eyes_x = -40
left_eyes_y = -175

nose_offset_x = 20
nose_offset_y = 13

def main(args=None): # node 초기화 및 전체 드로잉 시퀀스 수행
    rclpy.init(args=args)
    node = rclpy.create_node("doosan_pick_and_place", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool, set_tcp, movej, movel, wait,
            set_digital_output, set_singularity_handling,
            movesx, DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL,task_compliance_ctrl,set_desired_force,DR_FC_MOD_REL,move_spiral,DR_AXIS_Z,DR_TOOL,release_force,
            release_compliance_ctrl,move_periodic,check_force_condition
        )
        from DR_common2 import posj, posx
    except ImportError as e:
        print(f"Error importing Doosan API modules: {e}")
        return

    set_tool("Tool Weight_test")
    set_tcp("test_TCP")
    set_singularity_handling(ON)
    print("Speed and compliance setup completed")

    # 갤러리 이미지 처리 및 경로 발굴
    img_path = "/home/shindonghyun/doosan_ws/image/totoro_image.jpg"
    img = cv2.imread(img_path)
    gray_img = rgb2gray(img)
    thresh_val = threshold_otsu(gray_img)
    binary_img = gray_img < thresh_val
    skeleton = skeletonize(binary_img)

# ============================================================
# 이미지 처리(Image Processing & Skeleton Path Extraction)
# ============================================================

    # skel_image 끝점(end_point) 탐색 - 연결된 이웃 pixel의 1개 지점 반환
    def find_endpoints(skel_img):
        endpoints = []
        for y in range(1, skel_img.shape[0] - 1):
            for x in range(1, skel_img.shape[1] - 1):
                if skel_img[y, x] == 1:
                    neighbors = np.sum(skel_img[y-1:y+2, x-1:x+2]) - 1
                    if neighbors == 1:
                        endpoints.append((y, x))
        return endpoints
    
    # BFS 기반 skeleton 경로 추적, 시작점부터 연결된 pixel을 순차적으로 탐색
    def fast_trace_path(skel_img, start):
        visited = np.zeros_like(skel_img, dtype=bool)
        path = []
        q = deque()
        q.append(start)
        visited[start] = True
        while q:
            y, x = q.popleft()
            path.append((y, x))
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dy == 0 and dx == 0:
                        continue
                    ny, nx = y + dy, x + dx
                    if (0 <= ny < skel_img.shape[0]) and (0 <= nx < skel_img.shape[1]):
                        if skel_img[ny, nx] == 1 and not visited[ny, nx]:
                            visited[ny, nx] = True
                            q.append((ny, nx))
        return path

    start_point = (273, 135)
    skeleton_path = fast_trace_path(skeleton, start_point)
    robot_path = [(x, y) for y, x in skeleton_path]

    # 코 이미지 skeleton 경로 추출(이미지 → 이진화 → skeleton → BFS 경로 생성)>
    def nose_path():
        # 1. 이미지 불러오기 및 리사이즈 (210 x 297)
        img_path = "/home/shindonghyun/doosan_ws/image/totoro_nose.jpg"
        img = cv2.imread(img_path)
        # raw = 2408// 10
        # column = 3508 // 10
        # img = cv2.resize(img, (raw, column))  # (width, height)

        # 2. 흑백 변환 및 이진화
        gray_img = rgb2gray(img)
        thresh_val = threshold_otsu(gray_img)
        binary_img = gray_img < thresh_val  # 검정 선 추출을 위해 반전

        # 3. 스켈레톤 중심선 추출
        skeleton = skeletonize(binary_img)

        # 4. 끝점 찾기 함수
        def find_endpoints(skel_img):
            endpoints = []
            for y in range(1, skel_img.shape[0] - 1):
                for x in range(1, skel_img.shape[1] - 1):
                    if skel_img[y, x] == 1:
                        neighbors = np.sum(skel_img[y-1:y+2, x-1:x+2]) - 1
                        if neighbors == 1:
                            endpoints.append((y, x))
            return endpoints
        #642 871
        # 5. BFS 기반 경로 추적
        def fast_trace_path(skel_img, start):
            visited = np.zeros_like(skel_img, dtype=bool)
            path = []
            q = deque()
            q.append(start)
            visited[start] = True
            while q:
                y, x = q.popleft()
                path.append((y, x))
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        if dy == 0 and dx == 0:
                            continue
                        ny, nx = y + dy, x + dx
                        if (0 <= ny < skel_img.shape[0]) and (0 <= nx < skel_img.shape[1]):
                            if skel_img[ny, nx] == 1 and not visited[ny, nx]:
                                visited[ny, nx] = True
                                q.append((ny, nx))
            return path

        # 6. 시작점 설정 및 경로 추출
        endpoints = find_endpoints(skeleton)
        start_point = (100, 107)  # 수동 설정 가능: (y, x)
        skeleton_path = fast_trace_path(skeleton, start_point)

        # 7. 시각화 (10개마다 점찍기)
        img_vis = img.copy()
        for idx, (y, x) in enumerate(skeleton_path[::3]):
            cv2.circle(img_vis, (x, y), 2, (0, 0, 255), -1)
            cv2.putText(img_vis, str(idx + 1), (x + 3, y - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.15, (255, 0, 0), 1)

        # plt.figure(figsize=(8, 11))
        # plt.imshow(cv2.cvtColor(img_vis, cv2.COLOR_BGR2RGB))
        # plt.title("Skeleton Path (Resized to 210x297)")
        # plt.axis("off")
        # plt.show()

        # 8. 로봇 좌표로 변환
        nose_draw = [(x, y) for y, x in skeleton_path]
        print(len(nose_draw)//5)
        return nose_draw
    
    # 패턴 이미지 여러 개를 skeleton경로로 변환, 각 패턴 별 drawing path 생성
    def path_pattern():
        pat_list = []
        pat_list.append("/home/jin/Downloads/pattern_1.jpg")
        pat_list.append("/home/jin/Downloads/pattern_2.jpg")
        pat_list.append("/home/jin/Downloads/pattern_3.jpg")
        pat_list.append("/home/jin/Downloads/pattern_4.jpg")
        pat_list.append("/home/jin/Downloads/pattern_5.jpg")
        pat_list.append("/home/jin/Downloads/pattern_6.jpg")
        pat_list.append("/home/jin/Downloads/pattern_7.jpg")

        point_list= []
        point_list.append((160, 81))
        point_list.append((142,99))
        point_list.append((172,134))
        point_list.append((176,45))
        point_list.append((181,91))
        point_list.append((188,121))
        point_list.append((204,142))

        list_pattern = []  # 🔄 반복문 밖으로 위치 수정

        for i in range(0,7):    
            img_path = pat_list[i]
            img = cv2.imread(img_path)

            gray_img = rgb2gray(img)
            thresh_val = threshold_otsu(gray_img)
            binary_img = gray_img < thresh_val

            skeleton = skeletonize(binary_img)

            def find_endpoints(skel_img):
                endpoints = []
                for y in range(1, skel_img.shape[0] - 1):
                    for x in range(1, skel_img.shape[1] - 1):
                        if skel_img[y, x] == 1:
                            neighbors = np.sum(skel_img[y-1:y+2, x-1:x+2]) - 1
                            if neighbors == 1:
                                endpoints.append((y, x))
                return endpoints

            def fast_trace_path(skel_img, start):
                visited = np.zeros_like(skel_img, dtype=bool)
                path = []
                q = deque()
                q.append(start)
                visited[start] = True
                while q:
                    y, x = q.popleft()
                    path.append((y, x))
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dy == 0 and dx == 0:
                                continue
                            ny, nx = y + dy, x + dx
                            if (0 <= ny < skel_img.shape[0]) and (0 <= nx < skel_img.shape[1]):
                                if skel_img[ny, nx] == 1 and not visited[ny, nx]:
                                    visited[ny, nx] = True
                                    q.append((ny, nx))
                return path

            start_point = point_list[i]
            skeleton_path = fast_trace_path(skeleton, start_point)

            img_vis = img.copy()
            for idx, (y, x) in enumerate(skeleton_path[::2]):
                cv2.circle(img_vis, (x, y), 2, (0, 0, 255), -1)
                cv2.putText(img_vis, str(idx + 1), (x + 3, y - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.15, (255, 0, 0), 1)

            # plt.figure(figsize=(8, 11))
            # plt.imshow(cv2.cvtColor(img_vis, cv2.COLOR_BGR2RGB))
            # plt.title("Skeleton Path (Resized to 210x297)")
            # plt.axis("off")
            # plt.show()

            pattern_list = [(x, y) for y, x in skeleton_path]
            print(len(pattern_list)//5)
            list_pattern.append(pattern_list)  # 🔄 여기로 이동

        return list_pattern
    
    # 입 이미지 skeleton 경로 생성 함수
    def path_mouth():
        # 1. 이미지 불러오기 및 리사이즈 (210 x 297)
        img_path = "/home/jin/Downloads/IMG_1451.jpg"
        img = cv2.imread(img_path)
        # raw = 2408// 10
        # column = 3508 // 10
        # img = cv2.resize(img, (raw, column))  # (width, height)

        # 2. 흑백 변환 및 이진화
        gray_img = rgb2gray(img)
        thresh_val = threshold_otsu(gray_img)
        binary_img = gray_img < thresh_val  # 검정 선 추출을 위해 반전

        # 3. 스켈레톤 중심선 추출
        skeleton = skeletonize(binary_img)

        # 4. 끝점 찾기 함수
        def find_endpoints(skel_img):
            endpoints = []
            for y in range(1, skel_img.shape[0] - 1):
                for x in range(1, skel_img.shape[1] - 1):
                    if skel_img[y, x] == 1:
                        neighbors = np.sum(skel_img[y-1:y+2, x-1:x+2]) - 1
                        if neighbors == 1:
                            endpoints.append((y, x))
            return endpoints
        #642 871
        # 5. BFS 기반 경로 추적
        def fast_trace_path(skel_img, start):
            visited = np.zeros_like(skel_img, dtype=bool)
            path = []
            q = deque()
            q.append(start)
            visited[start] = True
            while q:
                y, x = q.popleft()
                path.append((y, x))
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        if dy == 0 and dx == 0:
                            continue
                        ny, nx = y + dy, x + dx
                        if (0 <= ny < skel_img.shape[0]) and (0 <= nx < skel_img.shape[1]):
                            if skel_img[ny, nx] == 1 and not visited[ny, nx]:
                                visited[ny, nx] = True
                                q.append((ny, nx))
            return path

        # 6. 시작점 설정 및 경로 추출
        endpoints = find_endpoints(skeleton)
        start_point = (127,111)  # 수동 설정 가능: (y, x)
        skeleton_path = fast_trace_path(skeleton, start_point)

        # 7. 시각화 (10개마다 점찍기)
        img_vis = img.copy()
        for idx, (y, x) in enumerate(skeleton_path[::1]):
            cv2.circle(img_vis, (x, y), 2, (0, 0, 255), -1)
            cv2.putText(img_vis, str(idx + 1), (x + 3, y - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.15, (255, 0, 0), 1)

        # plt.figure(figsize=(8, 11))
        # plt.imshow(cv2.cvtColor(img_vis, cv2.COLOR_BGR2RGB))
        # plt.title("Skeleton Path (Resized to 210x297)")
        # plt.axis("off")
        # plt.show()

        # 8. 로봇 좌표로 변환
        mouth_path = [(x, y) for y, x in skeleton_path]
        print(len(mouth_path)//5)
        return mouth_path

# ============================================================
# Rviz 시각화(RViz Marker Visualization)
# ============================================================

    marker_pub = node.create_publisher(Marker, "/skeleton_path_marker", 10)
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.ns = "skeleton_path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.002
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.points = []

    # RViz 마커 퍼블리셔 설정(skeleton 경로 및 눈, 코 마커를 시각화)
    def marker_timer_callback():
        if marker_timer_callback.idx < len(robot_path):
            x, y = robot_path[marker_timer_callback.idx]
            p = Point()
            p.x = (x + 300.0) / 1000.0
            p.y = (y - 220.0) / 1000.0
            p.z = 0.0
            marker.points.append(p)
            marker.header.stamp = node.get_clock().now().to_msg()
            marker_pub.publish(marker)
            marker_timer_callback.idx += 1

        elif not marker_timer_callback.eyes_drawn:
            # 여기서 눈 마커를 한 번만 추가
            def make_circle_marker(ns, marker_id, color, center, radius_mm, num_points=36):
                m = Marker()
                m.header.frame_id = "base_link"
                m.header.stamp = node.get_clock().now().to_msg()
                m.ns = ns
                m.id = marker_id
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.002
                m.color.a = 1.0
                m.color.r, m.color.g, m.color.b = color
                m.pose.orientation.w = 1.0

                cx, cy = center
                for i in range(num_points + 1):  # +1 to close the circle
                    angle = 2 * math.pi * i / num_points
                    x = cx + radius_mm * math.cos(angle)
                    y = cy + radius_mm * math.sin(angle)
                    p = Point()
                    p.x = x / 1000.0   # 오프셋 제거
                    p.y = y / 1000.0   # 오프셋 제거
                    p.z = 0.0
                    m.points.append(p)
                return m


            left_eye_marker = make_circle_marker(
                ns="left_eye", marker_id=1, color=(1.0, 0.0, 0.0),
                center=(434.0 + left_eyes_x, 56.0 + left_eyes_y),
                radius_mm=2.0
            )

            right_eye_marker = make_circle_marker(
                ns="right_eye", marker_id=2, color=(1.0, 0.0, 0.0),
                center=(434.0 + right_eyes_x, 56.0 + right_eyes_y),
                radius_mm=2.0
            )


            marker_pub.publish(left_eye_marker)
            marker_pub.publish(right_eye_marker)
           # === ✅ 코 마커 추가 (스켈레톤 기반 경로) ===
            nose_points = nose_path()[::3]  # 간격 조절
            nose_marker = Marker()
            nose_marker.header.frame_id = "base_link"
            nose_marker.header.stamp = node.get_clock().now().to_msg()
            nose_marker.ns = "nose"
            nose_marker.id = 3
            nose_marker.type = Marker.LINE_STRIP
            nose_marker.action = Marker.ADD
            nose_marker.scale.x = 0.002
            nose_marker.color.a = 1.0
            nose_marker.color.r = 1.0
            nose_marker.color.g = 0.0
            nose_marker.color.b = 0.0
            nose_marker.pose.orientation.w = 1.0

            for (x, y) in nose_points:
                p = Point()
                p.x = (x + 300.0) / 1000.0
                p.y = (y - 220.0) / 1000.0
                p.z = 0.0
                nose_marker.points.append(p)

            marker_pub.publish(nose_marker)
            marker_timer_callback.eyes_drawn = True

            marker_timer_callback.eyes_drawn = True  # 두 번 그리지 않게
    
    marker_timer_callback.idx = 0
    marker_timer_callback.eyes_drawn = False

# ============================================================
# 로봇 드롱잉 제어(Robot Drawing Motion Control)
# ============================================================

    # 입 경로를 따라 로봇 드로잉 수행(movex 기반 연속 이동)
    def draw_mouth():
        movel(posx(0,0,100,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

        mouthpath = path_mouth()
        mouth_pattern = mouthpath
        chunks = [[] for _ in range(10)]
        for (x, y) in mouth_pattern[::1]:
            p = posx([float(x)+310.0, float(y)-228.0, 109.0, 118.03, 180.0, 118.08])
            for chunk in chunks:
                if len(chunk) < 80:
                    chunk.append(p)
                    break

        idxss = 0
        while idxss < len(chunks):
            segment = chunks[idxss]
            if segment:
                print(f"[INFO] movesx 실행: chunk {idxss}, pose 수: {len(segment)}")
                movesx(segment, vel=VELOCITY, acc=ACC)
            idxss += 1     
        movel(posx(0,0,100,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

    # 패턴 그리기(토토로 배 패턴 이미지를 순차적으로 드로잉)
    def draw_pattern():
        movel(posx(0,0, 20 ,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

        patternpath = path_pattern()
        for i in range(0, 7): 
            pattern = patternpath[i]
            chunks = [[] for _ in range(1)]
            for (x, y) in pattern[::2]:
                p = posx([float(x)+300.0, float(y)-220.0, 108.5, 118.03, 180.0, 118.08])
                for chunk in chunks:
                    if len(chunk) < 80:
                        chunk.append(p)
                        break

            idx = 0
            while idx < len(chunks):
                segment = chunks[idx]
                if segment:
                    print(f"[INFO] movesx 실행: chunk {idx}, pose 수: {len(segment)}")
                    movesx(segment, vel=VELOCITY, acc=ACC)
                    movel(posx(0,0,100,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                idx += 1
        # for i in range(0,5):
    
    # 코 그리기(토토로 코 skeleton 경로를 로봇으로 드로잉)
    def draw_nose():
        nosepath = nose_path()
        nose_point = posx([434.0 + left_eyes_x + nose_offset_x, 56.0 + left_eyes_y+nose_offset_y, 109.0, 118.03, 180.0, 118.08])
        nose_ready = posx([434.0 + left_eyes_x + nose_offset_x, 56.0 + left_eyes_y+nose_offset_y, 109.0 + 50, 118.03, 180.0, 118.08])
        movel(nose_ready, v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        movel(nose_point, v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        nose_list= [[] for _ in range(10)]
        for (x, y) in nosepath[::3]:
            p = posx([float(x)+310.0, float(y)-205.0, 110.0, 118.03, 180.0, 118.08])
            for nose in nose_list:
                if len(nose) < 80:
                    nose.append(p)
                    break
        index = 0
        while index < len(nose_list):
            segment = nose_list[index]
            if segment:
                print(f"[INFO] movesx 실행: chunk {index}, pose 수: {len(segment)}")
                movesx(segment, vel=VELOCITY, acc=ACC)
            index += 1
        movel(posx(0,0, 100 ,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

    # 눈 그리기(힘제어(compliance control과 force control)기반 토토로 눈 드로잉 수행)
    def draw_eyes():
        #눈 그리기 
        time.sleep(1.0)

        # 왼쪽 눈
        left_eyes_point = posx([434.0 + left_eyes_x, 56.0 + left_eyes_y, 109.0, 118.03, 180.0, 118.08])
        left_eyes_ready = posx([434.0 + left_eyes_x, 56.0 + left_eyes_y, 109.0 + 50, 118.03, 180.0, 118.08])

        movel(left_eyes_ready, v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        movel(left_eyes_point, v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        # move_spiral(rev=3.0, rmax=10.0, lmax=30.0, vel=10, acc=20, time=10.0, axis=DR_AXIS_Z, ref=DR_TOOL)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        
        time.sleep(0.1)
        # set_stiffnessx([3000.0]*3 + [200.0]*3, time=0.0)
        # wait(0.5)
        set_desired_force(fd=[0, 0, -8, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)  
        time.sleep(3.0)
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()
        time.sleep(0.1)     
        # # 오른쪽 눈
        time.sleep(1.0)
        movel(posx(0,0, 100 ,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

        right_eyes_point = posx([434.0 + right_eyes_x, 56.0 + right_eyes_y, 109.0, 118.03, 180.0, 118.08])
        right_eyes_ready = posx([434.0 + right_eyes_x, 56.0 + right_eyes_y, 109.0 + 50, 118.03, 180.0, 118.08])

        movel(right_eyes_ready, v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        movel(right_eyes_point, v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)  
        time.sleep(3.0)
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()
        time.sleep(0.1)  
        # move_spiral(rev=9.5, rmax=20.0, lmax=50.0, vel=20, acc=50, time=20.0, axis=DR_AXIS_Z, ref=DR_TOOL)
        movel(posx(0,0, 100 ,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

        print("Pick-and-place sequence completed.")

    # 일반 펜을 내려놓고 붓펜으로 교체
    def brush_pen():
        get_pen = posx([223.88, -182.77, 117.37, 69.39, -179.99, 90.71])
        get_pen_up = posx([223.88, -182.77, 211.37, 69.39, -179.99, 90.71])
        movel(get_pen_up, v=20, a=20, vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        movel(get_pen, v=20, a=20, vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        release()
        time.sleep(1.0)
        movel(get_pen_up, v=20, a=20, vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        get_brushpen = posx([200.12, -58.35, 130.0, 152.13, -179.99, 151.76])
        ready_brushpen = posx([200.12, -58.35, 213.87, 152.13, -179.99, 151.76])
        movel(ready_brushpen, v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        movel(get_brushpen, v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        grip()
        time.sleep(3.0)
        movel(ready_brushpen, v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)

# ============================================================
# 힘제어(Force Control & Human Interaction)
# ============================================================

    # 사용자의 force 입력(Nudge)을 감지 후 드로잉 수행
    def wait_for_nudge_then_draw(threshold=20.0):
            print("Z축 방향으로 최소 20.0N 이상 누르면 다음 동작을 수행합니다.")
            try:
                while rclpy.ok():
                    while not check_force_condition(axis=DR_AXIS_Z, min=threshold, ref=DR_TOOL):
                        print("Nudge 감지됨! 후속 동작 수행 중")
                        brush_pen()
                        draw_eyes()
                        # draw_nose()
                        break
                    time.sleep(0.1)                        
                    
                print("경로 실행 중...")
                time.sleep(3.0)
                print("경로 실행 완료")        
            except Exception as e:
                print(f"예외 발생: {e}")   
    # # 펜 pick 동작 수행
    def pen_pick_and_place():
        # 펜잡기
        get_pen = posx([223.88, -182.77, 118.00, 69.39, -179.99, 90.71])
        get_pen_up = posx([223.88, -182.77, 211.37, 69.39, -179.99, 90.71])
        release()
        movel(get_pen_up, v=20, a=20, vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        movel(get_pen, v=20, a=20, vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        grip()
        time.sleep(3.0)
        movel(get_pen_up, v=20, a=20, vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
    # 그리퍼 open
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(0.5)
    # 그리퍼 close
    def grip():
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        wait(0.5)
    pen_pick_and_place()

    # 로봇 동작 시작
    JReady = [0, -20, 110, 0, 90, 0]
    movej(JReady, v=20, a=20)
    Ready = posx([434.0, 56.0, 109.0, 118.03, 180.0, 118.08])
    movel(Ready, v=20, a=20, vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)

    # 경로 이동 분할
    chunks = [[] for _ in range(10)]
    for (x, y) in robot_path[5::5]:
        p = posx([float(x)+300.0, float(y)-220.0, 109.0, 118.03, 180.0, 118.08])
        for chunk in chunks:
            if len(chunk) < 80:
                chunk.append(p)
                break
            
    node.create_timer(0.11, marker_timer_callback)

    idx = 0
    while idx < len(chunks):
        segment = chunks[idx]
        if segment:
            print(f"[INFO] movesx 실행: chunk {idx}, pose 수: {len(segment)}")
            movesx(segment, vel=VELOCITY, acc=ACC)
        idx += 1
    # # 넛지 
    draw_mouth()
    draw_pattern()
    draw_nose()
    wait_for_nudge_then_draw()

    # rclpy.spin(node)
    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()