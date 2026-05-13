########## YoloModel ##########

# 표준 및 외부 모듈 import
import os
import json
import time
from collections import Counter

import rclpy  # ROS2 Python client library
from ament_index_python.packages import get_package_share_directory  # 패키지 경로 찾기
from ultralytics import YOLO  # YOLOv8 모델 로딩
import numpy as np


# ROS2 패키지 및 자원 경로 설정
PACKAGE_NAME = "pick_and_place_voice"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

# YOLO 모델 및 클래스명 매핑 JSON 파일
YOLO_MODEL_FILENAME = "best.pt"
YOLO_CLASS_NAME_JSON = "class_name_tool.json"

# 실제 파일 경로 구성
YOLO_MODEL_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_MODEL_FILENAME)
YOLO_JSON_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_CLASS_NAME_JSON)


class YoloModel:
    def __init__(self):
        """
        클래스 초기화: YOLO 모델과 클래스 이름 정보를 로딩합니다.
        """
        self.model = YOLO(YOLO_MODEL_PATH)  # 학습된 YOLOv8 모델 불러오기
        with open(YOLO_JSON_PATH, "r", encoding="utf-8") as file:
            class_dict = json.load(file)  # JSON에서 클래스 사전 읽기
            # {이름: id} 형식으로 변환
            self.reversed_class_dict = {v: int(k) for k, v in class_dict.items()}

    def get_frames(self, img_node, duration=1.0):
        """
        특정 시간(duration) 동안 이미지 노드로부터 프레임들을 수집합니다.
        :param img_node: 카메라로부터 프레임을 받을 수 있는 ROS2 노드
        :param duration: 프레임을 수집할 시간 (초 단위)
        :return: 수집된 프레임 리스트
        """
        end_time = time.time() + duration
        frames = {}

        while time.time() < end_time:
            rclpy.spin_once(img_node)  # ROS2 콜백 실행
            frame = img_node.get_color_frame()  # 프레임 가져오기 (OpenCV 이미지)
            stamp = img_node.get_color_frame_stamp()  # 타임스탬프 (중복 방지)
            if frame is not None:
                frames[stamp] = frame
            time.sleep(0.01)  # 너무 빠르게 반복하지 않도록 sleep

        if not frames:
            print("No frames captured in %.2f seconds", duration)

        print("%d frames captured", len(frames))
        return list(frames.values())  # 리스트로 변환해서 반환

    # 원본 함수
    def get_best_detection(self, img_node, target):
        rclpy.spin_once(img_node)
        frames = self.get_frames(img_node)
        if not frames:  # Check if frames are empty
            return None

        results = self.model(frames, verbose=False)
        print("classes: ")
        print(results[0].names)
        detections = self._aggregate_detections(results)
        label_id = self.reversed_class_dict[target]
        print("label_id: ", label_id)
        print("detections: ", detections)

        matches = [d for d in detections if d["label"] == label_id]
        if not matches:
            print("No matches found for the target label.")
            return None, None
        best_det = max(matches, key=lambda x: x["score"])
        return best_det["box"], best_det["score"]
    
        '''
        영어 → 포르투갈어 이름 매핑
        target_aliases = {
            'hammer': 'Martelo',
            'pliers': 'Alicate',
            'screwdriver': 'Chave',
            'saw': 'Serrote'
        }
        '''

    # def get_best_detection(self, img_node, target):
    #     """
    #     지정한 target 객체에 대해 YOLO로 인식하고, 가장 신뢰도 높은 박스를 반환합니다.
    #     :param img_node: 이미지 노드 (get_color_frame 필요)
    #     :param target: 인식하고자 하는 객체 이름 (영어 또는 포르투갈어)
    #     :return: (바운딩 박스 [x1,y1,x2,y2], 신뢰도) 또는 (None, None)
    #     """

    #     # 영어 → 포르투갈어 대응 사전
    #     target_aliases = {
    #         'hammer': 'Martelo',
    #         'pliers': 'Alicate',
    #         'screwdriver': 'Chave',
    #         'saw': 'Serrote'
    #     }

    #     # target 이름 번역 (없으면 그대로 사용)
    #     translated_target = target_aliases.get(target.lower(), target)

    #     # ROS2 노드 콜백 처리
    #     rclpy.spin_once(img_node)

    #     # 일정 시간 동안 프레임 수집
    #     frames = self.get_frames(img_node)
    #     if not frames:
    #         return None, None

    #     # YOLO 추론 실행
    #     results = self.model(frames, verbose=False)

    #     print("classes: ")
    #     print(results[0].names)  # 모델이 감지 가능한 클래스명 출력

    #     # 여러 프레임에서 나온 감지 결과를 종합
    #     detections = self._aggregate_detections(results)

    #     # 클래스 이름 → 라벨 인덱스
    #     try:
    #         label_id = self.reversed_class_dict[translated_target]
    #     except KeyError:
    #         print(f":x: Target '{translated_target}' not found in class dictionary.")
    #         return None, None

    #     print("label_id: ", label_id)
    #     print("detections: ", detections)

    #     # 라벨 ID가 일치하는 감지들만 필터링
    #     matches = [d for d in detections if d["label"] == label_id]
    #     if not matches:
    #         print("No matches found for the target label.")
    #         return None, None

    #     # 신뢰도(score)가 가장 높은 감지를 반환
    #     best_det = max(matches, key=lambda x: x["score"])
    #     return best_det["box"], best_det["score"]

    def _aggregate_detections(self, results, confidence_threshold=0.5, iou_threshold=0.5):
        """
        여러 프레임에서 나온 감지 결과들을 종합하여 최종 박스를 구성합니다.
        :param results: YOLO 결과 리스트
        :param confidence_threshold: 신뢰도 필터링 기준
        :param iou_threshold: 같은 객체로 판단할 IoU 기준
        :return: 정제된 감지 결과 리스트 (box, score, label 포함)
        """
        raw = []  # 초기 감지 결과 수집
        for res in results:
            for box, score, label in zip(
                res.boxes.xyxy.tolist(),
                res.boxes.conf.tolist(),
                res.boxes.cls.tolist(),
            ):
                if score >= confidence_threshold:
                    raw.append({"box": box, "score": score, "label": int(label)})

        final = []  # 최종 감지 결과
        used = [False] * len(raw)  # 중복 처리 방지

        for i, det in enumerate(raw):
            if used[i]:
                continue
            group = [det]
            used[i] = True
            for j, other in enumerate(raw):
                if not used[j] and other["label"] == det["label"]:
                    # IoU가 기준 이상이면 같은 객체로 판단
                    if self._iou(det["box"], other["box"]) >= iou_threshold:
                        group.append(other)
                        used[j] = True

            # 그룹 평균 박스를 생성
            boxes = np.array([g["box"] for g in group])
            scores = np.array([g["score"] for g in group])
            labels = [g["label"] for g in group]

            final.append({
                "box": boxes.mean(axis=0).tolist(),  # 평균 위치
                "score": float(scores.mean()),        # 평균 신뢰도
                "label": Counter(labels).most_common(1)[0][0]  # 가장 빈도 높은 라벨
            })

        return final

    def _iou(self, box1, box2):
        """
        두 박스 간 IoU(Intersection over Union)를 계산합니다.
        :param box1: [x1, y1, x2, y2]
        :param box2: [x1, y1, x2, y2]
        :return: IoU 값 (0~1)
        """
        x1, y1 = max(box1[0], box2[0]), max(box1[1], box2[1])
        x2, y2 = min(box1[2], box2[2]), min(box1[3], box2[3])

        # 교집합 넓이
        inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)

        # 두 박스 넓이
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])

        # IoU 계산
        union = area1 + area2 - inter
        return inter / union if union > 0 else 0.0
