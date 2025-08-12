import copy
import cv2 as cv
import numpy as np
import mediapipe as mp
import csv
from collections import deque, Counter

from model import KeyPointClassifier
from model import PointHistoryClassifier
from utils import CvFpsCalc
from app import (  # 从原始文件引入这些函数
    calc_bounding_rect, calc_landmark_list,
    pre_process_landmark, pre_process_point_history,
)

class HandGestureDetector:
    def __init__(self, device=0, width=960, height=540,
                 min_detection_confidence=0.7, min_tracking_confidence=0.5):

        self.cap = cv.VideoCapture(device)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)

        self.history_length = 16
        self.point_history = deque(maxlen=self.history_length)
        self.finger_gesture_history = deque(maxlen=self.history_length)

        # MediaPipe Hands
        mp_hands = mp.solutions.hands
        self.hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

        # 加载分类器和标签
        self.keypoint_classifier = KeyPointClassifier()
        self.point_history_classifier = PointHistoryClassifier()

        with open('model/keypoint_classifier/keypoint_classifier_label.csv',
                  encoding='utf-8-sig') as f:
            self.keypoint_labels = [row[0] for row in csv.reader(f)]

        with open('model/point_history_classifier/point_history_classifier_label.csv',
                  encoding='utf-8-sig') as f:
            self.point_history_labels = [row[0] for row in csv.reader(f)]

    def read_frame(self):
        ret, image = self.cap.read()
        if not ret:
            return None, None
        image = cv.flip(image, 1)
        debug_image = copy.deepcopy(image)
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = self.hands.process(image)
        image.flags.writeable = True
        return debug_image, results

    def detect_gesture(self, image, results):
        gesture_text = ""
        if results.multi_hand_landmarks:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                brect = calc_bounding_rect(image, hand_landmarks)
                landmark_list = calc_landmark_list(image, hand_landmarks)

                pre_landmarks = pre_process_landmark(landmark_list)
                hand_sign_id = self.keypoint_classifier(pre_landmarks)
                gesture_text = self.keypoint_labels[hand_sign_id]

                # 动态手势轨迹分析
                if hand_sign_id == 2:
                    self.point_history.append(landmark_list[8])
                else:
                    self.point_history.append([0, 0])

                point_history_data = pre_process_point_history(image, self.point_history)
                if len(point_history_data) == self.history_length * 2:
                    fg_id = self.point_history_classifier(point_history_data)
                    self.finger_gesture_history.append(fg_id)
                    most_common = Counter(self.finger_gesture_history).most_common(1)[0][0]
                    gesture_text += f" + {self.point_history_labels[most_common]}"

        else:
            self.point_history.append([0, 0])

        return gesture_text

    def release(self):
        self.cap.release()
