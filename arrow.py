#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class ArrowDirectionDetector:
    def __init__(self, flip_horizontal=True):
        # 파란색 HSV 범위 (필요 시 파라미터로 튜닝 가능)
        self.blue_lower = np.array([100, 100, 50], dtype=np.uint8)
        self.blue_upper = np.array([130, 255, 255], dtype=np.uint8)

        # 흰색 HSV 범위 (화살표)
        self.white_lower = np.array([0, 0, 170], dtype=np.uint8)
        self.white_upper = np.array([179, 60, 255], dtype=np.uint8)

        self.flip_horizontal = flip_horizontal

    #[추가] 거리 인식 튜닝용 파라미터
        #    멀리 있는 표지판도 잡기 위해 기존보다 작게 설정
        self.min_sign_size = 20           # 기존 코드 w/h 30 → 20 정도로 완화
        self.min_arrow_area_ratio = 0.004 # 기존 0.01 → 0.004 (0.4%) 로 완화
    
    '''(화살표 윤곽선 무게중심)def detect_direction(self, frame):
        """
        frame: BGR 이미지
        return: direction("left"/"right"/"none"), debug_frame
        """
        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        debug_frame = frame.copy()

        # 1. 파란 원 표지판 검출
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv, self.blue_lower, self.blue_upper)

        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return "none", debug_frame

        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)

        # 너무 작은 건 무시
        if w < 30 or h < 30:
            return "none", debug_frame

        roi = frame[y:y + h, x:x + w]
        roi_h, roi_w = roi.shape[:2]

        # 2. 원 내부만 마스크 (ROI 중앙을 원 중심으로 가정)
        mask_circle = np.zeros((roi_h, roi_w), dtype=np.uint8)
        cx0, cy0 = roi_w // 2, roi_h // 2
        radius = min(roi_w, roi_h) // 2
        cv2.circle(mask_circle, (cx0, cy0), radius, 255, -1)

        roi_circle = cv2.bitwise_and(roi, roi, mask=mask_circle)

        # 3. 흰색 화살표 검출 (HSV에서 흰색 범위)
        hsv_roi = cv2.cvtColor(roi_circle, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv_roi, self.white_lower, self.white_upper)

        # 노이즈 제거
        mask_white = cv2.morphologyEx(
            mask_white, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=2
        )
        mask_white = cv2.morphologyEx(
            mask_white, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2
        )

        # 에지 추출
        edges = cv2.Canny(mask_white, 50, 150)

        contours_white, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                             cv2.CHAIN_APPROX_SIMPLE)

        if not contours_white:
            return "none", debug_frame

        arrow_contour = max(contours_white, key=cv2.contourArea)
        area = cv2.contourArea(arrow_contour)

        # 화살표가 충분히 크지 않으면 무시
        if area < 0.01 * roi_w * roi_h:
            return "none", debug_frame

        # 4. 화살표 컨투어의 중심 (centroid) 계산
        M = cv2.moments(arrow_contour)
        if M["m00"] == 0:
            return "none", debug_frame

        arrow_cx = M["m10"] / M["m00"]
        arrow_cy = M["m01"] / M["m00"]

        # 방향 판별
        direction = "right" if arrow_cx > roi_w / 2.0 else "left"

        # ===== 디버그용 그림 그리기 =====
        # 원 bounding box
        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # ROI 중앙선
        cv2.line(debug_frame,
                 (x + roi_w // 2, y),
                 (x + roi_w // 2, y + h),
                 (255, 0, 0), 2)
        # 화살표 컨투어 & 중심점
        arrow_contour_shifted = arrow_contour + np.array([[x, y]])
        cv2.drawContours(debug_frame, [arrow_contour_shifted], -1, (0, 0, 255), 2)
        cv2.circle(debug_frame,
                   (int(x + arrow_cx), int(y + arrow_cy)),
                   5, (0, 0, 255), -1)

        return direction, debug_frame'''


    '''(화살표 윤곽선 끝점))def detect_direction(self, frame):
        """
        frame: BGR 이미지
        return: direction("left"/"right"/"none"), debug_frame
        """
        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        debug_frame = frame.copy()

        # 1. 파란 원 표지판 검출
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv, self.blue_lower, self.blue_upper)

        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return "none", debug_frame

        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)

        # 너무 작은 건 무시
        if w < 30 or h < 30:
            return "none", debug_frame

        roi = frame[y:y + h, x:x + w]
        roi_h, roi_w = roi.shape[:2]

        # 2. 원 내부만 마스크 (ROI 중앙을 원 중심으로 가정)
        mask_circle = np.zeros((roi_h, roi_w), dtype=np.uint8)
        cx0, cy0 = roi_w // 2, roi_h // 2       # 원 중심(ROI 기준)
        radius = min(roi_w, roi_h) // 2
        cv2.circle(mask_circle, (cx0, cy0), radius, 255, -1)

        roi_circle = cv2.bitwise_and(roi, roi, mask=mask_circle)

        # 3. 흰색 화살표 검출 (HSV에서 흰색 범위)
        hsv_roi = cv2.cvtColor(roi_circle, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv_roi, self.white_lower, self.white_upper)

        # 노이즈 제거
        mask_white = cv2.morphologyEx(
            mask_white, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=2
        )
        mask_white = cv2.morphologyEx(
            mask_white, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2
        )

        # 에지 추출
        edges = cv2.Canny(mask_white, 50, 150)

        contours_white, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                             cv2.CHAIN_APPROX_SIMPLE)

        if not contours_white:
            return "none", debug_frame

        arrow_contour = max(contours_white, key=cv2.contourArea)
        area = cv2.contourArea(arrow_contour)

        # 화살표가 충분히 크지 않으면 무시
        if area < 0.01 * roi_w * roi_h:
            return "none", debug_frame

        # 4. 화살표 "끝점(tip)" 찾기
        #    → ROI 중심(cx0, cy0)에서 가장 멀리 떨어진 컨투어 점을 tip으로 사용
        pts = arrow_contour.reshape(-1, 2)          # (N,2) 형태로 변환
        dx = pts[:, 0].astype(np.float32) - float(cx0)
        dy = pts[:, 1].astype(np.float32) - float(cy0)
        dist2 = dx * dx + dy * dy                   # 제곱 거리
        idx = np.argmax(dist2)                      # 가장 먼 점 인덱스
        tip_x, tip_y = pts[idx]                     # tip (ROI 좌표계)

        # 방향 판별: tip이 ROI 중심보다 오른쪽이면 right, 왼쪽이면 left
        direction = "right" if tip_x > cx0 else "left"

        # ===== 디버그용 그림 그리기 =====
        # 원 bounding box (전체 ROI)
        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # ROI 중앙선
        cv2.line(debug_frame,
                 (x + roi_w // 2, y),
                 (x + roi_w // 2, y + h),
                 (255, 0, 0), 2)

        # 화살표 컨투어 & tip 위치
        arrow_contour_shifted = arrow_contour + np.array([[x, y]])
        cv2.drawContours(debug_frame, [arrow_contour_shifted], -1, (0, 0, 255), 2)

        tip_x_global = int(x + tip_x)
        tip_y_global = int(y + tip_y)
        cv2.circle(debug_frame, (tip_x_global, tip_y_global), 6, (0, 255, 255), -1)

        return direction, debug_frame'''

    def detect_direction(self, frame):
        """
        frame: BGR 이미지
        return: direction("left"/"right"/"none"), debug_frame
        """
        # 0. 좌우 반전 보정
        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        debug_frame = frame.copy()

        # 1. 파란 원 표지판 검출
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv, self.blue_lower, self.blue_upper)

        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return "none", debug_frame

        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)

        # 너무 작은 건 무시
        if w < self.min_sign_size or h < self.min_sign_size:
            return "none", debug_frame

        roi = frame[y:y + h, x:x + w]
        roi_h, roi_w = roi.shape[:2]

        # 2. 원 내부만 마스크 (ROI 중앙을 원 중심으로 가정)
        mask_circle = np.zeros((roi_h, roi_w), dtype=np.uint8)
        cx0, cy0 = roi_w // 2, roi_h // 2      # 원 중심 (ROI 좌표계)
        radius = min(roi_w, roi_h) // 2
        cv2.circle(mask_circle, (cx0, cy0), radius, 255, -1)

        roi_circle = cv2.bitwise_and(roi, roi, mask=mask_circle)

        # 3. 흰색 화살표 검출
        hsv_roi = cv2.cvtColor(roi_circle, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv_roi, self.white_lower, self.white_upper)

        mask_white = cv2.morphologyEx(
            mask_white, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=2
        )
        mask_white = cv2.morphologyEx(
            mask_white, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2
        )

        edges = cv2.Canny(mask_white, 50, 150)

        contours_white, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                             cv2.CHAIN_APPROX_SIMPLE)

        if not contours_white:
            return "none", debug_frame

        arrow_contour = max(contours_white, key=cv2.contourArea)
        area = cv2.contourArea(arrow_contour)

        # 화살표가 충분히 크지 않으면 무시 (면적 비율 완화)
        if area < self.min_arrow_area_ratio * roi_w * roi_h:
            return "none", debug_frame

        # 4. 화살표 head/tail 찾기 (PCA 기반)
        pts = arrow_contour.reshape(-1, 2).astype(np.float32)

        M = cv2.moments(arrow_contour)
        if M["m00"] == 0:
            return "none", debug_frame
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        # 중심 기준으로 좌표 이동
        pts_centered = pts - np.array([[cx, cy]], dtype=np.float32)

        # 공분산 행렬 → 주성분 벡터
        cov = np.cov(pts_centered.T)
        eigvals, eigvecs = np.linalg.eig(cov)
        idx_max = int(np.argmax(eigvals))
        main_axis = eigvecs[:, idx_max].real
        main_axis = main_axis / (np.linalg.norm(main_axis) + 1e-6)

        # 각 점을 주축 방향으로 투영
        proj = pts_centered @ main_axis  # (N,)

        head_idx = int(np.argmax(proj))   # 머리
        tail_idx = int(np.argmin(proj))   # 꼬리

        head_x, head_y = pts[head_idx]
        tail_x, tail_y = pts[tail_idx]

        # 5. 방향 판별: 꼬리/머리의 세로 위치 관계로 좌/우 결정
        #    (ROS 이미지 좌표계: 위쪽일수록 y가 작고, 아래쪽일수록 y가 큼)
        dy = tail_y - head_y

        if abs(dy) < 5:   # 세로 차이가 너무 작으면 애매 → none (필요 없으면 제거 가능)
            direction = "none"
        else:
            if tail_y > head_y:
                direction = "left"    # 꼬리가 머리보다 위쪽이면 왼쪽 화살표
            else:
                direction = "right"   # 꼬리가 머리보다 아래쪽이면 오른쪽 화살표

        # ===== 디버그용 그림 그리기 =====
        arrow_contour_shifted = arrow_contour + np.array([[x, y]])
        cv2.drawContours(debug_frame, [arrow_contour_shifted], -1, (0, 0, 255), 2)

        # ROI bounding box & 중앙선
        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.line(debug_frame,
                 (x + roi_w // 2, y),
                 (x + roi_w // 2, y + h),
                 (255, 0, 0), 2)

        # head / tail 전역 좌표에 찍기
        head_global = (int(x + head_x), int(y + head_y))
        tail_global = (int(x + tail_x), int(y + tail_y))
        cv2.circle(debug_frame, head_global, 6, (0, 255, 255), -1)  # 머리: 노란색
        cv2.circle(debug_frame, tail_global, 6, (0, 0, 255), -1)    # 꼬리: 빨간색

        return direction, debug_frame


class ArrowDetectorNode:
    def __init__(self):
        rospy.init_node('arrow_detector_node', anonymous=True)

        # 파라미터
        image_topic = rospy.get_param('~image_topic',
                                      '/usb_cam/image_raw/compressed')
        flip_horizontal = rospy.get_param('~flip_horizontal', True)

        rospy.loginfo("Subscribing image topic: %s", image_topic)
        rospy.loginfo("flip_horizontal: %s", flip_horizontal)

        self.detector = ArrowDirectionDetector(
            flip_horizontal=flip_horizontal
        )

        # 퍼블리셔: 방향 + 디버그 이미지
        self.pub_direction = rospy.Publisher(
            '/arrow_direction', String, queue_size=1
        )
        self.pub_debug = rospy.Publisher(
            '/arrow_debug_image/compressed', CompressedImage, queue_size=1
        )

        # 서브스크라이버
        self.sub = rospy.Subscriber(
            image_topic, CompressedImage, self.image_callback,
            queue_size=1, buff_size=2**24
        )

    def image_callback(self, msg):
        # CompressedImage → OpenCV BGR
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            rospy.logwarn("이미지 디코딩 실패")
            return

        direction, debug_frame = self.detector.detect_direction(frame)

        # 방향 publish
        s = String()
        s.data = direction
        self.pub_direction.publish(s)

        # 디버그 이미지 publish
        debug_msg = CompressedImage()
        debug_msg.header = msg.header
        debug_msg.format = "jpeg"
        debug_msg.data = np.array(
            cv2.imencode('.jpg', debug_frame)[1]
        ).tobytes()
        self.pub_debug.publish(debug_msg)

        # 필요하면 1초에 1번 정도 로그
        rospy.loginfo_throttle(1.0, "arrow_direction: %s", direction)


if __name__ == '__main__':
    try:
        node = ArrowDetectorNode()
        rospy.loginfo("Arrow detector node started.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
