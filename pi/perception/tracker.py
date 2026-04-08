import cv2
import numpy as np


class ColorTracker:
    def __init__(self):
        # Red color range (HSV)
        self.lower1 = np.array([0, 80, 50])
        self.upper1 = np.array([15, 255, 255])

        self.lower2 = np.array([165, 80, 50])
        self.upper2 = np.array([180, 255, 255])

    def track(self, frame):
        if frame is None:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self.lower1, self.upper1)
        mask2 = cv2.inRange(hsv, self.lower2, self.upper2)

        mask = mask1 + mask2

        # Clean noise
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Largest object
        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) < 150:
            return None

        x, y, w, h = cv2.boundingRect(largest)

        cx = x + w // 2
        cy = y + h // 2

        return {
            "center": (cx, cy),
            "bbox": (x, y, w, h),
            "area": cv2.contourArea(largest),
        }
