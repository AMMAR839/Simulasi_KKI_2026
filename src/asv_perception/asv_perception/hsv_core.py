from collections import defaultdict
from dataclasses import dataclass

import cv2
import numpy as np


HSV_RANGES = {
    "red": (
        (np.array([0, 90, 65]), np.array([12, 255, 255])),
        (np.array([168, 90, 65]), np.array([179, 255, 255])),
    ),
    "green": ((np.array([35, 65, 55]), np.array([90, 255, 255])),),
    "blue": ((np.array([90, 70, 45]), np.array([140, 255, 255])),),
}


@dataclass(frozen=True)
class HsvCandidate:
    class_id: str
    score: float
    x: float
    y: float
    width: float
    height: float
    area: float
    solidity: float


class StableHsvDetector:
    def __init__(
        self,
        stable_frames=5,
        min_area_px=120.0,
        box_area_ratio=0.008,
        max_width=640,
    ):
        self.stable_frames = max(1, int(stable_frames))
        self.min_area_px = max(1.0, float(min_area_px))
        self.box_area_ratio = max(0.0001, float(box_area_ratio))
        self.max_width = max(64, int(max_width))
        self._streaks = defaultdict(int)

    def detect(self, bgr_image, camera_name):
        if bgr_image is None or bgr_image.size == 0:
            self._update_streaks(set())
            return []

        original_h, original_w = bgr_image.shape[:2]
        scale = min(1.0, self.max_width / float(max(original_w, 1)))
        if scale < 1.0:
            image = cv2.resize(
                bgr_image,
                (int(original_w * scale), int(original_h * scale)),
                interpolation=cv2.INTER_AREA,
            )
        else:
            image = bgr_image

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image_area = float(image.shape[0] * image.shape[1])
        kernel = np.ones((5, 5), dtype=np.uint8)
        candidates = []

        for color, ranges in HSV_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < self.min_area_px:
                    continue
                hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
                solidity = area / max(hull_area, 1.0)
                if solidity < 0.55:
                    continue
                x, y, width, height = cv2.boundingRect(contour)
                class_id = self._classify(
                    color, camera_name, area / max(image_area, 1.0)
                )
                if class_id is None:
                    continue
                area_score = min(1.0, area / max(image_area * 0.03, 1.0))
                score = min(0.99, 0.45 + 0.35 * solidity + 0.20 * area_score)
                inv_scale = 1.0 / scale
                candidates.append(
                    HsvCandidate(
                        class_id=class_id,
                        score=score,
                        x=(x + width * 0.5) * inv_scale,
                        y=(y + height * 0.5) * inv_scale,
                        width=width * inv_scale,
                        height=height * inv_scale,
                        area=area * inv_scale * inv_scale,
                        solidity=solidity,
                    )
                )

        best_by_class = {}
        for candidate in candidates:
            current = best_by_class.get(candidate.class_id)
            if current is None or candidate.area > current.area:
                best_by_class[candidate.class_id] = candidate

        present = set(best_by_class)
        self._update_streaks(present)
        return [
            candidate
            for class_id, candidate in best_by_class.items()
            if self._streaks[class_id] >= self.stable_frames
        ]

    def _classify(self, color, camera_name, area_ratio):
        if color == "red":
            return "buoy_red"
        if camera_name == "down":
            return "underwater_box" if color in ("blue", "green") else None
        is_box = area_ratio >= self.box_area_ratio
        if color == "green":
            return "surface_box" if is_box else "buoy_green"
        if color == "blue":
            # The water surface often forms one very large blue contour in the
            # front camera. Underwater targets are validated by the side/down
            # camera, while compact front-camera blue contours are dock buoys.
            return "buoy_blue" if area_ratio <= 0.25 else None
        return None

    def _update_streaks(self, present):
        for class_id in list(self._streaks):
            if class_id not in present:
                self._streaks[class_id] = 0
        for class_id in present:
            self._streaks[class_id] += 1
