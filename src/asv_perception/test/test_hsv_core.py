import cv2
import numpy as np

from asv_perception.hsv_core import StableHsvDetector


def test_requires_stable_frames_before_publishing():
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (220, 100), (420, 300), (0, 255, 0), -1)
    detector = StableHsvDetector(stable_frames=5)

    for _ in range(4):
        assert detector.detect(image, "front") == []

    detections = detector.detect(image, "front")
    assert len(detections) == 1
    assert detections[0].class_id == "surface_box"
    assert detections[0].score > 0.7


def test_small_red_blob_is_buoy_and_noise_is_rejected():
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.circle(image, (320, 180), 18, (0, 0, 255), -1)
    image[10:12, 10:12] = (0, 255, 0)
    detector = StableHsvDetector(stable_frames=1, min_area_px=100)

    detections = detector.detect(image, "front")
    assert [item.class_id for item in detections] == ["buoy_red"]


def test_missing_frame_resets_stability():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (80, 60), (240, 200), (255, 0, 0), -1)
    detector = StableHsvDetector(stable_frames=2, min_area_px=80)

    assert detector.detect(image, "down") == []
    assert detector.detect(np.zeros_like(image), "down") == []
    assert detector.detect(image, "down") == []
    assert detector.detect(image, "down")[0].class_id == "underwater_box"


def test_large_blue_front_background_is_not_an_underwater_box():
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    image[120:, :] = (255, 80, 0)
    detector = StableHsvDetector(stable_frames=1, min_area_px=100)

    assert detector.detect(image, "front") == []
