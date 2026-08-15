import cv2
import numpy as np

from handeye_calibration_ros.charuco_detector import board_image_score


def test_large_sharp_board_scores_higher_than_small_blurry_board():
    sharp = np.zeros((720, 1280, 3), dtype=np.uint8)
    for x in range(0, sharp.shape[1], 16):
        sharp[:, x:x + 8] = 255
    xs = np.linspace(200, 900, 8)
    ys = np.linspace(120, 570, 5)
    large_corners = np.array(
        [(x, y) for y in ys for x in xs], dtype=np.float32).reshape(-1, 1, 2)
    blurry = np.full((720, 1280, 3), 127, dtype=np.uint8)
    small_corners = np.array(
        [(x, y) for y in np.linspace(320, 370, 5)
         for x in np.linspace(600, 680, 8)],
        dtype=np.float32).reshape(-1, 1, 2)
    sharp_score = board_image_score(sharp, large_corners, 40)['score']
    blurry_score = board_image_score(blurry, small_corners, 40)['score']
    assert sharp_score > 35.0
    assert blurry_score < 25.0
    assert sharp_score > blurry_score
