import cv2
import numpy as np

# Tạo điểm và đa giác
point = [0.5, 0.5]
pts = np.array([
    [0, 0],
    [1, 0],
    [2, 0],
    [2, 1],
    [3, 1],
    [3, 2],
    [2, 2],
    [1, 2],
    [1, 1],
    [0, 1]
])

contour = cv2.approxPolyDP(pts, epsilon=0.1, closed=True)

distance = cv2.pointPolygonTest(contour, point, True)


if distance > 0:
    print("Điểm nằm trong đa giác")
elif distance < 0:
    print("Điểm nằm ngoài đa giác")
else:
    print("Điểm nằm trên cạnh đa giác")