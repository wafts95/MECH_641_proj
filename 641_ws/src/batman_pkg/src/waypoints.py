import cv2
import numpy as np
import os

image = cv2.imread(os.path.join("/home/robotics/MECH_641_proj/641_ws/src/batman_pkg/media","batman.jpg"), 1);
grey_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

mean = np.mean(grey_img)
min_threshold = 0.66 * mean
max_threshold = 1.33 * mean
edges = cv2.Canny(grey_img, min_threshold, max_threshold)

img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

contours, _= cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(len(contours[0])) # or 1
cv2.drawContours(img, contours, -1, (0, 255, 0), 1)

cv2.namedWindow('contours')
cv2.moveWindow('contours', 40, 30)
cv2.imshow('contours', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
