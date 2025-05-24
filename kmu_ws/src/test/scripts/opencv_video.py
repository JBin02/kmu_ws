#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import cv2
import numpy as np

video_file = cv2.VideoCapture("highway.mp4")

while True:
    _, img = video_file.read()
    y, x, channel = img.shape
    margin_x1 = 50
    margin_x2 = 220
    margin_y = 270
    src1 = (margin_x1, y)
    src2 = (margin_x2, margin_y)
    src3 = (x - margin_x2, margin_y)
    src4 = (x - margin_x1, y)
    srcs = np.float32([src1, src2, src3, src4])
    dst1 = (80, y)
    dst2 = (80, 0)
    dst3 = (x - 80, 0)
    dst4 = (x - 80, y)
    dsts = np.float32([dst1, dst2, dst3, dst4])
    matrix = cv2.getPerspectiveTransform(srcs, dsts)
    warp_img = cv2.warpPerspective(img, matrix, [x, y])
    cv2.imshow("img", img)
    cv2.imshow("warp_img", warp_img)
    cv2.waitKey(30)
