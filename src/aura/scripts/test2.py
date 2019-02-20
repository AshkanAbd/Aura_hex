#!/usr/bin/env python2

import numpy as np
import cv2 as cv
import colorsys

cap = cv.VideoCapture(0)
while True:
    _, frame = cap.read()
    frame = cv.flip(frame, 1)
    frame = cv.GaussianBlur(frame, (5, 5), -1)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, (110, 100, 130), (130, 255, 255))
    edge = cv.Canny(mask, 100, 200)
    _, contours, _ = cv.findContours(edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        epsilon = 0.001 * cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, epsilon, True)
        x, y, h, w = cv.boundingRect(approx)
        if h * w < 1000: continue;
        # Founded
        if len(approx) >= 10:
            cv.drawContours(frame, [approx], -1, (0, 0, 255), 2)
            break
    cv.imshow("H detection", frame)
    if cv.waitKey(1) == 27: break;
