#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import track
import detect

global theta_div
theta_div = 0

def theta_norm():
    return theta_div

def main():
    global  theta_div
    cap = cv2.VideoCapture(0)
    ticks = 0
    lt = track.LaneTracker(2, 0.1, 150)
    ld = detect.LaneDetector(120)
    while cap.isOpened():
        prec_tick = ticks
        ticks = cv2.getTickCount()
        dt = (ticks - prec_tick) / cv2.getTickFrequency()

        ret, frame = cap.read()

        predicted = lt.predict(dt)

        lanes = ld.detect(frame)

        if predicted is not None:
            cv2.line(frame, (predicted[0][0], predicted[0][1]), (predicted[0][2], predicted[0][3]), (0, 0, 255), 5)
            cv2.line(frame, (predicted[1][0], predicted[1][1]), (predicted[1][2], predicted[1][3]), (0, 0, 255), 5)
            theta1 = np.arctan2((predicted[0][3] - predicted[0][1]), (predicted[0][2] - predicted[0][0]))
            theta2 = np.arctan2((predicted[1][3] - predicted[1][1]), (predicted[1][2] - predicted[1][0]))
            div = (np.pi - np.abs(theta1) - np.abs(theta2)) / 2
            theta_div = (np.pi - np.abs(theta1) - div) - np.pi/2
        lt.update(lanes)
        cv2.imshow('', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

main()