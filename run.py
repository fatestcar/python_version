import threading

import cv2
import numpy as np

import detect
import track
from PIDController import PIDController

global pos
lock = threading.Lock()


def camera_thread():
    global pos
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
            lock.acquire()
            try:
                pos = ((np.pi - np.abs(theta1) - div) - np.pi / 2) * 50 / np.pi
            finally:
                lock.release()
        lt.update(lanes)
        cv2.imshow('', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def pid_thread():
    global pos
    controller = PIDController()
    while True:
        lock.acquire()
        try:
            controller.controll(pos)
        finally:
            lock.release()


def main():
    global pos
    pos = 0
    # t1 = threading.Thread(target=camera_thread)
    t2 = threading.Thread(target=pid_thread)
    # t1.start()
    t2.start()
    # t1.join()
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
            lock.acquire()
            try:
                pos = ((np.pi - np.abs(theta1) - div) - np.pi / 2) * 50 / np.pi
            finally:
                lock.release()
        lt.update(lanes)
        cv2.imshow('', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    t2.join()


main()
