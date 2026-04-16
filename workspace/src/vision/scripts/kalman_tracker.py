import cv2
import numpy as np

class KalmanTracker:
    def __init__(self):
        self.kalman = cv2.KalmanFilter(4, 2)  # estado (x, y, vx, vy), medición (x, y)
        self.kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]], dtype=np.float32)
        self.kalman.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], dtype=np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.05
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
        self.initialized = False
        self.lost_counter = 0
        self.max_lost = 10
        
    def update(self, detected_center=None):
            if not self.initialized and detected_center is not None:
                self.kalman.statePre = np.array([[detected_center[0]], [detected_center[1]], [0], [0]], dtype=np.float32)
                self.kalman.statePost = np.array([[detected_center[0]], [detected_center[1]], [0], [0]], dtype=np.float32)
                self.initialized = True
                self.lost_counter = 0
                return detected_center

            if not self.initialized:
                return None

            predicted = self.kalman.predict()
            pred_center = (int(predicted[0]), int(predicted[1]))

            if detected_center is not None:
                measurement = np.array([[detected_center[0]], [detected_center[1]]], dtype=np.float32)
                corrected = self.kalman.correct(measurement)
                self.lost_counter = 0
                return (int(corrected[0]), int(corrected[1]))
            else:
                self.lost_counter += 1
                if self.lost_counter > self.max_lost:
                    self.initialized = False
                    return None
                return pred_center