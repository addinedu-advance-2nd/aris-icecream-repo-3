from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import *
import cv2
from mtcnn.mtcnn import MTCNN

class FaceDetectionThread(QThread):
    face_detected = pyqtSignal()
    no_face_detected = pyqtSignal()
    camera_error = pyqtSignal(str)

    def __init__(self, detector, capture):
        super().__init__()
        self.detector = detector
        self.capture = capture
        self.running = True

    def run(self):
        no_face_time = 0
        while self.running:
            ret, frame = self.capture.read()
            if not ret:
                self.camera_error.emit('카메라 오류가 발생했습니다.')
                return

            results = self.detector.detect_faces(frame)
            if len(results) > 0:
                no_face_time = 0
                print("얼굴이 감지되었습니다.")
            else:
                no_face_time += 1
                print("얼굴이 감지되지 않았습니다.")
                if no_face_time >= 15:
                    self.no_face_detected.emit()
                    break

            QThread.msleep(1000)  # 1초 대기

    def stop(self):
        self.running = False
        self.quit()
        self.wait()
