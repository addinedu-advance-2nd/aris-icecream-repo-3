import sys, os
import cv2
import threading

import rclpy as rp
from rclpy.node import Node
from bartendroid_msgs.srv import CameraService

from ultralytics import YOLO  
import numpy as np

''' 
Server
카메라(1) 노드
1. 아루코마커
    서비스 이름 : /info_camera1
    서비스 타입 : CameraService,
    서비스 데이터 : is_empty


2. 봉인씰
    서비스 이름 : /info_camera1
    서비스 타입 : CameraService, 
    서비스 데이터 : is_seal

    
3.  Request :
    Response :
'''

# 카메라 실행
# 아루코마커, 봉인씰 감지
class Video_Thread(threading.Thread):
    def __init__(self, camera_index, node, yolo_model_path):
        super().__init__()
        self.camera_index = camera_index
        self.node = node
        self.capture = cv2.VideoCapture(camera_index)

        # seal
        self.model = YOLO(yolo_model_path)
        self.zone_top_left = (80, 80)  # 왼쪽 상단 좌표 (x1, y1)
        self.zone_bottom_right = (500, 450)  # 오른쪽 하단 좌표 (x2, y2)


        if not self.capture.isOpened():
            self.node.get_logger().error(f"카메라 {camera_index}를 열 수 없습니다.")
            raise Exception(f"카메라 {camera_index}를 열 수 없습니다.")
        
        self.running = True
        self.markerDictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        #self.arucoParameter = cv2.aruco.DetectorParameters_create()
        self.arucoParameter = cv2.aruco.DetectorParameters()

        self.is_empty = [False, False, False]
        self.is_seal = True



    # 아루코마커 상태
    def run(self):
        i = 0
        self.node.get_logger().info("서버가 실행되고 있습니다. 비디오 스트림을 시작합니다.")
        while self.running:
            
            i += 1
            ret, frame = self.capture.read()
            if ret:
                # 아루코마커 준비
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.markerDictionary, parameters=self.arucoParameter)

                # 기본값으로 초기화
                self.is_empty = [False, False, False]   
                if ids is not None:
                    self.marker_ids = [int(id[0]) for id in ids]
                    cv2.aruco.drawDetectedMarkers(frame, corners)

                    for marker_id in self.marker_ids:
                        if marker_id < 3:
                            self.is_empty[marker_id] = True

                #     print(self.marker_ids)
                # else:
                #     print("감지된 ArUco 마커가 없습니다.")
                   
                # seal 준비
                frame2 = np.zeros_like(frame)
                cv2.convertScaleAbs(frame, frame2, 1.0, -50)

                # conf=0.8 설정으로 불필요한 검출 필터링
                detect_params = self.model.predict(source=[frame2], conf=0.7, save=False, verbose=False)
                DP = detect_params[0]
                annotated_frame = DP.plot()
                # 기본값으로 초기화
                self.is_seal = False  

                # 프레임 복사본 생성 (결과를 표시할 용도)
            

                for result in DP.boxes.data.tolist():  # 탐지된 결과를 순회
                    x1, y1, x2, y2, conf, cls = result  # 바운딩 박스 좌표와 다른 정보
                    cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                    if conf > 0.7:
                        self.is_seal = True  
                    else: 
                        self.is_seal = False

            print(f'Detection Result : is_empty : {self.is_empty}, is_seal : {self.is_seal}')

            cv2.imshow('YOLOv8 Ice Cream Detection and aruco marker', annotated_frame)              
            # cv2.imshow('Camera Server', frame)
            cv2.waitKey(1)

    def __del__(self):
        self.running = False
        self.capture.release()
        cv2.destroyAllWindows()

    def get_information(self):
        return self.is_empty, self.is_seal


class Server(Node):
    def __init__(self, yolo_model_path):
        super().__init__('camera_server')

        self.video_thread = Video_Thread(2, self, yolo_model_path)
        self.video_thread.start()

        self.server = self.create_service(
            CameraService, 
            '/info_camera1', 
            self.callback_service
        )
    
    def callback_service(self, request, response):
        response.is_empty, response.is_seal = self.video_thread.get_information()
        print(f'Server send : {response.is_empty} and {response.is_seal}')
        return response


def main(args=None):
    rp.init(args=args)

    directories = os.getcwd().split('/')
    if 'aris-repo-3' in directories:
        model_path = ''
        for directory in directories:
            model_path += directory + '/'
            if directory == 'aris-repo-3':
                break
        model_path += 'pretrained_yolo_model/model_seal/best.pt'

        server_node = Server(model_path)
        rp.spin(server_node)
    
        server_node.destroy_node()
    
    rp.shutdown()

if __name__ == '__main__':
    main()