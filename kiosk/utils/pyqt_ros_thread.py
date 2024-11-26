import sys, time

from PyQt5.QtCore import *

import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from bartendroid_msgs.srv import CameraService
from bartendroid_msgs.action import OrderInformation2

class RosThread(QThread):
    def __init__(self):
        super().__init__()
        self.executor = MultiThreadedExecutor()

        self.kiosk_node = Kiosk_Node()
        self.executor.add_node(self.kiosk_node)

    def run(self):
        print('Start Ros2 executor thread.')
        self.executor.spin()

    def __del__(self):
        self.executor.shutdown()

class Kiosk_Node(Node, QObject):
    camera_1_response_received = pyqtSignal(list)
    master_feedback_received = pyqtSignal(int)
    master_result_received = pyqtSignal(bool)

    def __init__(self):
        Node.__init__(self, 'kiosk_node')
        QObject.__init__(self)

        self.camera_1_service_client = self.create_client(
            CameraService, 
            '/info_camera1'
        )

        while not self.camera_1_service_client.wait_for_service(1.0):
            print('카메라 서버 생성을 기다리는 중...')
        print('카메라 서버 확인 완료')

        self.master_action_client = ActionClient(
            self, 
            OrderInformation2, 
            '/order_info2'
        )

        while not self.master_action_client.wait_for_server(1.0):
            print('마스터 서버 생성을 기다리는 중...')
        print('마스터 서버 확인 완료')

    def request_empty_position(self):
        # 카메라 1 서버에게 아루코 마커 위치를 요청하는 함수
        request_msg = CameraService.Request()
        future = self.camera_1_service_client.call_async(request_msg)
        future.add_done_callback(self.camera_service_1_response_callback)

    def camera_service_1_response_callback(self, future):
        # 카메라 1 서버가 응답하는 경우 이를 PyQt로 전송하는 콜백 함수
        self.camera_1_response_received.emit(future.result().is_empty)

    def send_order_goal(self, icecream_position, topping_position):
        # 마스터 액션 서버에게 주문 정보를 전달하는 함수
        order_goal_msg = OrderInformation2.Goal()
        order_goal_msg.icecream_position = icecream_position
        order_goal_msg.topping_position = topping_position

        self.master_action_client.wait_for_server()

        self.order_goal_future = self.master_action_client.send_goal_async(
            order_goal_msg, 
            feedback_callback=self.master_feedback_callback
        )

        self.order_goal_future.add_done_callback(self.master_result_callback)

    def master_feedback_callback(self, feedback_msg):
        # 마스터 액션 서버에게서 피드백(진행률)을 PyQt로 전송하는 콜백 함수
        progress_rate = feedback_msg.feedback.phase
        self.master_feedback_received.emit(progress_rate)

    def master_result_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('주문 정보 전송에 실패했습니다.')
            return
        print('주문 정보 전송에 성공했습니다.')

        self.order_goal_future = goal_handle.get_result_async()
        self.order_goal_future.add_done_callback(self.send_order_result)

    def send_order_result(self, future):
        result = future.result()
        print(f'응답 {result.status}으로 제조 과정이 완료되었습니다.')
        self.master_result_received.emit(result.result.is_complete)