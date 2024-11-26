import sys, os
import cv2
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading
import time

class DetectHand(threading.Thread):
    def __init__(self, node, yolo_model_path):
        super().__init__()
        self.node = node
        self.model = YOLO(yolo_model_path)
        self.cap = cv2.VideoCapture(4)
        self.zone_top_left = (70, 140)  # 왼쪽 상단 좌표 (x1, y1)
        self.zone_bottom_right = (540, 450)  # 오른쪽 하단 좌표 (x2, y2)
        self.running = True

    def run(self):
        last_publish_time = time.time()  # 마지막 퍼블리시 시간 초기화
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            detect_params = self.model.predict(source=[frame], conf=0.8, save=False, verbose=False)
            DP = detect_params[0]
            annotated_frame = DP.plot()  # 결과를 프레임에 그리기

            # 특정 구역 그리기
            cv2.rectangle(annotated_frame, self.zone_top_left, self.zone_bottom_right, (0, 255, 0), 2)


            check = False  # 체크 값을 초기화
            for result in DP.boxes.data.tolist():  # 탐지된 결과를 순회
                x1, y1, x2, y2, conf, cls = result  # 바운딩 박스 좌표와 다른 정보
                cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)

                # 사람의 위치가 특정 구역과 닿는지 확인
                if (x1 < self.zone_bottom_right[0] and x2 > self.zone_top_left[0] and
                    y1 < self.zone_bottom_right[1] and y2 > self.zone_top_left[1]):
                    check = True  # 특정 구역에 사람이 닿음
                    self.node.get_logger().info('객체가 영역 안에 있습니다.')
                    break  # 객체가 구역에 닿았으므로 루프 종료
                else:
                    check = check or False


            # 퍼블리시 주기 조정
            current_time = time.time()
            if current_time - last_publish_time >= 0.1:  # 0.5초 간격
                # 메시지 발행
                msg = Bool()
                msg.data = check
                self.node.collision_publisher.publish(msg)  # 퍼블리셔 이름
                last_publish_time = current_time  # 마지막 퍼블리시 시간 업데이트

            # 결과 프레임 보여주기
            cv2.imshow('collision', annotated_frame)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False

        # 웹캠 해제 및 창 닫기
        self.cap.release()
        cv2.destroyAllWindows()

    def stop(self):
        self.running = False


def main(args=None):
    rclpy.init()
    node = rclpy.create_node('collision_publisher')  # 노드이름
    node.collision_publisher = node.create_publisher(Bool, 'collision_status', 10)  # 퍼블리셔 생성
    
    directories = os.getcwd().split('/')
    if 'aris-repo-3' in directories:
        model_path = ''
        for directory in directories:
            model_path += directory + '/'
            if directory == 'aris-repo-3':
                break
        model_path += 'pretrained_yolo_model/hand/best.pt'

        video_thread = DetectHand(node, model_path)  # 비디오 스트림 처리 스레드 생성
        video_thread.start()  # 비디오 스레드 실행

        try:
            while rclpy.ok():
                rclpy.spin_once(node)
        except KeyboardInterrupt:
            pass
        finally:
            video_thread.stop()  # 비디오 스레드 정지
            video_thread.join()  # 스레드 종료 대기
            node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()