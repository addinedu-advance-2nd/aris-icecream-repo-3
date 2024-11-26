import sys, time, signal
from collections import deque

import rclpy as rp
from rclpy.action import ActionServer
from rclpy.node import Node
from bartendroid_msgs.action import OrderInformation2
from rclpy.executors import MultiThreadedExecutor
from bartendroid_msgs.srv import Input, Noninput, OrderInformation, CameraService
from std_msgs.msg import Bool
import traceback, threading
from tqdm import tqdm
import queue
'''
행동들로 꽉 찬 마스터 노드
 
Action Server
키오스크의 액션을 받는다

Subscriber
카메라(2) 퍼블리시를 받는다
'''


class SubscriberNode(Node):
    def __init__(self, q):
        super().__init__('collision_subscriber')
        self.sub = self.create_subscription(
            Bool,
            'collision_status',
            self.sub_callback,
            10
        )
        self.q = q
    def sub_callback(self,msg):
        if self.q.full():
            self.q.get()

        self.q.put(msg.data)  
        # self.get_logger().info(f'Collision status: {msg.data}')

class Master_Node(Node):
    def __init__(self, q):
        super().__init__('robot_node')

        self.q = q


        #Action 서버 생성(키오스크에서 n자리 아이스크림에 n토핑 정보 받아옴)
        self.action_server = ActionServer(
            self, 
            OrderInformation2, 
            '/order_info2', 
            self.execute_callback
        )
        # camer 1 서비스 클라이언트 생성(아루코마커 빈자리, 실 존재 확인)
        self.camera_1_service_client = self.create_client(
            CameraService, 
            '/info_camera1'
        )
        self.timer = self.create_timer(2.0, self.timer_callback)


        self.is_empty = None
        self.need_to_clean = False

    # 액션 서버 콜백 함수
    async def execute_callback(self, goal_handle):
        feedback_msg = OrderInformation2.Feedback()

        self.topping_position = goal_handle.request.topping_position
        self.icecream_position = goal_handle.request.icecream_position

        if self.topping_position == 3 and self.icecream_position == 3:
            print('하나의 주문을 완료했습니다. 청소를 해도 좋습니다.')
            result = OrderInformation2.Result()
            result.is_complete = True
            self.need_to_clean = True

            return result
        
        print(f'주문 접수 : 아이스크림 {self.icecream_position} 자리를 토핑{self.topping_position} 위치로')
        # 아이스크림 제조 시작
        flagstart = threading.Thread(target=self.flag)
        flagstart.start()
        self.home_L()
        feedback_msg.phase = 10
        goal_handle.publish_feedback(feedback_msg)
        self.motion_grab_capsule_1()
        feedback_msg.phase = 20
        goal_handle.publish_feedback(feedback_msg)
        self.motion_seal()
        feedback_msg.phase = 30
        goal_handle.publish_feedback(feedback_msg)
        # 비동기 서비스 호출
        seal_result = await self.send_request()
        # 봉인 씰이 존재했을 때
        if seal_result:
            self.motion_seal_2()
            result = OrderInformation2.Result()
            result.is_complete = False
            self._arm.stop_lite6_gripper()
            print('아이스크림 제조 실패')
            return result
        # 봉인 씰이 존재하지 않았을 때
        self.motion_grab_capsule_2()
        feedback_msg.phase = 40
        goal_handle.publish_feedback(feedback_msg)
        self.motion_place_capsule()
        feedback_msg.phase = 50
        goal_handle.publish_feedback(feedback_msg)
        self.motion_grab_cup()
        feedback_msg.phase = 60
        goal_handle.publish_feedback(feedback_msg)
        self.motion_topping()
        feedback_msg.phase = 70
        goal_handle.publish_feedback(feedback_msg)
        self.motion_make_icecream()
        feedback_msg.phase = 80
        goal_handle.publish_feedback(feedback_msg)
        self.serving()
        feedback_msg.phase = 90
        goal_handle.publish_feedback(feedback_msg)
        self.throw_away_trash()
        feedback_msg.phase = 100
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1)
        self._arm.stop_lite6_gripper()


        goal_handle.succeed()
        print('아이스크림 제조 완료')
        result = OrderInformation2.Result()
        result.is_complete = True
        return result


    async def send_request(self):

        request = CameraService.Request()
        self.get_logger().info('카메라 서비스 요청 전송...')
        future = self.camera_1_service_client.call_async(request)

        try:
            response = await future
            self.is_empty = response.is_empty
            self.is_seal = response.is_seal
            self.get_logger().info(f'응답 받음 - ArUco 상태: {self.is_empty}, 씰 상태: {self.is_seal}')
            return self.is_seal
        except Exception as e:
            self.get_logger().error(f'서비스 호출 실패: {str(e)}')
            return False

    
    def timer_callback(self):
        if self.need_to_clean:
            self.request_empty_position()


    def request_empty_position(self):
        request_msg = CameraService.Request()
        self.future = self.camera_1_service_client.call_async(request_msg)
        self.future.add_done_callback(self.camera_service_1_response_callback)

    
    def camera_service_1_response_callback(self, future):
        if all(future.result().is_empty):
            self.clear_tray()
    # 청소 모션
    def clear_tray(self):
        print('[----------------------------- 청소 중 -----------------------------]')
        self.need_to_clean = False
        print('청소 완료')

    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)


    

    def flag(self):
        state_check = False
        print("flag check")
        count = 0
        state_check = self.q.get()
        while True:
            # 정지 
            if  state_check == True:
                print("flag stop")
                self._arm.set_state(3)
                pass

            # 진행
            elif state_check is False:
                count += 1
                # False가 5회 이상 감지되면 종료
                if count >= 12:
                    #print("Confirmed flag pass")
                    self._arm.set_state(0)  
            
            state_check = self.q.get()
            # print(state_check)
            time.sleep(0.1)

# 로봇 동작 관련 코드는 보안 문제로 인해 모두 제거했습니다.
    def home_L(self):

        print('홈 포지션 동작')
        
    def motion_grab_capsule_1(self):

        print('캡슐 잡는 동작')

    def motion_seal(self):

        print('씰 확인 동작')
            

   # seal이 True일때 제자리에 가져다놓기
    def motion_seal_2(self):
        
        print('캡슐 원위치 동작')


    # seal이 False일때 동작
    def motion_grab_capsule_2(self):

        print('캡슐 잡는 동작')



       
    def motion_place_capsule(self):

        print("캡슐 프레스에 위치 동작")
        


    def motion_grab_cup(self):

        print("아이스크림 컵 잡는 동작")


    def motion_topping(self):

        print("토핑 받는 동작")



    def motion_make_icecream(self):

        print("아이스크림 받는 동작")


    def serving(self):

        print("서빙 하는 동작")
        

 
        
    def throw_away_trash(self):

            print("쓰래기 버리는 동작")



def main(args=None):
    rp.init(args=args)
    q = queue.Queue(maxsize=3)
    master_node = Master_Node(q)
    sub_node = SubscriberNode(q)
    executor = MultiThreadedExecutor()
    executor.add_node(master_node)
    executor.add_node(sub_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        master_node.destroy_node()
        sub_node.destroy_node()
        rp.shutdown()
        

if __name__ == '__main__':
    main()