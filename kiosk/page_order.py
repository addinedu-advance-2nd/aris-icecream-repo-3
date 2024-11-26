import sys

import rclpy as rp
from rclpy.node import Node

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

from utils.pyqt_ros_thread import RosThread

form_order_page = uic.loadUiType('kiosk/UI/page_order.ui')[0]
form_menu_position = uic.loadUiType('kiosk/UI/widgets/widget_order_menu_position.ui')[0]

# TODO : 토핑 이름 정보를 코드로 바꾸는 내용이 필요함.
topping_name_to_idx = {'죠리퐁':0, '코코볼':1, '해바라기씨':2}

class Popup_Order(QDialog, form_order_page):
    def __init__(self, ros_thread, order_information, available_positions):
        super().__init__()
        self.setupUi(self)
        # 윈도우 기본 프레임 제거
        self.setWindowFlag(Qt.WindowType.FramelessWindowHint) 

        self.event_loop = QEventLoop()

        self.num_total_order = 0
        self.num_success_order = 0

        self.ros_thread = ros_thread
        self.reconnect_ros_events()

        self.menu_widget_list = []
        self.is_all_order_complete = False 
        self.load_order_info(order_information, available_positions)

        self.btn_make.clicked.connect(self.send_order_information)

    def reconnect_ros_events(self):
        # self.ros_thread.kiosk_node.camera_1_response_received.disconnect_all()
        # self.ros_thread.kiosk_node.master_feedback_received.disconnect_all()
        # self.ros_thread.kiosk_node.master_result_received.disconnect()
        self.ros_thread.kiosk_node.camera_1_response_received.connect(self.get_available_seat)
        self.ros_thread.kiosk_node.master_feedback_received.connect(self.update_progressbar)
        self.ros_thread.kiosk_node.master_result_received.connect(self.goto_next_order)

    def load_order_info(self, order_information, available_positions):
        self.scroll_menu.setLayout(QVBoxLayout(self.scroll_menu))
        layout = self.scroll_menu.layout()

        for i, info in enumerate(order_information):
            widget = Widget_Menu_Position(info['name'], info['topping'], str(available_positions[i]))
            layout.addWidget(widget)
            self.menu_widget_list.append(widget)
            self.num_total_order += 1
        
        layout.addStretch(1)
        print(layout.count())

    def get_unfinished_order(self):
        result = []
        for menu_widget in self.menu_widget_list:
            if menu_widget.progressbar.value() == 0:
                result.append(menu_widget)
        return result
    
    def get_available_seat(self, is_empty_pos):
        self.is_available_seat = is_empty_pos
        self.event_loop.quit()

    def send_order_information(self):
        if self.num_success_order == self.num_total_order:
            return

        # if self.ros_thread.kiosk_node.is_master_busy:
        #     QMessageBox.information(self, 'Busy', '이전 작업을 마무리 하는 중입니다. 잠시만 기다려주세요.')

        self.ros_thread.kiosk_node.request_empty_position()
        self.event_loop.exec_()

        used_positions = [int(widget.label_position.text()) for widget in self.get_unfinished_order()]
        for pos in used_positions:
            if self.is_available_seat[pos]:
                QMessageBox.information(self, 'Wrong Position', '아이스크림이 올바른 위치에 놓이지 않으면 선택하신 토핑과 달라질 수 있어요.')
                return
        
        self.btn_make.clicked.disconnect(self.send_order_information)
        for menu_widget in self.get_unfinished_order():
            self.active_widget = menu_widget
            
            self.ros_thread.kiosk_node.send_order_goal(int(menu_widget.label_position.text()), topping_name_to_idx[menu_widget.label_topping_name.text()])
            print(f'{menu_widget.label_position.text()} 자리의 아이스크림에 {menu_widget.label_topping_name.text()} 토핑을 올려달라는 요청을 전송합니다.')

            self.event_loop.exec_()
        self.btn_make.clicked.connect(self.send_order_information)

        if self.num_success_order == self.num_total_order:
            self.btn_make.setText('완료')
            QMessageBox.information(self, '주문 완료 메세지', '주문이 모두 완료되었습니다.')

            self.ros_thread.kiosk_node.camera_1_response_received.disconnect()
            self.ros_thread.kiosk_node.master_feedback_received.disconnect()
            self.ros_thread.kiosk_node.master_result_received.disconnect()

            self.ros_thread.kiosk_node.send_order_goal(3, 3)
            self.close()
        else:
            QMessageBox.information(self, '주문 안내 메세지', '씰이 제거되지 않아 만들지 못한 아이스크림이 있습니다\n씰을 제거하고 다시 올려주신 뒤 버튼을 눌러주세요.')
            self.btn_make.setText('다시 제조하기')

    def update_progressbar(self, rate):
        self.active_widget.progressbar.setValue(rate)

    def goto_next_order(self, is_complete):
        print(f'주문 완료 상태 : {is_complete}')
        if not self.is_all_order_complete:
            if not is_complete:
                self.active_widget.progressbar.setValue(0)
            else:
                self.num_success_order += 1

            self.event_loop.quit()
        

class Widget_Menu_Position(QWidget, form_menu_position):
    def __init__(self, menu_name, topping_name, position):
        super().__init__()
        self.setupUi(self)

        self.label_menu_name.setText(menu_name)
        self.label_topping_name.setText(topping_name)
        self.label_position.setText(position)
        self.state = 0

        self.progressbar.setRange(0, 100)
        self.progressbar.setValue(0)

if __name__ == '__main__':
    rp.init()
    app = QApplication(sys.argv)

    sample_order1 = [
        {'name':'나주배 소르베', 'topping': '바닐라', 'price': 5000, 'count': 1}, 
        # {'name':'아몬드 봉봉', 'topping': '바닐라', 'price': 2000, 'count': 1}, 
        # {'name':'피스타치오 아몬드', 'topping': '레인보우', 'price': 3000, 'count': 1}, 
    ]
    positions = [0, 1, 2]

    ros_thread = RosThread()
    ros_thread.start()

    myWindow = Popup_Order(ros_thread, sample_order1, positions)
    myWindow.show()
    app.exec_() 

    rp.shutdown()
