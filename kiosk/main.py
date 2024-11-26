import sys
import pymysql
import cv2
import rclpy as rp

from mtcnn.mtcnn import MTCNN

from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

from page_admin import Popup_Admin
from page_topping import Popup_Topping
from page_order import Popup_Order
from page_recommend import Popup_Recommend

from widget_menu import MenuWidget, EmptyMenuWidget
from widget_cart import CartWidget

from utils.face_detection_thread import FaceDetectionThread
from utils.pyqt_ros_thread import RosThread
from page_recommend import AudioRecorderThread

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

# .ui 파일을 로드하여 MyKiosk 클래스 정의 준비
form_main_page = uic.loadUiType("kiosk/UI/page_main.ui")[0]

STYLE_DEFAULT = '''
background:rgb(255, 255, 255);
border: 2px solid #000000;
border-radius: 10px;
'''

STYLE_SELECT = '''
background:rgb(255, 120, 0);
border: 2px solid #000000;
border-radius: 10px;
'''

# 1. DB연결
connection = pymysql.connect(
    host='localhost',
    user='root',
    password='12345678',
    database='BARTENDROID'
)

# 메인윈도우 - 초기화면 + 메인화면 + 메뉴테이블(9)
class MyKiosk(QMainWindow, form_main_page):
    def __init__(self, conn, ros_thread):
        super().__init__()
        self.setupUi(self)

        # 윈도우 기본 프레임 제거
        # self.setWindowFlag(Qt.WindowType.FramelessWindowHint) 
        self.showMaximized() 
        self.setGeometry(0, 0, 1920, 1080)

        self.stackedWidget.setCurrentWidget(self.page_initial) 
        self.logo_click_count = 0  
        
        
        self.conn = conn
        self.ros_thread = ros_thread
        self.ros_thread.kiosk_node.camera_1_response_received.connect(self.get_available_seat)

        self.event_loop = QEventLoop()

        # 추가 기능 설정
        self.init_face_detectoin()
        self.init_audio_recording()

        # 로고 이미지 설정
        self.set_logo_img()

        # 화면보호기 이미지 설정
        self.set_ad_img()

        # 초기화면 클릭 시 이벤트 연결
        self.page_initial.mousePressEvent = self.go_to_main
        # 로고라벨 클릭시 이벤트 연결
        self.logo_label.mousePressEvent = self.go_to_admin

        # 구매버튼 클릭시 함수실행
        self.buy_btn.clicked.connect(self.go_to_confirm_order)

        # 카테고리 버튼 4개
        self.btn_list = [self.category_btn1, self.category_btn2, self.category_btn3, self.category_btn4]
        for btn in self.btn_list:
            btn.clicked.connect(self.show_menu)

        self.category_btn1.click()

        # 카트위젯 인스턴스
        self.cart_frame = self.findChild(QFrame, "cart_frame")  
        self.cart_area = self.findChild(QScrollArea, "cart_area") 
        cart_frame_layout = QVBoxLayout(self.cart_frame)
        cart_frame_layout.setContentsMargins(0, 0, 0, 0)  
        cart_frame_layout.addWidget(self.cart_area)  
        self.cart_area.setWidgetResizable(True)
        self.cart_area.setMinimumSize(self.cart_frame.size())       
        self.cart_content = QWidget()
        self.cart_layout = QVBoxLayout(self.cart_content)
        self.cart_area.setWidget(self.cart_content)

        self.maximum_cart_num = 3
        self.total_cart_num = 0  
        self.total_price = 0
        self.refresh_total_price()

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Escape:
            self.set_ad_img()

    def get_available_seat(self, is_empty_pos):
        print('[Kiosk thread] 응답 가공')
        self.available_positions = [i for i in range(3) if is_empty_pos[i]]
        self.event_loop.quit()

    def refresh_total_price(self):
        self.label_total_price.setText(f'{self.total_price} 원')

    # 로고 이미지 설정
    def set_logo_img(self):
        self.logo = self.findChild(QLabel, "logo_label")
        pixmap = QPixmap("img/logo.png")  
        # pixmap = pixmap.scaled(self.logo.width(), self.logo.height())
        self.logo.setPixmap(pixmap)
        self.logo.setScaledContents(True)
    
    # 화면보호기 이미지 설정
    def set_ad_img(self):
        self.stackedWidget.setCurrentWidget(self.page_initial) 

        self.ad_img_num = 1 

        self.ad = self.findChild(QLabel, "ad")
        pixmap = QPixmap(f"img/ad{self.ad_img_num}.png")  
        self.ad.setPixmap(pixmap)
        self.ad.setScaledContents(True)

        self.ad_timer = QTimer(self)
        
        self.ad_timer.timeout.connect(self.change_ad_img)
        self.ad_timer.start(3000)

    # 3초마다 화면보호기 이미지 변경
    def change_ad_img(self):
        self.ad_img_num += 1
        if self.ad_img_num > 4:
            self.ad_img_num = 1
        
        pixmap = QPixmap(f"img/ad{self.ad_img_num}.png")  
        self.ad.setPixmap(pixmap)
        self.ad.setScaledContents(True)

    # 메인화면으로 전환
    def go_to_main(self, event):
        print("Initial page clicked") 
        self.stackedWidget.setCurrentWidget(self.page_main)

        # 첫 카테고리 강제 클릭
        self.category_btn1.click()

        # 화면보호기 타이머 중지
        self.ad_timer.stop()

        # 얼굴 탐지 시작
        self.face_detection_thread.start()

    # 로고라벨 클릭체크, 관리자창으로 전환
    def go_to_admin(self, event):
        self.logo_click_count += 1
        print(f"로고클릭 {self.logo_click_count}번")
        
        if self.logo_click_count == 5:
            self.admin_window = Popup_Admin(connection)
            self.admin_window.show()
            print("Admin page clicked") 
            self.logo_click_count = 0       

    def go_to_confirm_order(self):
        # 메인화면 내 장바구니의 주문정보를 받아와 주문확인 페이지로 전달

        # 주문정보 만들기
        order_list = []
        for i in range(self.cart_layout.count()):
            item = self.cart_layout.itemAt(i)
            if item.widget():
                widget = item.widget()
                for _ in range(int(widget.cart_num.text())):
                    order_list.append({
                        'name':widget.cart_name.text(), 
                        'topping':widget.cart_top.text(), 
                        'price':widget.label_price.text(),
                        'num' : widget.cart_num.text()
                    })
        
        # 주문 확인 창 실행
        if len(order_list) > 0:
            self.ros_thread.kiosk_node.request_empty_position()
            print('요청 보내기')
            self.event_loop.exec_()
            print('응답 수신')

            if len(order_list) > len(self.available_positions):
                QMessageBox.information(self, 'Title - No position', '현재 트레이가 비워지지 않았습니다. 잠시 기다려주세요.')
            else:
                self.ros_thread.kiosk_node.camera_1_response_received.disconnect()
                self.confirm_window = Popup_Order(self.ros_thread, order_list, self.available_positions)
                self.confirm_window.exec()

                self.ros_thread.kiosk_node.camera_1_response_received.connect(self.get_available_seat)
                
                # 주문정보 DB 저장
                self.save_order(order_list)
                # 장바구니 초기화
                self.empty_cart()
                self.set_ad_img()
        else:
            QMessageBox.information(self, 'Title - No order', '장바구니가 비어있습니다.')

    def empty_cart(self):
        # 장바구니를 초기화(비움)
        layout = self.cart_layout
        while self.cart_layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
            else:
                layout.removeItem(item)
        
        self.total_cart_num = 0  
        self.total_price = 0
        self.refresh_total_price()

    def reset_category_btn_style(self):
        for btn in self.btn_list:
            btn.setStyleSheet(STYLE_DEFAULT)

    # 메뉴테이블 위젯 전환
    def show_menu(self):
        # 쿼리를 통해 카테고리에 맞는 아이스크림 테이블을 출력
        self.reset_category_btn_style()
        self.sender().setStyleSheet(STYLE_SELECT)
        target_category = self.sender().text()
        

        # 카테고리에 해당하는 데이터 추출
        cursor = self.conn.cursor(pymysql.cursors.DictCursor)
        query_expr = f'SELECT * FROM ICECREAM WHERE CATEGORY = "{target_category}"'
        cursor.execute(query_expr)
        result = cursor.fetchall()

        # 수직 정렬 레이아웃 생성 및 초기화
        if self.widget_list.layout() is None:
            vertical_layout = QVBoxLayout()
            self.widget_list.setLayout(vertical_layout)  # 중복 추가 방지
        else:
            vertical_layout = self.widget_list.layout()
            # 기존 레이아웃 초기화 - 모든 위젯과 하위 레이아웃 삭제
            while vertical_layout.count() > 0:
                item = vertical_layout.takeAt(0)
                
                # 위젯 삭제
                if item.widget() is not None:
                    item.widget().deleteLater()
                    
                # 하위 레이아웃 삭제
                elif item.layout() is not None:
                    sub_layout = item.layout()
                    while sub_layout.count() > 0:
                        sub_item = sub_layout.takeAt(0)
                        if sub_item.widget():
                            sub_item.widget().deleteLater()
                    sub_layout.deleteLater()
                
                # 최종적으로 레이아웃에서 제거
                vertical_layout.removeItem(item)

        vertical_layout.setSizeConstraint(QLayout.SetMinimumSize)
        # 새로운 위젯을 배치할 때 사용
        horizontal_layout = None  # 수평 레이아웃 초기화
        widgets_in_row = 0  # 현재 행에 추가된 위젯 수

        # 추출된 데이터 위젯으로 출력
        for index, row in enumerate(result):
            # 새로운 행을 시작할 때 수평 레이아웃 생성
            if widgets_in_row == 0:
                horizontal_layout = QHBoxLayout()
                horizontal_layout.setSpacing(0)
                vertical_layout.addLayout(horizontal_layout)

            # Menu_Widget 생성 및 수평 레이아웃에 추가
            icecream_info = {
                'id': row['ID'],
                'name': row['NAME'],
                'price': row['PRICE'], 
                'status': row['STATUS']
            }
            widget = MenuWidget(icecream_info, callback=self.go_to_topping)
            horizontal_layout.addWidget(widget)

            widgets_in_row += 1  # 행에 추가된 위젯 수 증가

            # 4개가 채워지면 다음 행으로 넘어가도록 초기화
            if widgets_in_row == 4:
                widgets_in_row = 0

        if widgets_in_row != 0:  # 마지막 행이 4개로 꽉 차지 않았을 경우
            for _ in range(4-widgets_in_row):
                horizontal_layout.addWidget(EmptyMenuWidget())

        vertical_layout.addStretch(1)



    # 카트위젯 전환
    def show_cart(self, order_info):
        print("show_cart 함수를 실행합니다, order_info :", order_info)

        # 1. CartWidget 인스턴스를 생성
        cart_item_widget = CartWidget(self, order_info['menu'], order_info['topping'], order_info['price'])
        # cart_item_widget.setFixedSize(QSize(290, 200))  # 장바구니 너비 186

        # CartWidget이 스크롤 영역의 가로 크기에 맞춰지도록 설정
        # cart_item_widget.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        # 주문 정보를 CartWidget의 라벨에 설정
        cart_item_widget.cart_name.setText(order_info['menu'])
        cart_item_widget.cart_num.setText(str(1))
        cart_item_widget.cart_top.setText(order_info['topping'])
        cart_item_widget.label_price.setText(f"{order_info['price']} 원")

        # 2. 기존 레이아웃의 addStretch 제거
        if self.cart_layout.count() > 0 and isinstance(self.cart_layout.itemAt(self.cart_layout.count() - 1), QSpacerItem):
            self.cart_layout.takeAt(self.cart_layout.count() - 1)

        # 3. 새로운 CartWidget을 장바구니 레이아웃에 추가
        self.cart_layout.addWidget(cart_item_widget)

        # 4. 장바구니 레이아웃에 빈 공간 추가
        self.cart_layout.addStretch()  # addStretch로 남은 공간을 공백으로 채움
        print(f'current cart : {self.cart_layout.count()}')
        # 5. 장바구니 UI 업데이트
        # self.cart_content.update()
        # self.cart_content.adjustSize()

        print(f"장바구니에 항목 추가됨: 메뉴 - {order_info['menu']}, 토핑 - {order_info['topping']}, 가격 - {order_info['price']}")

        

    # 토핑창으로 전환
    def go_to_topping(self, menu_name):
        
        if self.is_cart_full():
            return

        self.topping_window = Popup_Topping(menu_name, connection)
        self.topping_window.exec()
        
        order_info = self.topping_window.order_info # order_info 받아오는 부분
        if order_info:
            print('go_to_topping 함수 실행 order_info = ', order_info)
            self.show_cart(order_info)


        else:
            print('주문 정보가 없습니다.')

    # 장바구니에 3개 이상 담길 경우 경고
    def is_cart_full(self):
        if self.total_cart_num >= self.maximum_cart_num:
            QMessageBox.warning(self, "Order Error", "아직 초보 바텐드로이드에게\n3개 이상의 주문은 무리에요😭")
            return True
        return False

    # 주문정보 저장
    def save_order(self, order_list):
        print(order_list)
        cur = connection.cursor()

        customer_id = 1

        total_price = 0
        for order in order_list:
            total_price += int(order['price'][:-1])

        # 주문 정보 저장
        cur.execute(f"INSERT INTO REQUEST (TOTAL_PRICE, REQUEST_DT, CUSTOMER_ID) VALUES ({total_price}, NOW(), {customer_id});")

        connection.commit()

        # 주문 메뉴 정보 저장
        request_id = cur.lastrowid

        for order in order_list:
            # 아이스크림 ID 가져오기
            cur.execute(f"SELECT ID FROM ICECREAM WHERE NAME = '{order['name'].strip()}'")
            icecream_id = cur.fetchone()
            
            # 토핑 ID 가져오기
            cur.execute(f"SELECT ID FROM TOPPING WHERE NAME = '{order['topping'].strip()}'")
            topping_id = cur.fetchone()
            
            cur.execute(f"INSERT INTO MENU (REQUEST_ID, ICECREAM_ID, TOPPING_ID) VALUES ({request_id}, {icecream_id[0]}, {topping_id[0]});")
                 
            connection.commit()

    def init_face_detectoin(self):
        self.detector = MTCNN()  # 얼굴 탐지기 초기화
        self.capture = cv2.VideoCapture(0)  # 카메라 초기화
        self.face_detection_thread = FaceDetectionThread(self.detector, self.capture)
        
        self.face_detection_thread.no_face_detected.connect(self.on_no_face_detected)
        self.face_detection_thread.camera_error.connect(self.show_camera_error)
        self.is_timer_expired = False
    
    def on_no_face_detected(self):
        self.warning_dialog = QMessageBox(self)
        self.warning_dialog.setWindowTitle('No Face Detected')
        self.warning_dialog.setText('10초 후 화면보호기로 돌아갑니다.\n주문을 계속하시겠습니까?')
        self.warning_dialog.setStandardButtons(QMessageBox.Ok)
        
        # 타이머 설정 (5초 후에 자동으로 닫기)
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.after_no_face_detected)

        self.is_timer_expired = False  # 타이머 만료 상태 초기화
        self.timer.start(10000)

        result = self.warning_dialog.exec_()  # 모달 대화 상자 실행

        if self.is_timer_expired:
            # 타이머가 만료된 경우에는 아무것도 하지 않음
            return
        elif result == QMessageBox.Ok:
            print('OK 버튼이 클릭됨')
            self.timer.stop()  # 타이머 중지
            self.face_detection_thread.start()  # 얼굴 탐지 스레드 시작
    
    def after_no_face_detected(self):
        self.is_timer_expired = True

        if self.warning_dialog.isVisible():
            self.warning_dialog.close()

        # 열려있는 모든 창 닫기
        for widget in QApplication.topLevelWidgets():
            if isinstance(widget, QDialog) and widget.isVisible():
                widget.close()

            # 화면 보호기로 전환 
            self.empty_cart()
            self.set_ad_img()

    def show_camera_error(self, message):
        QMessageBox.warning(self, 'Camera Error', message)  # 메인 GUI에서 메시지 박스 표시

    def init_audio_recording(self):
        self.audio_recorder_thread = AudioRecorderThread()
        self.btn_help.clicked.connect(self.on_audio_recording) 

    def on_audio_recording(self):
        self.popup_recommend = Popup_Recommend(connection, self.audio_recorder_thread, callback=self.go_to_topping)
        self.audio_recorder_thread.start()  # 음성 녹음 시작 
        self.popup_recommend.exec_()       

if __name__ == "__main__":
    rp.init()
    app = QApplication(sys.argv)
    
    ros_thread = RosThread()
    ros_thread.start()

    window = MyKiosk(connection, ros_thread)
    window.show()

    app.exec_()
    connection.close()

    rp.shutdown()

