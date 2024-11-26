import sys
import pymysql

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import uic

# ! 비밀번호가 평문으로 저장되어있음에 주의할 것
PASSWD = '12345'

# TODO: 활성화/비활성화 되어있는 것의 디자인 수정이 필요
STYLE_ACTIVE = '''
    color: rgb(229, 79, 64);
    border: 2px solid rgb(229, 79, 64);
    border-radius: 7px; 
    font-family: 'Ubuntu';
    font-size: 11pt;
    height: 50px;
    margin-right: 5px;
'''
STYLE_DEACTIVE = '''
    color: rgb(45, 45, 45);
    border: 2px solid rgb(45, 45, 45);
    border-radius:7px;
    font-family: 'Ubuntu';
    font-size: 11pt;
    height: 50px;
    margin-right: 5px;
'''

form_admin_page = uic.loadUiType('kiosk/UI/page_admin.ui')[0]
form_single_menu = uic.loadUiType('kiosk/UI/widgets/widget_admin_menu.ui')[0]

class Popup_Admin(QDialog, form_admin_page):
    def __init__(self, conn):
        super().__init__()
        self.setupUi(self)
        # 윈도우 기본 프레임 제거
        self.setWindowFlag(Qt.WindowType.FramelessWindowHint) 

        # DB와의 연결을 위한 커넥터 저장
        self.conn = conn

        # 시작화면 지정
        self.stack_main.setCurrentWidget(self.page_password)    # page_password

        # 취소 버튼 이벤트 지정
        self.btn_cancel.clicked.connect(self.close)
        self.btn_quit.clicked.connect(self.close)

        self.entered_password = ''
        # 비밀번호 페이지 - 숫자 버튼 이벤트 지정
        for btn in self.keypads.findChildren(QPushButton):
            if btn.objectName()[-1].isalpha():
                continue
            btn.clicked.connect(self.enter_number)
        
        # 비밀번호 페이지 - 비밀번호 삭제 버튼 이벤트 지정
        self.btn_del.clicked.connect(self.delete_number)

        # 비밀번호 페이지 - 비밀번호 초기화 버튼 이벤트 지정
        self.btn_clear.clicked.connect(self.clear_number)

        # 비밀번호 페이지 - 비밀번호 확인 버튼 이벤트 지정
        self.btn_check_password.clicked.connect(self.check_password)
        
        # 메뉴 관리 페이지 - 상단 카테고리별 버튼 이벤트 지정
        self.btn_chocolate.clicked.connect(self.get_icecream_list)
        self.btn_fruit.clicked.connect(self.get_icecream_list)
        self.btn_plain.clicked.connect(self.get_icecream_list)
        self.btn_special.clicked.connect(self.get_icecream_list)
        self.btn_topping.clicked.connect(self.get_topping_list)
        self.btn_chocolate.click()

        # 초기 화면 로딩
        self.btn_chocolate.click()

    def enter_number(self):
        # 버튼에 쓰여있는 숫자를 덧붙여 입력. 최대 20자리만 입력 가능
        if len(self.entered_password) == 20:
            QMessageBox.information(self, "Password Error", "패스워드는 최대 20자리 입니다.")
            return

        n = self.sender().text()[-1]
        self.entered_password += n
        self.label_password.setText('*'*len(self.entered_password))
    
    def delete_number(self):
        # 입력된 비밀번호의 마지막 숫자를 제거.
        self.entered_password = self.entered_password[:-1]
        self.label_password.setText('*'*len(self.entered_password))
        
    def clear_number(self):
        # 입력된 비밀번호를 초기화(비움)
        self.entered_password = ''
        self.label_password.setText('')

    def check_password(self):
        # 패스워드가 일치하는지를 확인. 올바르지 않은 경우 경고창 후 초기화
        if self.entered_password == PASSWD:
            self.stack_main.setCurrentWidget(self.page_menu_manager)
        else:
            QMessageBox.critical(self, "Password Error", "올바르지 않은 패스워드입니다.")
            self.clear_number()

    def get_icecream_list(self):
        # 쿼리를 통해 카테고리에 맞는 아이스크림 테이블을 출력

        target_category = self.sender().text()

        # 카테고리에 해당하는 데이터 추출
        cursor = self.conn.cursor(pymysql.cursors.DictCursor)
        query_expr = f'select NAME, STATUS from ICECREAM where CATEGORY = "{target_category}"'
        cursor.execute(query_expr)
        result = cursor.fetchall()

        # 수직 정렬 레이아웃 생성 및 초기화
        if self.widget_list.layout() is None:
            self.widget_list.setLayout(QVBoxLayout(self))
        layout = self.widget_list.layout()

        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
            else:
                layout.removeItem(item)

        # 추출된 데이터 위젯으로 만들어 출력
        for row in result:
            widget = Menu_Widget(self, 'ICECREAM', row['NAME'], row['STATUS'])
            widget.setMinimumHeight(100)
            layout.addWidget(widget)
        layout.addStretch(1)
        cursor.close()


    def get_topping_list(self):
        # 쿼리를 통해 토핑 테이블을 출력
        # * addStretch(1)으로 해결

        cursor = self.conn.cursor(pymysql.cursors.DictCursor)
        query_expr = f'select NAME, STATUS from TOPPING'
        cursor.execute(query_expr)
        result = cursor.fetchall()

        # 수직 정렬 레이아웃 생성 및 초기화
        if self.widget_list.layout() is None:
            self.widget_list.setLayout(QVBoxLayout(self))
        layout = self.widget_list.layout()        

        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
            else:
                layout.removeItem(item)

        # 추출된 데이터 위젯으로 만들어 출력
        for row in result:
            widget = Menu_Widget(self, 'TOPPING', row['NAME'], row['STATUS'])
            widget.setMinimumHeight(100)
            layout.addWidget(widget)
        layout.addStretch(1)
        cursor.close()

class Menu_Widget(QWidget, form_single_menu):
    # 아이스크림/토핑 리스트를 출력할 때, 각 항목을 표현하는 위젯

    def __init__(self, parent, type, menu_name, is_not_soldout):
        super().__init__()
        self.setupUi(self)

        self.parent = parent
        # 종류 지정(DB테이블 명)
        self.type = type
        # 메뉴 이름 출력
        self.label_menu_name.setText(menu_name)

        # 현재 판매 상태에 맞게 스타일 적용
        self.is_soldout = not is_not_soldout
        if self.is_soldout:
            self.btn_soldout.setStyleSheet(STYLE_ACTIVE)
            self.btn_not_soldout.setStyleSheet(STYLE_DEACTIVE)
        else:
            self.btn_soldout.setStyleSheet(STYLE_DEACTIVE)
            self.btn_not_soldout.setStyleSheet(STYLE_ACTIVE)

        # 판매/품절 버튼 이벤트 지정
        self.btn_soldout.clicked.connect(self.change_to_soldout)
        self.btn_not_soldout.clicked.connect(self.change_to_not_soldout)

    def change_to_soldout(self):
        # 품절로 상태를 전환
        if self.is_soldout:
            QMessageBox.information(self, 'State Information', '이미 품절 상태입니다.')
            return
        QMessageBox.information(self, 'State Information', '품절 상태로 전환합니다.')
        
        # 쿼리를 통해 해당 메뉴의 상태를 0(품절)로 전환 후 저장
        cursor = self.parent.conn.cursor(pymysql.cursors.DictCursor)
        query_expr = f'update {self.type} set STATUS=0 where NAME="{self.label_menu_name.text()}";'
        cursor.execute(query_expr)
        cursor.close()
        self.parent.conn.commit()

        # 현재 상태 및 버튼 디자인 갱신
        self.is_soldout = not self.is_soldout
        self.btn_soldout.setStyleSheet(STYLE_ACTIVE)
        self.btn_not_soldout.setStyleSheet(STYLE_DEACTIVE)

    def change_to_not_soldout(self):
        # 판매로 상태를 전환
        if not self.is_soldout:
            QMessageBox.information(self, 'State Information', '이미 판매 중인 상태입니다.')
            return
        QMessageBox.information(self, 'State Information', '판매 상태로 전환합니다.')

        # 쿼리를 통해 해당 메뉴의 상태를 0(품절)로 전환 후 저장
        cursor = self.parent.conn.cursor(pymysql.cursors.DictCursor)
        query_expr = f'update {self.type} set STATUS=1 where NAME="{self.label_menu_name.text()}";'
        cursor.execute(query_expr)
        cursor.close()
        self.parent.conn.commit()

        # 현재 상태 및 버튼 디자인 갱신
        self.is_soldout = not self.is_soldout
        self.btn_soldout.setStyleSheet(STYLE_DEACTIVE)
        self.btn_not_soldout.setStyleSheet(STYLE_ACTIVE)

if __name__ == '__main__':
    conn = pymysql.connect(
        host='localhost',
        user='root',
        password='12345678',
        database='BARTENDROID'
    )

    app = QApplication(sys.argv)
    myWindow = Popup_Admin(conn)
    myWindow.show( )
    app.exec_( )
    conn.close()