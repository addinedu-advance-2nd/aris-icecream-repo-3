import sys
import pymysql
import urllib.request


from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt


form_main_menu = uic.loadUiType('kiosk/UI/widgets/widget_main_menu.ui')[0]
form_empty_main_menu = uic.loadUiType('kiosk/UI/widgets/widget_empty_main_menu.ui')[0]

'''
 QWidget : item
    QFrame : frame_menu 
    QLabel : menu_image, menu_price, menu_name
'''
class MenuWidget(QWidget, form_main_menu):
    def __init__(self, icecream_info, callback = None):
        super().__init__()
        self.setupUi(self)

        # 메뉴 - 이미지연결 
        self.pixmap_menu = QPixmap()
        if icecream_info['status'] == 0:
            self.pixmap_menu.load('img/soldout.png')
            callback = self.notify_soldout
        else:
            self.pixmap_menu.load(f'img/icecream{icecream_info["id"]}.png')
        
        self.menu_image.setPixmap(self.pixmap_menu)  # QLabel에 이미지 설정
        self.menu_image.setScaledContents(True)  # 이미지가 QLabel 크기에 맞춰지도록 설정
        # 메뉴 - 이름
        self.menu_name.setText(icecream_info['name'])
        # 메뉴 - 가격
        self.menu_price.setText(f"{icecream_info['price']} 원")  

        # 콜백 함수 저장
        self.callback = callback
        self.menu_name_text = icecream_info['name']

        # menu_image 클릭 이벤트 처리
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            # 콜백 함수가 정의된 경우 호출
            if self.callback:
                self.callback(self.menu_name_text) 

    def notify_soldout(self, menu_name):
        QMessageBox.information(self, 'Soldout', f'"{menu_name}"은(는) 현재 품절상태입니다.')

class EmptyMenuWidget(QWidget, form_empty_main_menu):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.menu_name.setText('')
        self.menu_price.setText('')  


if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = MenuWidget(None, None, None)
    window.show()
    app.exec_()

