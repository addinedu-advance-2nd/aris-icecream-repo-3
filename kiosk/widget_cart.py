import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout
from PyQt5 import uic

# UI 파일을 로드하여 클래스 형식으로 변환
form_cart = uic.loadUiType('kiosk/UI/widgets/widget_cart_menu.ui')[0]

'''
QWidget : cart_widget
    QFrame : cart_frame
        QPushButton : btn_del, btn_minus, btn_plus
        QLabel : cart_name, cart_num, cart_top, label_price
'''
class CartWidget(QWidget, form_cart):
    def __init__(self, parent, name, topping, price):
        super().__init__()
        self.setupUi(self)
        print(topping, name, price)

        self.main_window = parent

        self.single_price = price
        self.quantity = 1

        self.cart_name.setText(name)
        self.cart_top.setText(topping)
        self.label_price.setText(str(self.single_price))
        
        self.main_window.total_price += self.single_price
        self.main_window.total_cart_num += 1
        self.main_window.refresh_total_price()

        self.btn_del.clicked.connect(self.remove_cart_item)
        self.btn_minus.clicked.connect(self.decrease_quantity)
        self.btn_plus.clicked.connect(self.increase_quantity)

    # 장바구니 - 제거버튼
    def remove_cart_item(self):
        self.main_window.total_price -= self.quantity*self.single_price
        self.main_window.total_cart_num -= self.quantity
        self.main_window.refresh_total_price()

        self.setParent(None)

    # 장바구니 -버튼
    def decrease_quantity(self):
        print("버튼 눌림 확인")
        if self.quantity > 1:
            self.quantity -= 1
            self.cart_num.setText(str(self.quantity))

            self.main_window.total_price -= self.single_price
            self.main_window.total_cart_num -= 1
            self.main_window.refresh_total_price()

    # 장바구니 +버튼
    def increase_quantity(self):
        print("버튼 눌림 확인")

        if self.main_window.is_cart_full():
            return
        self.quantity += 1
        self.cart_num.setText(str(self.quantity))

        self.main_window.total_price += self.single_price
        self.main_window.total_cart_num += 1
        self.main_window.refresh_total_price()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CartWidget(None, 'ICECREAM', 'NAME', 'TOPPING', 'PRICE')
    window.show()
    app.exec_()
