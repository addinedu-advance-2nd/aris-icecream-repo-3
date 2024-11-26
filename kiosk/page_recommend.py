import sys
import time
import datetime
import pymysql

import wave
import sounddevice as sd

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import uic

from faster_whisper import WhisperModel 

import google.generativeai as genai

# API 키 설정 (발급받은 Google API 키 사용)
GOOGLE_API_KEY = ""
genai.configure(api_key=GOOGLE_API_KEY)

class AudioRecorderThread(QThread):
    def __init__(self):
        super().__init__()
        self.RATE = 16000
        self.CHANNELS = 1
        self.OUTPUT_FILENAME = f"data/audio/record_{datetime.datetime.now()}.wav"
        self.is_recording = False
        self.audio_data = []
        self.icecream_list = []
        self.model_stt = WhisperModel("base", device="cpu")
        self.model_gemini = genai.GenerativeModel('gemini-1.5-pro')

    def callback(self, indata, frames, time, status):
        if status:
            self.recording_error.emit(str(status))

        self.audio_data.append(indata.copy())

    def run(self):
        if self.is_recording:
            return
        print("녹음 시작...")
        self.is_recording = True
        self.audio_data = []  # 녹음 데이터 초기화

        with sd.InputStream(samplerate=self.RATE, channels=self.CHANNELS, dtype='int16', callback=self.callback):
            while self.is_recording:
                time.sleep(0.1)  # 스트리밍 대기

    def stop(self, is_emergency_stop):
        if not self.is_recording:
            print("녹음 중이 아닙니다.")
            return
        print("녹음 종료.")
        self.is_recording = False
        self.save_audio(self.audio_data, self.OUTPUT_FILENAME, is_emergency_stop)

    def save_audio(self, data, filename, is_emergency_stop):
        print(f"녹음된 파일이 {filename}에 저장되었습니다.")
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(2)  # 16비트 샘플링 깊이
            wf.setframerate(self.RATE)
            wf.writeframes(b''.join(frame.tobytes() for frame in data))
        if is_emergency_stop == True:
            return
        self.stt(self.OUTPUT_FILENAME)
    
    def stt(self, filename):
        # 변환할 오디오 파일 경로
        audio_file = filename  # VAD로 처리한 오디오 파일 경로

        # 음성 파일을 텍스트로 변환
        # Whisper 모델 로드 (여기서는 "base" 모델을 사용)
        #model = WhisperModel("base", device="cpu")
        segments, info = self.model_stt.transcribe(audio_file, language="ko")

        # 변환된 텍스트 출력
        print("Transcription:")

        wordss = [""]

        for segment in segments:
            # 텍스트를 ','와 띄어쓰기로 구분하여 나누기
            words = [word for word in segment.text.replace(" ", ",").split(",") if word]

            # 어절을 띄어쓰기를 경계로 합치고 한 줄의 글로 만들고 wordss에 저장
            join_words = " ".join(words)
            wordss[0] += " " + join_words

        print(wordss)

        self.gemini(wordss)

    def gemini(self, text):
        # 제공된 메뉴와 설명을 설정
        menu_data = [
            {"description": "깊고 풍부한 자모카 아이스크림에 고소한 아몬드와 초콜릿 훠지 시럽이 들어있는 제품 #커피 #초콜릿시럽 #아몬드", "name": "자모카 아몬드 훠지"},
            {"description": "부드러운 마스카포네 아이스크림과 마카롱, 초콜릿의 달콤한 만남 #마스카포네치즈 #라즈베리시럽 #마카롱 #하트초콜릿", "name": "봉쥬르~ 마카롱"},
            {"description": "이탈리안 디저트 파나코타와 상큼한 블루베리의 부드러운 만남 #블루베리 #우유푸딩", "name": "블루베리 파나코타"},
            {"description": "고소한 흑임자, 인절미 아이스크림에 쫄깃한 떡리본과 바삭한 프랄린 피칸이 쏙쏙 #피칸 #흑임자 #인절미", "name": "쫀떡궁합"},
            {"description": "달콤한 카라멜 아이스크림에 바삭한 달고나가 쏘옥~ #달고나 #카라멜", "name": "너는 참 달고나"},
            {"description": "부드럽게 즐기는 뉴욕식 정통 치즈케이크 아이스크림 #치즈 #그라함크래커", "name": "뉴욕 치즈케이크"},
            {"description": "쿨~한 민트 맛과 진한 초콜릿을 담은 달콤한 비밀 레시피 #민트 #초콜릿", "name": "마법사의 비밀 레시피"},
            {"description": "부드럽고 깔끔한 바닐라 아이스크림 #바닐라", "name": "바닐라"},
            {"description": "쿨한 당신의 선택! 상쾌한 민트향에 초코칩까지! #민트 #초콜릿칩", "name": "민트 초콜릿 칩"},
            {"description": "밀크 초콜릿, 다크 초콜릿, 화이트 무스 세 가지 아이스크림에 달콤 바삭한 초코볼이 더해진 아이스크림 #밀크초콜릿 #화이트무스 #다크초콜릿 #초콜릿칩 #초코프레첼", "name": "엄마는 외계인"},
            {"description": "부드럽고 달콤한 솜사탕과 함께 떠나는 이상한 나라로의 여행 #옐로우솜사탕 #핑크솜사탕 #블루솜사탕 #옐로우크런치 #핑크크런치", "name": "이상한 나라의 솜사탕"},
            {"description": "유산균이 살아 있는 오리지널 요거트 아이스크림 #요거트", "name": "31요거트"},
            {"description": "새콤상큼 딸기 과육이 듬뿍! #딸기 #딸기과육", "name": "베리베리 스트로베리"},
            {"description": "부드러운 바닐라향 아이스크림에, 달콤하고 진한 오레오 쿠키가 듬뿍 #바닐라 #오레오", "name": "오레오 쿠키 앤 크림"},
            {"description": "알폰소 망고와 우유 아이스크림의 부드러운 만남 #우유", "name": "알폰소 망고"},
            {"description": "입안 가득 즐거운 초콜릿, 아몬드로 더욱 달콤하게! #바닐라 #밀크초콜릿시럽 #초코아몬드", "name": "아몬드 봉봉"},
            {"description": "상큼한 파인애플, 오렌지, 라즈베리가 만드는 일곱빛깔 무지개 #오렌지 #파인애플 #라즈베리", "name": "레인보우 샤베트"},
            {"description": "나주배를 그대로 갈아만든 시원하고 달콤한 나주배 소르베 #나주배", "name": "나주배 소르베"},
            {"description": "딸기와 초콜릿이 치즈케이크에 반해버린 사랑의 맛 #치즈딸기 #크래클퍼지 #치즈케이크큐브 #딸기과육", "name": "사랑에 빠진 딸기"},
            {"description": "초콜릿 칩이 들어있는 진한 초콜릿 아이스크림 #초콜릿무스", "name": "초콜릿 무스"},
            {"description": "엄선된 녹차를 사용한 싱그러운 그린티 아이스크림 #그린티", "name": "그린티"},
            {"description": "바닐라향 아이스크림에 초콜릿 칩이 쏙쏙쏙! #바닐라 #초콜릿칩", "name": "초콜릿 칩"},
            {"description": "달콤한 밀크쉐이크 아이스크림과 치즈 아이스크림에 초콜릿과 치토스 볼이 가득! #초콜릿 #치즈 #밀크쉐이크 #치토스볼", "name": "치토스 밀크쉐이크 아이스크림"},
            {"description": "부드러운 우유 맛 아이스크림 속에 깊은 단 맛을 끌어내는 소금 아이스크림 #소금우유", "name": "소금 우유 아이스크림"},
            {"description": "진한 커피 아이스크림, 우유맛 아이스크림에 프레첼 볼과 초콜릿이 어우러진 맛 #초콜릿 #커피 #우유 #초코프레첼", "name": "내가 아인슈페너?!"},
            {"description": "블루베리와 딸기로 상큼함을 더한 치즈케이크 한 조각 #치즈 #블루베리시럽 #딸기시럽 #치즈케이크큐브", "name": "바람과 함께 사라지다"},
            {"description": "상큼한 청사과와 시원한 민트향이 기분까지 상쾌하게 #민트 #그린애플", "name": "애플 민트"},
            {"description": "블루베리 & 바닐라향에 입안에서 톡톡 터지는 캔디와 신나는 축제 #블루베리 #바닐라 #체리시럽 #딸기과육 #블루팝핑캔디", "name": "슈팅스타"},
            {"description": "피스타치오와 아몬드가 만나 고소함이 두 배! #피스타치오 #아몬드", "name": "피스타치오 아몬드"},
            {"description": "진하고 부드러운 정통 초콜릿 아이스크림 #초콜릿", "name": "초콜릿"},
            {"description": "달콤한 고구마와 연유 아이스크림에 쫀득한 찰떡 다이스가 쏙쏙! #고구마 #떡 #연유", "name": "찰떡이구마"},
            {"description": "향긋한 얼그레이와 달콤한 초콜릿이 만난, F까지 반하게 할 T(Tea,차) 아이스크림! #초콜릿 #얼그레이", "name": "너 T(tea)야??"},
            {"description": "체리과육이 탱글탱글 씹히는 체리 아이스크림 #체리 #체리과육", "name": "체리쥬빌레"},
        ]

        # 메뉴 데이터와 설명을 하나의 문자열로 연결
        menu_text = "\n".join([f"{item['name']}: {item['description']}" for item in menu_data])

        # gemini-1.5-flash 모델 초기화
        #model = genai.GenerativeModel('gemini-1.5-pro')

        # 사용자 요청을 포함하여 모델에게 추천을 요청
        taste_preference = text
        prompt = f"아래는 (메뉴: 메뉴설명)이야. '{taste_preference}'의 문맥과 가장 비슷한 설명의 메뉴를 무조건 3개 골라서 단답형으로 '메뉴1,메뉴2,메뉴3'과 같이 띄어쓰기 없이 ','로만 구분해서 대답해줘:\n\n{menu_text}"


        while True:
            try:
                response = self.model_gemini.generate_content(prompt)
                response_text_list = response.text.split(',')
                if len(response_text_list) != 3:
                    for i in range(0,5):
                        try:
                            response = self.model_gemini.generate_content(prompt)
                            response_text_list = response.text.split(',')
                            if len(response_text_list) == 3:
                                break
                        except:
                            continue
                break
            except:
                continue

        response_text_list[len(response_text_list)-1] = response_text_list[len(response_text_list)-1].strip()

        response_return = ['','','']
        for i in range(0,len(response_text_list)):
            response_return[i] = response_text_list[i].replace(" ", "")

        # 추천 결과를 출력
        self.icecream_list = response_return
        print(f'response_return: {response_return}')

form_waiting_msgbox = uic.loadUiType('kiosk/UI/msgbox_waiting.ui')[0]

class WaitingDialog(QDialog, form_waiting_msgbox):
    def __init__(self, parent=None): 
        super().__init__(parent)
        self.setupUi(self)
        self.setWindowTitle('바텐드로이드가 열심히 고민 중이이에요! 잠시만 기다려주세요 🤓')

form_recommend_page = uic.loadUiType('kiosk/UI/page_recommend.ui')[0]

class Popup_Recommend(QDialog, form_recommend_page):
    def __init__(self, conn, audio_recorder_thread, callback):
        super().__init__()
        self.setupUi(self)
        self.stackedWidget.setCurrentWidget(self.page_record)

        # 윈도우 기본 프레임 제거
        self.setWindowFlag(Qt.WindowType.FramelessWindowHint) 

        self.conn = conn
        self.audio_recorder_thread = audio_recorder_thread
        self.callback = callback

        self.btn_end.clicked.connect(self.stop_recording)

        self.btn_cancel_1.clicked.connect(self.close_window)
        self.btn_cancel_2.clicked.connect(self.close_window)
        self.set_recording_gif()

    def set_recording_gif(self):
        self.img_recording = self.findChild(QLabel, "img_recording")

        movie = QMovie("img/recording.gif")
        self.img_recording.setMovie(movie)

        self.img_recording.setScaledContents(True)  # 이미지 크기를 QLabel에 맞게 조정

        movie.start()

    def close_window(self):
       self.audio_recorder_thread.stop(True)
       self.close()

    def stop_recording(self):
        if self.audio_recorder_thread.is_recording:
            self.waiting_dialog = WaitingDialog(self)
            self.waiting_dialog.show()
            
            self.audio_recorder_thread.stop(False)
            print("녹음이 종료되었습니다.")

            self.go_to_recommend(self.waiting_dialog)

    def go_to_recommend(self, waiting_dialog):
        self.waiting_dialog = waiting_dialog
        self.waiting_dialog.close()

        self.stackedWidget.setCurrentWidget(self.page_recommend)       
        self.load_recommend_information() 
    
    def load_recommend_information(self):
        formatted_names = ', '.join([f"'{name.strip()}'" for name in self.audio_recorder_thread.icecream_list])

        cursor = self.conn.cursor(pymysql.cursors.DictCursor)
        query_expr = f'SELECT * FROM ICECREAM WHERE RECOMMENDED_NAME IN ({formatted_names})'
        cursor.execute(query_expr)
        recommend_info = cursor.fetchall()

        print('recommend info')
        print(recommend_info)

        self.recommend_pixmap = [QPixmap(), QPixmap(), QPixmap()]
        self.recommend_imgs = [self.img_recommend_1, self.img_recommend_2, self.img_recommend_3]
        self.recommend_name_labels = [self.label_recommend_name_1, self.label_recommend_name_2, self.label_recommend_name_3]
        self.recommend_price_labels = [self.label_recommend_price_1, self.label_recommend_price_2, self.label_recommend_price_3]
        self.recommend_prices = []
        self.recommend_frames = [self.frame_recommend_1, self.frame_recommend_2, self.frame_recommend_3]

        for i in range(3):
            # 각 토핑이 위치할 이미지 라벨, 라벨, 정보, 동작할 함수 지정
            img = self.recommend_imgs[i]
            label_name = self.recommend_name_labels[i]
            label_price = self.recommend_price_labels[i]
            pixmap = self.recommend_pixmap[i]
            info = recommend_info[i]
            
            # 토핑 이름 및 이미지 출력
            label_name.setText(info['NAME'])
            label_price.setText(f'{info["PRICE"]}원')
            self.recommend_prices.append(info["PRICE"])
            
            # 이미지를 url에서 로드, 품절이라면 품절 이미지 사용
            if info['STATUS'] == 0:
                pixmap.load(f'img/soldout.png')
            else:
                pixmap.load(f'img/icecream{info["ID"]}.png')
            img.setPixmap(pixmap)
            img.setScaledContents(True)
            # img.resize()

            # 판매중이라면 일반 이벤트 등록, 품절이라면 품절 안내 이벤트 등록
            if info['STATUS']:
                img.mousePressEvent = lambda event, name=info['NAME']: self.open_topping_window(name)
            else:
                img.mousePressEvent = self.show_soldout_message

    def open_topping_window(self, name):
        self.close()
        self.callback(name)
    
    def show_soldout_message(self, event):
        # 품절 안내 메세지 출력
        QMessageBox.information(self, 'Title', '이 토핑은 품절 상태입니다.')


if __name__ == '__main__':
    conn = pymysql.connect(
        host='localhost',
        user='root',
        password='12345678',
        database='BARTENDROID'
    )

    app = QApplication(sys.argv)

    myWindow = Popup_Recommend(conn)
    myWindow.show()
    app.exec_()

    conn.close()