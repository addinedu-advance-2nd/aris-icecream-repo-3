# 설치 및 실행 절차

## 공통 환경
- Ubuntu 22.04
- Python 3.10
- ROS2 Humble

## 키오스크 환경

1. mysql-server 설치 `sudo apt install mysql-server`
2. root 계정 생성 및 비밀번호 `12345678`로 지정
3. mysql에서 data/setup.sql 실행
4. `pip install pymysql`
5. `pip install opencv-contrib-python-headless`
6. `pip install mtcnn`
7. `pip install tensorflow==2.17.0`
8. `pip install sounddevice` 및 libportaudio2 설치(`sudo apt install libportaudio2`)
9. `pip install faster_whisper`
10. `pip install kss`
11. `pip install google-generativeai`
12. `pip install ipython`
13. 만약 numpy버전이 2.0이상이라면 `pip install numpy==1.26.4`
14. 로봇 구동 환경의 모든 노드를 실행 후 `kiosk/main.py`실행

## 로봇 구동 환경
*본 코드에서 실제 로봇팔을 동작시키는 부분은 제거되어있는 상태입니다.*

1. `pip install opencv-contrib-python`
2. `pip install ultralytics`
3. [xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK)를 사용하여 `robot/src/bartendroid/bartendroid/master.py`에 로봇 구동코드 작성
4. robot폴더에서 colcon build
5. `source install/setup.bash`
6. 로봇 노드 실행 `ros2 run bartendroid master`
7. 카메라 서버1(아루코 마커, 봉인씰 감지) 실행 `ros2 run bartendroid camera_server`
8. 충돌 감지 노드 실행 `ros2 run bartendroid obj_detect_pub`