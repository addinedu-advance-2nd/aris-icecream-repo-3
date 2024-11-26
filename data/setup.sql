DROP DATABASE BARTENDROID;

CREATE DATABASE BARTENDROID;

USE BARTENDROID;

CREATE TABLE CUSTOMER (
	ID INT NOT NULL AUTO_INCREMENT,
    NAME VARCHAR(20) NOT NULL,
    PRIMARY KEY (ID)
);

CREATE TABLE REQUEST (
	ID INT NOT NULL AUTO_INCREMENT,
    TOTAL_PRICE INT NOT NULL,
    REQUEST_DT DATETIME NOT NULL DEFAULT NOW(),
    CUSTOMER_ID INT NOT NULL,
    PRIMARY KEY (ID),
    FOREIGN KEY (CUSTOMER_ID) REFERENCES CUSTOMER(ID)
);

CREATE TABLE ICECREAM (
	ID INT NOT NULL AUTO_INCREMENT,
    CATEGORY VARCHAR(20) NOT NULL,
    NAME VARCHAR(20) NOT NULL,
    PRICE INT NOT NULL,
    IMG VARCHAR(255) NOT NULL,
    STATUS BOOLEAN NOT NULL DEFAULT 1,
    CONTENT VARCHAR(255) NOT NULL,
    RECOMMENDED_NAME VARCHAR(20) NOT NULL,
    PRIMARY KEY (ID)
);

CREATE TABLE TOPPING (
	ID INT NOT NULL AUTO_INCREMENT,
    NAME VARCHAR(20) NOT NULL,
    PRICE INT NOT NULL,
    IMG VARCHAR(255) NOT NULL,
    STATUS BOOL NOT NULL DEFAULT 1,
    PRIMARY KEY (ID)
);

CREATE TABLE MENU (
	ID INT NOT NULL AUTO_INCREMENT,
    REQUEST_ID INT NOT NULL,
    ICECREAM_ID INT NOT NULL,
    TOPPING_ID INT NOT NULL,
    PRIMARY KEY (ID),
    FOREIGN KEY (REQUEST_ID) REFERENCES REQUEST(ID),
    FOREIGN KEY (ICECREAM_ID) REFERENCES ICECREAM(ID),
    FOREIGN KEY (TOPPING_ID) REFERENCES TOPPING(ID)
);

# ICECREAM DATA INSERTION
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('스페셜','치토스 밀크쉐이크 아이스크림',5000,'https://www.baskinrobbins.co.kr/upload/product/main/6b7de3ba55a71e3e99dc341d5cb908a9.png', '달콤한 밀크쉐이크 아이스크림과 치즈 아이스크림에 초콜릿과 치토스 볼이 가득! #초콜릿 #치즈 #밀크쉐이크 #치토스볼', '치토스밀크쉐이크아이스크림');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('초콜릿','너 T(tea)야??',5000,'https://www.baskinrobbins.co.kr/upload/product/main/d07eb9cc2b781a21cfa3f18932473478.png', '향긋한 얼그레이와 달콤한 초콜릿이 만난, F까지 반하게 할 T(Tea,차) 아이스크림! #초콜릿 #얼그레이', '너T(tea)야??');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('초콜릿','내가 아인슈페너?!',5000,'https://www.baskinrobbins.co.kr/upload/product/main/19bf954288abc59f184a75737ab9d720.png', '진한 커피 아이스크림, 우유맛 아이스크림에 프레첼 볼과 초콜릿이 어우러진 맛 #초콜릿 #커피 #우유 #초코프레첼', '내가아인슈페너?!');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('스페셜','쫀떡궁합',5000,'https://www.baskinrobbins.co.kr/upload/product/main/9368bad2b705ebb121e4b1f9f0e1c57e.png', '고소한 흑임자, 인절미 아이스크림에 쫄깃한 떡리본과 바삭한 프랄린 피칸이 쏙쏙 #피칸 #흑임자 #인절미', '쫀떡궁합');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('과일','나주배 소르베',5000,'https://www.baskinrobbins.co.kr/upload/product/main/1db88b4ce08086f487c72ef55344558a.png', '나주배를 그대로 갈아만든 시원하고 달콤한 나주배 소르베 #나주배', '나주배소르베');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('과일','블루베리 파나코타',5000,'https://www.baskinrobbins.co.kr/upload/product/main/7f981fc6e7a526daee600f4cee803b43.png', '이탈리안 디저트 파나코타와 상큼한 블루베리의 부드러운 만남 #블루베리 #우유푸딩','블루베리파나코타');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('플레인','소금 우유 아이스크림',5000,'https://www.baskinrobbins.co.kr/upload/product/main/719e8584fd50151284b5ace3589f7a34.png', '부드러운 우유 맛 아이스크림 속에 깊은 단 맛을 끌어내는 소금 아이스크림 #소금우유','소금우유아이스크림');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('초콜릿','봉쥬르~ 마카롱',5000,'https://www.baskinrobbins.co.kr/upload/product/main/0b22f758c9d1d0551498caafc4512f5c.png', '부드러운 마스카포네 아이스크림과 마카롱, 초콜릿의 달콤한 만남! #마스카포네치즈 #라즈베리시럽 #마카롱 #하트초콜릿', '봉쥬르~마카롱');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('과일','알폰소 망고',5000,'https://www.baskinrobbins.co.kr/upload/product/main/df7fdd6f667571b6521ae17cd27e3843.png', '알폰소 망고와 우유 아이스크림의 부드러운 만남 #우유','알폰소망고');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('과일','애플 민트',5000,'https://www.baskinrobbins.co.kr/upload/product/main/eec8cef386cf6697768a89b384c07bf7.png', '상큼한 청사과와 시원한 민트향이 기분까지 상쾌하게 #민트 #그린애플','애플민트');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('스페셜','너는 참 달고나',5000,'https://www.baskinrobbins.co.kr/upload/product/main/cc95113a4bfca780bbfee39b16fbedf1.png', '달콤한 카라멜 아이스크림에 바삭한 달고나가 쏘옥~ #달고나 #카라멜','너는참달고나');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('초콜릿','마법사의 비밀 레시피',5000,'https://www.baskinrobbins.co.kr/upload/product/main/dd743197f80c722eb57389b4c1814d2f.png', '쿨~한 민트 맛과 진한 초콜릿을 담은 달콤한 비밀 레시피 #민트 #초콜릿','마법사의비밀레시피');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('초콜릿','엄마는 외계인',5000,'https://www.baskinrobbins.co.kr/upload/product/main/91c8668227bcf556c43a968b97e342e6.png', '밀크 초콜릿, 다크 초콜릿, 화이트 무스 세 가지 아이스크림에 달콤 바삭한 초코볼이 더해진 아이스크림 #밀크초콜릿 #화이트무스 #다크초콜릿 #초콜릿칩 #초코프레첼','엄마는외계인');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('초콜릿','민트 초콜릿 칩',5000,'https://www.baskinrobbins.co.kr/upload/product/main/fb92d70dee836652115c4f3b13175541.png', '쿨한 당신의 선택! 상쾌한 민트향에 초코칩까지! #민트 #초콜릿칩','민트초콜릿칩');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('스페셜','뉴욕 치즈케이크',5000,'https://www.baskinrobbins.co.kr/upload/product/main/60a04a3a5d1b0119f065d12ee7797b2c.png', '부드럽게 즐기는 뉴욕식 정통 치즈케이크 아이스크림 #치즈 #그라함크래커','뉴욕치즈케이크');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('과일','레인보우 샤베트',5000,'https://www.baskinrobbins.co.kr/upload/product/main/5ad63f3af7244a666d981a1497a39fe7.png', '상큼한 파인애플, 오렌지, 라즈베리가 만드는 일곱빛깔 무지개 #오렌지 #파인애플 #라즈베리','레인보우샤베트');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('과일','체리쥬빌레',5000,'https://www.baskinrobbins.co.kr/upload/product/main/f6609e3e3431d54beceeb1d3787403af.png', '체리과육이 탱글탱글 씹히는 체리 아이스크림 #체리 #체리과육','체리쥬빌레');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('스페셜','슈팅스타',5000,'https://www.baskinrobbins.co.kr/upload/product/main/a4b71e8b99743c93a7824331850b0a3d.png', '블루베리 & 바닐라향에 입안에서 톡톡 터지는 캔디와 신나는 축제 #블루베리 #바닐라 #체리시럽 #딸기과육 #블루팝핑캔디','슈팅스타');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('과일','사랑에 빠진 딸기',5000,'https://www.baskinrobbins.co.kr/upload/product/main/387609a495e841413a453ce22b555840.png', '딸기와 초콜릿이 치즈케이크에 반해버린 사랑의 맛 #치즈딸기 #크래클퍼지 #치즈케이크큐브 #딸기과육','사랑에빠진딸기');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('초콜릿','오레오 쿠키 앤 크림',5000,'https://www.baskinrobbins.co.kr/upload/product/main/414246bd9041530d6ad4d30d97aac87c.png', '부드러운 바닐라향 아이스크림에, 달콤하고 진한 오레오 쿠키가 듬뿍! #바닐라 #오레오','오레오쿠키앤크림');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('과일','베리베리 스트로베리',5000,'https://www.baskinrobbins.co.kr/upload/product/main/ea6608b4f72563b360da5c44c946ddc7.png', '새콤상큼 딸기 과육이 듬뿍! #딸기 #딸기과육','베리베리스트로베리');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('스페셜','31요거트',5000,'https://www.baskinrobbins.co.kr/upload/product/main/d56328637eaf86e3273ebc39c57aada7.png', '유산균이 살아 있는 오리지널 요거트 아이스크림 #요거트','31요거트');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('과일','바람과 함께 사라지다',5000,'https://www.baskinrobbins.co.kr/upload/product/main/01ecc320f5d3a6f32e5188eda373842d.png', '블루베리와 딸기로 상큼함을 더한 치즈케이크 한 조각 #치즈 #블루베리시럽 #딸기시럽 #치즈케이크큐브','바람과함께사라지다');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('스페셜','이상한 나라의 솜사탕',5000,'https://www.baskinrobbins.co.kr/upload/product/main/4db4f9967ad6f603837a40eede965ef0.png', '부드럽고 달콤한 솜사탕과 함께 떠나는 이상한 나라로의 여행 #옐로우솜사탕 #핑크솜사탕 #블루솜사탕 #옐로우크런치 #핑크크런치','이상한나라의솜사탕');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('스페셜','피스타치오 아몬드',5000,'https://www.baskinrobbins.co.kr/upload/product/main/868364b0ed6038d0c9aee0a10e50d4a9.png', '피스타치오와 아몬드가 만나 고소함이 두 배! #피스타치오 #아몬드','피스타치오아몬드');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('초콜릿','초콜릿 무스',5000,'https://www.baskinrobbins.co.kr/upload/product/main/0d67607c2ca4dde4ec24ac8109a343c2.png', '초콜릿 칩이 들어있는 진한 초콜릿 아이스크림 #초콜릿무스','초콜릿무스');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('플레인','그린티',5000,'https://www.baskinrobbins.co.kr/upload/product/main/a6bd7bcdd6bdb56f28df7e98f051abda.png', '엄선된 녹차를 사용한 싱그러운 그린티 아이스크림 #그린티','그린티');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('스페셜','찰떡이구마',5000,'https://www.baskinrobbins.co.kr/upload/product/main/69f8fcb8dadccc84a8ad1f86d203decd.png', '달콤한 고구마와 연유 아이스크림에 쫀득한 찰떡 다이스가 쏙쏙! #고구마 #떡 #연유','찰떡이구마');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('초콜릿','초콜릿',5000,'https://www.baskinrobbins.co.kr/upload/product/main/aff1c39b1653ddb7701abd9b4c8394ee.png', '진하고 부드러운 정통 초콜릿 아이스크림 #초콜릿','초콜릿');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('초콜릿','자모카 아몬드 훠지',5000,'https://www.baskinrobbins.co.kr/upload/product/main/f31388da0371388c2086a7c90990a097.png', '깊고 풍부한 자모카 아이스크림에 고소한 아몬드와 초콜릿 훠지 시럽이 들어있는 제품 #커피 #초콜릿시럽 #아몬드','자모카아몬드훠지');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('초콜릿','초콜릿 칩',5000,'https://www.baskinrobbins.co.kr/upload/product/main/505b2fe8d9c444d638665fbb40c6f8a5.png', '바닐라향 아이스크림에 초콜릿 칩이 쏙쏙쏙! #바닐라 #초콜릿칩','초콜릿칩');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('초콜릿','아몬드 봉봉',5000,'https://www.baskinrobbins.co.kr/upload/product/main/e7cb5667c3147ddb0b31e28d1f365980.png', '입안 가득 즐거운 초콜릿, 아몬드로 더욱 달콤하게! #바닐라 #밀크초콜릿시럽 #초코아몬드','아몬드봉봉');
INSERT INTO ICECREAM (CATEGORY, NAME, PRICE, IMG, CONTENT, RECOMMENDED_NAME) VALUES ('플레인','바닐라',5000,'https://www.baskinrobbins.co.kr/upload/product/main/901f131644310c0eb356cbab7ecc4738.png', '부드럽고 깔끔한 바닐라 아이스크림 #바닐라','바닐라');

SELECT * FROM ICECREAM;

# TOPPING DATA INSERTION
INSERT INTO TOPPING (NAME, PRICE, IMG) VALUES ('죠리퐁', 3000, 'https://upload.wikimedia.org/wikipedia/commons/thumb/a/aa/Jollypong_20201213_003.jpg/748px-Jollypong_20201213_003.jpg?20210103101033');
INSERT INTO TOPPING (NAME, PRICE, IMG) VALUES ('코코볼', 3000, 'https://i.dailymail.co.uk/1s/2024/08/12/15/88443821-13734805-image-a-3_1723473462301.jpg');
INSERT INTO TOPPING (NAME, PRICE, IMG) VALUES ('해바라기씨', 3000, 'https://s.alicdn.com/@sc04/kf/H376647df26854e31a1c8241a09cd31caA.jpg_720x720q50.jpg');

SELECT * FROM TOPPING;

# TEST CUSTOMER DATA INSERTION
INSERT INTO CUSTOMER (NAME) VALUES ('TEST');

SELECT * FROM CUSTOMER;

