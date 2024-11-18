#include <SPIFFS.h>
#include <TinyGPS++.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLECharacteristic.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // 서비스 UUID
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8" // 특성 UUID

// L298N 모터 제어 핀 정의
const int IN1 = 18;
const int IN2 = 19;
const int ENA = 5;
const int IN3 = 14;
const int IN4 = 12;
const int ENB = 13;

//bluetooth 관련 변수 정의
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
String command = "";  // 현재 명령을 저장할 변수
int pwmValue = 0;
bool isDriving = false; // 주행 시작 여부를 나타내는 플래그
float targetLatitude = 0.0;
float targetLongitude = 0.0; 

// 10Hz 설정을 위한 UBX 명령어 (바이트 배열)
byte setRateTo10Hz[] = { 
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 
    0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 
    0x7A, 0x12 
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected"); // 연결 메시지 출력
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected"); // 연결 해제 메시지 출력
    pServer->getAdvertising()->start(); // 연결 해제 시 다시 광고 시작
  }
};

//함수 전처리
void moveTowardTarget(float currentLat, float currentLon, float targetLat, float targetLon);
int findClosestRoadPoint(float currentLat, float currentLon);
int findClosestRoadPoint2(float currentLat, float currentLon);
void loadRoadPath();
float calculateBearing(float lat1, float lon1, float lat2, float lon2) ;
int findClosestRoadPoint(float currentLat, float currentLon);
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
void parseCoordinates(String data);
void stop();
void right();
void left();
void forward();
void backward();
void straight();

//사용자로부터 블루투스로 입력받은 데이터 처리
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String receivedData = pCharacteristic->getValue().c_str();  // std::string을 Arduino String으로 변환
    Serial.print("Received data: ");
    Serial.println(receivedData); // 수신한 데이터 출력

    if (receivedData.indexOf(',') != -1) {  // 좌표 값 포함
        parseCoordinates(receivedData);  // 좌표로 인식하여 파싱
    } else if (receivedData.toInt() > 0) {  // 숫자만 입력된 경우 PWM 값으로 인식
        pwmValue = receivedData.toInt();
        Serial.print("PWM Value set to: ");
        Serial.println(pwmValue);
    } else if (receivedData == "start") {  // start 명령으로 주행 시작
        isDriving = true;
        Serial.println("Starting autonomous driving...");
    } else if (receivedData == "stop") {  // stop 명령으로 주행 중지
        isDriving = false;
        stop(); // 미니카 정지
        delay(100);
        Serial.println("Autonomous driving stopped.");
    } else {
        command = receivedData;  // 명령어로 인식
    }
  }
};

// Bluetooth 데이터에서 위도와 경도를 파싱하는 함수
void parseCoordinates(String data) {
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
        targetLatitude = data.substring(0, commaIndex).toFloat();
        targetLongitude = data.substring(commaIndex + 1).toFloat();
        Serial.print("Target set to: ");
        Serial.print(targetLatitude, 6);
        Serial.print(", ");
        Serial.println(targetLongitude, 6);
    }
}

TinyGPSPlus gps;

struct GPSPoint {
    float latitude;
    float longitude;
};

GPSPoint roadPath[100];  // 도로 경로 배열
int pathLength = 0;

const float MAX_DISTANCE_FROM_ROAD = 2.0;  // 도로를 벗어나는 최대 허용 거리 (미터)


void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting BLE...");
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // GPS 통신 설정(,,RX,TX)
  delay(100);
  
  //GPS 업데이트 속도를 10Hz로 설정(0.1초마다 받아옴)
  Serial2.write(setRateTo10Hz, sizeof(setRateTo10Hz));
  
  // BLE 초기화
  BLEDevice::init("ESP32_S3_BLE");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // BLE 서비스 및 특성 생성
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                     CHARACTERISTIC_UUID,
                     BLECharacteristic::PROPERTY_READ |
                     BLECharacteristic::PROPERTY_WRITE
                   );

  pCharacteristic->setCallbacks(new MyCallbacks());  // 데이터 수신 콜백 설정
  pCharacteristic->setValue("Hello from ESP32-S3!");

  // 서비스 시작
  pService->start();

  // BLE 광고 시작 (클라이언트가 연결할 수 있도록)
  pServer->getAdvertising()->start();
  Serial.println("BLE advertising started...");

  // SPIFFS 초기화 및 도로 경로 불러오기
  if (!SPIFFS.begin(true)) {
      Serial.println("Failed to mount SPIFFS");
      return;
  }
  loadRoadPath();
}


//도로에 해당하는 좌표들을 담은 파일에서 데이터 불러오기
void loadRoadPath() {
    File file = SPIFFS.open("/service_roads.csv", "r");
    if (!file) {
        Serial.println("Failed to open road_path.csv");
        return;
    }
    //파일에 데이터가 존재하는 배열까지만 실행
    while (file.available() && pathLength < 100) {
        //파일에서 한 줄을 읽어 line변수에 저장(line에는 37.xxxx,127.xxxx 처럼 콤마(,)로 구분된 문자열이 저장됨)
        String line = file.readStringUntil('\n');   
        int commaIndex = line.indexOf(',');   //콤마의 위치를 찾아서 변수에 저장.
        //콤마가 존재할때만 실행
        if (commaIndex > 0) {
          //위도와 경도를 분리하여 저장
            //line.substring(0,commaIndex)는 문자열 시작부터 콤마 전까지의 부분을 추출
            //toFloat()를 통해 문자열로 된 값을 float형 실수로 변환
            roadPath[pathLength].latitude = line.substring(0, commaIndex).toFloat(); 
            //line.substring(commaIndex+1)는 콤마 이후의 문자열을 추출하여 경도 값으로 사용
            roadPath[pathLength].longitude = line.substring(commaIndex + 1).toFloat();
            //roadPath배열에 현재 줄의 위도와 경도를 저장한 후 pathLength를 1 증가시킴
            //다음 줄을 읽어 저장할 때 배열의 다음 인덱스를 사용하도록 하기 위함
            pathLength++;   
        }
    }
    file.close();
    Serial.println("Road path loaded from SPIFFS");
}

//실행문
void loop() {
    float currentLatitude = gps.location.lat();   //현재 위치의 위도와 경도 정보를 변수에 저장
    float currentLongitude = gps.location.lng();
  //시작 전 미니카 테스트
    if (command == "go"){
        forward();
        command="";
    }else if (command == "back") {
        backward();
        command = "";  // 명령 실행 후 초기화
    }else if (command == "stop") {
        stop();
        command = "";  // 명령 실행 후 초기화
    }else if (command == "left") {
        left();
        command = "";  // 명령 실행 후 초기화
    }else if (command == "right") {
        right();
        command = "";  // 명령 실행 후 초기화
    }else if (command == "straight") {
        straight();
        command = "";  // 명령 실행 후 초기화
    }else if (command == "cl") {
        Serial.print(currentLatitude, 6);
        Serial.print(", ");
        Serial.println(currentLongitude, 7);
        command="";
    }
    // GPS 모듈로 현재 위치 확인
    if (isDriving) {  // 주행이 시작된 상태일 경우에만 실행
        while (Serial2.available() > 0) {     //GPS모듈에서 읽을 수 있는 데이터가 있을 때 true
            gps.encode(Serial2.read());       //tinyGPS++라이브러리가 GPS데이터를 해석하도록 합니다.
            if (gps.location.isUpdated()) {   //GPS 위치 정보가 업데이트 된 경우
                //소수 6번째 자리 까지만 사용
                float currentLatitude = gps.location.lat();   //현재 위치의 위도와 경도 정보를 변수에 저장
                float currentLongitude = gps.location.lng();
                Serial.print("Current location: ");           //현재 위치 정보를 시리얼 모니터에 출력
                Serial.print(currentLatitude, 6);
                Serial.print(", ");
                Serial.println(currentLongitude, 7);
                
                //현재 위치과 목표 지점 사이의 거리를 계산하여 변수에 저장
                float distanceToTarget = calculateDistance(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
                Serial.print("Distance to target: ");
                Serial.println(distanceToTarget);
                
                if (distanceToTarget > 3) {  // 목표 지점에 도달하지 않은 경우
                  //
                  moveTowardTarget(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
                } else {
                  stop();
                  delay(100);
                  stop();
                  Serial.println("Arrived at target location.");
                  isDriving = false;  // 목표 도달 후 주행 멈춤
                }
            }
        }
    }
}

// 두 GPS 좌표 간 거리 계산 함수 (단위: 미터)
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371000; // 지구 반경 (미터)
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);
    float a = sin(dLat/2) * sin(dLat/2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

// 도로 경로 내 가장 가까운 좌표를 찾고 해당 좌표의 인덱스 반환
int findClosestRoadPoint(float currentLat, float currentLon) {
    float minDistance = MAX_DISTANCE_FROM_ROAD;
    int closestIndex = -1;
    //불러온 spiffs의 도로좌표들의 배열 수만큼 반복
    //현재위치와 가장 가까운 도로좌표를 찾아내서 해당 배열의 번수를 변수에 저장
    for (int i = 0; i < pathLength; i++) {
        //도로 좌표와 현재 좌표의 거리를 계산하여 변수에 저장
        float distance1 = calculateDistance(currentLat, currentLon, roadPath[i].latitude, roadPath[i].longitude);
        //만약 거리가 설정된 최대 거리보다 작을 시 minDistance값으로 갱신
        //갱신 이후로도 더 작은 값이 나오면 갱신
        //갱신될 때만 해당 인덱스를 closestIndex에 저장
        if (distance1 < minDistance) {
            minDistance = distance1;
            closestIndex = i;
        }
    }
    return closestIndex;
}

// 도로 경로 내 가장 가까운 좌표를 찾고 해당 좌표의 인덱스 반환
int findClosestRoadPoint2(float currentLat, float currentLon) {
    float minDistance = 10;
    int closestIndex = -1;
    //불러온 spiffs의 도로좌표들의 배열 수만큼 반복
    //현재위치와 가장 가까운 도로좌표를 찾아내서 해당 배열의 번수를 변수에 저장
    for (int i = 0; i < pathLength; i++) {
        //도로 좌표와 현재 좌표의 거리를 계산하여 변수에 저장
        float distance1 = calculateDistance(currentLat, currentLon, roadPath[i].latitude, roadPath[i].longitude);
        //만약 거리가 설정된 최대 거리보다 작을 시 minDistance값으로 갱신
        //갱신 이후로도 더 작은 값이 나오면 갱신
        //갱신될 때만 해당 인덱스를 closestIndex에 저장
        if (distance1 < minDistance) {
            minDistance = distance1;
            closestIndex = i;
        }
    }
    return closestIndex;
}

//전진 타이밍 수정 필요
// 목표 위치로 이동 방향 결정
void moveTowardTarget(float currentLat, float currentLon, float targetLat, float targetLon) {
    //findClosestRoadPoint에서 반환받은 값 변수에 저장
    int closestPointIndex = findClosestRoadPoint(currentLat, currentLon);
    //closestPointIndex값이 -1이면 근처에 2미터보다 가까운 도로 경로가 없다는 의미
    if (closestPointIndex == -1) {
         // 가까운 도로 경로 지점의 좌표를 찾고 방위각을 계산
         int nearestPointIndex = findClosestRoadPoint2(currentLat, currentLon);
         float nearestLat = roadPath[nearestPointIndex].latitude;
         float nearestLon = roadPath[nearestPointIndex].longitude;
         Serial.print(nearestLat, 6);
         Serial.print("");
         Serial.println(nearestLon, 7);
         float recoveryBearing = calculateBearing(currentLat, currentLon, nearestLat, nearestLon);
         if (recoveryBearing >= 45 && recoveryBearing <= 135) {
             right();  // 오른쪽으로 틀어 도로 복귀
             delay(100);  // 짧은 전환 시간으로 미세 조정
             straight();
          } else if (recoveryBearing >= 225 && recoveryBearing <= 315) {
             left();   // 왼쪽으로 틀어 도로 복귀
             delay(100);
             straight();  // 직진 상태로 정렬하여 천천히 복귀
             //멈춘 후 후진+방향전환 해서 복귀
             //stop을 여러번 하는 이유는 전진에서 후진으로 전환할 때 확실히 멈춘 상태에서 전환하기 위함
         } else if (recoveryBearing >= 135 && recoveryBearing < 180) {
             stop();
             delay(100);
             backward();  // 후진하여 도로 복귀
             right();
             delay(50);  // 후진 시간 조정
             stop();
             straight();
             delay(50);
             forward();
             //멈춘 후 후진+방향전환 해서 복귀
         } else if (recoveryBearing >= 180 && recoveryBearing < 225) {
             stop();
             delay(50);
             backward();  // 후진하여 도로 복귀
             left();
             delay(50);  // 후진 시간 조정
             stop();
             straight();
             delay(50);
             forward();
             //기본적으로 앞으로 주행
         } else {
             forward();  // 도로 복귀를 위한 전진
         }
    }else{
      float bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
      if (bearing >= 45 && bearing <= 180) {
          right();
          delay(100);  // 방향 전환 시간
          straight();  // 직진 상태로 정렬
      } else if (bearing >= 180 && bearing <= 315) {
          left();
          delay(100);  // 방향 전환 시간
          straight();  // 직진 상태로 정렬
      } else {
          forward();  //목적지로 가기 위한 전진
      }
    }
}

// 방위각 계산(현재위도,현재경도,목표위도,목표경도)
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float y = sin(radians(lon2 - lon1)) * cos(radians(lat2));
    float x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(radians(lon2 - lon1));
    return fmod((degrees(atan2(y, x)) + 360), 360);
}

// 모터 제어 함수들
void forward(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, pwmValue);
}

void backward(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, pwmValue);  // 속도 설정 (0~255)
}

void stop(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void right(){
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 255);
}

void left(){
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 255);
}

void straight(){
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}
