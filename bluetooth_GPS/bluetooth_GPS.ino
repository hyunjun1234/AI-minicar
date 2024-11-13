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
const int IN2 = 5;
const int ENA = 19;
const int IN3 = 17;
const int IN4 = 16;
const int ENB = 4;

//bluetooth 관련 변수 정의
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
String command = "";  // 현재 명령을 저장할 변수
int pwmValue = 0;
bool isDriving = false; // 주행 시작 여부를 나타내는 플래그
float targetLatitude = 0.0;
float targetLongitude = 0.0; 

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
void parseCoordinates(String data);
void stop();

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
  Serial2.begin(9600, SERIAL_8N1, 13, 12); // GPS 통신 설정(,,RX,TX)
  delay(100);
  
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

void loadRoadPath() {
    File file = SPIFFS.open("/service_roads.csv", "r");
    if (!file) {
        Serial.println("Failed to open road_path.csv");
        return;
    }

    while (file.available() && pathLength < 100) {
        String line = file.readStringUntil('\n');
        int commaIndex = line.indexOf(',');
        if (commaIndex > 0) {
            roadPath[pathLength].latitude = line.substring(0, commaIndex).toFloat();
            roadPath[pathLength].longitude = line.substring(commaIndex + 1).toFloat();
            pathLength++;
        }
    }
    file.close();
    Serial.println("Road path loaded from SPIFFS");
}

void loop() {
    // GPS 모듈로 현재 위치 확인
    if (isDriving) {  // 주행이 시작된 상태일 경우에만 실행
        while (Serial2.available() > 0) {
            gps.encode(Serial2.read());
            if (gps.location.isUpdated()) {
                float currentLatitude = gps.location.lat();
                float currentLongitude = gps.location.lng();
                Serial.print("Current location: ");
                Serial.print(currentLatitude, 6);
                Serial.print(", ");
                Serial.println(currentLongitude, 6);

                float distanceToTarget = calculateDistance(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
                Serial.print("Distance to target: ");
                Serial.println(distanceToTarget);

                if (distanceToTarget > 3) {  // 목표 지점에 도달하지 않은 경우
                    moveTowardTarget(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
                } else {
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

// 도로 경로 내 가장 가까운 좌표를 찾고 도로 유지 여부 확인
int findClosestRoadPoint(float currentLat, float currentLon) {
    float minDistance = MAX_DISTANCE_FROM_ROAD;
    int closestIndex = -1;
    for (int i = 0; i < pathLength; i++) {
        float distance = calculateDistance(currentLat, currentLon, roadPath[i].latitude, roadPath[i].longitude);
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i;
        }
    }
    return closestIndex;
}

// 목표 위치로 이동 방향 결정
void moveTowardTarget(float currentLat, float currentLon, float targetLat, float targetLon) {
    int closestPointIndex = findClosestRoadPoint(currentLat, currentLon);
    if (closestPointIndex == -1) {
         // 가까운 도로 경로 지점의 좌표를 찾고 방위각을 계산
        float nearestLat = roadPath[0].latitude;
        float nearestLon = roadPath[0].longitude;
        float recoveryBearing = calculateBearing(currentLat, currentLon, nearestLat, nearestLon);

        if (recoveryBearing >= 45 && recoveryBearing <= 135) {
            right();  // 오른쪽으로 틀어 도로 복귀
            delay(100);  // 짧은 전환 시간으로 미세 조정
            straight();  // 직진 상태로 정렬하여 천천히 복귀
            delay(200);
            forward();
        } else if (recoveryBearing >= 225 && recoveryBearing <= 315) {
            left();   // 왼쪽으로 틀어 도로 복귀
            delay(100);
            straight();  // 직진 상태로 정렬하여 천천히 복귀
            delay(200);
            forward();
        } else if (recoveryBearing >= 135 && recoveryBearing < 225) {
            backward();  // 후진하여 도로 복귀
            delay(300);  // 후진 시간 조정
            forward();
        } else {
            forward();  // 도로 복귀를 위한 전진
            delay(300);
            forward();
        }
        Serial.println("Off road! Adjusting direction to return to road.");
        return;
    }

    float targetRoadLat = roadPath[closestPointIndex].latitude;
    float targetRoadLon = roadPath[closestPointIndex].longitude;
    float bearing = calculateBearing(currentLat, currentLon, targetRoadLat, targetRoadLon);

    if (bearing >= 45 && bearing <= 135) {
        right();
        delay(200);  // 방향 전환 시간
        straight();  // 직진 상태로 정렬
    } else if (bearing >= 225 && bearing <= 315) {
        left();
        delay(200);  // 방향 전환 시간
        straight();  // 직진 상태로 정렬
    } else {
        forward();
    }
}

// 방위각 계산
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

void left(){
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite(ENB,255);
}

void right(){
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite(ENB,255);
}

void straight(){
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}
