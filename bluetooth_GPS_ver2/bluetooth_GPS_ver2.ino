#include <SPIFFS.h>
#include <TinyGPS++.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLECharacteristic.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // 서비스 UUID
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8" // 특성 UUID

// L298N 모터 제어 핀 정의
const int IN1 = 19;
const int IN2 = 18;
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
float mD = 2.0;
float currentLatitude = 0.0;
float currentLongitude=0.0;
unsigned long previousMillis = 0; // 마지막으로 좌표를 출력한 시각 저장
const unsigned long interval = 1000; // 1초 간격
float distance1=0.0;
float distance=0.0;
int targetRoadPointIndex=-1;
bool visited[100]={false};
bool firstRoadDirection=false;
float FRDcla=0.0;   //FirstRoadDirectioncurrentlatitude
float FRDclo=0.0;   //FirstRoadDirectioncurrentlongitude
int FRDcpi=-1;      //FirstRoadDirectionclosestpointindex
int FRDnpi=-1;      //FirstRoadDirectionnextpointindex
float FRDbtcp=0.0;  //FirstRoadDirectionbearingTocurrentPoint
float FRDbtnp=0.0;  //FirstRoadDirectionbearingToNextPoint
float TOLERANCE = 1.0;
float heading = 0.0; // 전역 변수로 선언
float prevLatitude=0.0;
float prevLongitude=0.0;

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
void moveTowardTargetUsingRoad(float currentLat, float currentLon, float targetLat, float targetLon);
int findClosestRoadPoint(float currentLat, float currentLon);
int findClosestRoadPoint2(float mD, float currentLat, float currentLon);
int findNextRoadPoint(int currentIndex, float currentLat, float currentLon, float targetLat, float targetLon);
void loadRoadPath();  //도로 좌표 저장
void adjustDirection(float recoveryBearing);

//거리, 각도 계산
float calculateBearing(float lat1, float lon1, float lat2, float lon2) ;
int findClosestRoadPoint(float currentLat, float currentLon);
float calculateDistance(float lat1, float lon1, float lat2, float lon2);

//목표좌표 저장(파싱)
void parseCoordinates(String data);
void stop();
void right();
void left();
void forward();
void backward();
void straight();

struct GPSPoint {
    float latitude;
    float longitude;
};

GPSPoint roadPath[100];  // 도로 경로 배열
int pathLength = 0;

//사용자로부터 블루투스로 입력받은 데이터 처리
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String receivedData = pCharacteristic->getValue().c_str();  // std::string을 Arduino String으로 변환
    Serial.print("Received data: ");
    Serial.println(receivedData); // 수신한 데이터 출력

    if (receivedData.indexOf(',') != -1) {  // 좌표 값 포함
        parseCoordinates(receivedData);  // 좌표로 인식하여 파싱
        Serial.print("Parsed Target Coordinates -> Lat: ");
        Serial.print(targetLatitude, 6);
        Serial.print(", Lon: ");
        Serial.println(targetLongitude, 7);
        
        //목표 지점 도로 좌표 계산
        targetRoadPointIndex = findClosestRoadPoint(targetLatitude, targetLongitude);
        if(targetRoadPointIndex == -1){
            while (targetRoadPointIndex == -1) {
                Serial.println("No valid road point near target found. Expanding range...");
                targetRoadPointIndex = findClosestRoadPoint2(mD, targetLatitude, targetLongitude);
                mD += 2; // 검색 범위 확장
            } 
        }
        mD=2.0;
        Serial.println("Find targetRoadPoint!");
        mD = 2; // 검색 범위 초기화
        firstRoadDirection = true;
        //미니카의 첫 진행방향 선정
        if(FRDcpi==-1){
          while(FRDcpi==-1){
              FRDcpi = findClosestRoadPoint2(mD, currentLatitude, currentLongitude);
              if (FRDnpi == -1 && FRDcpi!=-1) {
                  FRDnpi = findNextRoadPoint(FRDcpi, currentLatitude, currentLongitude, roadPath[targetRoadPointIndex].latitude, roadPath[targetRoadPointIndex].longitude);
              }
              mD++;
          }
        }
        mD=2.0;
        // 자율주행이 이미 시작된 상태라면 멈추도록 설정
        if (isDriving) {
            isDriving = false; // 자율주행 종료
            stop();
            delay(100);
            Serial.println("Driving stopped due to new target coordinates.");
        }
    } else if (receivedData.toInt() > 0) {  // 숫자만 입력된 경우 PWM 값으로 인식
        pwmValue = receivedData.toInt();
        Serial.print("PWM Value set to: ");
        Serial.println(pwmValue);
    } else if (receivedData == "start") {  // start 명령으로 주행 시작
        isDriving = true;
        Serial.println("Starting autonomous driving...");
    } else if (receivedData == "stopg") {  // stop 명령으로 주행 중지
        stop(); // 미니카 정지
        delay(100);
        isDriving = false;
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


const float MAX_DISTANCE_FROM_ROAD = 5.0;  // 도로를 벗어나는 최대 허용 거리 (미터)

//도로에 해당하는 좌표들을 담은 파일에서 데이터 불러오기
void loadRoadPath() {
    File file = SPIFFS.open("/service_roads_impro.txt", "r");
    if (!file) {
        Serial.println("Failed to open road_path.txt");
        return;
    }
    //파일에 데이터가 존재하는 배열까지만 실행
    while (file.available() && pathLength <= 104) {
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
            Serial.print("Loaded point: "); // 디버깅용 출력
            Serial.print("Lat: ");
            Serial.print(roadPath[pathLength].latitude, 6);
            Serial.print(", Lon: ");
            Serial.println(roadPath[pathLength].longitude, 7);
            //roadPath배열에 현재 줄의 위도와 경도를 저장한 후 pathLength를 1 증가시킴
            //다음 줄을 읽어 저장할 때 배열의 다음 인덱스를 사용하도록 하기 위함
            pathLength++;   
        }
    }
    file.close();
    Serial.println("Road path loaded from SPIFFS");
}

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

  // SPIFFS 초기화 및 도로 경로 불러오기
  if (!SPIFFS.begin(true)) {
      Serial.println("Failed to mount SPIFFS");
      return;
  }
  loadRoadPath();
  
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

  // GPS 데이터가 있을 경우 처리
  if (Serial2.available() > 0) {
      gps.encode(Serial2.read());
      if (gps.location.isUpdated()) {
          currentLatitude = gps.location.lat();
          currentLongitude = gps.location.lng();
          
          Serial.print("Current location: ");
          Serial.print(currentLatitude, 6);
          Serial.print(", ");
          Serial.println(currentLongitude, 7);
      }
  }
}

void adjustDirection(float recoveryBearing){
         float bearingDifference = recoveryBearing - heading;
         if(recoveryBearing<heading){
           bearingDifference = 360+bearingDifference;
         }
        
         Serial.print("Target Bearing: ");
         Serial.print(recoveryBearing);
         Serial.print(", Current Heading: ");
         Serial.print(heading);
         Serial.print(", Bearing Difference: ");
         Serial.println(bearingDifference);
         
         if(bearingDifference>0 && bearingDifference<45){
            right();
            delay(300);  // 방향 전환 시간
            straight();  // 직진 상태로 정렬
            return;
         }else if (bearingDifference >= 45 && bearingDifference < 90) {
            right();
            delay(800);  // 방향 전환 시간
            straight();  // 직진 상태로 정렬
            return;
         } else if (bearingDifference >= 90 && bearingDifference <= 135) {
      //급격한 변화를 위한 딜레이
            right();
            delay(1300);  // 방향 전환 시간
            straight();  // 직진 상태로 정렬
            return;
         }else if (bearingDifference >= 135 && bearingDifference < 225) {
            stop();
            delay(100);
            backward();
            if((bearingDifference >=0 && bearingDifference < 135) || (bearingDifference>=225 && bearingDifference < 360)) {
              stop();
              return;
            }
        }else if (bearingDifference >= 225 && bearingDifference < 270) {
      //급격한 변화를 위한 딜레이
            left();
            delay(1300);  // 방향 전환 시간
            straight();  // 직진 상태로 정렬
            return;
        }else if (bearingDifference >= 270 && bearingDifference <= 315) {
            left();
            delay(800);  // 방향 전환 시간
            straight();  // 직진 상태로 정렬
            return;
        }else if (bearingDifference>315 && bearingDifference<360){
            left();
            delay(300);  // 방향 전환 시간
            straight();  // 직진 상태로 정렬
            return;
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
        if (visited[i]) continue; // 이미 방문한 도로 좌표는 건너뛴다
        //도로 좌표와 현재 좌표의 거리를 계산하여 변수에 저장
        distance1 = calculateDistance(currentLat, currentLon, roadPath[i].latitude, roadPath[i].longitude);
        //만약 거리가 설정된 최대 거리보다 작을 시 minDistance값으로 갱신
        //갱신 이후로도 더 작은 값이 나오면 갱신
        //갱신될 때만 해당 인덱스를 closestIndex에 저장
        if (distance1 < minDistance) {
            minDistance = distance1;
            closestIndex = i;
        }
    }
    Serial.print("Closest Road Point Index: ");
    Serial.println(closestIndex);
    return closestIndex;
}

// 현재 위치에서 가장 적합한 다음 도로 지점을 찾는 함수
int findNextRoadPoint(int currentIndex, float currentLat, float currentLon, float targetLat, float targetLon) {
    float targetBearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
    float minBearingDifference = 360.0; // 초기화: 최대 값
    float minDistance = 1e6;            // 초기화: 큰 값
    int nextPointIndex = -1;

    // 도로 배열 전체를 탐색
    for (int i = 0; i < pathLength; i++) {
        if (i == currentIndex || visited[i]) continue; // 현재 지점 또는 이미 방문한 지점은 제외

        float nextLat = roadPath[i].latitude;
        float nextLon = roadPath[i].longitude;

        // 현재 위치에서 다음 도로 지점까지의 거리 및 방위각
        float distanceToNextPoint = calculateDistance(currentLat, currentLon, nextLat, nextLon);
        float bearingToNextPoint = calculateBearing(currentLat, currentLon, nextLat, nextLon);

        // 목표 방향과의 방위각 차이 계산
        //fabs -> 절댓값
        float bearingDifference = fabs(targetBearing - bearingToNextPoint);
        if (bearingDifference > 180.0) {
            bearingDifference = 360.0 - bearingDifference; // 차이 값은 0~180 사이로 유지
        }

        // 최적의 다음 지점 선택 기준
        // - 목표 방향과의 방위각 차이가 작음
        // - 현재 위치와의 거리가 가까움
        if (bearingDifference < minBearingDifference ||
            (bearingDifference == minBearingDifference && distanceToNextPoint < minDistance)) {
            minBearingDifference = bearingDifference;
            minDistance = distanceToNextPoint;
            nextPointIndex = i;
        }
    }

    return nextPointIndex;
}

// 도로 경로 내 가장 가까운 좌표를 찾고 해당 좌표의 인덱스 반환
int findClosestRoadPoint2(float mD, float currentLat, float currentLon) {
    float minDistance = MAX_DISTANCE_FROM_ROAD+mD;
    int closestIndex = -1;
    //불러온 spiffs의 도로좌표들의 배열 수만큼 반복
    //현재위치와 가장 가까운 도로좌표를 찾아내서 해당 배열의 번수를 변수에 저장
    for (int i = 0; i < pathLength; i++) {
        //도로 좌표와 현재 좌표의 거리를 계산하여 변수에 저장
        float distance = calculateDistance(currentLat, currentLon, roadPath[i].latitude, roadPath[i].longitude);
        //만약 거리가 설정된 최대 거리보다 작을 시 minDistance값으로 갱신
        //갱신 이후로도 더 작은 값이 나오면 갱신
        //갱신될 때만 해당 인덱스를 closestIndex에 저장
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i;
        }
    }
    return closestIndex;
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
  delay(100);
}

void left(){
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 255);
  delay(100);
}

void straight(){
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}

void moveTowardTargetUsingRoad(float currentLat, float currentLon, float targetLat, float targetLon) {
    // 현재 위치에서 가장 가까운 도로 지점 찾기
    int closestPointIndex = findClosestRoadPoint(currentLat, currentLon);
    if(closestPointIndex == -1){
      while(closestPointIndex == -1) {
        Serial.println("Find on more large range");
        closestPointIndex=findClosestRoadPoint2(mD,currentLat,currentLon);
        mD=mD+2;
      }
    }
    mD=2;

    //목적지에 도달 시 정지
    if (closestPointIndex == targetRoadPointIndex) {
        Serial.println("Reached the target road point. Stopping the car.");
        stop();
        delay(200);
        isDriving = false;  // 주행 중지
        
        mD=2;
    }
    
    // 현재 위치에서 가장 적합한 다음 도로 지점 선택
    int nextPointIndex = findNextRoadPoint(closestPointIndex, currentLat, currentLon, targetLat, targetLon);

    if (nextPointIndex == -1) {
        Serial.println("No valid next road point found. Stopping...");
        stop();
    }

    float nextLat = roadPath[nextPointIndex].latitude;
    float nextLon = roadPath[nextPointIndex].longitude;

    visited[closestPointIndex] = true; // 현재 위치를 방문 상태로 기록
    Serial.print("Moving to next road point: ");
    Serial.print(nextLat, 6);
    Serial.print(", ");
    Serial.println(nextLon, 6);

    // 다음 지점으로 이동
    float roadBearing = calculateBearing(currentLat, currentLon, nextLat, nextLon);
    Serial.print("Moving Toward Road Point Index ");
    Serial.print(nextPointIndex);
    Serial.print(" -> Bearing: ");
    Serial.println(roadBearing);
    forward();
    adjustDirection(roadBearing);
}


//실행문
void loop() {
    // 현재 시간 저장
    unsigned long currentMillis = millis();

    // GPS 데이터가 있을 경우 처리
    if (Serial2.available() > 0) {
        gps.encode(Serial2.read());
        if (gps.location.isUpdated()) {
            currentLatitude = gps.location.lat();
            currentLongitude = gps.location.lng();

            // 1초 간격으로 출력
            if (currentMillis - previousMillis >= interval) {
                previousMillis = currentMillis; // 마지막 출력 시각 갱신
                Serial.print("Current location: ");
                Serial.print(currentLatitude, 6);
                Serial.print(", ");
                Serial.println(currentLongitude, 7);
            }

            if (prevLatitude != 0.0 && prevLongitude != 0.0) {
                heading = calculateBearing(prevLatitude, prevLongitude, currentLatitude, currentLongitude);
                Serial.print("Heading: ");
                Serial.println(heading);
            }
            
            prevLatitude = currentLatitude;
            prevLongitude = currentLongitude;
            
        }
    }
  //시작 전 미니카 테스트
    if (command == "go"){
        forward();
        command="";
    }else if (command == "back") {
        backward();
        command = "";  // 명령 실행 후 초기화
    }else if (command == "stop") {
        stop();
        delay(100);
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
      if(firstRoadDirection==true){
        while(firstRoadDirection){
          FRDbtcp=calculateBearing(currentLatitude,currentLongitude,roadPath[FRDcpi].latitude, roadPath[FRDcpi].latitude);
          forward();
          adjustDirection(FRDbtcp);
          float distance2 = calculateDistance(currentLatitude, currentLongitude, roadPath[FRDcpi].latitude, roadPath[FRDcpi].longitude);
          Serial.print("Distance to first road point: ");
          Serial.println(distance2);
          // WDT 리셋
          
            
        // 거리가 허용 오차 이내이면 정지
          if (distance2 <= TOLERANCE) {
             Serial.println("Reached first road point. Stopping the car.");
             stop();
             delay(100);
             break;
          } 
        }
      }
      visited[FRDcpi] = true; // 현재 위치를 방문 상태로 기록
      delay(500);
      if(firstRoadDirection==true){
        while(firstRoadDirection){
          FRDbtnp=calculateBearing(currentLatitude,currentLongitude,roadPath[FRDnpi].latitude, roadPath[FRDnpi].latitude);
          adjustDirection(FRDbtnp);
          forward();
          float distance3 = calculateDistance(currentLatitude, currentLongitude, roadPath[FRDnpi].latitude, roadPath[FRDnpi].longitude);
          Serial.print("Distance to next road point: ");
          Serial.println(distance3);
          
        // 거리가 허용 오차 이내이면 정지
          if (distance3 <= TOLERANCE) {
            Serial.println("Reached second road point. Stopping the car.");
            stop();
            delay(100);
            firstRoadDirection=false;
            delay(50);
          } 
        }
      }
      visited[FRDnpi] = true; // 현재 위치를 방문 상태로 기록
      delay(200);
      //현재 위치과 목표 지점 사이의 거리를 계산하여 변수에 저장
      float distanceToTarget = calculateDistance(currentLatitude, currentLongitude, roadPath[targetRoadPointIndex].latitude, roadPath[targetRoadPointIndex].longitude);
      Serial.print("Distance to target: ");
      Serial.println(distanceToTarget);
      if (distanceToTarget <= 3) {
         stop();
         delay(100);
         Serial.println("Arrived at target location.");
         isDriving = false;  // 목표 도달 후 주행 멈춤
      }
      moveTowardTargetUsingRoad(currentLatitude, currentLongitude, roadPath[targetRoadPointIndex].latitude, roadPath[targetRoadPointIndex].longitude);
    }
}
