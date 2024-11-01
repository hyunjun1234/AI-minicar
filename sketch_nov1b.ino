#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;               // GPS 객체 생성
HardwareSerial SerialGPS(2);    // GPS와 연결된 시리얼 포트 설정

const int ppsPin = 18;          // PPS 핀이 연결된 GPIO 핀

volatile bool ppsTriggered = false; // PPS 신호가 발생했는지 확인하는 플래그
volatile bool stopMeasurement = false; // 측정 중지 플래그


void IRAM_ATTR handlePPS() {
  // PPS 핀에서 신호가 발생할 때 호출되는 인터럽트 핸들러
  ppsTriggered = true;
}

void setup() {
  Serial.begin(115200);         // 디버깅용 시리얼 모니터 설정
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);  // GPS 모듈 설정 (TX=17, RX=16)

  pinMode(ppsPin, INPUT);       // PPS 핀을 입력 모드로 설정
  attachInterrupt(digitalPinToInterrupt(ppsPin), handlePPS, RISING); // PPS 핀에 상승 엣지 인터럽트 설정

  Serial.println("Waiting for GPS signal...");
}

void loop() {
  // 시리얼 모니터에서 'stop' 명령어를 입력하면 측정 중지
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input == "stop") {
      stopMeasurement = true;
    }
  }

  if (stopMeasurement) {
    Serial.println("Measurement stopped by user.");
    while (true);  // 측정을 멈추고 프로그램 멈춤
  }
  // PPS 신호가 발생하면 시간 정보 출력
  if (ppsTriggered) {
    ppsTriggered = false; // 플래그 초기화

    if (gps.time.isValid()) {  // GPS 모듈로부터 유효한 시간 데이터가 수신된 경우에만 출력
      int hour = (gps.time.hour() + 9) % 24;  // UTC 시간에 9시간 더하고 24로 나눈 나머지 (한국 시간)
      int minute = gps.time.minute();
      int second = gps.time.second();
      
      Serial.print("PPS signal received! Current Time (KST): ");
      Serial.print(hour);         // 시(hour) 출력
      Serial.print(":");
      Serial.print(minute);       // 분(minute) 출력
      Serial.print(":");
      Serial.println(second);     // 초(second) 출력
    } else {
      Serial.println("PPS signal received, but time data is not valid yet.");
    }
  }

  // GPS 모듈에서 데이터 수신
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());  // GPS 데이터를 TinyGPS++로 파싱
    
    if (gps.location.isUpdated()) {  // 위치 정보가 업데이트되었을 때만 출력
      Serial.print("Latitude: ");
      Serial.print(gps.location.lat(), 6);  // 위도 출력
      Serial.print(" Longitude: ");
      Serial.println(gps.location.lng(), 6);  // 경도 출력
      
      Serial.print("Altitude (m): ");
      Serial.println(gps.altitude.meters());  // 고도 출력
      
      Serial.print("Speed (km/h): ");
      Serial.println(gps.speed.kmph());       // 속도 출력
      
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value()); // 위성 수 출력

      // UTC 시간을 한국 시간(KST)으로 변환하여 출력
      if (gps.time.isValid()) {
        int hour = (gps.time.hour() + 9) % 24;  // 한국 시간 (UTC + 9)
        int minute = gps.time.minute();
        int second = gps.time.second();
        
        Serial.print("Time (KST): ");
        Serial.print(hour);         // 시(hour) 출력
        Serial.print(":");
        Serial.print(minute);       // 분(minute) 출력
        Serial.print(":");
        Serial.println(second);     // 초(second) 출력
      } else {
        Serial.println("Time data is not valid yet.");
      }
      
      Serial.println();
    }
  }
}
