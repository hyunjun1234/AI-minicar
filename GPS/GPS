#include <TinyGPSPlus.h>

// TinyGPSPlus 객체 생성
TinyGPSPlus gps;

// HardwareSerial 객체 생성 (UART2)

// 전역 변수 선언
float currentLatitude = 0.0;
float currentLongitude = 0.0;

void setup()
{
  // 시리얼 모니터 시작
  Serial.begin(115200);
  
  // Serial2를 GPIO4(RX) 및 GPIO12(TX)로 설정
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX = GPIO16, TX = GPIO17
  
  Serial.println("GPS 데이터 수신 시작");
}

void loop()
{
  // Serial2에서 데이터가 있을 경우 처리
  while (mySerial.available() > 0)
  {
    char c = mySerial.read(); // '==' -> '=' 수정
    gps.encode(c);
  }

  // GPS 위치 데이터가 업데이트되고 유효할 경우
  if (gps.location.isUpdated() && gps.location.isValid())
  {
    currentLatitude = gps.location.lat();
    currentLongitude = gps.location.lng();
    
    // 시리얼 모니터에 위도와 경도 출력
    Serial.print(currentLatitude, 6);
    Serial.print(",");
    Serial.println(currentLongitude, 6);
  }
}
