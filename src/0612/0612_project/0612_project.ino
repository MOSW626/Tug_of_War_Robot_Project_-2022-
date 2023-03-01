// 코드 설명 : 값을 기준으로 해서 움직임만 간단히 제어하기
// ① MPU 6050 에서 Z속도, X가속도, Y가속도 값을 받기
// ② AcZ <= 22000 일 때, 모터 가동에 대한 함수 실행
// ②-1 GyX < 600 && GyY < 50 => 모터 속도는 delta(GyY)값에 비례
// ②-2 GyX > 600 && GyY > 50 => 모터 속도는 delta(GyY)값에 비례

// -1*delta(GyY)*0.8 = output
// delta(GyY) = GyY(i) - GyY(f)

//----------libraries------------
#include<Wire.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

const int MPU=0x68;  //MPU 6050 의 I2C 기본 주소
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int output;

void setup(){
  Wire.begin();      //Wire 라이브러리 초기화
  Wire.beginTransmission(MPU); //MPU로 데이터 전송 시작
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     //MPU-6050 시작 모드로
  Wire.endTransmission(true); 
  Serial.begin(115200);
  Serial.println("CLEARDATA");
  Serial.println("LABEL, AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ");
  
  //모터 출력핀 초기화
  pinMode (6, OUTPUT);
  pinMode (9, OUTPUT);
  pinMode (10, OUTPUT);
  pinMode (11, OUTPUT);

  //모터 동작 OFF
  analogWrite(6,LOW);
  analogWrite(9,LOW);
  analogWrite(10,LOW);
  analogWrite(11,LOW);
}

void loop(){
  Wire.beginTransmission(MPU);    //데이터 전송시작
  Wire.write(0x3B);               // register 0x3B (ACCEL_XOUT_H), 큐에 데이터 기록
  Wire.endTransmission(false);    //연결유지
  Wire.requestFrom(MPU,14,true);  //MPU에 데이터 요청
  //데이터 한 바이트 씩 읽어서 반환
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  Tmp=Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();
  
  //시리얼 모니터에 출력
  Serial.print("DATA,");
  Serial.print(AcX);
  Serial.print(",");
  Serial.print(AcY);
  Serial.print(",");
  Serial.print(AcZ);
  Serial.print(",");
  Serial.print(Tmp);
  Serial.print(",");
  Serial.print(GyX);
  Serial.print(",");
  Serial.print(GyY);
  Serial.print(",");
  Serial.print(GyZ);
  Serial.println();
  delay(333);
}
