/*
코드 설명 : 값을 기준으로 해서 움직임만 간단히 제어하기
① MPU 6050 에서 Z속도, X가속도, Y가속도 값을 받기
② AcZ <= 22000 일 때, 모터 가동에 대한 함수 실행
  ②-1 GyX < 600 && GyY < 50 => 모터 속도는 delta(GyY)값에 비례
  ②-2 GyX > 600 && GyY > 50 => 모터 속도는 delta(GyY)값에 비례

  -1*delta(GyY)*0.8 = output
  delta(GyY) = GyY(i) - GyY(f)

*/

//----------libraries------------
#include <Wire.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>


//------------------------------------------------------------
#define OUTPUT_TEAPOT 1                    // Processing을 통해 MPU6050 센서를 Visualize 하고 싶은 경우 1, 아니면 0으로 선언합니다
#define MIN_ABS_SPEED 30                  // 모터의 최저속도를 설정합니다.   0 ~ 255 값 중 선택
#define OUTPUT_READABLE_YAWPITCHROLL    // Yaw, Pitch, Roll 값을 얻기 위해 선언합니다
#define INTERRUPT_PIN 2                  // MPU6050 센서의 INT 핀이 꽂혀있는 번호를 설정합니다. 보통 2번
#define LED_PIN 13                         // Arudino Uno의 13번핀 LED를 동작 중에 반짝거리게 하려고 선언합니다 

 
//------------------------------------------------------------
//MPU 객체를 선언합니다
MPU6050 mpu;
// MPU control/status vars
bool blinkState = false;    // LED를 반짝거리게 하기 위한 변수
bool dmpReady = false;        // set true if DMP init was successful
uint8_t mpuIntStatus;        // holds actual interrupt status byte from MPU
uint8_t devStatus;            // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;        // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;            // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];        // FIFO storage buffer
 
 
//------------------------------------------------------------
// MPU6050 센서를 통해 쿼터니언과 오일러각, Yaw, Pitch, Roll 값을 얻기 위해 선언합니다
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 
 
//------------------------------------------------------------
// Processing으로 MPU6050 센서를 Visualize 하기 위한 변수
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
 
 
//------------------------------------------------------------
// PID 제어용 변수 선언
double kp = 2.8;
double ki = 0.5;
double kd = 15;
 
 
// 기울일 각도 선택 
// 제가 만든 밸런싱로봇에는 184.0도가 가장 최적의 평형각도였습니다
// 각도가 180도를 기준으로 +-를 설정해주시면 됩니다
double originalSetpoint = 270;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.3;
 
// PID 제어용 input, output 변수를 선언합니다
double input, output;
 
// PID값을 설정해준다
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);
 
 
// 모터 제어용 변수 선언
// EnA, EnB는 속도제어용(pwm), IN1,2,3,4는 방향제어용 핀입니다
int ENA = 10;
int IN1 = 12;
int IN2 = 6;
int IN3 = 9;
int IN4 = 8;
int ENB = 11;
 
// motorController 객체 생성, 맨 끝 파라미터 1,1은 각각 좌측, 우측모터의 최대속도(%) 입니다.
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, 1, 1);
 
 
 
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
 
void dmpDataReady() {
    mpuInterrupt = true;
}

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
