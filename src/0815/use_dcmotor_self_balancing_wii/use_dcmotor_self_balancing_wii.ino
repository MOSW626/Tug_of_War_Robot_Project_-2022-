/* 사용 제품 스펙
IMU : mpu6050
Motor Driver : Sabertooth Simplified
PID controll : 가변저항을 통한 PID 계수 조절
*/

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>
#include <SoftwareSerial.h>             // motor driver를 위한 핀 설정
#include <SabertoothSimplified.h>       // motro driver

//              변수 설정
SoftwareSerial SWSerial(NOT_A_PIN, 11);          // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial);               // Use SWSerial as the serial port.

MPU6050 mpu;

int16_t ax, ay, az;   // MPU6050 센서의 x, y 및 z 방향의 가속도 값
int16_t gx, gy, gz;   // MPU6050 센서의 x, y 및 z 방향 각속도 값

#define Pin_steering_right      12               // 조향 명령 오른쪽 핀 연결
#define Pin_steering_letf       13               // 조향 명령을 위한 핀 연결 왼쪽

#define Pin_Switch_Box        3                // 모터 동기화와 I 제어 중에서 선택하기 위한 박스 스위치의 핀 연결

#define Pin_PID_Regulation_P      A1               // P 구성 요소를 변경하기 위한 전위차계용 핀 연결
#define Pin_PID_Regulation_I      A2               // I 구성 요소를 변경하기 위한 전위차계용 핀 연결
#define Pin_PID_Regulation_D      A3               // D 구성 요소를 변경하기 위한 전위차계용 핀 연결


int LoopTime_Target = 9;                           // 100Hz에 도달하는 데 필요한 루프 지속 시간(ms)
int LoopTime_Adjusted = LoopTime_Target;          // 강제 중단이 있는 마지막 루프 시간
int LoopTime_Bisher = LoopTime_Target;             // 강제 중단 없는 마지막 루프 시간
unsigned long LoopTime_Start = 0;		 // 루프 시작 시간

float Theta;					 // 현재 기울기 각도
float Theta_f;                               // 경사각의 목표값, 즉 0°
float Theta_Border;                             // Segway가 종료되는 최대 허용 경사각

float ACC_angle;				 // 가속도계에서의 각도
float GYRO_rate;		                 // 자이로 센서의 각속도

float Kp,Ki,Kd,K;                                // PID 제어 상수, 미분부, 적분부, 미분부, 전체부
int Motor;                                       // PID 제어에서 얻은 모터 제어 값
int Motor_right, Motor_left;                   // 두 엔진의 값
float K_Motor_left, K_Motor_right;             // 두 모터의 동기 구동에 대한 보정 계수

int Switch_Box;                                // 상자의 스위치 위치를 쿼리하는 변수

int Steering_Input_right = 0;                  // 우측 조향 명령을 감지하기 위한 변수
int Steering_Input_left = 0;                   // 왼쪽으로의 조향 명령을 감지하기 위한 변수
float Steering_max;                               // 조향 명령으로 모터 제어가 변경되어야 하는 최대값
float Steering_right, Steering_left;             // 현재 및 좌우로 조향할 때 점차적으로 증가하는 제어 값


// ***********************************************************************************
// ****************************** Calibration ***************************************
// ***********************************************************************************


void calibrateSensors()              // 센서 제로 값의 일회성 결정(각각 50회 측정의 평균값)
   {
    
     // ================================================ =============== =
     // ========== 센서 해상도 변경 ==============
     // ================================================ =============== =
    
     // ================================================ ================================================= ==
     // 장치에서 RAW Accel/자이로 측정을 읽습니다
     // 출력 값 -ACC : 해상도 2G : 16384/g 해상도 4G : 8192/g
     // 출력 값 자이로 : 해상도 250 °/S : 131/°/s 해상도 500 °/s : 65.5/°/s
     // ================================================ ================================================= ==
    
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
       
     
    
    // 처음에 두 엔진의 "굉음"을 피하십시오.
    
    ST.motor(1, 0);
    ST.motor(2, 0);
    
   
        
    Theta_f = 0.0;                 // 기울기 각도의 목표값
    Theta_Border = 30.0;              // 최대 허용 경사각
    
    
    // *******************************************************
    // ********** K - PID 제어 값 *************
    // *******************************************************
    
    
    Kp = analogRead(Pin_PID_Regulation_P) * 25.0 / 1023.0;        // 전위차계로 고정된 차이 비율
    Ki = 0.1;                                                   // 전위차계로 고정된 일체형 부품(그러나 모터 수정을 켤 수 있으므로 처음에는 0.1로 고정됨)  
    Kd = analogRead(Pin_PID_Regulation_D) * 100.0 / 1023.0;       // 전위차계로 고정된 차동 부품
    K = 1.0;                                                    // 총 점유율


    // **************************************************
    // ********** K - 모터 값 *************
    // **************************************************


    pinMode(Pin_Switch_Box, INPUT);      // I 제어와 모터 동기화 중에서 선택하기 위한 핀
    
    K_Motor_right = 1.0;                  // 오른쪽 모터에 대한 보정 계수
    K_Motor_left = 0.8;                   // 왼쪽 모터의 보정 계수
      
    
    // **********************************************
    // ********** 스티어링 값 *************
    // **********************************************
    
    
    Steering_max = 25.0;                      // 모터 제어가 조향 명령으로 MAXIMUM을 변경해야 하는 값
    Steering_right = 0.0;                    // 오른쪽으로 조향할 때 현재 추가 값
    Steering_left = 0.0;                     // 왼쪽으로 조향할 때 현재 추가 값
 
    pinMode(Pin_steering_right, INPUT);      // 오른쪽 스티어링용 핀이 입력으로 선언되었습니다.
    pinMode(Pin_steering_letf, INPUT);       // 왼쪽 스티어링 핀이 입력으로 선언됨
      
   }

// ********************************************************************************************
// ****************** 직렬 인터페이스로 값 출력 ******************************
// ********************************************************************************************

void Value_output()
   {
    /*
    Serial.print(Theta);
    Serial.print("     ");
    Serial.println(Motor);
    Serial.print("     ");
    */
    
    Serial.print("a_y = ");
    Serial.print(ay/16384.0);
    Serial.print("    a_z = ");
    Serial.print(az/16384.0);
    Serial.print("    ACC_angle = ");
    Serial.print(ACC_angle,0);
    Serial.print("    GYRO_rate = ");
    Serial.print(GYRO_rate,0);
    Serial.print("    Theta: ");
    Serial.println(Theta,0);
    
    /*
    Serial.print("   Motor: ");
    Serial.print(Motor);
    Serial.print("    Motor_right: ");
    Serial.print(Motor_right);
    Serial.print("    Motor_left: ");
    Serial.println(Motor_left);
    */
    
   }


// ******************************************************************************************************
// ***************************************** PID 제어 **********************************************
// ******************************************************************************************************

float error;
float last_error = 0;
float pTerm;
float iTerm;
float dTerm;
float integrated_error = 0;
int GUARD_GAIN = 40;           // 최대 적분 각도 오차

   int pid(float Theta_currently, float Theta_Spec, float Thetaspeed)
      {
       error = Theta_Spec - Theta_currently;
       
       pTerm = Kp * error;                                                         // 차이 점유율
       
       
       integrated_error = integrated_error + error;
   
       iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);          // 중요한 부분
  
       
       dTerm = Kd * Thetaspeed / 100.0;                                 // 차동 성분; :100 사용 가능한 값을 얻으십시오!
       
       /*
       Serial.print("    K_p: ");
       Serial.print(pTerm);
       Serial.print("    K_d: ");
       Serial.println(dTerm);
       */
  
       last_error = error;
  
       // Serial.println(K*(pTerm + iTerm + dTerm));
       
  
       return constrain(K*(pTerm + iTerm + dTerm), -127, 127);                     // 제한 [-127.127] 내에서 두 엔진에 대한 엔진 값의 출력
      } 

 
 

// ******************************************************************************************************
// ************************************** 칼만 필터 모듈 ******************************************
// ******************************************************************************************************


    float Q_angle  =  0.001;		// E(alpha2) = 0.001
    float Q_gyro   =  0.003;  		// E(bias2) = 0.003
    float R_angle  =  0.001;  		// Sz = 0.03   !!! 숫자가 클수록 각도가 변화에 덜 민감하게 반응합니다 !!!
    float x_angle = 0;
    float x_bias = 0;
    float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
    float dt, y, S;
    float K_0, K_1;

  float kalmanCalculate(float newAngle, float newRate, int looptime)
     {
      dt = float(looptime)/1000;    // 즉, 초
      x_angle = x_angle + dt * (newRate - x_bias);
      P_00 = P_00 - dt * (P_10 + P_01) + Q_angle * dt;
      P_01 = P_01 - dt * P_11;
      P_10 = P_10 - dt * P_11;
      P_11 = P_11 + Q_gyro * dt;

      y = newAngle - x_angle;
      S = P_00 + R_angle;
      K_0 = P_00 / S;
      K_1 = P_10 / S;

      x_angle +=  K_0 * y;
      x_bias  +=  K_1 * y;
      P_00 -= K_0 * P_00;
      P_01 -= K_0 * P_01;
      P_10 -= K_1 * P_00;
      P_11 -= K_1 * P_01;

      return x_angle;
     }



// ********************************************************************
// ******** PUI 매개변수를 변경하기 위한 키보드 프롬프트 *********
// ********************************************************************

int Keyboard_input()
   {
    if(!Serial.available())    return 0;
   
    char param = Serial.read();                            // 매개변수 바이트 가져오기
  
    if(!Serial.available())    return 0;
  
    char cmd = Serial.read();                              // 명령 바이트 가져오기
  
    Serial.flush();                                         // Serial 통신 이후 남아있는 값 제거
  
    switch (param)
       {
        case 'p':
           if(cmd=='+')    Kp++;
           if(cmd=='-')    Kp--;
           break;
        case 'i':
           if(cmd=='+')    Ki += 0.1;
           if(cmd=='-')    Ki -= 0.1;
           break;
        case 'd':
           if(cmd=='+')    Kd++;
           if(cmd=='-')    Kd--;
           break;
       case 'k':
           if(cmd=='+')    K += 0.2;
           if(cmd=='-')    K -= 0.2;
           break;
       case 'l':
           if(cmd=='+')    K_Motor_left += 0.1;
           if(cmd=='-')    K_Motor_left -= 0.1;
           break;
       case 'r':
           if(cmd=='+')    K_Motor_right += 0.1;
           if(cmd=='-')    K_Motor_right -= 0.1;
           break;
     
       default:
           Serial.print("?");           Serial.print(param);
           Serial.print(" ?");          Serial.println(cmd);
      }
  
    Serial.println();
    Serial.print("K:");                      Serial.print(K);
    Serial.print("   Kp:");                  Serial.print(Kp);
    Serial.print("   Ki:");                  Serial.print(Ki);
    Serial.print("   Kd:");                  Serial.print(Kd);
    Serial.print("   K_Motor_left:");       Serial.print(K_Motor_left);
    Serial.print("   K_Motor_right:");      Serial.println(K_Motor_right);
   } 





// ****************************************************************************
// ****************************** SETUP ***************************************
// ****************************************************************************
void setup()
   {
    Wire.begin(); 
     
    //SWSerial.begin(9600);     // 이것은 DIP 스위치로 선택한 전송 속도입니다.
     
    Serial.begin(9600);    // 직렬 모니터가 값을 확인하기 위한 전송 속도
    
    // initialize device
    mpu.initialize();
            
    calibrateSensors();       // 센서의 일회성 교정을 위한 서브프로그램
   }

// ***************************************************************************************************************************************************
// ***************************************************************************************************************************************************
// ********************************************************************** 메인 루프 **************************************************************
// ***************************************************************************************************************************************************
// ***************************************************************************************************************************************************

void loop()
   {

   // *******************************************************
   // ********************* 센서 쿼리 *******************
   // *******************************************************
   
    // ===========================================================================
    // 장치에서 원시 가속도/자이로 측정값 읽기
    // 출력 값-Acc: 분해능 2g: 16384/g 분해능 4g: 8192/g
    // 출력값 자이로: 분해능 250°/s: 131/°/s 분해능 500°/s: 65.5/°/s
    // ===========================================================================
    
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   
    ACC_angle = atan(ay * 1.0 / az * 1.0) * 180.0 / 3.141592654;       // 해상도 2g: 16384/g
   
    //ACC_angle = atan((ay/16384.0) / (az/16384.0)) * 180.0 / 3.141592654; // 해상도 2g: 16384/g
    
    GYRO_rate = gx/65.5;                                 // 분해능 500°/s: 65.5/°/s

     
     
   // *******************************************************
   // ********** K - PID 제어 값 *************
   // *******************************************************
    
    
    Kp = analogRead(Pin_PID_Regulation_P) * 25.0 / 1023.0;                     // 전위차계로 고정된 차이 비율; 최대 = 25
    Kd = analogRead(Pin_PID_Regulation_D) * 100.0 / 1023.0;                    // 전위차계로 고정된 차동 부품; 최대 = 100

    Switch_Box = digitalRead(Pin_Switch_Box);                            // 상자의 스위치 상태에 대한 핀 쿼리
    
    if (Switch_Box == HIGH)                                                // 내가 제어하는 ​​상자의 스위치를 통해 활성화됨
       {
        Ki = analogRead(Pin_PID_Regulation_I) * 2.0 / 1023.0;                  // 냄비로 고정된 적분항; 최대 = 2
       }
    else                                                                     // 박스 엔진 컨트롤의 스위치로 활성화
        {
         K_Motor_right = analogRead(Pin_PID_Regulation_I) * 2.0 / 1023.0;     // 두 모터의 동기 구동에 대한 보정 계수; 최대 = 2
        }

      
     
     
     // ********************************************************************************
     // ****************** 칼만 필터, PWM 계산 및 모터 값 *****************
     // ********************************************************************************
     
     
     Theta = kalmanCalculate(ACC_angle, GYRO_rate, LoopTime_Adjusted);	// 칼만 필터로 계산된 각도

     
     if (Theta > Theta_Border || Theta < (Theta_Border * (-1)))
        {
         // ===============================================
         // 너무 큰 경사각으로 인한 취소!
         // ===============================================
         
         ST.motor(1, 0);
         ST.motor(2, 0);
        }
     else
        {
         // =========================
         // 경사각 OK
         // =========================
      
 
         Motor = pid(Theta, Theta_f, GYRO_rate);      // 모터 제어를 위한 PWM 값 계산
     
         Motor_right = K_Motor_right * Motor;            // 오른쪽 모터에 대한 K 계수 동기 모터 속도 계산

         Motor_left = K_Motor_left * Motor;              // 왼쪽 모터의 K 계수 동기 모터 속도 계산
     
         
          
         // **************************************************************************************
         // ***** 조향이 작동되었는지 확인하고 모터 제어를 변경합니다. *****
         // **************************************************************************************
     
     
         Steering_Input_right = digitalRead(Pin_steering_right);   // 오른쪽 스티어링 핀 쿼리

         if (Steering_Input_right == HIGH)
            {     
              // ******************************************
              // *** 스티어링 오른쪽을 눌렀습니다. ***
              // ******************************************
          
              if (Motor_right >= 0)    // 세그웨이가 직진하거나 정지해 있습니다. 어떤 모터가 쿼리되는지는 중요하지 않습니다.
                 {
                  Motor_right = Motor_right - (int)Steering_right;   // 인수를 곱해 볼 수도 있습니다(예: * (1 - 0.1).
                  Motor_left = Motor_left + (int)Steering_right;     // 인수를 곱해 볼 수도 있습니다(예: * (1 + 0.1)).
                 }
              else                      // segway fährt gerade rückwärts
                 {
                  Motor_right = Motor_right + (int)Steering_right;   // 인수를 곱해 볼 수도 있습니다(예: * (1 + 0.1).
                  Motor_left = Motor_left - (int)Steering_right;     // 인수를 곱해 볼 수도 있습니다(예: * (1 - 0.1).
                 }
                 
             Steering_right = Steering_right + 0.05;                                 // 스티어링이 너무 급격하지 않도록 쿼리당 0.1만큼만 늘리는 것이 좋습니다!
             
             if (Steering_right > Steering_max) Steering_right = Steering_max;        // 최대 조향 값을 초과해서는 안 됩니다!
             
             //Steering_right = constrain(Steering_right, 0, Steering_max);          // 간격 [0,steering_max]로 가져온 오른쪽 스티어링 값.
            } 
         else
            {
             Steering_right = 0.0;
            }
    
    
         Steering_Input_left = digitalRead(Pin_steering_letf);    // 왼쪽으로 조종하기 위해 핀 쿼리

         if (Steering_Input_left == HIGH)
            {     
              // *****************************************
              // *** 왼쪽 스티어링을 눌렀습니다. ***
              // *****************************************
          
              if (Motor_left >= 0)    // 세그웨이가 직진하거나 정지해 있습니다. 어떤 엔진이 쿼리되는지는 중요하지 않습니다.
                 {
                  Motor_right = Motor_right + (int)Steering_left;   // 인수를 곱해 볼 수도 있습니다(예: * (1 + 0.1).
                  Motor_left = Motor_left - (int)Steering_left;     // 인수를 곱해 볼 수도 있습니다(예: * (1 - 0.1).
                 }
              else                      // segway fährt gerade rückwärts
                 {
                  Motor_right = Motor_right - (int)Steering_left;   // 인수를 곱해 볼 수도 있습니다(예: * (1 - 0.1).
                  Motor_left = Motor_left + (int)Steering_left;     // 인수를 곱해 볼 수도 있습니다(예: * (1 + 0.1).
                 }
                 
             Steering_left = Steering_left + 0.05;                                 // 스티어링이 너무 급격하지 않도록 쿼리당 0.1만큼만 늘리는 것이 좋습니다!
             
             if (Steering_left > Steering_max) Steering_left = Steering_max;        // 최대 조향 값을 초과해서는 안 됩니다!
             
             //Steering_left = constrain(Steering_left, 0, Steering_max);          // 왼쪽 스티어링 값을 [0,steering_max] 간격으로 가져왔습니다.
            } 
         else
            {
             Steering_left = 0.0;
            }
       
        
        
     
         // *******************************************************************************************
         // ******************************** 모터 제어  ***********************************
         // *******************************************************************************************
        
         
         Motor_right = constrain(Motor_right, -127, 127);          // 오른쪽 엔진 값을 [-127,127] 간격으로 가져왔습니다.
         Motor_left = constrain(Motor_left, -127, 127);            // 왼쪽 엔진 값을 [-127,127] 간격으로 가져왔습니다.
         
                      
     /*
         // 낮은 모터 값에서 응답 동작을 개선하기 위해 선형 구동 함수 대신 루트 함수 사용
         // ======================================================================================================================================
         
         if (Motor_right >= 0)     // 오른쪽 모터가 앞으로 회전
            { 
             Motor_right = sqrt(127 * Motor_right);             // 낮은 엔진 값에서 응답 동작을 개선하기 위해
              
             ST.motor(2, Motor_right);      
            }
         else                       // 오른쪽 모터가 뒤로 회전
            {
             Motor_right = -sqrt(127 * -Motor_right);           // 낮은 엔진 값에서 응답 동작을 개선하기 위해
             
             ST.motor(2, Motor_right);               
            }


         if (Motor_left >= 0)      // 왼쪽 모터가 앞으로 회전
            {
             Motor_left = sqrt(127 * Motor_left);               // 낮은 엔진 값에서 응답 동작을 개선하기 위해
             Value_output
             ST.motor(1, Motor_left);               
            }
         else                       // 왼쪽 모터가 뒤로 회전
            {
             Motor_left = -sqrt(127 * -Motor_left);             // 낮은 엔진 값에서 응답 동작을 개선하기 위해
             
             ST.motor(1, Motor_left);  
            }
         */
         
         ST.motor(1, Motor_left);
         ST.motor(2, Motor_right);
         
        } 


   // ************************************************************************ 
   // *********************** 측정값 출력 **************************
   // ************************************************************************

    Value_output();         // 값 출력 함수


   // ******************************************************************
   // *********************** 키보드 쿼리 **************************
   // ******************************************************************

   // Keyboard_input();



   // **********************************************************************
   // *********************** 루프 타이밍 제어 **************************
   // **********************************************************************

     LoopTime_Bisher = millis() - LoopTime_Start;        // 마지막 루프 이후 시간
     
     if(LoopTime_Bisher < LoopTime_Target)
        {
         delay(LoopTime_Target - LoopTime_Bisher);         // 동일한 루프 시간을 얻기 위한 지연
        }
     
     LoopTime_Adjusted = millis() - LoopTime_Start;     // 마지막 루프의 업데이트된 지속 시간은 LoopTime_f = 예를 들어 10msek와 같아야 합니다!
     LoopTime_Start = millis();                          // 루프의 새로운 시작 시간
   
 }