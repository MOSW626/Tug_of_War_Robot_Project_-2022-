#include <I2Cdev.h>


//   arduino ==> 셀프밸런싱로봇을 만들어 실험해본 코드

//----------------------------- -------------------------------
// PID 제어, 모터 제어, MPU6050센서 값을 받기 위한 선언문
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//MPU 객체를 선언합니다
MPU6050 mpu;
//------------------------------------------------------------
#define OUTPUT_READABLE_YAWPITCHROLL  // Yaw, Pitch, Roll 값을 얻기 위해 선언합니다
#define INTERRUPT_PIN 2               // MPU6050 센서의 INT 핀이 꽂혀있는 번호를 설정합니다. 보통 2번
//#define LED_PIN 13                    // Arudino Uno의 13번핀 LED를 동작 중에 반짝거리게 하려고 선언합니다

//int led = 13;

unsigned long timer;  // 시간 정의
extern volatile unsigned long timer0_millis;



// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

void i2cSetup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
#define OUTPUT_TEAPOT 1  // Processing을 통해 MPU6050 센서를 Visualize 하고 싶은 경우 1, 아니면 0으로 선언합니다
//------------------------------------------------------------
// MPU control/status vars
bool blinkState = false;  // LED를 반짝거리게 하기 위한 변수
bool dmpReady = false;    // set true if DMP init was successful
uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
uint8_t devStatus;        // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;      // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;       // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];   // FIFO storage buffer


//------------------------------------------------------------
// MPU6050 센서를 통해 쿼터니언과 오일러각, Yaw, Pitch, Roll 값을 얻기 위해 선언합니다
// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// ================================================================
// ===                         PID Setup                        ===
// ================================================================
double originalSetpoint;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.3;
double fKp = 10000, fKi = 200, fKd = 1.6;  //front Kpid          50 > 75
double bKp = 75, bKi = 200, bKd = 0.8;  //back Kpid         32 > 20

/*
1. 먼저 Kp를 조정합니다. 
  Kp값이 너무 작으면 로봇이 쉽게 넘어지며
  Kp값이 너무 크면 로봇이 앞뒤로 심하게 흔들리게 됩니다.
  로봇이 앞뒤로 조금씩 흔들리는 상태가 최적의 상태입니다.

2. Kp가 설정되면 다음은 Kd를 조정합니다.
  최적은 Kd값은 로봇이 안정을 유지하는 동안 진동을 감소시키며
  손으로 밀어도 로봇이 바로 복귀되게 합니다.

3. 마지막으로 Ki를 조정합니다.
  Kp와 Kd가 설정되더라도 안정된 상태로 도달하는 동안 진동을 하게 됩니다.
  최적의 Ki값은 로봇이 안정되는 데 걸리는 시간을 단축시킬 수 있습니다.
*/

// PID 제어용 input, output 변수를 선언합니다
double input, output;

// PID값을 설정해준다
PID pid(&input, &output, &setpoint, fKp, fKi, fKd, DIRECT);

void PID_RL() {
 /* double gap = input - setpoint;  // Pitch 값은 back으로 갈 수록 값이 커진다.
                                  // 누우면 gap은 음수, 일어서면 gap은 양수의 값을 지니게 됨.
  if (setpoint >= -30) {
    pid.SetTunings(bKp, bKi, bKd);
  } else if (setpoint < -30) {
    if (gap <= 0)  // 뒤로 누움
    {
      pid.SetTunings(bKp, bKi, bKd);
    } else if (gap > 0)  // 앞으로 감
    {
      pid.SetTunings(fKp, fKi, fKd);
    }
  }
  */
  pid.SetTunings(fKp, fKi, fKd);
}


// ================================================================
// ===                       Motor Setup                        ===
// ================================================================
#define MIN_ABS_SPEED 200  // 모터의 최저속도를 설정합니다.   0 ~ 255 값 중 선택

// 모터 제어용 변수 선언
// EnA, EnB는 속도제어용(pwm), IN1,2,3,4는 방향제어용 핀입니다
int ENA = 10;
int IN1 = 12;
int IN2 = 6;
int IN3 = 9;
int IN4 = 8;
int ENB = 11;

// motorController 객체 생성, 맨 끝 파라미터 1,1은 각각 좌측, 우측모터의 최대속도(%) 입니다.
LMotorController motorController(ENA, IN2, IN1, ENB, IN4, IN3, 100, 100);



// ================================================================
// ===                         Visualize                        ===
// ================================================================
//------------------------------------------------------------
// Processing으로 MPU6050 센서를 Visualize 하기 위한 변수
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

void MPU6050_connect() {
  // initialize device
     Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  // devStatus 값이 0 이면 정상작동, 0이 아니면 오작동입니다
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();


    // MPU6050 센서가 정삭작동하면 PID 제어용 코드를 초기화합니다
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  } else {  // MPU6050 센서가 오작동한 경우
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
                 Serial.print(F("DMP Initialization failed (code "));
                 Serial.print(devStatus);
        Serial.println(F(")"));
  }
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                         MPU6050                          ===
// ================================================================
void PID_update_and_motor_control() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    //no mpu data - performing PID calculations and output to motors

    pid.Compute();                                // 루프를 돌면서 pid 값을 업데이트합니다
    motorController.move(output, MIN_ABS_SPEED);  // pid 연산으로 나온 output 값을 motorController로 전송합니다. (모터제어)
  }
}
void etc_need_setup() {
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
}
void Get_ypr_input() {
  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;

  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

#ifdef OUTPUT_READABLE_YAWPITCHROLL  // 센서를 통해 구한 Yaw, Pitch, Roll 값을 Serial Monitor에 표시합니다 \
    Serial.print("ypr\t"); \
    Serial.print(ypr[0] * 180/M_PI); \
    Serial.print("\t"); \
    Serial.print(ypr[1] * 180/M_PI); \
    Serial.print("\t"); \
    Serial.println(ypr[2] * 180/M_PI);

#endif
  // PID 제어를 하기 위해 input 변수에 Pitch 값을 넣습니다
  input = ypr[1] * 180 / M_PI-1;
  
}
void check_imu_working() {
  // MPU6050 센서가 정상작동하는 경우에만 PID제어를 해야하므로 아래와 같이 if-else문을 작성합니다
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
  }
  // MPU6050 센서가 정상작동하는 경우
  else if (mpuIntStatus & 0x02) {
    Get_ypr_input();
    PID_RL();
    MPU6050_visualize();
  }
}
void MPU6050_visualize() {
#ifdef OUTPUT_TEAPOT  // Processing으로 MPU6050센서의 움직임을 Visualize 하기 위한 코드
  // display quaternion values in InvenSense Teapot demo format:
  teapotPacket[2] = fifoBuffer[0];
  teapotPacket[3] = fifoBuffer[1];
  teapotPacket[4] = fifoBuffer[4];
  teapotPacket[5] = fifoBuffer[5];
  teapotPacket[6] = fifoBuffer[8];
  teapotPacket[7] = fifoBuffer[9];
  teapotPacket[8] = fifoBuffer[12];
  teapotPacket[9] = fifoBuffer[13];
  Serial.write(teapotPacket, 14);
  teapotPacket[11]++;  // packetCount, loops at 0xFF on purpose
#endif
}

// ================================================================
// ===                        Rebound                           ===
// ================================================================
void Rebound(double theta_0, double theta_b, unsigned long time_start, unsigned long time_b, unsigned long T)
// theta_0는 기본 세팅 각도, theta_b는 바운드 될 각도, time_start는 반동 시작 타이밍, time_b는 바운드 될 각도의 유지 시간, T은 주기
{
  timer = millis();
  setpoint = theta_0;  // degree(90) = 0, degree(60) = -30
  if (timer >= time_start && timer <= time_start + time_b) {
    setpoint = theta_b;
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if (timer >= T) {
    timer0_millis = 0;
  }
  digitalWrite(LED_BUILTIN, LOW);
}

// ================================================================
// ===                         Setup                            ===
// ================================================================
void setup() {
  i2cSetup();
  Serial.begin(115200);
  Serial.print("test Theta_set,Theta_input, test\n");
  while (!Serial);  // wait for Leonardo enumeration, others continue immediately
  MPU6050_connect();
  // 로봇이 작동 중 13번 LED를 깜빡거리기 위해 OUTPUT으로 초기화합니다
 // pinMode(LED_PIN, OUTPUT);
  //pinMode(led, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
}


// ================================================================
// ===                          Loop                            ===
// ================================================================
void loop() {
  Rebound(0,0, 1000, 72, 2000);
  PID_update_and_motor_control();
  etc_need_setup();
  check_imu_working();
  //   Serial.println(timer);
  //   Serial.println(",");
  //   Serial.println(setpoint);
  Serial.print("test ");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(input);
  Serial.print(" test\n");
}
