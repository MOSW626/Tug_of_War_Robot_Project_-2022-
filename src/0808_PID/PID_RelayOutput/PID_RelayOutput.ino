/********************************************************
 * PID RelayOutput Example
 * Same as basic example, except that this time, the output
 * is going to a digital pin which (we presume) is controlling
 * a relay.  the pid is designed to Output an analog value,
 * but the relay can only be On/Off.
 *
 *   to connect them together we use "time proportioning
 * control"  it's essentially a really slow version of PWM.
 * first we decide on a window size (5000mS say.) we then 
 * set the pid to adjust its output between 0 and that window
 * size.  lastly, we add some logic that translates the PID
 * output into "Relay On Time" with the remainder of the 
 * window being "Relay Off Time"
 * 
 * PID 릴레이 출력 예
 * 이번에는 출력되는 것을 제외하고는 기본 예제와 동일
 * (추정)이 제어하는 ​​디지털 핀으로 이동합니다.
 * 릴레이. pid는 아날로그 값을 출력하도록 설계되었으며,
 * 그러나 릴레이는 On/Off만 가능합니다.
 *
 * 그것들을 함께 연결하기 위해 "시간 비례"를 사용합니다.
 * control" 이것은 본질적으로 PWM의 정말 느린 버전입니다.
 * 먼저 창 크기(5000mS라고 함)를 결정한 다음
 * 0과 해당 창 사이에서 출력을 조정하도록 pid를 설정합니다.
 * 크기. 마지막으로 PID를 변환하는 로직을 추가합니다.
 * 나머지를 "Relay On Time"으로 출력
 * 창은 "Relay Off Time"임
 ********************************************************/

#include <PID_v1.h>
#define RelayPin 6

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;
void setup()
{
  windowStartTime = millis();
  
  //initialize the variables we're linked to
  Setpoint = 100;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);   //제한을 걸음

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = analogRead(0);
  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if(millis() - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;      //????
  }
  if(Output < millis() - windowStartTime) digitalWrite(RelayPin,HIGH);
  else digitalWrite(RelayPin,LOW);

}
