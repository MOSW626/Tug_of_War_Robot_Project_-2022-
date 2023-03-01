/********************************************************
 * PID Adaptive Tuning Example
 * One of the benefits of the PID library is that you can
 * change the tuning parameters at any time.  this can be
 * helpful if we want the controller to be agressive at some
 * times, and conservative at others.   in the example below
 * we set the controller to use Conservative Tuning Parameters
 * when we're near setpoint and more agressive Tuning
 * Parameters when we're farther away.
 * 
 * PID 적응 튜닝 예
 * PID 라이브러리의 장점 중 하나는 다음을 수행할 수 있다는 것입니다.
 * 언제든지 튜닝 매개변수를 변경할 수 있습니다. 이것은 될 수있다
 * 컨트롤러가 어떤 부분에서 공격적이기를 원할 때 유용합니다.
 * 시대에 따라 보수적이다. 아래 예에서
 * Conservative Tuning Parameters를 사용하도록 컨트롤러를 설정했습니다.
 * 우리가 설정값에 가깝고 더 공격적인 튜닝일 때
 * 우리가 더 멀리 있을 때 매개변수.
 ********************************************************/

#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(0);
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = analogRead(0);
  
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive(적극적인) tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
  myPID.Compute();
  analogWrite(3,Output);
}
