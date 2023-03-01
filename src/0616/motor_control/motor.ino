void f_50()
{
//최대속도의 50%로 정회전
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
  analogWrite(11, 127);
  digitalWrite(6, HIGH);
  digitalWrite(12, LOW);
  analogWrite(10, 127);
  delay(3000);
}

void b_50()
{
//최대속도의 50% 역회전
  digitalWrite(8, LOW);
  digitalWrite(9,HIGH);
  analogWrite(11, 127);
  digitalWrite(6, LOW);
  digitalWrite(12,HIGH);
  analogWrite(10, 127);
  delay(3000);
}

void f()
{    
//최대속도로 정회전
  digitalWrite(8, HIGH);
  digitalWrite(9,LOW);
  analogWrite(11, 255);
  digitalWrite(6, HIGH);
  digitalWrite(12,LOW);
  analogWrite(10, 255);
  delay(3000);
}

void b()
{  
//최대속도로 역회전
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
  analogWrite(11, 255);
  digitalWrite(6, LOW);
  digitalWrite(12, HIGH);
  analogWrite(10, 255);
  delay(3000);
}

void s()
{  
//정지 (7번핀에 HIGH를 주어도 PWM 핀에 값을 0을 주었기 때문에 정지함)
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
  analogWrite(11, 0);
  digitalWrite(6, HIGH);
  digitalWrite(12, LOW);
  analogWrite(10, 0);
  delay(3000);
}
