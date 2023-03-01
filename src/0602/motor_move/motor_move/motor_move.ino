void setup() {
  Serial.begin(115200);

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

void loop() {
  Forward(); //전진
  delay(1000);
  Reverse(); //후진
  delay(1000);
  Stop(); //모터 정지
  delay(1000);
}
