/*
 * very basic sketch for PLX DAQ test
 * for new "Version 2" of PLX DAQ
 */

int i = 0;

void setup() {

  // open serial connection
    Serial.begin(9600);

  // define 2 rows: first named "Counter", second named "millis"
    Serial.println("CLEARDATA");
    Serial.println("LABEL,Time,Counter,millis");
}

void loop() {

  // simple print out of number and millis
  // output "DATA,TIME,4711,13374"
    Serial.print("DATA,TIME,");
    Serial.print(i++); Serial.print(",");
    Serial.println(millis());
}
