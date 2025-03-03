//lab2
#include<Servo.h> // include the file for the servo motor 
Servo servo1;// create the servo object to control servo #1
Servo servo2;// create the servo object to control servo #2
//NOTE :12 SERVOS CAN BE CONTROLLED MOST OF THE AURDINO BOARDS

void setup() {
  servo1.attach(7);
  servo2.attach(8);
  Serial.begin(9600);

}

void loop() {
  int theta1_d; // desired servo angle 
  int analog_input0; // analog input for the PIN A0
  float voltage0; // voltage input for the pun A0
 
  for (theta1_d=0;theta1_d<=180;theta1_d++){
    servo1.write(theta1_d);
    delay(15);
    }
    // MOVE SERVO #1 FROM 180 TO 0 DEGREE IN SMALL INCREAMENT 
    for (theta1_d=180;theta1_d>=0;theta1_d--){
      servo1.write(theta1_d);
      delay(15);
      }

      //Read the analog input for the PIN A0
      analog_input0=analogRead(A0);

      //ananlog input are trypiucally 10bit 
      // the input range is from 0to1023 (1111111111 binary in base 10)
      voltage0=analog_input0/1023.0*5; // convert input to V
      Serial.print("\nanalog_input0 =");
      Serial.print(analog_input0);

}
