// BLINK PROGRAM 
// i had made some changss compare to prof as the code stop after 26 secibd

int pin1=13; // pin 13 usually connected to the LED
void setup (){
  pinMode(pin1,OUTPUT);//pinmode is the aurdino lib function
  Serial.begin(9600);// 9600 is the std frequency for the communication please use
  //this as beginer
  
  }
  void loop(){

    // floats are used in arduino as it consumed less space compare to double
    float time_s = micros() * 1.0e-6;// time in second 

    if (time_s>=26) {
      
      Serial.print("Program ends");
      digitalWrite(pin1,LOW);
      while(true);
      }
    digitalWrite(pin1,HIGH); // set pin 1 to the high for LED to ON 
    delay(1000);
    digitalWrite(pin1,LOW);
    delay(1000);
    // time_s=micros()*1.0e-6; // returns time in micro second
    // millus() --> this will return the time in millisecond 

    Serial.print("\ntime(s)=");
    Serial.print(time_s);
    
    
    }
