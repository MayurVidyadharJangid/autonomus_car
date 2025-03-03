//Question 2 

int pin1=13;
void setup(){
  
  pinMode(pin1,OUTPUT);
  Serial.begin(9600);
  }
  int pin_state=LOW;
  float time_p=0.0; // prev time
  void loop(){
    float time_clock,dt,f,pi=3.14159;
    while(1){
    time_clock =micros()*1.0e-6;
    dt= time_clock-time_p;
    if( (dt>0.25)&&(time_clock<2*pi)){
      f=sin (time_clock);
      
      
          Serial.print("time_clock");
           Serial.print(",");
          Serial.print(f);
          Serial.print("\n");
          time_p=time_clock;
          break;
      }
    }
    
    }
  
