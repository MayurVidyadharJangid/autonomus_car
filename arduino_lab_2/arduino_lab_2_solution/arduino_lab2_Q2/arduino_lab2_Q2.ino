
int pin_LED = 13;

void setup() {
	
	pinMode(pin_LED,OUTPUT); // config LED pin as output
	pinMode(3,INPUT_PULLUP); // config pin 3 (ie D3) as input
	// testing 
	Serial.begin(115200); 
}

void loop() {
	int d3;
	float voltage3, t_s, dt;

	t_s = micros()*1.0e-6;
	
	d3 = digitalRead(3); // read digital input for pin 3

	if( d3 == LOW ) voltage3 = 0.0; 
	if( d3 == HIGH ) voltage3 = 5.0;

	// note we set it with d3 and not voltage3
	digitalWrite(pin_LED, d3);
	
	// print each data point on a new line with a comma in between
	// so we can cut and paste the csv file
	Serial.print(t_s);
	Serial.print(",");
	Serial.print(voltage3);
	Serial.print("\n");
	
	// quick and dirty approach to approximately print out every 1s.
	// the time to print out and for the functions will result 
	// in somewhat longer intervals.
//	delay(1000);

	// accurate appoach to wait 1s by contunutally reading the clock
	// -- like checking the time on your watch waiting for a certain
	// time to pass 
	while(1) {
		dt = micros()*1.0e-6 - t_s;
		if(dt > 1.0) break;
	}

	// end program when t > 60s
	if(t_s > 60.0) exit(0);

}

