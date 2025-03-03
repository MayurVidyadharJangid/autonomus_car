
// 1) The objective of this question is to determine how long it takes
// to execute an Arduino library function, in this case the sin 
// function.  All functions require a certain amount of time to 
// execute.  Due to the computational limitations of an Arduino board 
// it’s often important to determine how long various parts of the 
// program take to execute.  The clock function micros() can be used 
// to measure the execution time of a function.  This is accomplished 
// by measuring the time t1 before the function is called and 
// measuring the time t2 after the function is called.  
// The difference dt = t2 – t1 is the time required for the function 
// to execute (assuming the time to execute micros() is negligible).  
// This value dt might vary depending on the argument of the function 
// and other tasks the Arduino is doing.  Therefore, it’s best to 
// repeatedly calculated dt in a for loop with a large number of 
// iterations (e.g. 1000) and calculate the average, minimum, 
// and maximum values of dt.

// Following the approach outlined above, determine the average, 
// minimum, and maximum time it takes to execute the function 
// sin(t) using a for loop for i from 1 to 1000, where t is the 
// clock time in microseconds.

void setup() {
	Serial.begin(115200); 
}

void loop() {
	// note: you could use float or unsigned long int to store
	// the time in microseconds.
	// unsigned long int is more accurate for large values of time
	// since there is no truncation error.
	// float is easier to convert to time in seconds.
	// try both and compare.
	float ave,min,max,sum,dt;
	float y, t1, t2;
	int i;
	
	// note the for loop could be put in setup() or loop().  if it's 
	// in loop it will print out average, minimum, and maxiumum repeatedly
	sum = 0.0;
	max = 0.0;
	min = 1000000.0;
	for(i=1;i<=1000;i++) {
		t1 = micros();
		y = sin(t1);
		t2 = micros();
		dt = t2 - t1;
		sum += dt;
		if(dt > max) max = dt;
		if(dt < min) min = dt;
	}
	ave = sum / 1000;
	
	Serial.print("\ny = ");
	Serial.print(y);
	
	Serial.print("\nmin = ");
	Serial.print(min);
	Serial.print("\nave = ");
	Serial.print(ave);
	Serial.print("\nmax = ");
	Serial.print(max);

	// give some time to print out
	delay(500);
}

