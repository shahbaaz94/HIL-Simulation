
//#include <Servo.h>

#define BIT(a) (1 << (a))

#include <Arduino.h>
#include <math.h>

#include <avr/io.h> // I/O and register definitions
#include <avr/interrupt.h> // interrupt functions and definitions

void task1();

//Servo servo1, servo2;

float read_ADC_voltage(int channel, int n);
float calculate_mu(float wb,float wf,float &r);
void pidctrl(float t, float dt, float y, float &u);

// all the initialization for register level program
void registersetup();

volatile float t1_offset = 0.0;
const float prescale_inv = 8.0 / 16.0e6;
volatile int pw1;

// initial time of test
float t0;

//static volatile float y1 = -1, y2 = -1, y3 = -1;

void setup() 
{	
	// Serial startup sequence below ensures reliable / predictable startup /////
	char ch;
	
	Serial.begin(2000000);
	
	// wait until Serial Monitor or save_serial program is started
	while( !Serial ) delay(1);
	
	// note: the following code is the best way to start an Arduino 
	// program in my opinion since you can control the start time exactly
	// and you can wait until you are ready to watch the experiment.
	while(1) {
		if( Serial.available() > 0 ) { // is there incoming data
			ch = Serial.read(); // read one character / byte
			if( ch == 's' ) break;
		}
		delay(10); // leave some time for printing message above, etc.
	}
	
	// give some time for user to look at robot
	delay(3000);
	
	// NOTE: you can try using the following line here to make sure each line 
	// is completely output before continuing -- this might make the 
	// output more reliable
	// Serial.flush() // completely write serial output before continuing		
	
	// end of Serial startup sequence ///////////////////////////////////	

	// I recommend using pins 7 and 8 for the servo pins
	// -- you should avoid using pins 5, 6, and 11 which
	// are used by the car simulator board for analog output.
	
	// you should also avoid using pins 2 and 3 to avoid
	// potential conflict with the simulator board.
	
	//servo1.attach(7); 
	//servo2.attach(8);

	registersetup();
	
	//t0 = micros()*1.0e-6; // initial time (s)

	while(1) task1();

	delay(1000);
	exit(0); // not using loop()	
}

void registersetup(){
	
	// configure output pins(pin 7,8), manually send 1-2ms pulse within 20ms cycle	
	DDRD |= BIT(7);
	DDRB |= BIT(0);
	
	// configure input pins(analog pin 1,3,5)
//	DDRC &= ~BIT(1);
//	DDRC &= ~BIT(3);
//	DDRC &= ~BIT(5);
//  PORTC |= BIT(1);
//  PORTC |= BIT(3);
//  PORTC |= BIT(5);
  
	cli();
	
	// clear timer1 control register (ie use default values)
	TCCR1A = 0;	TCCR1B = 0;
	
	// set timer1 prescaler to 8
	// real time = counts *prescale/16e6. 
	// And we need precise control over the scale of 1ms to 2ms(milliseconds) in 20ms loop	
	// if prescale = 64(default) ---> 1ms to 2ms = counts * 64/16e6 ---> counts = 250 to 500
	// if prescale = 8, counts = 2000 to 4000, for 20 ms counts = 40000, control resolution: 0.5microseconds
	// if prescale = 1, counts = 16000 to 32000, but for 20ms counts= 3200000 > 65536, out of range
	// if prescale = 256, counts = 62.5 to 125, control may not be so precise
	// if prescale = 1024, counts = 15.625 to 31.25, same problem
	// to conclude, we choose prescale = 8
	TCCR1B |= BIT(CS11); 
  //TCCR1B |= BIT(CS10); 
	
	// set timer1 interrupt mask / register
	TIMSK1 |= BIT(OCIE1A);
	TIMSK1 |= BIT(OCIE1B);
  //TIMSK1 = BIT(TOIE1);
  
	// set timer compare registers
	OCR1A = 3000; // initial value pw = 1500 ---> counts = 3000
	OCR1B = 40000;
	
	TCNT1 = 0;
	
	PORTD |= BIT(7); // send out initial servo pulse = 1500

  ADMUX = 0;
  ADMUX |= BIT(MUX0);
  // set ADC reference (max input voltage) to 5V (Vcc) from micrcontroller
  // note: default (0) takes the reference from the Aref pin
  ADMUX |= BIT(REFS0);
 
  // set ADC control and status register A
  ADCSRA = 0;  
  
  ADCSRA |= BIT(ADEN); // ADC enable
  //ADCSRA |= BIT(ADIE); // ADC interrupt enable,only for auto mode

  ADCSRA |= BIT(ADPS1) | BIT(ADPS2); // 64 prescaler
  // this gives a conversion time of 60 microseconds for one channel

  ADCSRA |= BIT(ADSC);
  
	sei();
	
}

ISR(TIMER1_COMPA_vect)
{
  
  PORTD &= ~BIT(7); // turn of pin7(falling edge)
  
}

ISR(TIMER1_COMPB_vect)
// timer1 output compare match interrupt function
{
  
  TCNT1 = 0 ;
  t1_offset += 40000.0 * prescale_inv;
  
  // send the latest pw1 to simulator
  OCR1A = 2*pw1;
  
  PORTD |= BIT(7); // turn on pin7(rising edge)  
}

ISR(TIMER1_OVF_vect)
// timer1 overflow interrupt function
{
  t1_offset += 65536.0 * prescale_inv;
}

void task1() 
{		
	int n;
	float y1 = -1, y2 = -1, y3 = -1;
	float wb, wf;
	float u1, u2;
	int pw2;
	int w1, w2;	
	float t;
	float dt = 0.001; //time step

	const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
	const float wmax = 810.0; // maximum back wheel speed (rad/s)	
	const float V_bat = 12.0; // lipo battery voltage
	const float V_bat_inv = 1/V_bat;	

	float mu,slipratio;

	t = TCNT1 * prescale_inv;
	cli();
	t += t1_offset;
	sei();	

	// read clock time (s)
	//t = micros()*1.0e-6 - t0;
  
	// read car outputs ///////////////

	// note that y1 (drive motor), y2 (right front wheel), and 
	// y3 (left front wheel) are the outputs for the car system

	// *** I recommend using pins A1, A3, and A5 for analog inputs ***
	
	n = 200; // number of ADC samples in averaging filter
	
	// TODO: use ADC&average to replace function below
	y1 = read_ADC_voltage(1,n);
	y2 = read_ADC_voltage(3,n);
	y3 = read_ADC_voltage(5,n);
	
	// use the following scale for the output y1
	// w = 0 	-> y1 = 2.5 V
	// w = wmax -> y1 = 5.0 V
	// y1 = 2.5 + w * wmax_inv * 2.5 ->
	
	// back wheel angular velocity (rad/s)
	wb = (y1 - 2.5) * 0.4 * wmax;

	// front wheel angular velocity (rad/s)
	wf = (y2 - 2.5) * 0.4 * wmax;

	// calculate inputs
	mu = calculate_mu(wb,wf,slipratio);
	
	pidctrl(t, dt, mu, u1);
	
	// step input for u1 at t = 5s and down to zero at t = 10s
	// if(t > 20) {
		// u1 = 0.0;
	// } else {
		// u1 = 12.0;
	// }	
	
	// note the maximum input u1 is V_bat (12V)
	// anything more than 12V or less than -12V
	// will have no effect on the simulator
	// use the following if statement to prevent
	// asking for inputs out of range
	if( u1 > V_bat )  u1 = V_bat;
	if( u1 < -V_bat ) u1 = -V_bat;	
	
	u2 = 0.0;

	// convert inputs to actuator values pw1, pw2

	// convert motor voltage to pulse width
	// u1 = (w1 - PW_0) * PW_R * V_bat ->
	w1 = u1 * V_bat_inv * (PW_MAX - PW_0) + PW_0; 
	
	// saturate input if out of range
	if(w1 > PW_MAX) w1 = PW_MAX;
	if(w1 < PW_MIN) w1 = PW_MIN;	
	
	pw1 = w1;
	
//	if(w2 > PW_MAX) w2 = PW_MAX;
//	if(w2 < PW_MIN) w2 = PW_MIN;

	// set pw2 for testing purposes
	pw2 = 1750;
	
	// set car actuators
	//servo1.writeMicroseconds(pw1);
	//servo2.writeMicroseconds(pw2);	
	
	// print out for testing purposes
	
	Serial.print(t);
	Serial.print(",");
	
	Serial.print(y1);
	Serial.print(",");
	
	Serial.print(y2);
	Serial.print(",");

	Serial.print(y3);
	Serial.print(",");	
	
	Serial.print(u1);
	Serial.print(",");	
	
	Serial.print(wb);
	Serial.print(",");		
	
	Serial.print(wf);
	Serial.print(",");	

	Serial.print(mu);
	Serial.print(","); 

	Serial.print(slipratio);
	Serial.print("\n"); 

	// Serial.print(PINC & BIT(3));
	// Serial.print(","); 

	// Serial.print(analogRead(A3));
	// Serial.print(","); 

	// Serial.print(PINC & BIT(5));
	// Serial.print("\n");
  
	delay(30);
}


float read_ADC_voltage(int channel, int n)
// average = digital low pass filter
// for some value of n it's very good (eg n=200)
{
	int i,j;
	int analread;
	float input, voltage;
	unsigned long int sum;
	const float ADC_to_V = 1.0/1023.0*5;   

  if(channel == 1) {
    ADMUX &= ~BIT(MUX1);
    ADMUX &= ~BIT(MUX2);
    ADMUX |= BIT(MUX0);
  }
  else if (channel == 3) {
    //ADMUX = 0;
    ADMUX |= BIT(MUX0)| BIT(MUX1);
  }
  else if(channel == 5) {
    //ADMUX = 0;
    ADMUX |= BIT(MUX0) | BIT(MUX2);
  }
  ADCSRA |= BIT(ADSC);
  
	sum = 0;
	for(i=0;i<n;i++) {
    while( ADCSRA & BIT(ADSC) ) j++;
  
    // read the ADC (10-bits) // 0 - 1023
    analread = ADC;
    ADCSRA |= BIT(ADSC);
    //analread = (PINC & BIT(channel)); 
		//if(analread != 0) sum += analread;
    sum += analread;
    //sum += analogRead(channel);
	}
	
	input = (float)sum / n; // average analog input	
	
//	voltage = input / 1023.0 * 5;
	voltage = input * ADC_to_V; // much faster than expression above
	
	return voltage;
}

//ISR(ADC_vect)
//// ADC conversion complete interrupt
//// note:
//// arduino time functions such as micros() and serial 
//// communication can work in this interrupt function
//{
//  static volatile int i = 0;
//  int adc1,n=200;
//  float input, voltage;
//  volatile unsigned long int sum = 0;
//  const float ADC_to_V = 1.0/1023.0*5; 
//    
//  ADCSRA |= BIT(ADSC);
//
//  if(i = 0){
//    y3 = (float)sum / n * ADC_to_V;
//    ADMUX &= ~BIT(MUX1);
//    ADMUX &= ~BIT(MUX2);
//    ADMUX |= BIT(MUX0);
//    sum = 0;
//    adc1 = ADC; // read the ADC (10-bits) // 0 - 1023
//    sum += adc1;
//    i++;
//  }
//  else if(i < n){
//    adc1 = ADC; // read the ADC (10-bits) // 0 - 1023
//    sum += adc1;
//    i++;
//  }
//  else if (i = n){
//    y1 = (float)sum / n * ADC_to_V;
//    sum = 0;
//    ADMUX |= BIT(MUX0)|BIT(MUX1);
//    adc1 = ADC; // read the ADC (10-bits) // 0 - 1023
//    sum += adc1;
//    i++;
//  }
//  else if(i < 2 * n){
//    adc1 = ADC; // read the ADC (10-bits) // 0 - 1023
//    sum += adc1;
//    i++;
//  }
//  else if (i = 2*n){
//    y2 = (float)sum / n * ADC_to_V;
//    sum = 0;
//    ADMUX |= BIT(MUX0)|BIT(MUX2);
//    adc1 = ADC; // read the ADC (10-bits) // 0 - 1023
//    sum += adc1;
//    i++;
//  }
//  else if (i < 3*n){
//    adc1 = ADC; // read the ADC (10-bits) // 0 - 1023
//    sum += adc1;
//    i++;
//  }
//  else if (i = 3*n) {
//    i = 0;
//  } 
//  
//}

float calculate_mu(float wb,float wf,float &r)
{
	float mu;
	float B, C, D, E, rmax;
	
	float tol = 1.0e-10, Rw = 3.2e-2; /// (m) 3.2 cm tire radius
	
	
	// checking for / 0 is not needed because of tol
	r = ( wb * Rw - wf * Rw ) / ( fabs(wf * Rw) + tol );	
	
//	B=10; C=1.9; D=1; E=0.97; // dry
	B=12; C=2.3; D=0.82; E=1; // wet	
//	B=5; C=2; D=0.3; E=1; // snow
//	B=4; C=2; D=0.1; E=1; // ice
	
	// maximum slip ratio for model -- prevents too much extrapolation
	rmax = 10.0;

	// limit range of r
	if( r > rmax )  r = rmax;
	if( r < -rmax ) r = -rmax;

	mu = D*sin( C*atan( B*r-E*(B*r-atan(B*r)) ) );

	return mu;
}

void pidctrl(float t, float dt, float y, float &u){

	float kp, ki, kd;
	float e, ed, z, ei_max;
	float V_batt = 12.0;
	float r;
	

	// use static floats for local variables that need to be 
	// remembered between function calls.
	// static variables are like global variables that can only
	// be used by the function.
	// regular local variables get erased at the end of the function.
	static float tp = 0.0; // previous time (initial value = 0)
	static float ei = 0.0; // integral state (initial value = 0)
	static float ep = 0.0; // previous error	

	// PID controller gains
	
	// this pid is for normal speed control
	kp = 16.0; //kp = 1 --- wf = 50; kp = 10 --- wf = 500; kp = 16 --- wf = 800 propotional just like mu  
	// but if kp > 16 traction control effect will be worse but can improve time response
	ki = 0.0;  // traction control will be worse while improve time response, somehow prohibit mu to control speed
	kd = 0.0;  // will make traction control impossible
	
//	// this pid is for max speed control
//	kp = 20.0; //kp = 1 --- wf = 50; kp = 10 --- wf = 500; kp = 16 --- wf = 800 propotional just like mu  
//	// but if kp > 16 traction control effect will be worse but can improve time response
//	ki = 2.0;  // traction control will be worse while improve time response, somehow prohibit mu to control speed
//	kd = 0.0;  // will make traction control impossible

    // for normal speed testing
    if (t < 40) r = 0.4;
    else r = -0.5;
//
//    // for max speed testing
//    if (t < 50) r = 0.8;
//    else r = -1.0;

	
	// calculate dt
	dt = t - tp; // measure sampling period dt
	tp = t; // save previous sample time

	// calculate controller error
	e = r - y;
	ed = (e - ep) / dt; // ed = rd - yd

	// standard integration I = ei
	ei += e*dt;	// I += e*dt
	
	// anti-windup / saturation integration logic ///////
	if (ki > 0.0) {
		ei_max = 0.14*V_batt / ki; // want ki*ei < 0.14*V_batt
	}
	else {
		ei_max = 0.0;
	}

	// set z
	if ((ei > ei_max) && (e > 0)) {
		// positive out of bounds, positive integration
		z = 0; // stop integration
	}
	else if ((ei < -ei_max) && (e < 0)) {
		// negative out of bounds, negative integration		
		z = 0; // stop integration
	}
	else { // either in bounds or integration reduces integral
		z = e; // normal integration
	}

	// NOTE: if you use this you must comment out ei += e*dt
	ei += z*dt;

	////////////////////////////////////////////	

	// PID controller
	u = kp*e + ki*ei + kd*ed;	

	if (u >  V_batt) u = V_batt;
	if (u < 0) u = 0;
}


void loop()
{
	// not used
}
