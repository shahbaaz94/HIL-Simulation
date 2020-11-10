
// sign macro function --- Translated to register level, commented line will be marked "*" ZYW
#define SIGN(a) ( (a) >= 0.0 ? 1.0 : -1.0 )
// bit a
#define BIT(a) (1 << (a))

#include <Arduino.h>
#include <math.h>

#include <avr/io.h> // I/O and register definitions
#include <avr/interrupt.h> // interrupt functions and definitions


void sim_step(float &t, float x[], float u[], float dt, float y[]);

const int NS = 5; // number of state variables (order)
const int MS = 2; // number of inputs
const int PS = 2; // number of outputs

volatile float t1_offset = 0.0;
const float prescale_inv = 64.0 / 16.0e6;

	
float calculate_mu_bw(float r);

// external interrupt on pin 2
//void interrupt1();

// external interrupt on pin 3
//void interrupt2();

// all the initialization for register level program
void registersetup();

void task1();

// note all global variables shared with the interrupt functions
// should be volatile so they will always be updated

// you should also watch out for atomic access type problems
// -- use noInterrupts() and interrupts() for critical code
// that can't be interrupted
volatile int pw1 = 1500, pw2 = 1500;

// note the following pins are used by the simulator board 
// -- they should not be used on the controller board either
// for safety in case the boards are mixed up to prevent
// pins from blowing out.

// pins 2 and 3 are used for DI to measure the servo pulse width
//const int U1_PIN = 2, U2_PIN = 3;

// pins 5, 6, and 11 are used for analog / PWM outputs 
// pin 5  - y1 (drive motor)
// pin 6  - y2 (right front wheel)
// pin 11 - y3 (left front wheel)
//const int Y1_PIN = 5, Y2_PIN = 6, Y3_PIN = 11; 

// start time of the simulation
float t0;

void setup() 
{	
	Serial.begin(2000000);
	
	// Serial startup sequence below ensures reliable / predictable startup /////

	// NOTE: the serial startup sequence shouldn't be used for the simulator
	// since the serial monitor or save_serial may not
	// always be connected -- just start program immediately
	
	// basically you only want to use the startup sequence
	// if you want to use save_serial with the program
	// -- for other cases remove or comment it out

	// give some time for user to look at robot
	delay(3000);	
	
  // set external interrupt on pin 2
  //attachInterrupt(digitalPinToInterrupt(2),interrupt1,CHANGE);

  // set external interrupt on pin 3
  //attachInterrupt(digitalPinToInterrupt(3),interrupt2,CHANGE);
  
  registersetup();

	// program loop
	while(1) {
		task1();
	}
	
	// don't use loop() in the program
	delay(1000);
	exit(0);	
}

	
void loop()
{
	// not used
}	

void registersetup()
{
	//* Manually enable the pull-up function, when PUD == 0, pull-up is enabled
	MCUCR &= ~BIT(4);
	
	// configure PWM pins as outputs (digital 5, 6, 11)
	//pinMode(Y1_PIN,OUTPUT);
	//pinMode(Y2_PIN,OUTPUT);
	//pinMode(Y3_PIN,OUTPUT);
	DDRD |= BIT(5);
	DDRD |= BIT(6);
	DDRB |= BIT(3);
	
	// note on the Arduino Uno and Nano only DIO pins 2 and 3 
	// can be used for external interrupts	
	
	// configure external interrupt pins as inputs (2, 3)
	//pinMode(U1_PIN,INPUT_PULLUP);
	//pinMode(U2_PIN,INPUT_PULLUP);	
	DDRD &= ~BIT(2);
	DDRD &= ~BIT(3);
	PORTD |= BIT(2);
	PORTD |= BIT(3);
	
	//* noInterrupts(); // disable interrupts -- cli()
	cli();
	
	// set external interrupt on pin 2
	//attachInterrupt(digitalPinToInterrupt(2),interrupt1,CHANGE);

	// set external interrupt on pin 3
	//attachInterrupt(digitalPinToInterrupt(3),interrupt2,CHANGE);	
	
	// INITIALIZATION: TIMER & INTERRUPTS //////////////
	
	// clear timer0-2 control register (ie use default values)
	TCCR1A = 0;	TCCR1B = 0;
	// TCCR0A = 0;	TCCR0B = 0;
	// TCCR2A = 0;	TCCR2B = 0;
   
	// set timer1 prescaler to 64 (default value)
	// note: CS12 (ie CSx2) is used instead of CS02, etc.  
	// note: different timers have different settings and prescaler tables
	TCCR1B |= BIT(CS11) | BIT(CS10); 
	
	// set timer1 overflow interrupt bit in interrupt mask / register
	TIMSK1 = BIT(TOIE1);
	
	// set external interrupt control register A to
	// interrupt on any change for INT0(pin2) and INT1(pin3)
	EICRA |= BIT(ISC00); EICRA &= ~BIT(ISC01);
	//EICRA |= BIT(ISC10); EICRA &= ~BIT(ISC11);
	 
	// enable external interrupt for INT0 and INT1
	EIMSK |= BIT(INT0);
	//EIMSK |= BIT(INT1);		
	
	
	// set timer0(pin5,6 pwm output) and timer2(pin11 pwm output) prescaler to 64 (default)
	// prescaler other than 64 will change other funtions so don't change it
	//TCCR0B |= BIT(CS01) | BIT(CS00); 	
	//TCCR2B = BIT(CS22);
	
	// set timer0 and timer2 to fast pwm mode(WGM:011), with non-inverting mode (10 for A&B)
	TCCR0A = BIT(COM0A1) | BIT(COM0B1) | BIT(WGM01) | BIT(WGM00);
	TCCR2A = BIT(COM2A1) | BIT(COM2B1) | BIT(WGM21) | BIT(WGM20);
	
	
	
	// set timer1 initial value using 2-byte register value 
	TCNT1 = 0;

	//* interrupts(); // enable interrupts -- sei()
	sei();	
	
}

void task1()
{
	int output;
	float y1, y2, y3;
	int w1, w2;
	
	const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
	const float V_bat = 12.0; // lipo battery voltage
	const float PW_R = 1.0 / (PW_MAX - PW_0);
	const float THETA_MAX = 3.14159/2;
	const float wmax = 810.0; // maximum back wheel speed (rad/s)
	const float wmax_inv = 1/wmax;	
	
  float tc; // clock time (s)
	static float t; // current simulation time (s)	
	static float dt; // time step (s)
	static float x[NS+1];  // state vector
	static float u[MS+1]; // input vector u
	static float y[PS+1]; // output vector y
	static int init = 0; // initialization flag
	
	float wb; // back wheel angular velocity (rad/s)	
	float wf; // front wheel angular velocity (rad/s)
	
	
	// set simulation parameters
	dt = 0.001; // time step

	// initialization section
	if( !init ) { 
		// first call to sim_step(...) just sets ICs and parameters
		sim_step(t,x,u,dt,y);
		init = 1;
	}

	// read control inputs ///////////////////////////////
	
	// TODO: generate fault signal if pw1 and pw2 out of range
	// or crash / terminate program
	// or generate error message on screen

	// pw1 - ESC
	// 1000 = -V_bat	
	// 1500 = 0V
	// 2000 = V_bat

	// pw2 - steering servo -- TODO: saturate at +/-45 deg or so ?
	// 1000 = -PI/2 rad (-90 deg)
	// 1500 = 0 rad (0 deg)
	// 2000 = PI/2 (90 deg)

	// TODO: should probably have atomic access here
	// or change pw1, pw2 to single byte variables	
	
	// saturate pw1 and pw2 between 1000 and 2000 us
	w1 = pw1; // servo1 pulse width (us)
//	w2 = pw2; // servo2 pulse width (us)
	


	if(w1 > PW_MAX) w1 = PW_MAX;
	if(w1 < PW_MIN) w1 = PW_MIN;

//	if(w2 > PW_MAX) w2 = PW_MAX;
//	if(w2 < PW_MIN) w2 = PW_MIN;
	


	// convert pulse width to motor voltage
	u[1] = (w1 - PW_0) * PW_R * V_bat; // V	

	// prevent u[1] from going out of range
	if( u[1] > V_bat )  u[1] = V_bat;
	if( u[1] < -V_bat ) u[1] = -V_bat;

	// disturbance torque Td(t) (Nm)
	u[2] = 0.0;

	// note this input (steering angle) is not currently used in the model
	// TODO: add steering model
//	u[3] = (w2 - PW_0) * PW_R * THETA_MAX; // rad

	// register level timer -- 	
	
	tc = TCNT1 * prescale_inv;
	
	cli();
	tc += t1_offset;
	sei();
	
	// read clock time
	//tc = micros()*1.0e-6 - t0; // s

  //!!!for testing purposes only!!!
//    if(tc > 20){
//      u[1] = 0.0;
//    }else {
//      u[1] = 12.0;
//    }  

	// real-time simulation step
	// simulate car until simulation time is equal to clock time
	while( t < tc ) {
		// Euler simulation step of dt
		sim_step(t,x,u,dt,y);
	}

	// set simulation outputs based on model outputs
	
	wb = y[1]; // back wheel velocity (rad/s)
 	wf = y[2]; // front wheel velocity (rad/s) 
	
	// use the following scale for the output y1
	// w = 0 	-> y1 = 2.5 V
	// w = wmax -> y1 = 5.0 V
	y1 = 2.5 + wb * wmax_inv * 2.5;

	// note that division / is time consuming on arduino compared
	// to multiplication so precompute constants involving division
	// as much as possible like above
  
  	// limit / saturate y1	
	if( y1 < 0.0 ) y1 = 0.0; // V
	if( y1 > 5.0 ) y1 = 5.0; // V

	// calculate y2 -- front wheel velocity output

	// use the following scale for the output y1
	// w = 0 	-> y1 = 2.5 V
	// w = wmax -> y1 = 5.0 V
	y2 = 2.5 + wf * wmax_inv * 2.5;	
	
  	// limit / saturate y2	
	if( y2 < 0.0 ) y2 = 0.0; // V
	if( y2 > 5.0 ) y2 = 5.0; // V	
	
	// not currently using y3 for the model	
	
	y3 = 3.0;

	// pin 5  - y1 (drive motor)
	// pin 6  - y2 (right front wheel)
	// pin 11 - y3 (left front wheel)	 
  
	// PWM output values range from 0 to 255
	
	output = (int)(y1*51); // 1V = 51 counts since 5V = 255
	//analogWrite(Y1_PIN,output);
	OCR0B = output;
	
	output = (int)(y2*51); // 1V = 51 counts since 5V = 255
	//analogWrite(Y2_PIN,output);
	OCR0A = output;
	
	output = (int)(y3*51); // 1V = 51 counts since 5V = 255
	//analogWrite(Y3_PIN,output);
	OCR2A = output;
	
	// output for testing purposes
	
	Serial.print(tc);
	Serial.print(",");

  Serial.print(t);
  Serial.print(",");
  
	Serial.print(wb);	
	Serial.print(",");
	
	Serial.print(wf);	
	Serial.print(",");	
	
	Serial.print(u[1]);
	Serial.print("\n");

	delay(10);
}	


//void interrupt1() 
// external interrupt on pin 2
ISR(INT0_vect)
{
	int input;
	static int input_p = 0;
	static float t0 = 0;
  
	
	//input = digitalRead(U1_PIN);
	input = PIND & BIT(2);
	
	// TODO: check for micros() overflow
	
	// rising edge
	if( (input > 0) && (input_p == 0) ) {
		//t0 = micros();
		t0 = TCNT1 * prescale_inv + t1_offset;
    //t0 = TCNT1;
	}
	
	// falling edge
	if( (input == 0) && (input_p > 0) ) {
		//pw1 = micros() - t0;
		pw1 = (TCNT1 * prescale_inv + t1_offset - t0) * 1.0e6;
    //pw1 = (TCNT1 - t0) * prescale_inv * 1.0e6;
	}
	
	// save previous input state for next function call
	input_p = input;
}


//void interrupt2() 
// external interrupt on pin 3
//ISR(INT1_vect)
//{
//	int input;
//	static int input_p = 0;
//	static unsigned long int t0 = 0;
//	
//	//input = digitalRead(U2_PIN);	
//	//input = PIND & BIT(3);
//
//  
//	// rising edge
//	if( (input == HIGH) && (input_p == LOW) ) {
//		//t0 = micros();
//		//t0 = TCNT1 * prescale_inv + t1_offset;
//    t0 = TCNT1;
//	}
//	
//	// falling edge
//	if( (input == LOW) && (input_p == HIGH) ) {
//		//pw2 = micros() - t0;
//		//pw2 = (TCNT1 * prescale_inv + t1_offset - t0) * 1.0e6; 
//    pw2 = (TCNT1 - t0) * prescale_inv * 1.0e6;
//	}
	
	// save previous input state for next function call
//	input_p = input;
//}


ISR(TIMER1_OVF_vect)
{
	t1_offset += 65536.0 * prescale_inv;
  //Serial.print(t1_offset);
  //Serial.print("\n");
	// note the timer has 65536 counts when it rolls since 
	// the count goes from 0 to 65535 -- 65536 values	
}


void sim_step(float &t, float x[], float u[], float dt, float y[])
{
	int i;
	static float xd[NS+1]; // derivative vector at time t	
	
	// note static variables are used to store variables 
	// that need to be remembered between function calls.
	
	// 1. set model parameters (m, k, b, etc.)
	static float L, R, kb; // electrical model parameters
	static float J, km, b, fc; // mechanical model parameters
	static int init = 0; // initialization flag
	
	// new parameters for traction model
	static float m, Rw, g, Q, GR;
	float mu, Ft, wm, v, r, wb, wf;
	float tol = 1.0e-10;
	
	// new constants to speed up the simulation
	static float Fn, GR_inv, L_inv, J_inv, m_inv;	
	
	// initialization section (gets executed only once
	// the first time you call the function)
	if( !init ) {
		
		L = 0.003; ///
		R = 0.141; ///
		// note due to energy conservation kb = km always
		kb = km = 0.00574; /// 
		J = 0.0001; /// this might also include the load inertia (wheel, gears, etc.)
		b = 3.97e-6; ///
		fc = 0.0; /// 0.008 max
		
		// new parameters for traction model
		m = 1.136; /// (kg) total car mass
		Rw = 3.2e-2; /// (m) 3.2 cm tire radius
		g = 9.8; /// g (m/s^2)
		Q = 0.37; /// rear weight distribution
		GR = 2.5; /// gear ratio
				
		// new constants to speed up the simulation ///////
		
		Fn = m*g*Q; /// normal force on back wheels	(assume constant)
		
		// constants to replace slow / with faster *
		GR_inv = 1/GR;
		L_inv = 1/L;
		J_inv = 1/J;
		m_inv = 1/m;
		
		////////////////////////////////////
	
		// 2. set initial conditions (IC)
		t = 0.0; // initial time	
		x[1] = 0.0; // initial current, i (A)
		x[2] = 0.0; /// initial velocity of motor, wm (rad/s)
		x[3] = 0.0; // initial angle, theta (rad)	

		// new states for the traction model
		x[4] = 0.0; // initial forward velocity v (m/s)
		x[5] = 0.0; // initial x position (m)

		init = 1; // initialization complete
		
		return; // first call to sim_step(...) just sets ICs and parameters
		
	} // end of initialization section
	
	// 4. calculate the derivative vector xd at time t
	
	// gear equations ////////
	/// wb = 1 / GR * wm
	/// tau_b = GR * tau_m	
	
	// calculate slip ratio r

	wm = x[2]; /// motor angular velocity, wm (rad/s)
	wb = GR_inv * wm; /// back wheel angular velocity, wb (rad/s)
	
	v = x[4]; // forward velocity v (m/s)

	// checking for / 0 is not needed because of tol
	r = ( wb * Rw - v ) / ( fabs(v) + tol ); ///
	
	// calculate friction coefficient
	mu = calculate_mu_bw(r); ///
	
	// outputs of interest for plotting
	y[1] = wb; // back wheel velocity (rad/s)
	
	// calculate front wheel angular velocity wf
	// v = wf * Rw -> wf = v / Rw
	wf = v / Rw;
	
	y[2] = wf; // front wheel velocity (rad/s) 	
	
	// calculate tire force
	Ft = mu * Fn;
	
	/// tau_b = GR * tau_m -> tau_m = tau_b / GR
	
	// DC motor equations (modified for tire torque Rw*Ft)
	xd[1] = (-x[1]*R - kb*x[2] + u[1]) * L_inv; // di/dt
	xd[2] = (km*x[1] - b*x[2] - fc*SIGN(x[2]) - (Rw*Ft)*GR_inv - u[2]) 
				* J_inv; /// dw/dt
	xd[3] = x[2]; // dth/dt = w

	// note that combining state variable equation models
	// normally requires exchange / sharing of coupling 
	// terms / variables between both sets of equations
	// -- in this case the tire force Ft
	
	// algebraic constrants may also occur with 
	// resulting coupling forces (lagrange multipliers, etc.)
	// -- requires differential algebraic equation (DAE) solvers 

	// new state-variable equations for the traction model
	xd[4] = Ft * m_inv; // dv/dt
	xd[5] = x[4]; // dx/dt = v

	// 5. apply Euler's equation, x = x + dx, note x is a vector
	// this part is always the same
	// but calculating xd will normally be different
	for(i=1;i<=NS;i++) x[i] = x[i] + xd[i]*dt; 
		
	t = t + dt; // increment time
	
}


float calculate_mu_bw(float r)
// BW model
// similar to Burckhardt model but with added term a2 for dip in curve
{
	float mu;
	float a1, a2, a3, c1, c2, rmax;

	// note a2 is mainly for wet road dip in curve

	// the ai parameters can be readily estimated off-line or on-line
	a1 = 1.63; a2 = -0.9; a3 = -0.1; rmax = 1.0; // wet
	
	// rmax is the maximum slip ratio for model -- prevents too much 
	// extrapolation from the a3 term

	// normally c1 and c2 should be constant for different road conditions
	c1 = -27;
	c2 = -15; 

	// limit range of r
	if( r > rmax )  r = rmax;
	if( r < -rmax ) r = -rmax;
	
	if( r < 0 ) {
		r = fabs(r);
		mu = -( a1 * (1-exp(c1*r)) + a2 * (1-exp(c2*r)) + a3 * r );
	} else {
		mu = a1 * (1-exp(c1*r)) + a2 * (1-exp(c2*r)) + a3 * r;	
	}
	
	return mu;
}

