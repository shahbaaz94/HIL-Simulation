
// DC motor + traction model

#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>

using namespace std;

// sign macro function
#define SIGN(a) ( (a) >= 0.0 ? 1.0 : -1.0 )

double sim_step(double &t, double x[], double u[], double dt, double y[]);

const int NS = 5; // number of state variables (order)
const int MS = 2; // number of inputs
const int PS = 4; // number of outputs
	
// wf - front wheel speed	
// note: this model assumes the front wheel has no slip
// -- perfect rolling, v = wf*Rw -> wf = v / Rw	
// *** important formula to calculate slip ratio
// wb back wheel speed
// wm motor speed
	
double calculate_mu(double r);

double calculate_mu_bw(double r);

void pidctrl(double t, double dt, double y, double r, double &u);
void autopid(double &kp, double &ki, double &kd, double y2, double t); // TODO: not fully tested, not reliable

int main()
{
	int i;
	double t; // current time (seconds)	
	double dt; // time step (s)
	double tf; // final time (s)	
	double x[NS+1];  // state vector
	double u[MS+1]; // input vector u
	double y[PS+1]; // outputs of interest for plotting

	// open output text file -- you can view this file with notepad++, etc.
	// the csv extension indicates it's a file with columns separated by
	// commas which can be opened and plotted with Excel or Matlab
	ofstream fout("sim.csv"); 

	// note you need to close the file in excel before re-writing it
	if ( !fout ) {
		cout << "\nerror opening output file";
	}	
	
	// use scientific notation to avoid loss of precision
	fout << scientific;
	
	// 8 digits of precision
	fout.precision(7);

	// label the columns (the % is so matlab plotting m-files ignore this line)
	fout << "%time(s),x1,x2,x3,x4,x5,u1,u2,y1,y2,y3,y4\n";
	
	// 3. set simulation parameters
	// - try an initial guess for dt (say 0.01) and then run the simulation.
	// - keep reducing dt until the solution / output plots converge (stop changing)
	dt = 0.001; // time step

	// dt should be at least 10x smaller than the smallest
	// time constant for the system

	tf = 100.0; // final simulation time (s)

	// first call to sim_step(...) just sets ICs and parameters
	sim_step(t,x,u,dt,y);

	cout << "\nsimulation is starting";

	u[1] = 12.0; // motor voltage V(t)
	u[2] = 0.0; // disturbance torque Td(t)

	
	double r;	
	double mu;

	

	// simulation loop
	while(t < tf) {
		
		// save time and state variables and input into a file
		fout << t;
		for(i=1;i<=NS;i++) fout << "," << x[i];		
		
		// calculate control inputs
		
		for(i=1;i<=MS;i++) fout << "," << u[i];		

		// Euler simulation step of dt
		// simulate from t to t+dt
		mu = sim_step(t,x,u,dt,y);

		// cout << "time: " << t << " y[1](backwheel) = " << y[1] << "  y[2](frontwheel) = " << y[2];
		


		// y1: wb  y2: wf
		// in wet condition(sim_step default), max mu = 0.8
		// if mu = 0, wf = 0; mu = 0.2, wf = 200; mu = 0.5, wf = 500; mu = 0.8, wf = 800; propotional just like kp
		// but if mu > 0.8, then wheels start to slip(different velocity); larger the mu, larger the slip effect
		
		// for normal speed testing
		/*if (t < 50) r = 0.4;
		else if (t < 60) r = -0.5;*/

		// for max speed testing
		if (t < 50) r = 0.8;
		else if (t < 60) r = -1.0;
		

		pidctrl(t, dt, mu, r, u[1]); // launch control and speed control by control mu, target speed = 800
		//pidctrl(t, dt, y[2], 800, u[1]); // launch control and speed control by control mu, target speed = 800
		//pidctrl(t, dt, fabs(y[1] - y[2]), 0, u[1]);	// traction control by control velocity difference, didn't work

		//cout << " new voltage: " << u[1] << endl;		
		
		//autopid(kp, ki, kd, y[2], t);		
		
		// outputs of interest for plotting
		for(i=1;i<=PS;i++) {
			// limit slip ratio if too large so plot scale isn't too large
			if( fabs(y[3]) > 10 ) y[3] = 10*SIGN(y[3]);
			fout << "," << y[i];
		}		

		// each row represents a given time
		if( t < tf ) fout << "\n"; // move on to next row

	} // end while

	// close output file
	fout.close();

	cout << "\nsimulation is complete\n\n";

	return 0;
}


void pidctrl(double t, double dt, double y, double r, double &u){

	double v;
	double kp, ki, kd;
	double e, ed, z, ei_max;
	double V_batt = 12.0;
	

	// use static floats for local variables that need to be 
	// remembered between function calls.
	// static variables are like global variables that can only
	// be used by the function.
	// regular local variables get erased at the end of the function.
	static double tp = 0.0; // previous time (initial value = 0)
	static double ei = 0.0; // integral state (initial value = 0)
	static double ep = 0.0; // previous error	

	// PID controller gains
	
	// this pid is for normal speed control
	//kp = 16.0; //kp = 1 --- wf = 50; kp = 10 --- wf = 500; kp = 16 --- wf = 800 propotional just like mu  
	//// but if kp > 16 traction control effect will be worse but can improve time response
	//ki = 0.0;  // traction control will be worse while improve time response, somehow prohibit mu to control speed
	//kd = 0.0;  // will make traction control impossible
	
	// this pid is for max speed control
	kp = 20.0; //kp = 1 --- wf = 50; kp = 10 --- wf = 500; kp = 16 --- wf = 800 propotional just like mu  
	// but if kp > 16 traction control effect will be worse but can improve time response
	ki = 4.0;  // traction control will be worse while improve time response, somehow prohibit mu to control speed
	kd = 0.0;  // will make traction control impossible

	
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





double sim_step(double &t, double x[], double u[], double dt, double y[])
{
	int i;
	static double xd[NS+1]; // derivative vector at time t	
	
	// note static variables are used to store variables 
	// that need to be remembered between function calls.
	
	// 1. set model parameters (m, k, b, etc.)
	static double L, R, kb; // electrical model parameters
	static double J, km, b, fc; // mechanical model parameters
	static int init = 0; // initialization flag
	
	// new parameters for traction model
	static double m, Rw, g, Q, GR;
	double mu, Ft, Fn, wm, v, r, wb, wf;
	double tol = 1.0e-10;
	
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
		
		return 0; // first call to sim_step(...) just sets ICs and parameters
		
	} // end of initialization section
	
	// 4. calculate the derivative vector xd at time t
		
	Fn = m*g*Q; /// normal force on back wheels
	
	// gear equations ////////
	/// wb = 1 / GR * wm
	/// tau_b = GR * tau_m	
	
	// calculate slip ratio r

	wm = x[2]; /// motor angular velocity, wm (rad/s)
	wb = (1/GR)*wm; /// back wheel angular velocity, wb (rad/s)
	
	v = x[4]; // forward velocity v (m/s)

	// checking for / 0 is not needed because of tol
	r = ( wb * Rw - v ) / ( fabs(v) + tol ); ///
	
	// calculate friction coefficient
	mu = calculate_mu(r); ///
	//cout << " miu value is : " << mu << endl;

	// outputs of interest for plotting
	
	y[1] = wb; // back wheel velocity (rad/s)
	
	// calculate front wheel angular velocity wf
	// v = wf * Rw -> wf = v / Rw
	wf = v / Rw;
	
	y[2] = wf; // front wheel velocity (rad/s) 	
	
	y[3] = r;
	
	y[4] = mu;
	
	// calculate tire force
	Ft = mu * Fn;
	
	/// tau_b = GR * tau_m -> tau_m = tau_b / GR
	
	// DC motor equations (modified for tire torque Rw*Ft)
	xd[1] = (-x[1]*R - kb*x[2] + u[1])/L; // di/dt
	xd[2] = (km*x[1] - b*x[2] - fc*SIGN(x[2]) - (Rw*Ft)/GR - u[2])/J; /// dw/dt
	xd[3] = x[2]; // dth/dt = w

	// note that combining state variable equation models
	// normally requires exchange / sharing of coupling 
	// terms / variables between both sets of equations
	// -- in this case the tire force Ft
	
	// algebraic constrants may also occur with 
	// resulting coupling forces (lagrange multipliers, etc.)
	// -- requires differential algebraic equation (DAE) solvers 

	// new state-variable equations for the traction model
	xd[4] = Ft / m; // dv/dt
	xd[5] = x[4]; // dx/dt = v

	// 5. apply Euler's equation, x = x + dx, note x is a vector
	// this part is always the same
	// but calculating xd will normally be different
	for(i=1;i<=NS;i++) x[i] = x[i] + xd[i]*dt; 
		
	t = t + dt; // increment time

	return mu;
	
}


// magic formula for tire friction

// F = N*D*sin(C*atan(B*r-E*(B*r−atan(B*r))))

// coefficient table from
// https://www.mathworks.com/help/physmod/sdl/ref/tiremagicformula.html
// https://www.mathworks.com/help/physmod/sdl/ref/tireroadinteractionmagicformula.html

// Surface	B		C		D		E
// Dry		10		1.9		1		0.97
// Wet		12		2.3		0.82	1
// Snow		5		2		0.3		1
// Ice		4		2		0.1		1

double calculate_mu(double r)
{
	double mu;
	double B, C, D, E, rmax;
	
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


double calculate_mu_bw(double r)
// BW model
// similar to Burckhardt model but with added term a2 for dip in curve
{
	double mu;
	double a1, a2, a3, c1, c2, rmax;

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




void autopid(double &kp, double &ki, double &kd, double y2, double t){

	// define variables outside the loop
	volatile int trflag = 1, tpflag = 1, tsflag = 1;
	volatile double tr, tp, settlemax, po, ts, e, ess, essp = 100,dk = 0.001;
	volatile double sample[50] = { 0.0 }, samplemax;
	volatile double error[50] = {1.0};
	volatile int kptuned=0,kituned=0,kdtuned=0;
	volatile int i, n=0;
		
	tr = 0; tp = 0; ts = 0; // initialize for later use		
	e = 0;
		
	//record rising time
	if (trflag == 1 && (800.0 - y2 / 800.0) < 0.1){
		tr = t;
		trflag = 0;
	}		
		
	n++;
		
	//record peak time and settling max
	if (n > 50 && tpflag == 1){			
		for(i=0;i<50;i++){	
			// "push forward the old data"
			sample[i] = sample[i+1];
		}
		sample[50] = y2;
			
		for(i=0;i<50;i++){	
			// mark max value in samples				
			if(sample[i] > sample[i+1]) sample[i] = samplemax;
			else sample[i+1] = samplemax;
		}
			
		if (sample[25] == samplemax){	
			// if the first "middle point" of the sample is max, then it's the peak time
			// if not accurate, raise sample number "n"
			tp = t;
			settlemax = y2;
			po = (settlemax - 800.0) / 800.0;
			tpflag = 0;		
		}			
	}
		
	//record settling time
	if(tpflag == 0 && tsflag==1){	// record after peak time is reached
		for(i=0;i<50;i++){
			// "push forward the old data"
			error[i] = error[i+1];
			e += error[i];
		}
		error[50] = fabs((y2-800)/800);
		e += error[50];
		ess = e / 50;	// 	steadystate error
		e = 0; // clear data for next loop's calculation
				
		if (ess < 0.02){
			ts = t;
			tsflag = 0;
		}			
	}			
	
	// if time response obtained, compare parameter and adjust kp,ki,kd to optimized value
	if (tr != 0 && tp != 0 && ts != 0){
		cout << "rise time = " << tr << "; peak time = " << tp << "; settling time = " << ts << endl;
		cout << "steady state error = " << ess << "; percentage overshoot = " << po << endl;
		
		if(po < 1.0 && kptuned == 0){
			kp += dk;
		}
		else kptuned =1;
		
		if(kptuned){
			if(po > 0.05 && kituned == 0){
				ki += dk;			
			}
			else kituned = 1;
		}
		
		if (kptuned*kituned){
			if(ess < essp && kdtuned ==0){
				kd += dk;					
			}
			else kdtuned = 1;			
		}
		
		if (kptuned*kituned*kdtuned){
			cout << "optimized kp = " << kp << "; ki = " << ki << "; kd = " << kd << endl;			
		}
	}
}


