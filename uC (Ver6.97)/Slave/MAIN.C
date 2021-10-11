//------------------------------------------------------------------------
//	TO DO LIST: 
//



#include "p30f2010.h"
#include "math.h"



//-----------------------ACTION MACROS-------------------------------

#define LAMP_ON()		PORTEbits.RE4 = 1;
#define LAMP_OFF()		PORTEbits.RE4 = 0;

#define GRIPPER_OPEN()	PORTEbits.RE3 = 0;		// realy output = 12V
#define GRIPPER_CLOSE()	PORTEbits.RE3 = 1;		// realy output = AGND

#define BRAKE_ON()		PORTEbits.RE2 = 1;
#define BRAKE_OFF()		PORTEbits.RE2 = 0;



//----------------------CONTROLLER VARIABLES--------------------------

// MATH
#define pi				3.141592653589793238462643383279
#define _2pi			6.283185307179586476925286766559
#define piover180		0.017453292519943295769236907684

// PWM
#define PWM_PERIOD		1000
#define PWM_PERIODx2 	2000 

// ENCODER
#define IPERREV			2000			// impulse per revolution: 4 x 500
#define MIN_POS			-20000
#define MAX_POS			20000
#define NULL_POSITION	10000
#define MAX_COUNTER		20000
#define imptorad		0.78539816
#define control_period	0.004
#define half_period		0.002



double command;			// calculated command
double pos;				// motor position in radians
double vel;				// motor velocity
double ovel;			// old velocity
double dpos;			// desired position in radians
double dvel;			// desired velocity
double dacc;			// desired acceleration
double fe;				// estimated friction
double me;				// estimated load



//-------------------------SPI VARIABLES-----------------------------

#define CONTROL_DATA_SIZE	9
#define x_slave		0
#define v_slave		4
#define ctr_slave	8
char control_data[CONTROL_DATA_SIZE+1];


#define PATH_DATA_SIZE		13
#define xref	0
#define vref	4
#define aref	8
#define ctr		12
char path_data[PATH_DATA_SIZE+1];


#define XV_DATA_SIZE	32
#define x1	0
#define v1  4
#define x2	8
#define v2  12
#define x3	16
#define v3  20
#define x4	24
#define v4  28
char xv_data[XV_DATA_SIZE];

#define AUX_DATA_SIZE	8
#define aux_param1	0
#define aux_param2	4
char aux_data[AUX_DATA_SIZE];


#define STEP1	0
#define STEP2	1
char comm_status;

char data_idx;

char validData;



//--------------------------------------------------------------------

#define DIN1			0x01
#define DIN2			0x02
#define DIN3TW			0x04
#define VALVE			0x08
#define MOTORBRAKE		0x10
#define CONTROL_TYPE	0x20

char statusByte;			// holds flag bit infos
char controlByte;
//	bits:	00000000
//			||||||||->	DIN1
//			|||||||-->	DIN2 
//			||||||--->	DIN3/Thermal Warning
//			|||||---->  VALVE
//			||||----->  MOTOR BRAKE ON-OFF
//			|||------>	x
//			||------->	x
//			|-------->	x



//-------------------------MISC VARIABLES-----------------------------

double tmp1;
double tmp2;


//------------------------FUCTIONS------------------------------------

void HardwareConfig() {

	//----------------DIGITAL INPUTS----------------------------

	TRISDbits.TRISD0 = 1;	// DIN1
	TRISDbits.TRISD1 = 1; 	// DIN2
	TRISEbits.TRISE5 = 1;	// DIN3

	//---------------DIGITAL OUTPUTS----------------------------

	TRISEbits.TRISE3 = 0;	// Relay control
	TRISEbits.TRISE4 = 0; 	// Digital out (used for LED) 

	PORTEbits.RE3 = 0;		// relay off
	PORTEbits.RE4 = 0;		// lamp off

	//---------------MOTOR CONTROL------------------------------

	TRISEbits.TRISE0 = 0;	// PWM1L
	TRISEbits.TRISE1 = 0;	// DIR
	TRISEbits.TRISE2 = 0; 	// BREAK

	PORTEbits.RE0 = 0; 
	PORTEbits.RE1 = 0; 
	PORTEbits.RE2 = 0; 

	//----------------PWM SETTINGS------------------------------

	PTCON = 0;
	PTCONbits.PTMOD = 0;		// free running mode
	PTPER = PWM_PERIOD;
	PWMCON1 = 0;
	PWMCON1bits.PMOD1 = 1;		// independent outputs
	PWMCON1bits.PMOD2 = 1;		// independent outputs
	PWMCON1bits.PMOD3 = 1;		// independent outputs
	PWMCON1bits.PEN1L = 1;		// enable PWM output pin
	PWMCON2 = 0;
	PWMCON2bits.OSYNC = 1;		// override syncronized!!!

	//---------------SPI CONFIG---------------------------------

	data_idx = 0;
	comm_status = STEP1;
	validData = 0;

	SPI1STAT = 0;
	SPI1STATbits.SPISIDL = 1;	// stop in idle mode
	SPI1CON = 0;	
	SPI1CONbits.SSEN = 1;		// slave select sincronization
	SPI1CONbits.CKE = 1;
	SPI1CONbits.CKP = 0;
	
	TRISFbits.TRISF2 = 1;		// SDI input pin
	TRISFbits.TRISF3 = 0;		// SDO output pin
	TRISEbits.TRISE8 = 1;		// SCK input pin
	TRISBbits.TRISB2 = 1; 		// SS input pin

	IFS0bits.SPI1IF = 0;
	IEC0bits.SPI1IE = 1;
	
	SPI1STATbits.SPIEN = 1;		// enable SPI

	//----------------QEI CONFIG--------------------------------

	ADPCFG |= 0x003C;			// make pins digital inputs, including the SS pin for the SPI
	POSCNT = 0;
	MAXCNT = MAX_COUNTER;
	QEICON = 0;
	QEICONbits.QEIM = 7;		// 4x mode
	QEICONbits.UPDN_SRC = 1;	// from control logic
	DFLTCON = 0;
	DFLTCONbits.CEID = 1;

	//----------------TIMER1 CONFIG------------------------------

	PR1 = 4000;				// to get 200us period with a 80MHz crystal
	T1CON = 0;
	IFS0bits.T1IF = 0;		// clear interrupt flag
    IEC0bits.T1IE = 1;		// enable timer interrupt

	//----------------TIMER2 CONFIG------------------------------

	PR2 = 20000;			// to get 8mS period with a 80MHz crystal
	T2CON = 0;
	T2CONbits.TCKPS = 2;	// prescaler 1:8
	IFS0bits.T2IF = 0;		// clear interrupt flag
    IEC0bits.T2IE = 1;		// enable timer interrupt
}

void SendCommand() {
	if (command<0) {
		if (command <-1) command =-1;
		PORTEbits.RE1 = 1;	// DIR
		PDC1 = -command * PWM_PERIODx2;
	} else {
		if (command > 1) command = 1;
		PORTEbits.RE1 = 0;	// DIR
		PDC1 = command * PWM_PERIODx2;
	}
}

void MakeMeasurement() {
	ovel = vel;
	vel = POSCNT;
	POSCNT = NULL_POSITION;
	vel = (vel - NULL_POSITION)*imptorad;
	pos = pos + (ovel+vel)*half_period;

	if (pos>MAX_POS) pos = MAX_POS;
	if (pos<MIN_POS) pos = MIN_POS;
}

void StartControl() {

	// PWM
	PORTEbits.RE1 = 0;		// DIR = 0 <=> negative
	PTMR = 0;				// reset pwm timer
	PTCONbits.PTEN = 1;		// pwm module ON

	// ENCODER
	POSCNT = NULL_POSITION;

	// CONTROL
	pos 		= 3.11;
	vel 		= 1.11;
	command 	= 0;
	me 			= 0;
	fe			= 0;
}

//---------------------CONTROL ALGORITHM--------------------------------

#define w			1.14
#define A			4

#define lamda 		10

#define Ks	 		0.01
#define gamma_f		0.0 //0.0000001
#define gamma_m		0.0 //0.00000001

double S;

void AdaptiveControl() {
	tmp1 = dvel - vel;

	S = tmp1 + lamda * (dpos - pos);
	me 	= me + gamma_m * (dacc + lamda * tmp1) * S;	

	if (me<0.0) me = 0.0;

	if (vel>0.0) {
		fe = fe - gamma_f * S;
    	command = -fe + me * (dacc + lamda * tmp1) + Ks * S;
	} else {
		fe = fe + gamma_f * S;
    	command = fe + me * (dacc + lamda * tmp1) + Ks * S;
	}
}

void CalculateCommand() {

	dpos = *((double*)(&path_data[xref]));
	dvel = *((double*)(&path_data[vref]));
	dacc = *((double*)(&path_data[aref]));

	AdaptiveControl();

	if (command > 0.95) command = 0.95;
	if (command <-0.95) command =-0.95;
}

void SaveContext() { 
	statusByte = 0;

	if (PORTDbits.RD0) {
		statusByte = statusByte + DIN1;

		// home position
		pos = 0;
	} 

	if (PORTDbits.RD1) {
		statusByte = statusByte + DIN2;
	}

	if (PORTEbits.RE5) {
		statusByte = statusByte + DIN3TW;
	}

	*((double*)(&control_data[x_slave])) = pos;
	*((double*)(&control_data[v_slave])) = vel;
	control_data[ctr_slave] = statusByte;

	*((double*)(&aux_data[aux_param1])) = dpos;
	*((double*)(&aux_data[aux_param2])) = dvel;
}


//------------------------MAIN FUCTION-------------------------------
//
//

int main() {

	HardwareConfig();

	StartControl();


	//--------------------NOTICE!!!!!!!!!!-----------------------
	// put the first byte of control_data into SPI1BUF register
	// after finding the home position (x = known, v = 0)
	// SPI communication should start after this event!!!!!!
	SaveContext();
	SPI1BUF = control_data[0];
	
	
	while (1) {
		if (validData) {
			//------------------------------------------------------------
			// restart control monitor. If no control signal arrives
			// within the a predefined amount of time
			TMR2 = 0;
			T2CONbits.TON = 1;

			//------------------------------------------------------------
			// unblock the controller, if was blocked
			BRAKE_OFF();

			if (path_data[ctr] & CONTROL_TYPE) {
				MakeMeasurement();
				CalculateCommand();
				SendCommand();
			} else {
				MakeMeasurement();
				command = path_data[0] / 128.0;
				SendCommand();
			}

			//------------------------------------------------------------
			// we save the actual parameters of this actuator
			SaveContext();

			//----------------NOTICE!!!!!!!!!-----------------------------
			// at the end of every cycle you must load into SPI1BUF the 
			// first byte of the actual control_data (recently measured)
			SPI1BUF = control_data[0];

			validData = 0;
		}

	}

	return 0;
}

void __attribute__((__interrupt__)) _SPI1Interrupt(void) {

	IFS0bits.SPI1IF = 0;

	//---------------------------------------------------
	// restart SPI traffic monitor; after one period 
	// SPI communication status will be reset
	// NOTICE!!! at this point validData must be null
	// to avoid SPI overrun (calculations must be finished).
	TMR1 = 0;
	T1CONbits.TON = 1;



	if (comm_status==STEP1) {
		//-----------------------------STEP1-----------------------------
		// receive path data (xref,vref,aref,ctr)
		// send back actual (x,v,ctr)

		path_data[data_idx] = SPI1BUF;

		data_idx++;

		if (data_idx<CONTROL_DATA_SIZE) {
			SPI1BUF = control_data[data_idx];
		} else {
			SPI1BUF = 0;
		}

		if (data_idx==PATH_DATA_SIZE) {
			data_idx = 0;
			comm_status = STEP2;

			// this loaded data will be sent in STEP2
			SPI1BUF = aux_data[data_idx];
		}
	} else {
		//-----------------------------STEP2-----------------------------
		// receive xv data (x,v)
		// no data sent back
		xv_data[data_idx] = SPI1BUF;

		data_idx++;	

		SPI1BUF = data_idx<AUX_DATA_SIZE ? aux_data[data_idx]:0;
	}
}

void __attribute__((__interrupt__)) _T1Interrupt(void)
{
	IFS0bits.T1IF = 0;

	T1CONbits.TON = 0;

	if (data_idx==XV_DATA_SIZE && comm_status==STEP2) {
		validData = 1;
	}

	comm_status = STEP1;
	data_idx = 0;
}

void __attribute__((__interrupt__)) _T2Interrupt(void)
{
	IFS0bits.T2IF = 0;

	T2CONbits.TON = 0;

	BRAKE_ON();

	command = 0;
	SendCommand();
}
