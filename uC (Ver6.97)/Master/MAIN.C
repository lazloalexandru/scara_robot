
#include <pic18fxx2.h> 
#include <math.h>
#include "path.h"
#include "params.h"



//-------------------------------------------------------
// these variables are used for data exchange via SPI
// communication

#define NUM_SLAVES		4

#define PATH_DATA_SIZE	13
#define xref	0
#define vref	4
#define aref	8
#define ctr		12

char path_data[NUM_SLAVES][PATH_DATA_SIZE+1];

#define CONTROL_DATA_SIZE	9
#define x_slave		0
#define v_slave		4
#define ctr_slave	8

#define XV_PAIR_SIZE	8

char received_data[NUM_SLAVES][CONTROL_DATA_SIZE+1];

#define AUX_DATA_SIZE	8
char aux_data[NUM_SLAVES][AUX_DATA_SIZE];

#define DIN1			0x01
#define DIN2			0x02
#define DIN3_TW 		0x04
#define VALVE			0x08
#define MOTOR_BRAKE 	0x10
#define CONTROL_TYPE	0x20

char data_idx;
char slave_idx;
char slave_data_idx;

#define NONE				-1



//--------------------------------------------------------
//	other
//

char 	goFlag;



//--------------------------------------------------------
// Path Generator
//

#define 	Ts		0.004

#define 	STAGE1	0
#define 	STAGE2	2
#define 	STAGE3	3
#define 	STOPPED	4

char 	path_stage[NUM_SLAVES];
double	time[NUM_SLAVES];				// time
double 	t;
int 	i;
char	valid_path_params;
char	num_active_tracks;

unsigned long global_time;

double _xx0[4];

//--------------------------------------------------------
//	USART
//

#define MANUAL_DATA			0x69
#define TRACKING_DATA		0x96
#define NO_CONTROL			0xD3
#define CONTROL_MANUAL		0xA5
#define CONTROL_TRACKING	0x5A
#define HOME_SEARCH			0xBC
#define START_CONTROL		0x3E
#define STOP_CONTROL		0xE6
char controlState;
char controller_enabled;

#define WAIT_COMMAND		0x0
#define WAIT_DATA			0x1
#define WAIT_MANUAL_DATA	0x2
#define WAIT_TRACKING_DATA	0x3
char commState;


#define MANUAL_DATA_SIZE		4
#define TRACKING_DATA_SIZE		16

#define MAX_USART_DATA_SIZE		16
char usart_rdata[MAX_USART_DATA_SIZE];
char usart_ridx;

#define MONITORED_DATA_SIZE		73
char usart_sdata[MONITORED_DATA_SIZE];
char usart_sidx;
char valid_sdata;

char rxData;

char manual_ctr[MANUAL_DATA_SIZE];
char valid_manual_ctr;
double tracking_ctr[NUM_SLAVES];
char we_have_new_target;



//--------------------------------------------------------
//	HOME SEARCH
//

#define PWM_HOME1	-20
#define PWM_HOME2	-20
#define PWM_HOME3	-70
#define PWM_HOME4	-20
char homes;
char went_home;


//--------------------------------------------------------
//----------------INTERRUPT SERVICE ROUTINE---------------
//--------------------------------------------------------

void interrupt low_priority ISR_low(void) {

	if ((TMR2IE)&&(TMR2IF)) {
		TMR2IF = 0;
		
		LATE2 = !LATE2;

		if (controller_enabled==START_CONTROL) {
			global_time++;

			switch (controlState) {

				case CONTROL_TRACKING:

					if (valid_path_params) {

						if (!num_active_tracks) {
							valid_path_params = 0;
							LATD7 = 0;
						}


						//------------------------------------------------------------------------------------------
						//										Fill out PATH DATA
						//------------------------------------------------------------------------------------------
	
			
						for (i=0; i<NUM_SLAVES; i++) {

							path_data[i][ctr] = 0b00100000;	// TRACKING Control
	
							if (_nseg[i]==3) {
	
								//----------------------------------------------------------------------------------
								//	Three Segment Path
								//----------------------------------------------------------------------------------

								t = time[i];
	
								switch (path_stage[i]){
									case STAGE1: {
										*((double *)(&path_data[i][aref])) = _amax[i];
										*((double *)(&path_data[i][vref])) = _amax[i]*t;
										*((double *)(&path_data[i][xref])) = _x0[i]+0.5*_amax[i]*t*t;
		
										t += Ts;
										if (t>=_dt1[i]) {
											t = 0;
											path_stage[i] = STAGE2;
										}
										break;
									}
									case STAGE2: {
										*((double *)(&path_data[i][aref])) = 0;
										*((double *)(&path_data[i][vref])) = _vmax[i];
										*((double *)(&path_data[i][xref])) = _xt1[i]+_vmax[i]*t;
		
										t += Ts;
										if (t>=_dt2[i]) {
											t = 0;
											path_stage[i] = STAGE3;
										}
										break;
									}
									case STAGE3: {
								
										*((double *)(&path_data[i][aref])) = -_amax[i];
										*((double *)(&path_data[i][vref])) = _vmax[i]-_amax[i]*t;
										*((double *)(&path_data[i][xref])) = _xt2[i]+_vmax[i]*t-0.5*_amax[i]*t*t;
	
										t += Ts;
										if (t>=_dt1[i]) {
											t = 0;
											path_stage[i] = STOPPED;
											num_active_tracks--;
										}
										break;
									}
									case STOPPED: {
										*((double *)(&path_data[i][aref])) = 0;
										*((double *)(&path_data[i][vref])) = 0;
										*((double *)(&path_data[i][xref])) = _xf[i];
										break;
									}
								}
	
								time[i] = t;

							} else {
	
								//----------------------------------------------------------------------------------
								//	Two Segment Path
								//----------------------------------------------------------------------------------

								t = time[i];

								switch (path_stage[i]){

									case STAGE1: {
										*((double *)(&path_data[i][aref])) = _amax[i];
										*((double *)(&path_data[i][vref])) = _amax[i]*t;
										*((double *)(&path_data[i][xref])) = _x0[i]+0.5*_amax[i]*t*t;

										t += Ts;
										if (t>=_dt1[i]) {
											t = 0;
											path_stage[i] = STAGE2;
										}
										break;
									}
									case STAGE2: {
										*((double *)(&path_data[i][aref])) = -_amax[i];
										*((double *)(&path_data[i][vref])) = _vt1[i]-_amax[i]*t;
										*((double *)(&path_data[i][xref])) = _xt1[i]+_vt1[i]*t-0.5*_amax[i]*t*t;

										t += Ts;
										if (t>=_dt1[i]) {
											t = 0;
											path_stage[i] = STOPPED;
											num_active_tracks--;
										}
										break;
									}
									case STOPPED: {
										*((double *)(&path_data[i][aref])) = 0;
										*((double *)(&path_data[i][vref])) = 0;
										*((double *)(&path_data[i][xref])) = _xf[i];
										break;
									}
								}

								time[i] = t;
							}
						}
					} else {
						path_data[0][ctr] = 0;	// Manual Control
						path_data[1][ctr] = 0;	// Manual Control
						path_data[2][ctr] = 0;	// Manual Control
						path_data[3][ctr] = 0;	// Manual Control

						path_data[0][0] = 0;
						path_data[1][0] = 0;
						path_data[2][0] = 0;
						path_data[3][0] = 0;
					}

					break;

				case CONTROL_MANUAL:
		
					if (valid_manual_ctr) {
						path_data[0][0] = manual_ctr[0];
						path_data[1][0] = manual_ctr[1];
						path_data[2][0] = manual_ctr[2];
						path_data[3][0] = manual_ctr[3];
				
						valid_manual_ctr = 0;
					} else {
	
						path_data[0][0] = 0;
						path_data[1][0] = 0;
						path_data[2][0] = 0;
						path_data[3][0] = 0;
					}

					path_data[0][ctr] = 0;	// Manual Control
					path_data[1][ctr] = 0;	// Manual Control
					path_data[2][ctr] = 0;	// Manual Control
					path_data[3][ctr] = 0;	// Manual Control

					break;
	
				case HOME_SEARCH:
	
					if (homes==0x0F) {
				
						_x0[0] = 0;
						_x0[1] = 0;
						_x0[2] = 0;
						_x0[3] = 0;
		
						went_home = 1;
	
						controlState = NO_CONTROL;
	
						LATD4 = 0;
						LATD5 = 0;
					}

					if (received_data[0][ctr_slave] & DIN1) {
						homes = homes | 0x1;
					}

					if (received_data[1][ctr_slave] & DIN1) {
						homes = homes | 0x2;
					}

					if (received_data[2][ctr_slave] & DIN1) {
						homes = homes | 0x4;
					}

					if (received_data[3][ctr_slave] & DIN1) {
						homes = homes | 0x8;
					}

					path_data[0][0] = homes&0x1 ? 0:PWM_HOME1;
					path_data[1][0] = homes&0x2 ? 0:PWM_HOME2;
					path_data[2][0] = homes&0x4 ? 0:PWM_HOME3;
					path_data[3][0] = homes&0x8 ? 0:PWM_HOME4;


					path_data[0][ctr] = 0;	// Manual Control
					path_data[1][ctr] = 0;	// Manual Control
					path_data[2][ctr] = 0;	// Manual Control
					path_data[3][ctr] = 0;	// Manual Control

					break;
				default:
					path_data[0][0] = 0;
					path_data[1][0] = 0;
					path_data[2][0] = 0;
					path_data[3][0] = 0;


					path_data[0][ctr] = 0;	// Manual Control
					path_data[1][ctr] = 0;	// Manual Control
					path_data[2][ctr] = 0;	// Manual Control
					path_data[3][ctr] = 0;	// Manual Control

					break;
			}

	
			//------------------------------------------------------------------------------------------
			//										SPI DATA TRANSFER
			//------------------------------------------------------------------------------------------

			//------------------------------------------------------------------
			//--------------------------STEP 1----------------------------------
			//------------------------------------------------------------------

			for (data_idx=0; data_idx<PATH_DATA_SIZE; data_idx++) {
				for (slave_idx=0; slave_idx<NUM_SLAVES; slave_idx++) {
					//------------------------------------
					// Select Slave
					//
					LATA = 0xFF;

					switch (slave_idx) {
						case 0: 
							LATA0 = 0;
							break;
						case 1: 
							LATA1 = 0;
							break;
						case 2: 
							LATA2 = 0;
							break;
						case 3: 
							LATA3 = 0;
							break;
						case 4: 
							LATA4 = 0;
							break;
						default: 
							break;
					}

					//------------------------------------
					// Data Transfer
					//
					SSPBUF = path_data[slave_idx][data_idx];
	
					while (BF==0);
		
					if (data_idx<CONTROL_DATA_SIZE) {
						received_data[slave_idx][data_idx] = SSPBUF;
					}
				}
			}


			_xx0[0] = *((double *)(&received_data[0][x_slave]));
			_xx0[1] = *((double *)(&received_data[1][x_slave]));
			_xx0[2] = *((double *)(&received_data[2][x_slave]));
			_xx0[3] = *((double *)(&received_data[3][x_slave]));


			//------------------------------------------------------------------
			//--------------------------STEP 2----------------------------------
			//------------------------------------------------------------------

			for (slave_data_idx=0; slave_data_idx<NUM_SLAVES; slave_data_idx++) {
				for (data_idx=0; data_idx<XV_PAIR_SIZE; data_idx++) {
					for (slave_idx=0; slave_idx<NUM_SLAVES; slave_idx++) {
						//------------------------------------
						// Select Slave
						//
						LATA = 0xFF;
						switch (slave_idx) {
							case 0: 
								LATA0 = 0;
								break;
							case 1: 
								LATA1 = 0;
								break;
							case 2: 
								LATA2 = 0;
								break;
							case 3: 
								LATA3 = 0;
								break;
							case 4: 
								LATA4 = 0;
								break;
							default: 
								break;
						}

						//------------------------------------
						// Data Transfer
						//
						SSPBUF = received_data[slave_data_idx][data_idx];
						while (BF==0);
						
						if (slave_data_idx==0) {
							aux_data[slave_idx][data_idx] = SSPBUF;
						}
					}
				}
			}

			//------------------------------------
			// Disable all Slaves
			//
			LATA = 0xFF;

			//------------------------------------------------------------------------------------
			//						LOAD MONITORED DATA for USART TRANSMIT
			//------------------------------------------------------------------------------------

			if (!valid_sdata) {
				usart_sidx = 0;
	
				for (slave_data_idx=0; slave_data_idx<NUM_SLAVES; slave_data_idx++) {
					for (data_idx=0; data_idx<CONTROL_DATA_SIZE; data_idx++) {
						usart_sdata[usart_sidx] = received_data[slave_data_idx][data_idx];
						usart_sidx++;
					}
				}

				for (slave_data_idx=0; slave_data_idx<NUM_SLAVES; slave_data_idx++) {
					for (data_idx=0; data_idx<AUX_DATA_SIZE; data_idx++) {
						usart_sdata[usart_sidx] = aux_data[slave_data_idx][data_idx];
						usart_sidx++;
					}					
				}

				*((unsigned long *)(&usart_sdata[usart_sidx])) = global_time;
				usart_sidx = usart_sidx + 4;

				usart_sdata[MONITORED_DATA_SIZE-1] = 0xA5;
				usart_sidx = 0;
				valid_sdata = 1;
			}
		}
	}
}

void interrupt ISR() {
	
	//**********************************************************************
	//**********************************************************************
	//							USART DATA TRANSFER
	//**********************************************************************
	//**********************************************************************
	
	if ((RCIE)&&(RCIF)) {
		RCIF = 0;
		
		rxData = RCREG;

		if (commState == WAIT_COMMAND) {
			
			switch(rxData) {
				case NO_CONTROL:
					if (controller_enabled!=START_CONTROL) break;

					commState = WAIT_COMMAND;
					controlState = NO_CONTROL;
					
					LATD4 = 0;
					LATD5 = 0;
					break;
				case START_CONTROL:
					global_time = 0;
					controller_enabled = START_CONTROL;
					break;	
				case STOP_CONTROL:
					controller_enabled = 0;

					LATD4 = 0;
					LATD5 = 0;
					break;	
				case HOME_SEARCH:
					if (controller_enabled!=START_CONTROL) break;

					commState = WAIT_COMMAND;
					controlState = HOME_SEARCH;

					homes = 0;

					LATD4 = 1;
					LATD5 = 1;
					break;
				case CONTROL_MANUAL:
					if (controller_enabled!=START_CONTROL) break;

					commState = WAIT_COMMAND;
					controlState = CONTROL_MANUAL;

					LATD4 = 1;
					LATD5 = 0;
					break;
				case CONTROL_TRACKING:
					if (controller_enabled!=START_CONTROL) break;

					valid_path_params = 0;
					commState = WAIT_COMMAND;
					controlState = CONTROL_TRACKING;

					LATD4 = 0;
					LATD5 = 1;
					break;
				case MANUAL_DATA:
					commState = WAIT_MANUAL_DATA;
					usart_ridx = 0;
					break;
				case TRACKING_DATA:
					commState = WAIT_TRACKING_DATA;
					usart_ridx = 0;
					break;
				default:
					commState = WAIT_COMMAND;					
					break;
			}

		} else {
    		// WAIT_DATA
			usart_rdata[usart_ridx] = rxData;
			usart_ridx++;

			switch(commState) {
				case WAIT_MANUAL_DATA:
					if (usart_ridx == MANUAL_DATA_SIZE) {

						manual_ctr[0] = usart_rdata[0];
						manual_ctr[1] = usart_rdata[1];
						manual_ctr[2] = usart_rdata[2];
						manual_ctr[3] = usart_rdata[3];
	
						valid_manual_ctr = 1;

						commState = WAIT_COMMAND;
					}
					break;
				case WAIT_TRACKING_DATA:
					if (usart_ridx == TRACKING_DATA_SIZE) {
						if (!we_have_new_target) {
							tracking_ctr[0] = *((double *)(&usart_rdata[0]));
							tracking_ctr[1] = *((double *)(&usart_rdata[4]));
							tracking_ctr[2] = *((double *)(&usart_rdata[8]));
							tracking_ctr[3] = *((double *)(&usart_rdata[12]));
						}

						we_have_new_target = 1;

						commState = WAIT_COMMAND;
					}
					break;
				default:
					break;
			}
		}
	}
}

//--------------------------------------------------------
//------------------------FUNCTIONS-----------------------
//--------------------------------------------------------

void HardwareInit() {

//--------------- Control ----------------------------

	controller_enabled = 1;
	valid_sdata = 0;
	valid_manual_ctr = 0;
	we_have_new_target = 0;
	went_home =  0;
	valid_path_params = 0;

//--------------------- INTERRUPTS ----------------------------

	PEIE = 1;			// peripheral interrupt enable
	GIE = 1;
	IPEN = 1;
	
//--------------------- TIMER2 ----------------------------

	global_time = 0;

//	PR2 = 0x7C;				// 500Hz @ Fosc = 40MHz	
	PR2 = 0xF9;				// 250Hz @ Fosc = 40MHz	
	T2CON = 0x4E;
	TMR2IP = 0;
	TMR2IF = 0;
	TMR2IE = 1;

//--------------------- LEDs ------------------------------

	TRISA5 = 0;
	TRISE2 = 0;

	LATA5 = 0;
	LATE2 = 0;

//--------------------- Digital I/O -----------------------

	TRISB2 = 1;		// digital input
	TRISB3 = 1;		// digital input
	TRISB4 = 1;		// digital input
	TRISB5 = 1;		// digital input
	
	TRISD4 = 0;		// digital output
	TRISD5 = 0;		// digital output
	TRISD6 = 0;		// digital output
	TRISD7 = 0;		// digital output

	LATD4 = 0;
	LATD5 = 0;
	LATD6 = 0;
	LATD7 = 0;

//------------------- USART Setup --------------------------

	usart_sidx = 0;
	usart_ridx = 0;

	TRISC6 = 0;		// TX pin output
	TRISC7 = 1;		// RX pin input

	TXSTA = 0;
	CREN = 1;		// continuous receive 
	BRGH = 1;		// high baud rate
	TXEN = 1;		// enable transmit
	SPBRG = 129;	// baud constand for 19200 @ 40MHz
//	SPBRG = 64;		// baud constand for 9600 @ 40MHz
	RCIP = 1;		// high priority
	RCIF = 0;
	RCIE = 1;
	SPEN = 1;		// enable module

	// related

	controlState = NO_CONTROL;
	commState = WAIT_COMMAND;	

//--------------------- SPI Setup --------------------------

	slave_idx = 0;
	data_idx = 0;
	slave_data_idx = 0;

	TRISC5 = 0; // SDO
	TRISC3 = 0; // SCK
	TRISC4 = 1; // SCK

	TRISA0 = 0;	// SLAVE1
	TRISA1 = 0;	// SLAVE2
	TRISA2 = 0;	// SLAVE3
	TRISA3 = 0;	// SLAVE4
	TRISA4 = 0;	// SLAVE5

	LATA0 = 1; // Disable
	LATA1 = 1; // Disable
	LATA2 = 1; // Disable
	LATA3 = 1; // Disable
	LATA4 = 1; // Disable
   	
	SSPBUF = 0;
	SSPSTAT = 0;
	CKP = 0; 		// idle state for clock low level
	CKE = 1; 		// transmit on rising edge of SCK
	SSPCON1 = 0x01;

	SSPEN	= 1;
}

char CalcPathParams(char idx) {

	if (_x0[idx] < x_min[idx]) _x0[idx] = x_min[idx];
	if (_x0[idx] > x_max[idx]) _x0[idx] = x_max[idx];
	if (_xf[idx] < x_min[idx]) _xf[idx] = x_min[idx];
	if (_xf[idx] > x_max[idx]) _xf[idx] = x_max[idx];

	if (_xf[idx]==_x0[idx]) return 0;

	_amax[idx] = (_xf[idx]<_x0[idx] ? -1:1)*a_max[idx];
	_vmax[idx] = (_xf[idx]<_x0[idx] ? -1:1)*v_max[idx];

	if (v_max[idx]*v_max[idx]/a_max[idx] < fabs(_xf[idx]-_x0[idx])) {
		// three segment path

		_nseg[idx] = 3;

		_xt1[idx] = _vmax[idx]*_vmax[idx]/(2*_amax[idx]) + _x0[idx];
		_xt2[idx] = _xf[idx] - (_xt1[idx] - _x0[idx]);

		_dt1[idx] = _vmax[idx]/_amax[idx];
		_dt2[idx] = (_xt2[idx]-_xt1[idx])/_vmax[idx];
				
		_vt1[idx] = 0;
	} else {
		// two segment path

		_nseg[idx] = 2;

		_dt1[idx] = sqrt((_xf[idx]-_x0[idx])/_amax[idx]);
		_xt1[idx] = (_x0[idx]+_xf[idx])/2;
		_vt1[idx] = _amax[idx]*_dt1[idx];

		_xt2[idx] = 0;
		_dt2[idx] = 0;
	}

	return 1;
}

int counter;
char temp;

void main() {
	goFlag = 0;

	HardwareInit();

	// give some time for slave startup
	for (counter=0; counter<20000; counter++);
	
	goFlag = 1;
		
	while (1) {

		if (controlState==CONTROL_TRACKING && went_home && we_have_new_target && !valid_path_params) {
			
			_x0[0] = _xx0[0];
			_x0[1] = _xx0[1];
			_x0[2] = _xx0[2];
			_x0[3] = _xx0[3];
			
			_xf[0] = tracking_ctr[0];
			_xf[1] = tracking_ctr[1];
			_xf[2] = tracking_ctr[2];
			_xf[3] = tracking_ctr[3];

			num_active_tracks = 0;
			temp = 0;
			
			for (counter=0; counter<NUM_SLAVES; counter++) {
				time[counter] = 0;
				path_stage[counter] = STOPPED;

				if (CalcPathParams(counter)) {
					temp = 1;				// itt ez nem kell me a num_active_tracks ugyanazt jelzi
					path_stage[counter] = STAGE1;
					num_active_tracks++;
				}
			}

			if (temp) {
				valid_path_params = 1;
				LATD7 = 1;
			}

			we_have_new_target = 0;
		}

		if (valid_sdata) {
			while(!TRMT);
			TXREG = usart_sdata[usart_sidx];
			usart_sidx++;
			
			if (usart_sidx==MONITORED_DATA_SIZE) {
				valid_sdata = 0;
			}
		}
	}
}

