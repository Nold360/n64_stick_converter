/*
 * N64_Stick_Converter_PCB_v3.c
 *
 * Created: 25.02.2015 17:48:54
 * Author: Jakob Schäfer
 *
 * ONLY FOR YOUR OWN PERSONAL USE! COMMERCIAL USE PROHIBITED!
 * NUR FÜR DEN EIGENGEBRAUCH! GEWERBLICHE NUTZUNG VERBOTEN! 
 *
 * fusebyte low:	0x42
 * fusebyte high:	0xDF
 *
 * Note: Using -o1 optimization level for the AVR/GNU C compiler
 *		 is recommended.
 *
 * --------------------------------------------------------------
 * ATtiny24A pin	|	function
 * (PDIP / SOIC)	|
 * -------------------------------------------------------------|
 * 1				|	VCC =
 * 					|	N64 controller PCB pin no. 5.
 * 					|	Bypass to GND with 100 nF capacitor
 * -------------------------------------------------------------|
 * 2				|	N64 controller PCB pin no. 6
 * -------------------------------------------------------------|
 * 3				|	N64 controller PCB pin no. 3
 * -------------------------------------------------------------|
 * 4				|	RESET for programming
 * 					|	Connect with 10 kOhm resistor to VCC
 * -------------------------------------------------------------|
 * 5				|	calibration slider switch
 * 					|	(leave floating or short to GND)
 * 					|	change switch positon for calibration
 * -------------------------------------------------------------|
 * 6				|	N64 controller PCB pin no. 2
 * -------------------------------------------------------------|
 * 7				|	N64 controller PCB pin no. 1
 * 					|	MOSI for programming
 * -------------------------------------------------------------|
 * 8				|	extended range mode button (active low)
 *					|	short to GND to use extended range mode
 * -------------------------------------------------------------|
 * 9				|	SCK for programming
 * -------------------------------------------------------------|
 * 10				|	calibration button 1 (active low)
 * -------------------------------------------------------------|
 * 11				|	calibration button 2 (active low)
 *					|	short both buttons to GND for calibr.
 * -------------------------------------------------------------|
 * 12				|	X axis of the stick potentiometer
 * -------------------------------------------------------------|
 * 13				|	Y axis of the stick potentiometer
 * -------------------------------------------------------------|
 * 14				|	GND =
 * 					|	N64 controller PCB pin no. 4
 * -------------------------------------------------------------|
 *
 *
 * If you want to increase/decrease the range of the stick, then try
 * out new values for the MIN_RANGE and MAX_RANGE constants below:
 */ 

 
/******************************************************************************
Macros & Defines
******************************************************************************/
// clock frequency
#define F_CPU 1000000UL

// +/- minimum range that will be achieved for each axis in standard range mode
#define MIN_RANGE_STD 81
// max. range limit in standard range mode; higher values will be clipped
#define MAX_RANGE_STD 84

// +/- minimum range that will be achieved for each axis in extended range mode
#define MIN_RANGE_XTD 91
// max. range limit in extended range mode, higher values will be clipped
#define MAX_RANGE_XTD 94

/******************************************************************************
Includes
******************************************************************************/ 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>

/******************************************************************************
Prototypes
******************************************************************************/
// returns a 16 bit ADC value of the potentiometer stick's x axis (0 - 1023)
uint16_t GetX(void);

// returns a 16 bit ADC value of the potentiometer stick's y axis (0 - 1023)
uint16_t GetY(void);

// scales the 16 bit ADC value down to 8 bits: result = raw16 * factor c / 256
uint8_t ScaleDown(uint16_t raw16, uint8_t c);

// rotates a byte left by one bit
uint8_t RotateLeft(uint8_t cData);

// rotates a byte left by one bit
uint8_t RotateRight(uint8_t cData);

// calculates the c factors and saves them into EEPROM
void Calibration(void);

/******************************************************************************
Globals
******************************************************************************/
// factors for x & y axis in standard range mode
uint8_t EEMEM cx_std = 0;						
uint8_t EEMEM cy_std = 0;			

// factors for x & y axis in extended range mode
uint8_t EEMEM cx_xtd = 0;						
uint8_t EEMEM cy_xtd = 0;

// for detecting first power on
uint8_t	EEMEM firstPowerOn = 1;				

// stores the position of the calibration slider switch
uint8_t EEMEM calibSwitch;

/******************************************************************************
Fuses
******************************************************************************/
__fuse_t __fuse __attribute__((section (".fuse"))) = {	.low		= 0x42, 
														.high		= HFUSE_DEFAULT,
														.extended	= EFUSE_DEFAULT};
  
int main(void)
{	
	int16_t xSteps, ySteps;
	uint16_t x, y, xOld, yOld;
	uint8_t xNeutral8, yNeutral8;
	uint8_t xWheel = 0b11001100;
	uint8_t yWheel = 0b00110011;
	uint16_t xNeutral16, yNeutral16;
	uint8_t xFactor, yFactor, maxRange;
	
	
	// set up the ports immediately
	DDRA = (1<<DDA6)|(1<<DDA7);
	DDRB = (1<<DDB0)|(1<<DDB1);
	PORTA = (1<<PORTA2)|(1<<PORTA3)|(1<<PORTA5);
	PORTB = (1<<PORTB2);
	
	// deactivate timer0, timer1 and USI peripherals for saving power
	PRR |= (1<<PRTIM0)|(1<<PRTIM1)|(1<<PRUSI);
	
	// now wait a little bit
	_delay_ms(250);	
		
	// ADC setup
	DIDR0 = (1<<ADC0D)|(1<<ADC1D);			// digital input disable for PORTA0+1	
	ADMUX = 0x01;							// channel 1 
	ADCSRA = (1<<ADPS0)|(1<<ADPS1);			// prescaler = 8 ==> f_ADC = 1 MHz/8 = 125 kHz
	ADCSRA |= (1<<ADEN);					// enable ADC

		
	// extended range mode if ext. range mode button is pushed
	if ( !(PINA&(1<<PORTA5)) ){
		xFactor = eeprom_read_byte(&cx_xtd);
		yFactor = eeprom_read_byte(&cy_xtd);
		maxRange = MAX_RANGE_XTD;
	}
	// standard range mode otherwise
	else{
		xFactor = eeprom_read_byte(&cx_std);
		yFactor = eeprom_read_byte(&cy_std);
		maxRange = MAX_RANGE_STD;	
	}
	
	// first AD conversion; initialize analog circuitry
	xNeutral16 = GetX();	
	
	// get x axis neutral position
	xNeutral16 = GetX();					
	xNeutral8 = ScaleDown(xNeutral16, xFactor);
	xOld = xNeutral8;
	
	// get y axis neutral position
	yNeutral16 = GetY();						
	yNeutral8 = ScaleDown(yNeutral16, yFactor);
	yOld = yNeutral8;	
	
	// execute calibration if:
	// a) microcontroller is powered on for the first time or
	// b) the calibration switch's position has been changed or
	// c) both calibration button's have been pushed
	if (	(eeprom_read_byte(&firstPowerOn)) || 
			((PINB&(1<<PORTB2)) != eeprom_read_byte(&calibSwitch)) ||
			!(PINA&((1<<PORTA2)|(1<<PORTA3)))	)	Calibration();
	
	
    while(1)
    {	
		
		// get x axis position
		x = GetX();
		// scale down
		x = ScaleDown(x, xFactor);
		// limit position to  +/- maxRange
		if ( (x>xNeutral8) && ((x-xNeutral8) > maxRange) ) x = xNeutral8 + maxRange;
		if ( (x<xNeutral8) && ((xNeutral8-x) > maxRange) ) x = xNeutral8 - maxRange;
						
		// get y axis position
		y = GetY();			
		// scale down
		y = ScaleDown(y, yFactor);		
		// limit position to  +/- maxRange
		if ( (y>yNeutral8) && ((y-yNeutral8) > maxRange) ) y = yNeutral8 + maxRange;
		if ( (y<yNeutral8) && ((yNeutral8-y) > maxRange) ) y = yNeutral8 - maxRange;
	
		// calculate the amount of steps (= increments or decrements) for both axes
		xSteps =  (int16_t) x - xOld;
		ySteps =  (int16_t) y - yOld;
		
		// store current stick position for the next cycle
		xOld = x;
		yOld = y;
		
		// while there are still steps left...
		while ( (xSteps!=0) || (ySteps!=0) ){
			
			// rotate the x wheel...					
			if (xSteps<0){
				xWheel = RotateLeft(xWheel);
				xSteps++;				
			}						
			if (xSteps>0){
				xWheel = RotateRight(xWheel);
				xSteps--;			
			}	
			
			// rotate the y wheel...		
			if (ySteps>0){
				yWheel = RotateRight(yWheel);
				ySteps--;				
			}			
			if (ySteps<0){
				yWheel = RotateLeft(yWheel);
				ySteps++;			
			}		
			
			// and put out the new XA/XB and YA/YB values:			
			PORTB = (PORTB&0b11111100)|(xWheel & 0b00000011); 
			PORTA = (PORTA&0b00111111)|(yWheel & 0b11000000);
		}	
		
    }	
	
}


uint16_t GetX(void){	
	// select ADC channel 1
	ADMUX = 0x01;		
	// start AD conversion				
	ADCSRA |= (1<<ADSC);		
	// wait until conversion is finished	
	while (ADCSRA & (1<<ADSC));	
	return ADC;
}

uint16_t GetY(void){	
	// select ADC channel 0
	ADMUX = 0x00;					
	// start AD conversion	
	ADCSRA |= (1<<ADSC);			
	// wait until conversion is finished
	while (ADCSRA & (1<<ADSC));
	return ADC;
}

uint8_t RotateLeft (uint8_t cData){
	uint8_t result;
	if ( cData & (1<<7) )
		result = (cData<<1)|(1<<0);
	else 
		result = (cData<<1);	
	return result;
}

uint8_t RotateRight (uint8_t cData){
	uint8_t result;
	if ( cData & (1<<0) )
		result = (cData>>1)|(1<<7);
	else
		result = (cData>>1);	
	return result;
}

void Calibration(void){	
	
	uint16_t temp;
	uint16_t xNeutral16, yNeutral16;
	uint16_t xMin, xMax, yMin, yMax;
	uint16_t counter = 0;
	uint16_t xFactor, yFactor;
	
	// reset firstPowerOn variable in EEPROM
	eeprom_update_byte(&firstPowerOn, 0x00);
	// store the calibration slider switch's position
	eeprom_update_byte(&calibSwitch, (PINB&(1<<PORTB2)) );	
						
	// get stick's neutral position
	xNeutral16 = GetX();		
	yNeutral16 = GetY();
				
	// reset both axes' min and max values
	xMin = xNeutral16;								
	xMax = xNeutral16;
	yMin = yNeutral16;
	yMax = yNeutral16;
		
	// do forever	
	while (1)
	{
				
		// check the x axis for new min and max values
		temp = GetX();				
		if (temp > xMax) xMax = temp;					
		if (temp < xMin) xMin = temp;
				
		// check the y axis for new min and max values 
		temp = GetY();				
		if (temp > yMax) yMax = temp;					
		if (temp < yMin) yMin = temp;	
		
		// increase counter			
		counter++;
				
		// periodically calculate and store the c factors
		if (counter>4000)
		{	
			// reset counter
			counter = 0;	
						
			// x axis (standard mode): use the difference between neutral and min or neutral and max, whatever is smaller		
			if ( (xMax - xNeutral16) < (xNeutral16 - xMin) ){
				temp = xMax - xNeutral16;
			}
			else{
				temp = xNeutral16 - xMin;		
			}
			// calculate x axis factor (standard mode)			
			xFactor = ((MIN_RANGE_STD*256)/temp);							
			// if remainder, add one	
			if ( ((MIN_RANGE_STD*256)%temp) > 0  ) xFactor++;			
			// store the c factor in EEPROM				
			eeprom_update_byte(&cx_std, (uint8_t) xFactor);	
					
			// y axis (standard mode): use the difference between neutral and min or neutral and max, whatever is smaller
			if ( (yMax - yNeutral16) < (yNeutral16 - yMin) )
				temp = yMax - yNeutral16;
			else
				temp = yNeutral16 - yMin;		
			// calculate y axis factor (standard mode)					
			yFactor = ((MIN_RANGE_STD*256)/temp);			
			// if remainder, add one		
			if ( ((MIN_RANGE_STD*256)%temp) > 0  ) yFactor++;
			// store the c factor in EEPROM	
			eeprom_update_byte(&cy_std, (uint8_t) yFactor);
			
			
			// x axis (extended range mode): use the difference between neutral and min or neutral and max, whatever is smaller
			if ( (xMax - xNeutral16) < (xNeutral16 - xMin) )
				temp = xMax - xNeutral16;
			else
				temp = xNeutral16 - xMin;
			// calculate x axis factor (extended range mode)	
			xFactor = ((MIN_RANGE_XTD*256)/temp);
			// if remainder, add one
			if ( ((MIN_RANGE_XTD*256)%temp) > 0  ) xFactor++;
			// store the c factor in EEPROM	
			eeprom_update_byte(&cx_xtd, (uint8_t) xFactor);
			
			// y axis (extended range mode): use the difference between neutral and min or neutral and max, whatever is smaller
			if ( (yMax - yNeutral16) < (yNeutral16 - yMin) )
				temp = yMax - yNeutral16;
			else
				temp = yNeutral16 - yMin;
			// calculate y axis factor (extended range mode)	
			yFactor = ((MIN_RANGE_XTD*256)/temp);
			// if remainder, add one
			if ( ((MIN_RANGE_XTD*256)%temp) > 0  ) yFactor++;
			// store the c factor in EEPROM	
			eeprom_update_byte(&cy_xtd, (uint8_t) yFactor);
		}
	}				
}

uint8_t ScaleDown(uint16_t raw16, uint8_t c){
	return  (uint8_t) ( (raw16*c) >> 8);	
}
