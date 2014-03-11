/*
 * LCDDisplay_Scnr.c
 *
 * Created: 3/6/2014 1:00:08 PM
 *  Author: HWQ
 */ 
//Definitions for MCU Clock Frequency
#define 	F_CPU   16000000

#define bit_is_set(sfr,bit) \
(_SFR_BYTE(sfr) & _BV(bit))
#define bit_is_clear(sfr,bit) \
(!(_SFR_BYTE(sfr) & _BV(bit)))


#define MISO 6			//PORTB
#define MOSI 5			//PORTB
#define CS 4			//PORTB
#define SCK 7			//PORTB
#define INTRPT_OUT 2	//PORTB Pin2 is the Interrupt Line to Master
#define PWM_OUT 3       //PORTB Pin3 is the LCD Back-light PWM ground
//digital inputs from rotary encoder 3+4 buttons
#define RENC3BTN 0      //RotaryEncoder3 Button  PORTB
#define RENC4BTN 1      //RotaryEncoder4 Button  PORTB

//PORTC is used for the LCD, do not assign elsewhere

//analog inputs from potentiometers of joysticks
#define JOY1X 0 //Joystick 1 X pot 0  PORTA
#define JOY1Y 1 //Joystick 1 Y pot 1  PORTA
#define JOY2X 2 //Joystick 2 X pot 2  PORTA
#define JOY2Y 3 //Joystick 2 Y pot 3  PORTA
//digital inputs from joystick 1+2 buttons and rotary encoder 1+2 buttons
#define JOY1BTN 4 //Joystick1 Button  PORTA
#define JOY2BTN 5 //Joystick2 Button  PORTA
#define RENC1BTN 6 //RotaryEncoder1 Button  PORTA
#define RENC2BTN 7 //RotaryEncoder2 Button  PORTA


#define RENCPORT PORTD // the port the encoders are on
#define RENC1A 0 //RotaryEncoder 1 A  PORTD
#define RENC1B 1 //RotaryEncoder 1 B  PORTD
#define RENC2A 2 //RotaryEncoder 2 A  PORTD
#define RENC2B 3 //RotaryEncoder 2 B  PORTD
#define RENC3A 4 //RotaryEncoder 3 A  PORTD
#define RENC3B 5 //RotaryEncoder 3 B  PORTD
#define RENC4A 6 //RotaryEncoder 4 A  PORTD
#define RENC4B 7 //RotaryEncoder 4 B  PORTD

//SPI COMMANDS for LCD and Joystick/Encoder Controller
#define RSTAVR 0x1F //reset the AVR, soft reset.
#define REQSTAT 0x10 //request the sensor status, acknowledge the interrupt signal
#define PSHSTA 0x11 //push the sensor status message through the buffer
//#define SMPLLO 0x12 //
//#define SMPLHI 0x1E //
#define ROTENC1ZERO 0x12
#define ROTENC2ZERO 0x13
#define ROTENC3ZERO 0x14
#define ROTENC4ZERO 0x15
#define LCDCMDLO 0x80 //LCD Command Lowest Allowable Value
#define LCDCMDHI 0x9F //LCD Command Highest Allowable Value
#define LCDCHAR0LO 0x00 //LCD Character Block0 Lowest Allowable Value, Custom character area
#define LCDCHAR0HI 0x0F  //LCD Character Block0 Highest Allowable Value, Custom character area
#define LCDCHAR1LO 0x20 //LCD Character Block1 Lowest Allowable Value, English character area
#define LCDCHAR1HI 0x7F  //LCD Character Block1 Highest Allowable Value, English character area
#define LCDCHAR2LO 0xA0 //LCD Character Block2 Lowest Allowable Value, Kanji/Greek character area
#define LCDCHAR2HI 0xFF //LCD Character Block2 Highest Allowable Value, Kanji/Greek character area

#define Reset_AVR() wdt_enable(WDTO_30MS); while(1) {} // this macro will reset the AVR by the watchdog timer

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "lcd.h"
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <string.h>


/*********************************************************
					  Global Variables
*********************************************************/

volatile unsigned char value=0;
volatile int frame_index=0;
//unsigned int buttonState[64];
//unsigned int prev_buttonState[64];
int potState[4];
int prev_potState[4];
int encState[4];
int prev_encState[4];
unsigned int prev_BtnState;
volatile int sensorBuffer[9];
unsigned char btnPressed =0;
unsigned int samples = 5; // default value for Buttons.  de-bounce/scan samples to take for each scan cycle per button, 
							//can be changed by sending SPI byte 0x12-0x1E (5 to 65 samples per button, per scan)
unsigned char analogSamples = 5; // this is the default number of ADC samples to take each scan for the Potentiometers

unsigned int potScanDecay = 300;  // this is the number of program cycles to stay in potentiometer fine adjust mode (i.e. +/- 1)
int potScanDelta = 2;  // this is the measured +/- delta (i.e. wake up) a pot must have to send an interrupt when NOT in fine adjust mode (i.e. the flutter to avoid)
unsigned int potFineAdjMode = 0; // this is the fine adjust FLAG
unsigned int decay; //used to track the cycles of code that go by for the decay of the fine-scan mode before it ends and must be 'awakened to reenter

volatile char LCDBuffer[80]; //characters for display buffer 
volatile int LCDBuffer_index=0; //Index to track character placement
char LCDDisplay[80]; //characters for display  
int LCDDisplay_index=0; //Index to track character placement
unsigned char LCDCharacter; //these are the characters pulled from the buffer
char LCDCommandMode =0; // LCD CommandMode Flag

signed char LCD_Cursor=0; // tracks LCD cursor location
unsigned char LCD_Row=0;
unsigned char LCD_Col=0;


//====rotary encoder variables
int RotaryEncCounter[4] = {128,128,128,128};

/*********************************************************
			Interrupt Service Routines
*********************************************************/
/*
Usage:		Automatic as long as Global Interrupts are enabled (sei())
Purpose:	Initialize I/O, Setup SPI Hardware
Parameters:	None
Return:		None
*/
ISR (SPI_STC_vect)
{   
	cli();	//Halt Interrupts
	value=SPDR;	//Get the data from the SPI bus
		
		//Sensors
		//Check for Sensor Commands
		
		if (value==REQSTAT)      // this means the Master is requesting a new sensor status message, acknowledging the interrupt
		{frame_index = 0; SPDR = sensorBuffer[frame_index++];PORTB |= (1<<INTRPT_OUT); }  // clear the interrupt line HIGH if the request is received
		if (value==PSHSTA)      // this means the Master is pushing the data through the SPI register to receive the sensor status message
		{SPDR = sensorBuffer[frame_index++];}
		if (value==RSTAVR)      // reset the AVR if commanded by the SPI Master
		{Reset_AVR();}
		//if ((value >= SMPLLO) && (value <= SMPLHI)) // this means the Master is resetting the Sample input value from the default value, to
		//{samples = (value-17)*5 ;}// (value - 17)  x 5, = [[[ 5 - 65 samples possible]]]
		if (value == ROTENC1ZERO) // this indicates a command to zero the rotary encoder counter
		{RotaryEncCounter[0] =128;}
			if (value == ROTENC2ZERO) // this indicates a command to zero the rotary encoder counter
			{RotaryEncCounter[1] =128;}
				if (value == ROTENC3ZERO) // this indicates a command to zero the rotary encoder counter
				{RotaryEncCounter[2] =128;}
					if (value == ROTENC4ZERO) // this indicates a command to zero the rotary encoder counter
					{RotaryEncCounter[3] =128;}
		
		//LCD
		//Anything within these limits must be allowable LCD commands or text characters to the LCD so handle that way
		if (((value >= LCDCMDLO) && (value <= LCDCMDHI))|| ((value >= LCDCHAR1LO) && (value <= LCDCHAR1HI)) || 
		((value >= LCDCHAR0LO) && (value <= LCDCHAR0HI)) || ((value >= LCDCHAR2LO) && (value <= LCDCHAR2HI))) // if the value is any character allowed
		{LCDBuffer[LCDBuffer_index] = value; 
			LCDBuffer_index++; 
			if (LCDBuffer_index >= 80) {LCDBuffer_index=0;} // reset the index if it reaches past the end of the max size and wrap around data
			} // add the character to the LCDBuffer, and increment index to next position.  
	
			
	sei();	//Re-enable interrupts
}



/*********************************************************
						Functions
*********************************************************/
void ioinit (void)
{
	//1 = Output, 0 = Input
	DDRA =0X00;	//Set PORTA as an input
	PORTA |= (1<<JOY1BTN)|(1<<JOY2BTN)|(1<<RENC1BTN)|(1<<RENC2BTN); //set PORTA PullUp Resistors for Joystick 1+2,
																	// Rotary Encoder 1+2 Buttons
		
	DDRB = (1<<MISO) | (1<<INTRPT_OUT)| (1<<PWM_OUT);	//Set MISO and Interrupt Outbound as an output, 
														//OC0 Timer0 PWM as an output for the LED Back-light, 
														//all other SPI as inputs
	PORTB = (1<<MOSI)|(1<<CS)|(1<<SCK)|(1<<RENC3BTN)|(1<<RENC4BTN);	//Enable pull-ups on SPI Lines & Button Inputs
	PORTB |= (1<<INTRPT_OUT); //set Interrupt Outbound output HIGH
		
	
	DDRC = 0xFF;	//Set all of PORTC as output for the LCD (C7 unused, reserved)
	PORTC = 0x00; //set PORTC outputs LOW
	
	DDRD = 0x00;	//Set all of Port D as inputs for the 4 rotary encoders (A + B pin for each)
	PORTD = 0xFF;   //set PortD pull-ups ON for the 4 rotary encoders
	
	
	//Setup the SPI Hardware
	SPCR = (1<<SPE) | (1<<SPIE)|(1<<SPR1)|(1<<SPR0); //Enable SPI, Enable SPI Interrupts, set clock with SPR0 and SPR1
	
	TCCR0 |= _BV(WGM00) | _BV(WGM01) | _BV(CS00)| _BV(COM01);
	//fast PWM Mode for Timer0, Pre-scaler divider 1, Clear OC0 on compare match
	
}

unsigned char sample_BtnsPortA(unsigned char col,unsigned int cycles)
{
	unsigned int total =0;
	for (unsigned int i = 0; i < cycles; i++)
	{	total += (PINA & (1<<col));
	}
	total /= cycles;
	
	return total;
}
unsigned char sample_BtnsPortB(unsigned char col,unsigned int cycles)
{
	unsigned int total =0;
	for (unsigned int i = 0; i < cycles; i++)
	{	total += (PINB & (1<<col));
	}
	total /= cycles;
	
	return total;
}


//==========ADC Read function======
int ADC_Read(uint8_t adctouse, unsigned char samples)
{
	int ADCval =0;
	
	ADMUX = adctouse;         // use "adctouse" ADC
	ADMUX |= (1 << REFS0);    // use AVcc as the reference
	//ADMUX &= ~(1 << ADLAR);   // clear for 10 bit resolution
	ADMUX |= (1 << ADLAR);   // set for 8 bit resolution
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 pre scale for 8Mhz
	ADCSRA |= (1 << ADEN);    // Enable the ADC

	for (char i=0; i<samples; i++)
	  {ADCSRA |= (1 << ADSC);    // Start the ADC conversion

	  while(ADCSRA & (1 << ADSC));      // this line waits for the ADC to finish

	  //ADCval = ADCL;  //this is for 10bit operation
	  //ADCval = (ADCH << 8) + ADCval;      //this is for 10bit operation  // ADCH is read so ADC can be updated again
      ADCval = ADCval + ADCH; // this uses the top 8 bits ADCH, when using 8 bit mode
	  }
	ADCval = ADCval/samples;  // this averages and  
	return ADCval;
}

void scanPots (void)
{
	sensorBuffer[0] = ADC_Read (JOY1X, analogSamples) ; //test read of ADC port JOY1X
	sensorBuffer[1] = ADC_Read (JOY1Y, analogSamples) ; //test read of ADC port JOY1Y
	sensorBuffer[2] = ADC_Read (JOY2X, analogSamples) ; //test read of ADC port JOY2X
	sensorBuffer[3] = ADC_Read (JOY2Y, analogSamples) ; //test read of ADC port JOY2Y
	potState[0] = sensorBuffer[0];// set the JOY1X value locally
	potState[1] = sensorBuffer[1];// set the JOY1Y value locally
	potState[2] = sensorBuffer[2];// set the JOY2X value locally
	potState[3] = sensorBuffer[3];// set the JOY2Y value locally	
}


void scanEncoders (void)
{

static int oldA[4];
static int oldB[4];

int newA[4]; 
int newB[4];
if (PIND & _BV(RENC1A)) {newA[0]=1;} else {newA[0]=0;}  //rotary encoder 1A
if (PIND & _BV(RENC1B)) {newB[0]=1;} else {newB[0]=0;}  //rotary encoder 1B
if (PIND & _BV(RENC2A)) {newA[1]=1;} else {newA[1]=0;}  //rotary encoder 1A
if (PIND & _BV(RENC2B)) {newB[1]=1;} else {newB[1]=0;}  //rotary encoder 1B
if (PIND & _BV(RENC3A)) {newA[2]=1;} else {newA[2]=0;}  //rotary encoder 1A
if (PIND & _BV(RENC3B)) {newB[2]=1;} else {newB[2]=0;}  //rotary encoder 1B	
if (PIND & _BV(RENC4A)) {newA[3]=1;} else {newA[3]=0;}  //rotary encoder 1A
if (PIND & _BV(RENC4B)) {newB[3]=1;} else {newB[3]=0;}  //rotary encoder 1B

for (int i=0; i<4; i++)
{
if ((newA[i] == newB[i]) && (oldA[i] != newA[i]) && (oldB[i] == newB[i]))	//
	{RotaryEncCounter[i]--;} // direction is left, minus one from counter
	else if ((newA[i] == newB[i]) && (oldB[i] != newA[i]) && (oldA[i] == newA[i]))	
	{RotaryEncCounter[i]++;} //direction is right, add one to counter
	else if ((newA[i] != newB[i]) && (oldA[i] != newA[i]) && (oldB[i] == newB[i]))	//
	{RotaryEncCounter[i]++;} // direction is left, minus one from counter
	else if ((newA[i] != newB[i]) && (oldB[i] != newB[i]) && (oldA[i] == newA[i]))
    {RotaryEncCounter[i]--;}	//direction is right, add one to counter	

if (RotaryEncCounter[i] > 255) {RotaryEncCounter[i] = 255;}
else if (RotaryEncCounter[i] < 1) {RotaryEncCounter[i] = 1;}
				
oldA[i] = newA[i];
oldB[i] = newB[i];
}

 sensorBuffer[4] = RotaryEncCounter[0]; //encoder1
 sensorBuffer[5] = RotaryEncCounter[1]; //encoder2
 sensorBuffer[6] = RotaryEncCounter[2]; //encoder3
 sensorBuffer[7] = RotaryEncCounter[3]; //encoder4
 encState[0] = sensorBuffer[4];// set the JOY1X value locally
 encState[1] = sensorBuffer[5];// set the JOY1Y value locally
 encState[2] = sensorBuffer[6];// set the JOY2X value locally
 encState[3] = sensorBuffer[7];// set the JOY2Y value locally
}



void getButtonsPortA(unsigned int ADCpin, unsigned char bit)
{
	if (sample_BtnsPortA(ADCpin, samples) == 0) {sensorBuffer[8] |= (1<<bit);} // set the PRESSED 1 value in the bit Bit spot (Btn x)
	else {sensorBuffer[8] &= ~(1<<bit);} // set the RELEASED 0 value in the buffer
}
void getButtonsPortB(unsigned int ADCpin, unsigned char bit)
{
	if (sample_BtnsPortB(ADCpin, samples) == 0) {sensorBuffer[8] |= (1<<bit);} // set the PRESSED 1 value in the bit Bit spot (Btn x)
	else {sensorBuffer[8] &= ~(1<<bit);} // set the RELEASED 0 value in the buffer
}

//==============LCD Functions ==================
void setLCDBrightness(unsigned char brightness)
{
	//SetBrightness on the LED Back-light of the LCD
	OCR0 = 255-brightness;	//inverted value to make sense, where 0 = off and 255 = full bright
}

void updateLCDCursor()
{//this function recalculates the cursor for the display
if (LCD_Cursor==-1) {LCD_Cursor=31;} // move the cursor pointer to Row2 location
if (LCD_Cursor==32) {LCD_Cursor=0;}	// move the cursor pointer to Row1 location	
if (LCD_Cursor>=0 && LCD_Cursor<16) {LCD_Row=0; LCD_Col=LCD_Cursor;} // set the row and column
if (LCD_Cursor>15 && LCD_Cursor<33) {LCD_Row=1; LCD_Col=LCD_Cursor-16; }  // set the row and column	
	
}

//===============INT (MAIN) =======================
int main(void)
{
    	//SetLCDBrightnessto OFF to start with
    	setLCDBrightness(0);

    	lcd_init (LCD_DISP_ON_CURSOR_BLINK);
		
		
	decay = potScanDecay; // set decay
	
	ioinit ();	//Initialize the I/O and the SPI port
	sei();	//Enable Global Interrupts
	
	//send the potentiometer values at startup one time
	scanPots ();
		//set the prev_potState to the current for the next time around
		prev_potState[0] = potState[0];
		prev_potState[1] = potState[1];
		prev_potState[2] = potState[2];
		prev_potState[3] = potState[3];
		
	//scan the encoders 
	scanEncoders ();
		//set the prev_potState to the current for the next time around
		prev_encState[0] = encState[0];
		prev_encState[1] = encState[1];
		prev_encState[2] = encState[2];
		prev_encState[3] = encState[3];
		
  				
	while(1)
    {
        
		//UPDATE THE LCD DISPLAY BUFFER TO MATCH THE LCD INCOMING BUFFER===================
		if (LCDDisplay_index != LCDBuffer_index)
		{LCDDisplay[LCDDisplay_index] = LCDBuffer[LCDDisplay_index]; 
		 LCDCharacter = LCDBuffer[LCDDisplay_index];
			
		  
				//Since something new has arrived, lets take a look and see what to do
				
				//===========LCD COMMANDS CHECKED HERE=============
				if (LCDCommandMode)
				{
					//Set the LCD Brightness
					if ((LCDCharacter >= 170) && (LCDCharacter <= 195 ))
					{setLCDBrightness((LCDCharacter - 170)*10 ); LCDCharacter =0;}
					
					//Clear the LCD Screen
					if (LCDCharacter == 0x01 )
					{lcd_clrscr ();}
					
					//Move Cursor Right One
					if (LCDCharacter == 0x02 )
					{LCD_Cursor++; updateLCDCursor(); lcd_gotoxy (LCD_Col, LCD_Row);}
						
					//Move Cursor Left One
					if (LCDCharacter == 0x03 )
					{LCD_Cursor--; updateLCDCursor(); lcd_gotoxy (LCD_Col, LCD_Row);}
						
					//ScrollRight
					if (LCDCharacter == 0x0B )
					{lcd_command(0x1C);}
					//	
					//ScrollLeft
					if (LCDCharacter == 0x0A )
					{lcd_command(0x18);}
					//}
						
					//Turn Display ON, clean memory
					if (LCDCharacter == 0x0C )
					lcd_init (LCD_DISP_ON);
									
					//Turn Display OFF, clear memory
					if (LCDCharacter == 0x08 )
					lcd_init (LCD_DISP_OFF);
					
					//Turn Cursor ON
					if (LCDCharacter == 0x0D )
					{lcd_command(0x0E);}
					
					//Turn Box Cursor Blink ON
					if (LCDCharacter == 0x0E )
					{lcd_command(0x0F);}
						
					
					//Set Cursor Position
					if (LCDCharacter >= 128 && LCDCharacter <= 208)  // 128dec / 0x80+ cursor location
					{LCD_Cursor = LCDCharacter-128; updateLCDCursor(); lcd_gotoxy (LCD_Col, LCD_Row);}
					
				}
				
				//as long as we're not in command mode, check to see if it is a character to be displayed, if so print it with wraparound
				if (!LCDCommandMode && LCDCharacter!=0xFE)
				{
				
				if (((LCDCharacter >= LCDCHAR1LO) && (LCDCharacter <= LCDCHAR1HI)) || 
				((LCDCharacter >= LCDCHAR0LO) && (LCDCharacter <= LCDCHAR0HI)) || 
				((LCDCharacter >= LCDCHAR2LO) && (LCDCharacter <= LCDCHAR2HI)))  //// any other non-printed special characters to add ????????
						{
						lcd_putc(LCDCharacter); 
						LCD_Cursor++; // move the cursor pointer right after printing a character
						updateLCDCursor(); // update the cursor tracking variables
						}
				}
				
		        //=======>>>>>> special characters like Backspace, Carriage Return, and others??? should handle them special here
		
		
		//turn command mode off after we checked for characters and commands
		LCDCommandMode=0;
			
		// add one to the index and next time around see if we're still not synced with buffer and repeat
		LCDDisplay_index++; 
		if (LCDDisplay_index >= 80) {LCDDisplay_index=0;} //wrap around if we are at the end
		
		
		//set COMMAND MODE if Command Character received for the next look
		if (LCDCharacter==0xFE) {LCDCommandMode=1;} // set Commmand mode and do nothing else, move on to the next character
		}
		
		
		
		//if (updateLCD==1)
		//{lcd_gotoxy(0,0); lcd_putc(LCDCharacter); updateLCD=0; setLCDBrightness(0);}
        // LCDBuffer

		//END OF LCD CONTROL======================
		
			
		//this scans the Joystick potentiometers on PORTA A0, A1, A2, A3  [Bytes 0-3] & moving averages them
		scanPots ();
		PORTB &= ~(1<<INTRPT_OUT); //drive interrupt line to signal Master to request initial pot state
		
		//Check to see if a movement is within Fine-Scan Delta, if so, go into Fine Adjust mode
		if (potFineAdjMode==0)
		{
			//check to see if any of the four pots values have changed since the last scan,
			//if so raise the interrupt line to signal the Master to request the state message
			if (potState[0] < (prev_potState[0]-potScanDelta)  || potState[0] > (prev_potState[0]+potScanDelta) ||
				potState[1] < (prev_potState[1]-potScanDelta)  || potState[1] > (prev_potState[1]+potScanDelta) ||
				potState[2] < (prev_potState[2]-potScanDelta)  || potState[2] > (prev_potState[2]+potScanDelta) ||
				potState[3] < (prev_potState[3]-potScanDelta)  || potState[3] > (prev_potState[3]+potScanDelta))
				{PORTB &= ~(1<<INTRPT_OUT); potFineAdjMode = 1; decay=potScanDecay;
				//set the prev_potState to the current for the next time around
				prev_potState[0] = potState[0];
				prev_potState[1] = potState[1];
				prev_potState[2] = potState[2];
				prev_potState[3] = potState[3];
			}
		}
		
		
		//Check to see if a movement is made and handle while in Fine Adjust mode
		if (potFineAdjMode==1)
		{
			//check to see if any of the four pots values have changed since the last scan,
			//if so raise the interrupt line to signal the Master to request the state message
			if (potState[0] != prev_potState[0] || potState[1] != prev_potState[1] ||
				potState[2] != prev_potState[2] || potState[3] != prev_potState[3])
				{PORTB &= ~(1<<INTRPT_OUT); 
				//set the prev_potState to the current for the next time around
				prev_potState[0] = potState[0];
				prev_potState[1] = potState[1];
				prev_potState[2] = potState[2];
				prev_potState[3] = potState[3];
			}
			decay--; if (decay == 0) {potFineAdjMode = 0;} // if decay has run to zero, then exit fine-scan mode
		}
		
			//scan the encoders
			scanEncoders ();
			
			if (encState[0] != prev_encState[0] || encState[1] != prev_encState[1] || encState[2] != prev_encState[2] || encState[3] != prev_encState[3])
			{PORTB &= ~(1<<INTRPT_OUT); 
			//set the prev_potState to the current for the next time around
			prev_encState[0] = encState[0];
			prev_encState[1] = encState[1];
			prev_encState[2] = encState[2];
			prev_encState[3] = encState[3];
			}
		
		// get Button state and set bits if pressed
		getButtonsPortA(JOY1BTN,0); getButtonsPortA(JOY2BTN,1); getButtonsPortA(RENC1BTN,2); getButtonsPortA(RENC2BTN,3);
		getButtonsPortB(RENC3BTN,4); getButtonsPortB(RENC4BTN,5); 
		
		if (sensorBuffer[8] != prev_BtnState)
		{PORTB &= ~(1<<INTRPT_OUT);  // if the button scanned condition is different than the last scan, activate the interrupt line LOW
		prev_BtnState = sensorBuffer[8];} // set the previous state to the recent state for next time around
		
		

	}
	return (0);
}