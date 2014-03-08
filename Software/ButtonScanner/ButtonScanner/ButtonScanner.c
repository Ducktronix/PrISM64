/*
Test Button Scanner w/SPI Code and Interrupt
Change History:  3/4/2014	Original Release	Version:  1.0

written by H. Wayne "Duck" Qualkinbush   3/4/2014   
for Ducktronix LLC     www.Ducktronix.com     email: info@ducktronix.com

Software is licensed "GNU General Public License (GPL) version 3" Read more at: https://www.gnu.org/licenses/gpl.html

Written for use on the Atmel ATMega32A.  
Porting will likely be required if this code is used on other MCU's; refer to the data sheets.


Principle of Operation:  
This Atmel AVR ATMega32A powered Button Scanner will control a traditional 8x8 matrix of buttons/switches.  Each button should have a diode wired in series
to ensure proper operation.  The micro-controller sets the 'ground' side of the button 'rows' in a pull-up INPUT configuration seeking ground.  
One 'column' at a time, the AVR changes an OUTPUT from a HIGH (normal) to a LOW to determine if any row/column button is pressed.  
If a pressed condition exists, then the AVR stores that row/col location into a buffer to be passed over SPI lupon request from the Master.  
Also, the code checks to see if the button state is different than the last time (i.e. just pressed, or just released) and, if so, 
sets an outbound interrupt line HIGH. The Master device should check for the interrupt line at a frequent rate, and if detected, should then 
send the 0xAA byte over SPI to begin the button state message transfer. When the AVR receives the 0XAA byte it resets the interrupt line LOW.  
The Master device should follow up with 0xFF bytes used to push the remaining data out of the buffer.  
The data from the buffer is 8 bytes long, sent by row, MSB first.  So, the first byte retrieved will be Row 0, then Row1...Row 7.And that is the last byte.
Each of the 8 bit positions represents a column, where a 0 represents NOT-PRESSED, and a 1 represents PRESSED buttons.

Some examples of the MSB byte flags:
0b00000000 = nothing pressed
0b10000000 = column 0 pressed
0b10000010 = columns 0 and 6 pressed

And lastly, if the third byte received is 0b00100000, then that would mean the button in the 3rd row, 3rd column, was PRESSED.  

There is also an SPI command byte that will cause the AVR to reset - 0x55.  And lastly, the button scanner takes 10 samples of each button by default.  
If the Master wants to change this, a byte can be sent over SPI between 0 and 80, which will then replace the default value until the next reset.  
This value is volatile and not stored in EEPROM, so it will be required to be set during each setup.

*Ensure SPI settings are configured properly, otherwise the operation will be questionable at best.
*I have noticed better performance and less data corruption when setting the MASTER at a speed slowed i.e. SPR0 | SPR1, while this
*device remains at SPI clock of SPR1

*/
#define F_CPU 16000000UL  // 16 MHz
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define MISO 6
#define MOSI 5
#define CS 4
#define SCK 7
#define INTRPT_OUT 3

#define SPOT1 0 //slider pot 1
#define SPOT2 1 //slider pot 2
#define RPOT1 2 //Rotary pot 1
#define RPOT2 3 //Rotary pot 2
#define ABTN1 4 //Arcade Button 1
#define ABTN2 5 //Arcade Button 2
#define ABTN3 6 //Arcade Button 3
#define ABTN4 7 //Arcade Button 4


#define Reset_AVR() wdt_enable(WDTO_30MS); while(1) {} // this macro will reset the AVR by the watchdog timer

/*********************************************************
					  Global Variables
*********************************************************/

volatile unsigned char value=0;
volatile int frame_index=0;
unsigned int buttonState[64];
unsigned int prev_buttonState[64];
int potState[4];
int prev_potState[4];
unsigned char prev_ArcadeBtnState;
int sensorBuffer[13];
unsigned char btnPressed =0;
unsigned int samples = 5; // default value for Button Matrix/ArcadeButtons.   de-bounce/scan samples to take for each scan cycle per button, 
							//can be changed by sending SPI byte 0x12-0x1E dec. (5 to 65 samples per button, per scan)
unsigned char analogSamples = 5; // this is the default number of ADC samples to take each scan for the Potentiometers

unsigned int potScanDecay = 400;  // this is the number of program cycles to stay in potentiometer fine adjust mode (i.e. +/- 1)
int potScanDelta = 2;  // this is the measured +/- delta (i.e. wake up) a pot must have to send an interrupt when NOT in fine adjust mode (i.e. the flutter to avoid)
unsigned int potFineAdjMode = 0; // this is the fine adjust FLAG
unsigned int decay; //used to track the cycles of code that go by for the decay of the fine-scan mode before it ends and must be 'awakened to reenter

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
		if (value==0x10)      // this means the Master is requesting a new 64 button status message
		{frame_index = 0; SPDR = sensorBuffer[frame_index++];PORTB &= ~(1<<INTRPT_OUT);}  // clear the interrupt line if the request is received
		if (value==0x11)      // this means the Master is pushing the data through the SPI register to receive the 64 button status message
		{SPDR = sensorBuffer[frame_index++];}
		if (value==0x1F)      // reset the AVR if commanded by the SPI Master
		{Reset_AVR();} 
		if (value > 0x11 && value < 0x1F ) // this means the Master is resetting the Sample input value from the default value, to
		{samples = (value-17)*5 ;}// (value-17)*5 = [[5-65 possible values]]
			
	sei();	//Re-enable interrupts
}

/*********************************************************
						Functions
*********************************************************/
void ioinit (void)
{
	//1 = Output, 0 = Input
	DDRB = (1<<MISO) | (1<<INTRPT_OUT);	//Set MISO and Interrupt Outbound as an output, all other SPI as inputs
	PORTB = (1<<MOSI)|(1<<CS)|(1<<SCK);	//Enable pull-ups on SPI Lines
	PORTB &= ~(1<<INTRPT_OUT); //set Interrupt Outbound output low
	//PortB Pins B0, B1, B2 are reserved
	
	DDRA =0X00;	//Set PORTA as an input 
	PORTA |= (1<<ABTN1)|(1<<ABTN2)|(1<<ABTN3)|(1<<ABTN4); //set PORTA PullUp Resistors for the Arcade Buttons
	
	DDRC = 0xFF;	//Set C0 output  THIS PORT IS FOR THE 8 ROW SCANNING (Pull LOW to enable the scan of the row)
	PORTC = 0xFF; //set C0 output high
	
	DDRD = 0x00;	//Set Port D as input THIS PORT IS FOR THE 8 COLUMN SCANNING (Read for a LOW to read the column)
	PORTD = 0xFF; //set PortD pull-ups ON
	
	
	//Setup the SPI Hardware
	SPCR = (1<<SPE) | (1<<SPIE)|(1<<SPR1)|(1<<SPR0); //Enable SPI, Enable SPI Interrupts, set clock with SPR0 and SPR1
}

unsigned char sample_portD(unsigned char col,unsigned int cycles)
{
	unsigned int total =0;
	for (unsigned int i = 0; i < cycles; i++)	
		{	total += (PIND & (1<<col));	
		}
	total /= cycles;
	
	return total;
}

unsigned char sample_ArcadeBtns(unsigned char col,unsigned int cycles)
{
	unsigned int total =0;
	for (unsigned int i = 0; i < cycles; i++)
	{	total += (PINA & (1<<col));
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
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescale for 8Mhz
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
	sensorBuffer[8] = ADC_Read (SPOT1, analogSamples) ; //test read of ADC port SPOT1
	sensorBuffer[9] = ADC_Read (SPOT2, analogSamples) ; //test read of ADC port SPOT2
	sensorBuffer[10] = ADC_Read (RPOT1, analogSamples) ; //test read of ADC port RPOT1
	sensorBuffer[11] = ADC_Read (RPOT2, analogSamples) ; //test read of ADC port RPOT2
	
	potState[0] = sensorBuffer[8];// set the SPOT1 value locally
	potState[1] = sensorBuffer[9];// set the SPOT2 value locally
	potState[2] = sensorBuffer[10];// set the RPOT1 value locally
	potState[3] = sensorBuffer[11];// set the RPOT2 value locally
}

void getArcadeButton(unsigned int ADCpin, unsigned char bit)
{
	if (sample_ArcadeBtns(ADCpin, samples) == 0) {sensorBuffer[12] |= (1<<bit);} // set the PRESSED 1 value in the bit Bit spot (ArcadeBtnx)
	else {sensorBuffer[12] &= ~(1<<bit);} // set the RELEASED 0 value in the buffer
}


/*********************************************************
						Main Code
*********************************************************/
int main (void)
{
	decay = potScanDecay; // set decay
		
	ioinit ();	//Initialize the I/O and the SPI port
	sei();	//Enable Global Interrupts

	//send the pot value at startup one time
	scanPots ();
	PORTB |= (1<<INTRPT_OUT);

	//set the prev_potState to the current for the next time around
	prev_potState[0] = potState[0];
	prev_potState[1] = potState[1];
	prev_potState[2] = potState[2];
	prev_potState[3] = potState[3];	
	
	while(1)
	{				
				
					
				//this scans the values of each button and returns it in an array
				//cycle through rows
				for (int row=0; row<8; row++)
					{   PORTC = (0xFF & ~(1<<row)); // return PORTC to all HIGH, and then set LOW the row of interest
						//scroll through each column
						for (int col=0; col<8; col++)
						{if (sample_portD(col,samples) == 0) {sensorBuffer[row] |= (1<<col); // set the PRESSED 1 value in the buffer	
							buttonState[col+(row*8)] = 1;} // set the PRESSED 1 value locally																				
					    else {sensorBuffer[row] &= ~(1<<col); // set the RELEASED 0 value in the buffer	
							buttonState[col+(row*8)] = 0;}// set the RELEASED 0 value locally
						if ((buttonState[col+(row*8)] == 1 && prev_buttonState[col+(row*8)] == 0) || (buttonState[col+(row*8)] == 0 && prev_buttonState[col+(row*8)] == 1)) 
							{PORTB |= (1<<INTRPT_OUT);} // if the button scanned condition is different than the last scan, activate the interrupt line
							prev_buttonState[col+(row*8)] = buttonState[col+(row*8)]; // set the previous state to the recent state for next time around
							}
						}
						
				
				//this scans the slider/rotary potentiometers on PORTA A0, A1, A2, A3  [Bytes 8-11] & moving averages them
				  scanPots ();
				
				//Check to see if a movement is within Fine-Scan Delta, if so, go into Fine Adjust mode
				if (potFineAdjMode==0)
					{
					//check to see if any of the four pots values have changed since the last scan, 
					//if so raise the interrupt line to signal the Master to request the state message
					if (potState[0] < (prev_potState[0]-potScanDelta)  || potState[0] > (prev_potState[0]+potScanDelta) ||
						potState[1] < (prev_potState[1]-potScanDelta)  || potState[1] > (prev_potState[1]+potScanDelta) ||
						potState[2] < (prev_potState[2]-potScanDelta)  || potState[2] > (prev_potState[2]+potScanDelta) ||
						potState[3] < (prev_potState[3]-potScanDelta)  || potState[3] > (prev_potState[3]+potScanDelta))
							{PORTB |= (1<<INTRPT_OUT); potFineAdjMode = 1; decay=potScanDecay;
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
						{PORTB |= (1<<INTRPT_OUT);						
							//set the prev_potState to the current for the next time around
							prev_potState[0] = potState[0];
							prev_potState[1] = potState[1];
							prev_potState[2] = potState[2];
							prev_potState[3] = potState[3];
						}
					decay--; if (decay == 0) {potFineAdjMode = 0;} // if decay has run to zero, then exit fine-scan mode
				}
				
								
				// get ArcadeButtonPresses and set bits if pressed
				getArcadeButton(ABTN1,0); getArcadeButton(ABTN2,1); getArcadeButton(ABTN3,2); getArcadeButton(ABTN4,3); 
		
				if (sensorBuffer[12] != prev_ArcadeBtnState)
				{PORTB |= (1<<INTRPT_OUT); // if the button scanned condition is different than the last scan, activate the interrupt line
				prev_ArcadeBtnState = sensorBuffer[12];} // set the previous state to the recent state for next time around		
	}
	
	return (0);

}


