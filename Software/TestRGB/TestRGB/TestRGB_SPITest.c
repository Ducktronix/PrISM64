/*
Test SPI Code
*/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>


/*********************************************************
					  Global Variables
*********************************************************/
volatile char myBuffer[512];		//Temporary Buffer that holds the incoming data from the SPI Bus. This data is parsed into the Red,Green and Blue Frame Buffers
volatile unsigned int frame_index = 0;	//Keeps track of the current index
volatile uint8_t new_frame;			//Flag telling the main firmware that enough data has been received and a new image can be displayed onto the matrix


volatile uint8_t NUM_BOARDS=0, RUN_COUNT=0;	
//volatile char command_mode=0;
volatile char value=0;


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
	value=SPDR;	//Get the data from the SPI bus and put it into the temporary frame buffer
		myBuffer[frame_index++] = value;	//Add value to buffer and increment the index.
		//SPDR = value;	//Send the value to the next board in the system.
	sei();	//Re-enable interrupts
}

/*********************************************************
						Main Code
*********************************************************/
int main (void)
{
		
	ioinit ();	//Initialize the I/O and the SPI port
	sei();	//Enable Global Interrupts
	while(1)
	{
        if (PINB & (1<<0)) //If CS goes high, SPI com is complete, SS is PB0
		{
			new_frame=1;
		}
		//New frame is set when enough bytes have been received for the whole system
		if(new_frame==1){
			new_frame=0;	//Reset the frame flag.
			
			//do work with mybuffer here
				
				
		}

		//do more here
		
		
	}
	
	return (0);

}

/*********************************************************
						Functions
*********************************************************/
void ioinit (void)
{
	//1 = Output, 0 = Input
	DDRB = (1<<3);	//Set MISO as an output, all other SPI as inputs
	PORTB = (1<<2)|(1<<0)|(1<<1);	//Enable pull-ups on SPI Lines
	
	DDRA = (1<<0);	//Set A0 output 
	PORTA = 0x00; //set all outputs low
	
	//Setup the SPI Hardware
	SPCR = (1<<SPE) | (1<<SPIE)|(1<<SPR1)|(1<<SPR0); //Enable SPI, Enable SPI Interrupts
}









void delay_us(uint8_t x)
{
	TIFR0 = (1<<TOV0); //Clear any interrupt flags on Timer2
	TCNT0 = 256 - x; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click
	while( (TIFR0 & (1<<TOV0)) == 0);

	//Double the delay because we are using 16MHz and 8 prescalar

	TIFR0 = (1<<TOV0); //Clear any interrupt flags on Timer2
	TCNT0 = 256 - x; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click
	while( (TIFR0 & (1<<TOV0)) == 0);
}

void delay_ms(uint16_t x)
{
	for ( ; x > 0 ; x--)
	{
		delay_us(250);
		delay_us(250);
		delay_us(250);
		delay_us(250);
	}
}

void write_to_EEPROM(unsigned int Address, unsigned char Data)
{
    //Interrupts are globally disabled!
	
	while(EECR & (1<<EEPE)); //Wait for last Write to complete
	//May need to wait for Flash to complete also!
	EEAR = Address;			//Assign the Address Register with "Address"
	EEDR=Data;				//Put "Data" in the Data Register
	EECR |= (1<<EEMPE); 	//Write to Master Write Enable
	EECR |= (1<<EEPE);  	//Start Write by setting EE Write Enable
	
}

unsigned char read_from_EEPROM(unsigned int Address)
{
	//Interrupts are globally disabled!
	
	while(EECR & (1<<EEPE));	//Wait for last Write to complete
	EEAR = Address;				//Assign the Address Register with "Address"
	EECR |= (1<<EERE); 			//Start Read by writing to EER
	return EEDR;				//EEPROM Data is returned
}


