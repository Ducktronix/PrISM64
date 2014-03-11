// SPI Master test program

#include <RGBMatrixSimple.h>
#include <buttonMatrix.h>

//Define the "Normal" 8 bit colors
#define BLACK	0
#define RED     0xE0
#define GREEN	0x1C
#define BLUE	0x03
#define YELLOW	RED|GREEN
#define MAGENTA	RED|BLUE
#define TEAL	BLUE|GREEN
#define WHITE	(RED|GREEN|BLUE)-0x60

//Define the SPI Pin Numbers
#define DATAOUT 51//MOSI
#define DATAIN  50//MISO 
#define SPICLOCK  52//sck
#define SS  53//ss
#define BTN_MATRIX_CS 49 //ss
#define MATRIX_CS 48 //ss
#define JOYENCDR_CS 47 //ss
#define BTNSCAN_INTRPT 20 //Button Scanner Interrupt
#define JOYENCDR_INTRPT 21 //Button Scanner Interrupt

const int BtnScannerRequestByte = 0x10;
const int BtnScannerPushByte = 0x11;
const int BtnScannerReset = 0x1F;


const byte potColor[] = {BLACK, WHITE, MAGENTA, BLUE, TEAL, GREEN, YELLOW, RED};

//int GLOBAL incoming;
int inBtnMatrixBuffer[13]; // allows function to pass array of buffer for use everywhere
int inJoyEncoderBuffer[9]; // allows function to pass array of buffer for use everywhere

byte color = 0x00;
byte MatrixFillColor = BLACK;
byte LCDBrightness = 0;
byte OldMatrixFillColor = BLACK;
byte OldLCDBrightness = 0;

byte ISR_BtnScannerFlag = 0;
byte ISR_JoyEncoderFlag = 0;

//Define the variables we'll need later in the program
RGBMatrixSimple Matrix (MATRIX_CS); 

//encoder counter
int counter[4] = {0,0,0,0};
int oldCounter[4] = {99,99,99,99}; //a non-0 value value forces it to update at startup
int tik;
int refreshLCDtiks = 8000;

//=====SPI FUNCTIONS=======
unsigned char spi_transfer(unsigned char data)
{
               SPDR = data;                    // Start the transmission
  		while (!(SPSR & (1<<SPIF)))     // Wait for the end of the transmission
  		{
  		};
  		return SPDR; 
}

unsigned char spi_send(int SSpin, unsigned char data)
{
  digitalWrite(SSpin, LOW); //enable / SS LOW of desired SPI slave device
  int incoming = spi_transfer(data); //transfer the byte
  digitalWrite(SSpin, HIGH); //disable / SS HIGH of desired SPI slave device
}

void setup() 
{  
  
  //Set the pin modes for the SPI bus
  pinMode(BTNSCAN_INTRPT, INPUT);
  pinMode(JOYENCDR_INTRPT, INPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK,OUTPUT);
  pinMode(SS,OUTPUT); // not using it, but required for SPI to function
  pinMode(BTN_MATRIX_CS,OUTPUT);
  pinMode(MATRIX_CS,OUTPUT);
  pinMode(JOYENCDR_CS,OUTPUT);
  digitalWrite(BTN_MATRIX_CS,HIGH);
  digitalWrite(MATRIX_CS,HIGH);
  digitalWrite(JOYENCDR_CS,HIGH);
  digitalWrite(BTNSCAN_INTRPT,HIGH); // turn on pullups
  digitalWrite(JOYENCDR_INTRPT,HIGH); //turn on pullups
  digitalWrite(SS,HIGH);
  
  //SPI Bus setup !!!!! This has to come after pinmode and HIGH on SS/CS pins are accomplished!!!!
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);	//Enable SPI HW, Master Mode, set clock divider SPR1 & SPR0 
 
  Serial.begin(115200);

delay (2500); // wait for everything to boot up

    for (byte i=0; i<4; i++)
    {  Matrix.fill(0xE0);
    delay (50); Matrix.blank(); delay (50);
    }
    Serial.println("Welcome!!!");

BtnMatrixReset(); // reset the devices at startup
JoyEncoderReset(); // reset the devices at startup


//get an initial sample of the sub-systems
ISR_BtnScanIntrpt();
ISR_JoyEncoderIntrpt();

//set the pixel color with the potentiometer reading
color = potColor[inJoyEncoderBuffer [0]/32];


//finally, last step is to activate the Interrupt lines
attachInterrupt(3, ISR_BtnScanIntrpt, FALLING);
attachInterrupt(2, ISR_JoyEncoderIntrpt, FALLING);

//LCDcursorOff (); // set up the LCD without the cursor showing
LCDcursorBlinkOn ();

spi_send(JOYENCDR_CS, 0x12); //reset the encoder counters 
spi_send(JOYENCDR_CS, 0x13); //reset the encoder counters 
spi_send(JOYENCDR_CS, 0x14); //reset the encoder counters 
spi_send(JOYENCDR_CS, 0x15); //reset the encoder counters 
} 

 //===============================ISR_BtnScanIntrpt()=================================================
void ISR_BtnScanIntrpt()
{

 //request the button state to get prepared
 digitalWrite(BTN_MATRIX_CS, LOW); //enable / SS LOW of desired SPI slave device
 SPDR = BtnScannerRequestByte;                    // Start the transmission
  		while (!(SPSR & (1<<SPIF)))     // Wait for the end of the transmission
  		{
  		};
 int incoming = SPDR; 
 digitalWrite(BTN_MATRIX_CS, HIGH); //disable / SS HIGH of desired SPI slave device
 
 digitalWrite(BTN_MATRIX_CS, LOW); 
  //ask for the 64 button state values and buffer
  for (int i = 0; i<13; i++)
   {  SPDR = BtnScannerPushByte;                    // Start the transmission
  		while (!(SPSR & (1<<SPIF)))     // Wait for the end of the transmission
  		{
  		};
   //Serial.println(inBtnMatrixBuffer[i]);
   inBtnMatrixBuffer[i] = SPDR; 
   }
   //Serial.println("======BTN_MATRIX_CS======");
  digitalWrite(BTN_MATRIX_CS, HIGH);
  
//======check Arcade Buttons=======
if (inBtnMatrixBuffer [12] &(1<<0)) // check button 1, bit 0
{MatrixFillColor = RED;}
if (inBtnMatrixBuffer [12] &(1<<1)) // check button 1, bit 1
{MatrixFillColor = GREEN;}
if (inBtnMatrixBuffer [12] &(1<<2)) // check button 1, bit 2
{MatrixFillColor = BLUE;}
if (inBtnMatrixBuffer [12] &(1<<3)) // check button 1, bit 3
{MatrixFillColor = BLACK;}

//Set the ISR BtnScanner FLAG for processing in MAIN
ISR_BtnScannerFlag = 1;
}

//===========================ISR_JoyEncoderIntrpt()===================================================
void ISR_JoyEncoderIntrpt()
{
 //request the button state to get prepared
 digitalWrite(JOYENCDR_CS, LOW); //enable / SS LOW of desired SPI slave device
 SPDR = BtnScannerRequestByte;                    // Start the transmission
  		while (!(SPSR & (1<<SPIF)))     // Wait for the end of the transmission
  		{
  		};
 int incoming = SPDR; 
 digitalWrite(JOYENCDR_CS, HIGH); //disable / SS HIGH of desired SPI slave device
 
 digitalWrite(JOYENCDR_CS, LOW); 
  //ask for the joystick pot, encoder, and button state values and buffer
  for (int i = 0; i<9; i++)
   {  SPDR = BtnScannerPushByte;                    // Start the transmission
  		while (!(SPSR & (1<<SPIF)))     // Wait for the end of the transmission
  		{
  		};
   inJoyEncoderBuffer[i] = SPDR; 
   }
   digitalWrite(JOYENCDR_CS, HIGH);

//check Joystick and Encoder Buttons and do the quick actions
if (inJoyEncoderBuffer [8] &(1<<0)) // check Joystick1 button, bit 0
{MatrixFillColor = MAGENTA; }  // send the command

if (inJoyEncoderBuffer [8] &(1<<1)) // check Joystick2 button, bit 1
{MatrixFillColor = YELLOW; }

if (inJoyEncoderBuffer [8] &(1<<2)) // check Encoder1 button, bit 2
{spi_send(JOYENCDR_CS, 0x12); MatrixFillColor = BLUE;} //reset the encoder counter to 128 (middle position)

if (inJoyEncoderBuffer [8] &(1<<3)) // check Encoder2 button, bit 3
{spi_send(JOYENCDR_CS, 0x13); MatrixFillColor = GREEN;} //reset the encoder counter to 128 (middle position)

if (inJoyEncoderBuffer [8] &(1<<4)) // check Encoder3 button, bit 4
{spi_send(JOYENCDR_CS, 0x14); MatrixFillColor = WHITE;} //reset the encoder counter to 128 (middle position)

if (inJoyEncoderBuffer [8] &(1<<5)) // check Encoder4 button, bit 5
{spi_send(JOYENCDR_CS, 0x15); MatrixFillColor = BLACK;} //reset the encoder counter to 128 (middle position)
 
//check Slider pot1 and change color
color = potColor[inJoyEncoderBuffer [0]/32];

//check Rotary pot1 and use to change backlighting
LCDBrightness = inJoyEncoderBuffer [2]/10; //255 divided by 10 gets us between 0-25 as required by the controller

//update rotarty encoder counters
for (int i=0; i<4; i++)
//{if (inJoyEncoderBuffer[4+i] == 0) {counter[0+i] --;}
//if (inJoyEncoderBuffer[4+i] == 255) {counter[0+i] ++;}
//}
{counter[i] = (inJoyEncoderBuffer[4+i]-128);} // center is 128
ISR_JoyEncoderFlag = 1;
}



void loop() 
{

if (MatrixFillColor != OldMatrixFillColor)
{noInterrupts(); // critical, time-sensitive code here
Matrix.fill(MatrixFillColor);
interrupts(); OldMatrixFillColor = MatrixFillColor;}

if (LCDBrightness != OldLCDBrightness)
{noInterrupts();
LCDbrightness(LCDBrightness); 
interrupts(); OldLCDBrightness = LCDBrightness;}




//Button Scanner Has New Data, do the processing here
if (ISR_BtnScannerFlag)
{
//Serial.println("====BUTTON SCANNER DATA====");
//for (int i=0; i<13; i++){
//Serial.print(inBtnMatrixBuffer[i]);}
//Serial.println("====                     ====");
noInterrupts();  //turn off interrupts when sending any outgoing SPI functions, i.e. to the LCD or LED Matrix to prevent msg. collision
matchLED_w_Btns();
interrupts(); // turn interrupts back on

ISR_BtnScannerFlag =0;
}


//Joystick Encoder Has New Data, do the processing here
if (ISR_JoyEncoderFlag)
{
//Serial.println("====JOYSTICK ENCODER DATA====");

//for (int i=0; i<9; i++){
//Serial.println(inJoyEncoderBuffer[i]);}
//Serial.println("====                     ====");

//for (int i=0; i<4; i++){

//Serial.println(counter[i]);}
//Serial.println("====                     ====");

//noInterrupts();  //turn off interrupts when sending any outgoing SPI functions, i.e. to the LCD or LED Matrix to prevent msg. collision
//do stuff here that uses SPI outgoing, like the LCD or LED matrix

//char Counter [8];
//LCDclear();  delayMicroseconds(500);

//itoa(counter[0], Counter, 10);
//LCDsetCursor(0); delayMicroseconds(500);
//LCDprintS(Counter);delayMicroseconds(500);
//interrupts(); // turn interrupts back on

ISR_JoyEncoderFlag =0;
}

//Frame Update for LCD 
if (tik > refreshLCDtiks)
{tik=0; 

char Counter[8];

noInterrupts();//turn off interrupts when sending any outgoing SPI functions, i.e. to the LCD to prevent msg. collision
if (counter[0] != oldCounter[0])
{itoa(counter[0], Counter, 10);

LCDsetCursor(0);  
LCDprintS(Counter);LCDprintS("  "); 

oldCounter[0] = counter[0];}

if (counter[1] != oldCounter[1])
{itoa(counter[1], Counter, 10);

LCDsetCursor(8);  
LCDprintS(Counter);LCDprintS("  "); 

oldCounter[1] = counter[1];}

if (counter[2] != oldCounter[2])
{itoa(counter[2], Counter, 10);

LCDsetCursor(16);  
LCDprintS(Counter);LCDprintS("  "); 

oldCounter[2] = counter[2];}

if (counter[3] != oldCounter[3])
{itoa(counter[3], Counter, 10);

LCDsetCursor(24);  
LCDprintS(Counter);LCDprintS("  "); 
oldCounter[3] = counter[3];}
interrupts(); // turn interrupts back on}
}

tik++; 


}//===============  END OF VOID LOOP()   ===========================


//LED Button Matrix Scanner Reset
void BtnMatrixReset()
{spi_send(BTN_MATRIX_CS, BtnScannerReset); delay (400);}

//LCD Joystic Encoder Scanner Reset
void JoyEncoderReset()
{spi_send(JOYENCDR_CS, BtnScannerReset); delay (400);}


//======= LCD FUNCTIONS ========
void LCDcursorLeft ()
{spi_send(JOYENCDR_CS, 0xFE); spi_send(JOYENCDR_CS, 0x03);
}

void LCDcursorRight ()
{spi_send(JOYENCDR_CS, 0xFE); spi_send(JOYENCDR_CS, 0x02);
}

void LCDcursorOn ()
{spi_send(JOYENCDR_CS, 0xFE); spi_send(JOYENCDR_CS, 0x0D);
}
void LCDcursorOff ()
{LCDdisplayOn ();}

void LCDcursorBlinkOn ()
{spi_send(JOYENCDR_CS, 0xFE); spi_send(JOYENCDR_CS, 0x0E);
}

void LCDcursorBlinkOff ()
{LCDcursorOn ();}


void LCDdisplayOn ()
{spi_send(JOYENCDR_CS, 0xFE); spi_send(JOYENCDR_CS, 0x0C);
}

void LCDdisplayOff ()
{spi_send(JOYENCDR_CS, 0xFE); spi_send(JOYENCDR_CS, 0x08);
}

void LCDsetCursor (byte pos)
{
spi_send(JOYENCDR_CS, 0xFE); spi_send(JOYENCDR_CS, (0x80 + pos)); 
}

void LCDshiftRight ()
{
spi_send(JOYENCDR_CS, 0xFE); spi_send(JOYENCDR_CS, 0x0B);
}

void LCDshiftLeft ()
{
spi_send(JOYENCDR_CS, 0xFE); spi_send(JOYENCDR_CS, 0x0A);
}


void LCDprintS(char LCDstring[])
{
  for (int i=0; i< strlen(LCDstring); i++)
  {spi_send(JOYENCDR_CS, LCDstring[i]); }
}

void LCDprintC(char LCDchar)
{spi_send(JOYENCDR_CS, LCDchar);}

void LCDbrightness(byte level)
{
 //level between 0 (off) and 10(full)
 if (level<26) 
 {level += 170;} else level=170;
 spi_send(JOYENCDR_CS, 0xFE); spi_send(JOYENCDR_CS, level);
}

void LCDclear(void)
 {spi_send(JOYENCDR_CS, 0xFE); spi_send(JOYENCDR_CS, 0x01); }
//===========================================================

void matchLED_w_Btns()
{
  for (byte row = 0; row<8; row++)
  {
    for (byte col = 0; col<8; col++)
    {
      if (inBtnMatrixBuffer [row] & (1<<col)) // button for testing
       {//Set the Button Scanner Samples to maximum to light up LED
         Matrix.Pixel(color, (0b0000111 ^ row) +1, col+1);; // this flips the first 3 bit places of the byte because the rows are inverted in the LED Matrix
       }
     }    
    } 
  
}

unsigned char Bit_Reverse( unsigned char x ) 
{ 
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa); 
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc); 
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0); 
    return x;    
}



