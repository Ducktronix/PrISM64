/*
  Dream2635_EK - LProgram for interfacing with Sparkfun's MIDI breakout and the Dream 2635-EK development kit.
  Dream "When sound quality counts, count on Dream"  http://www.dream.fr/
  General MIDI information: http://www.dream.fr/pdf/Serie2000/Soundbanks/GMBK9764.pdf
  E-Drum Information: http://www.dream.fr/pdf/Serie2000/Firmwares/Firm263xEDrum.pdf
  
  This program was developed for use in the PrISM64 project.
  Brought to you by Ducktronix LLC,  https://www.ducktronix.com/
  Created by H. Wayne "Duck" Qualkinbush  Feb 2014
  Released into the public domain.
*/


#include <Dream2635_EK.h>

// defines for MIDI Shield components only
const byte BUTTON1 = 2;
const byte BUTTON2 = 3;
const byte BUTTON3 = 4;
const byte BUTTON4 = 5;
const byte BUTTON5 = 6;

#define OFF 1
#define ON 2
#define WAIT 3

byte incomingByte;

byte velocity;

byte program;
byte program2;
byte program3;
byte variation;
byte variation2;
byte variation3;
const byte channel =  1;
const byte channel2 = 2;
const byte channel3 = 3;

int action=2; //1 =note off ; 2=note on ; 3= wait



volatile static unsigned long newTik = 1; // track the timer tiks

int tempo = 110; // desired tempo in Beats Per Minutes BPM
const float systemClockFreq = 16000000; //16MHz clock on Arduino
volatile int seqStep = 96; //start sequencestep at 96

boolean play;

// initialize the Dream and MIDI serial Library etc.
Dream2635_EK Dream (0); 


// Variables will change:
volatile int button1State;             // the current reading from the input pin
volatile int button2State;             // the current reading from the input pin
volatile int button3State;             // the current reading from the input pin
volatile int button4State;             // the current reading from the input pin
volatile int button5State;             // the current reading from the input pin

volatile byte lastButton1State = HIGH;   // the previous reading from the input pin
volatile byte lastButton2State = HIGH;   // the previous reading from the input pin
volatile byte lastButton3State = HIGH;   // the previous reading from the input pin
volatile byte lastButton4State = HIGH;   // the previous reading from the input pin
volatile byte lastButton5State = HIGH;   // the previous reading from the input pin

volatile static unsigned long lastDebounce1Time = 0;  // the last step the button was pressed
volatile static unsigned long lastDebounce2Time = 0;  // the last step the button was pressed
volatile static unsigned long lastDebounce3Time = 0;  // the last step the button was pressed
volatile static unsigned long lastDebounce4Time = 0;  // the last step the button was pressed
volatile static unsigned long lastDebounce5Time = 0;  // the last step the button was pressed

const static unsigned long debounceDelay = 1;    // the debounce tiks

int SoundCtrl_1 =0; // this is a test of an analog pot used to act as a controller
int prevSoundCtrl_1 =0;



void setup() {

 pinMode(BUTTON1,INPUT);
 pinMode(BUTTON2,INPUT);
 pinMode(BUTTON3,INPUT);
 pinMode(BUTTON4,INPUT);
 pinMode(BUTTON5,INPUT);
 pinMode(A5,INPUT);
 
digitalWrite(BUTTON1,HIGH); 
digitalWrite(BUTTON2,HIGH);
digitalWrite(BUTTON3,HIGH);
digitalWrite(BUTTON4,HIGH);
digitalWrite(BUTTON5,HIGH);


  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = (systemClockFreq/256*60/tempo/seqStep*4)-1; // this formula figures out the timing of the sequence steps per beat
                                                        // to compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();



//start serial port
Serial.begin(31250); delay (200);

// set up the Drums/GM synth chipset
  program =82;
  variation = 1;
  program2 =1;
  variation2 = 0;
  program3 =39;
  variation3 = 0;
  
  
  Dream.MIDIreset(); // reset the MIDI system
  delay (200);
  Dream.allChannels_reset_controllers();
  //Dream.MIDI_Send3(0x99,0x5B,0x45); // set GM Reverb 0x00 to 0x7F
  
  //set the Drum Kit Reverb
  

  
  //clear all MIDI sounds & notes on all channels, set polyphony on
 Dream.allChannels_poly_on();
 Dream.allChannels_allNotes_off();
 Dream.allChannels_allSounds_off();
  
   //set in the MIDI Instrument program desired and the variation desired on Channel 1
  //Dream.prog_change(channel,program);
  Dream.instr_change(channel,program, variation);
  Dream.instr_change(channel2,program2, variation2);
  Dream.instr_change(channel3,program3, variation3);
  
  Dream.volume(channel, 120); // set channel volume
  Dream.volume(channel2, 70); // set channel volume
  Dream.volume(channel3, 70); // set channel volume
  
  Dream.pan(channel,64);
  Dream.sustain(channel,0);
  
  //for tempo control, we use a timer
  //tik = millis(); //start tik timer
  
  //start/stop variable
  play = true;
  
//do a count in of 1,2,3,4 at tempo
//Dream.note_on(10,01,100); //play "one"
//delay (tempo * 8);
//Dream.note_on(10,02,100); //play "one"
//delay (tempo * 8);
//Dream.note_on(10,03,100); //play "one"
//delay (tempo * 8);
//Dream.note_on(10,04,100); //play "one"
//delay (tempo * 8);

} // end of void setup()






//==============This is a timer used for drum machine/sequencer/buttons=============
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
newTik++; if (newTik > 96) {newTik = 1;}


//update controller values
//take 4 samples and average
for (int i = 0; i<4; i++) {SoundCtrl_1 += (analogRead(A5)/8);}
SoundCtrl_1 = SoundCtrl_1/4;
if (SoundCtrl_1 != prevSoundCtrl_1) {Dream.Sound_Ctrlr_7(channel, SoundCtrl_1); prevSoundCtrl_1 = SoundCtrl_1;}
//repeat this process for every controller knob, slider, etc. during each tik, or possibly every couple of tiks depending on response required
// do NOT send these types of messages unless changes occur in the sensor, because it floods the channel with unecessary noise
  
//===========Button Handling Here=================

//BUTTON 1-------------->>>>>>
byte reading1 = digitalRead(BUTTON1);
if (reading1 == LOW && lastButton1State == HIGH && (newTik-lastDebounce2Time)>debounceDelay)
{lastDebounce1Time = newTik;
lastButton1State = reading1; doButton1Press();}

if (reading1 == HIGH && lastButton1State == LOW && (newTik-lastDebounce2Time)>debounceDelay)
{lastDebounce1Time = newTik; 
lastButton1State = reading1; doButton1Release();}

//BUTTON 2-------------->>>>>>
byte reading2 = digitalRead(BUTTON2);
if (reading2 == LOW && lastButton2State == HIGH && (newTik-lastDebounce2Time)>debounceDelay)
{lastDebounce2Time = newTik; 
lastButton2State = reading2; doButton2Press();}

if (reading2 == HIGH && lastButton2State == LOW && (newTik-lastDebounce2Time)>debounceDelay)
{lastDebounce2Time = newTik;
lastButton2State = reading2; doButton2Release();}

//BUTTON 3-------------->>>>>>
byte reading3 = digitalRead(BUTTON3);
if (reading3 == LOW && lastButton3State == HIGH && (newTik-lastDebounce3Time)>debounceDelay)
{lastDebounce3Time = newTik; 
lastButton3State = reading3; doButton3Press();}

if (reading3 == HIGH && lastButton3State == LOW && (newTik-lastDebounce3Time)>debounceDelay)
{lastDebounce3Time = newTik;
lastButton3State = reading3; doButton3Release();}

//BUTTON 4-------------->>>>>>
byte reading4 = digitalRead(BUTTON4);
if (reading4 == LOW && lastButton4State == HIGH && (newTik-lastDebounce4Time)>debounceDelay)
{lastDebounce4Time = newTik;
lastButton4State = reading4; doButton4Press();}

if (reading4 == HIGH && lastButton4State == LOW && (newTik-lastDebounce4Time)>debounceDelay)
{lastDebounce4Time = newTik;
lastButton4State = reading4; doButton4Release();}

//BUTTON 4-------------->>>>>>
byte reading5 = digitalRead(BUTTON5);
if (reading5 == LOW && lastButton5State == HIGH && (newTik-lastDebounce5Time)>debounceDelay)
{lastDebounce5Time = newTik;
lastButton5State = reading5; doButton5Press();}

if (reading5 == HIGH && lastButton5State == LOW && (newTik-lastDebounce5Time)>debounceDelay)
{lastDebounce5Time = newTik;
lastButton5State = reading5; doButton5Release();}

  
// ================SEQUENCER HERE================  
if (play)
{seqStep++; if (seqStep>95) seqStep=0; // increment the timing sequence steps by one and loopback at 96
Dream.timingClock(); // send a timing clock message every tick

//if (seqStep == 0 || seqStep == 8 || seqStep == 16 || seqStep == 24 )
//{
//  Dream.note_on(channel,note,100); //play program note middle C - one octave  
//}


if (seqStep == 0 || seqStep == 48 || seqStep == 60)
{
Dream.note_on(10,0x24,100); //play kick drum note  
}

if (seqStep == 24 || seqStep == 72)
{
Dream.note_on(10,0x28,100); //play snare drum note  
}

if (seqStep == 0 || seqStep == 12 || seqStep == 24 || seqStep == 36 || seqStep == 48 || seqStep == 60 || seqStep == 72 || seqStep == 84)
{
Dream.note_on(10,0x2A,65); //play closed HH drum note  

}



//CHORD
if (seqStep == 0 || seqStep == 48)
{
Dream.note_on(channel2,0x3C,100); //play chord note
Dream.note_on(channel2,0x3F,100); //play chord note 
Dream.note_on(channel2,0x43,100); //play chord note 
Dream.note_on(channel3,0x3C-36,100); //play bass note 
}

if (seqStep == 47 || seqStep == 96)
{
Dream.note_off(channel2,0x3C,100); //play chord note
Dream.note_off(channel2,0x3F,100); //play chord note 
Dream.note_off(channel2,0x43,100); //play chord note
Dream.note_off(channel3,0x3C-36,100); //play bass note 
}


} // end of sequencer

} // end of ISR Timer

//FUNCTIONS USED IN ISR Timer======
void doButton1Press()
  {
  Dream.note_on(channel,0x3C,127);
  }
void doButton1Release()
  {
  Dream.note_off(channel,0x3C,127);
  }
void doButton2Press()
  {
  Dream.note_on(channel,0x3F,127);
  }
void doButton2Release()
  {
  Dream.note_off(channel,0x3F,127);
  }
void doButton3Press()
  {
  Dream.note_on(channel,0x43,127); 
  }
void doButton3Release()
  {
  Dream.note_off(channel,0x43,127);
  }
void doButton4Press()
  {
    if (play) 
          {Dream.allChannels_allNotes_off(); // if it was playing and pausing, then stop all sounds/notes and restart the sequence
          seqStep = 1024; //start sequencestep at 1024
          }  
  play = !play; // toggle the play/pause state
  }
void doButton4Release()
  {
  return;
  }  
 void doButton5Press()
  {
  program++; if (program>128) {program=1;} // add one to the program
  Dream.instr_change(channel,program, variation); // assign the new program instrument
  Dream.note_on(channel, 0x3C, 127); // play a sample note
  }
void doButton5Release()
  {
  return;
  }   
  
  
  
  
  //=============================




void loop () {


///DO YOUR OTHER STUFF HERE
  
  
  
  
  
  
  
  
  //*************** MIDI OUT ***************//

 // pot = analogRead(0);
 // note = pot/8;  // convert value to value 0-127

//tempo machine  
//if (millis() >= (tik + tempo) && play)
//{tik = millis();
//seqStep++; if (seqStep>31) seqStep=0;
//} 

 
  //*************** MIDI LOOPBACK ******************//
 /* if(Serial.available() > 0)
  {
    byte1 = Serial.read();
    byte2 = Serial.read();
    byte3 = Serial.read();

    Midi_Send(byte1, byte2, byte3);
  }
  /*
  
  
  //*************** MIDI IN ***************/
  /*
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // wait for as status-byte, channel 1, note on or off
    if (incomingByte== 144) // Note on
    { 
      action = OFF;
    }
    else if (incomingByte== 128) // Note off
    { 
      action = ON;
    }
    else if (note==0 && action != WAIT) // note on, wait for note value
    { 
      note=incomingByte;
    }
    else if (note!=0 && action != WAIT)  // velocity
    { 
      velocity=incomingByte;
      if(action == ON){ 
        Midi_Send(0x90,note,velocity); 
      }
      if(action == OFF){ 
        Midi_Send(0x80,note,velocity); 
      }
      note=0;
      velocity=0;
      action=WAIT;
    }
    else{
    }
  }
*/
}



