/*
  Dream2635_EK.cpp - Library for interfacing with Sparkfun's MIDI Breakout and the Dream 2635-EK development kit with 263xE-Drum on ch.10 and General MIDI on 1-9, 11-16.
  Dream "When sound quality counts, count on Dream"  http://www.dream.fr/
  General MIDI information: http://www.dream.fr/pdf/Serie2000/Soundbanks/GMBK9764.pdf
  E-Drum Information: http://www.dream.fr/pdf/Serie2000/Firmwares/Firm263xEDrum.pdf
  
  This library was developed for use in the PrISM64 project.
  Brought to you by Ducktronix LLC,  https://www.ducktronix.com/
  Created by H. Wayne "Duck" Qualkinbush  Feb 2014
  Released into the public domain.
*/


#include <Arduino.h>
#include <Dream2635_EK.h>

Dream2635_EK::Dream2635_EK (int nul)
{  //nothing to do until the Serial port is opened
   // uses the standard MIDI baud rate of 31250
	}
	
void Dream2635_EK::note_on(byte channel, byte note, byte vel)
{//this is used to turn a note ON over a channel 1-16.  
//It subtracts one from the channel  from the channels because the documentation tables index them at 1, whereas the messages are indexed at 0
// notes are 0-127 and velocity is 0 min - 127 max
MIDI_Send3(0x90 + (validChannel(0, channel-1)), valid127Data(0, note), valid127Data(100, vel));
}	
void Dream2635_EK::note_off(byte channel, byte note, byte vel)
{//this is used to turn a note OFF on a channel 1-16.  
//It subtracts one from the channel  from the channels because the documentation tables index them at 1, whereas the messages are indexed at 0
// notes are 0-127 and velocity is 0 min - 127 max
MIDI_Send3(0x80 + (validChannel(0, channel-1)), valid127Data(0, note), valid127Data(0, vel));
}	
void Dream2635_EK::pitch_bend(byte channel, byte high, byte low)
{//this is used to bend the pitch wheel of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//lowest pitch values are 0x00,0x00, middle is 0x00,0x40, and highest is 0x7F,0x7F.  
// This message uses two 7 bit words to express wheel movements, 0xxxxxxx 0xxxxxxx, not sure how 
//they increment/decrement with an actual wheel controller
MIDI_Send3(0xE0 + (validChannel(0, channel-1)), valid127Data(0x00,high), valid127Data(0x40,low));
}	
void Dream2635_EK::chan_pressure(byte channel, byte pressure)
{//this is used to set the channel pressure value to a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable values are 0-127
MIDI_Send2(0xD0 + (validChannel(0, channel-1)), (valid127Data(0, pressure)));
}
void Dream2635_EK::prog_change(byte channel, byte program)
{//this is used to change the instrument program on a channel 1-16, but not the variation.  
//It subtracts one from the channel and program values because the documentation tables index them at 1, whereas the messages are indexed at 0
MIDI_Send2(0xC0 + (validChannel(0, channel-1)), (valid127Data(0, program-1)));
}	
void Dream2635_EK::instr_change(byte channel, byte program, byte variation)
{//this is used to change the instrument program and variation on a channel 1-16.  
//It subtracts one from the channel and program values because the documentation tables index them at 1, whereas the messages are indexed at 0
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x00, (valid127Data(0, variation)));  // send CTRL-0 followed by the variation change number
MIDI_Send2(0xC0 + (validChannel(0, channel-1)), (valid127Data(0, program-1))); //send a new program command to lock the change in
}	
void Dream2635_EK::key_aftertouch(byte channel, byte note, byte val)
{//this is used to change the key after-touch message on a channel 1-16.  
//It subtracts one from the channel and program values because the documentation tables index them at 1, whereas the messages are indexed at 0
//values for the after-touch value are 0-127
MIDI_Send3(0xA0 + (validChannel(0, channel-1)), (valid127Data(0, note)), (valid127Data(0, val)));  // send After-touch message with note and value
}	
void Dream2635_EK::mod_wheel(byte channel, byte val)
{//this is used to set the modulation wheel value to a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable mod wheel values are 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x01, (valid127Data(0, val)));
}
void Dream2635_EK::portamento_time(byte channel, byte val)
{//this is used to set the portamento time value to a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable time values are 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x05, (valid127Data(0, val)));
}
void Dream2635_EK::volume(byte channel, byte val)
{//this is used to set the volume of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable volume values are 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x07, (valid127Data(100, val)));
}
void Dream2635_EK::pan(byte channel, byte val)
{//this is used to set the panning of a mono instrument channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, 64 = center
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x0A, (valid127Data(64, val)));
}
void Dream2635_EK::balance(byte channel, byte val)
{//this is used to set the panning of a stereo instrument channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable  balance values are 0-127, 64 = center
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x08, (valid127Data(64, val)));
}
void Dream2635_EK::expression(byte channel, byte val)
{//this is used to set the expression of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable expression values are 0-127, default is 127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x0B, (valid127Data(127, val)));
}
void Dream2635_EK::sustain(byte channel, byte val)
{//this is used to set the sustain value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable sustain values are 0-127, default is 0>=(off)<=63, 64>=(on)<=127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x40, (valid127Data(0, val)));
}
void Dream2635_EK::portamento(byte channel, byte val)
{//this is used to set the portamento value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable portamento values are 0-127, default is 0>=(off)<=63, 64>=(on)<=127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x41, (valid127Data(0, val)));
}
void Dream2635_EK::bank_sel(byte channel, byte val)
{//this is used to set the Bank Select of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable Bank Select values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x00, (valid127Data(0, val)));
}


//========Foot & Breath Controller========
void Dream2635_EK::Foot_Ctrlr(byte channel, byte val)
{//this is used to set the Foot Controller value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable values are 0-127, default is 0-127
//For the E-Drums, Foot Ctrlr val 0-11; note 46 will play Open High-Hat
//For the E-Drums, Foot Ctrlr val 12-64; note 46 will play Half-Open High-Hat
//For the E-Drums, Foot Ctrlr val 65-127; note 46 will play Closed High-Hat
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x04, (valid127Data(0, val)));
}
void Dream2635_EK::Breath_Ctrlr(byte channel, byte val)
{//this is used to set the Breath Controller value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x02, (valid127Data(0, val)));
}




//========General Purpose Controllers========
void Dream2635_EK::GenPurp_Ctrlr_1(byte channel, byte val)
{//this is used to set the General Purpose Controller 1 value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x10, (valid127Data(0, val)));
}
void Dream2635_EK::GenPurp_Ctrlr_2(byte channel, byte val)
{//this is used to set the General Purpose Controller 2 value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x11, (valid127Data(0, val)));
}
void Dream2635_EK::GenPurp_Ctrlr_3(byte channel, byte val)
{//this is used to set the General Purpose Controller 3 value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x12, (valid127Data(0, val)));
}
void Dream2635_EK::GenPurp_Ctrlr_4(byte channel, byte val)
{//this is used to set the General Purpose Controller 4 value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x13, (valid127Data(0, val)));
}
void Dream2635_EK::GenPurp_Ctrlr_5(byte channel, byte val)
{//this is used to set the General Purpose Controller 5 value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x50, (valid127Data(0, val)));
}
void Dream2635_EK::GenPurp_Ctrlr_6(byte channel, byte val)
{//this is used to set the General Purpose Controller 6 value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x51, (valid127Data(0, val)));
}
void Dream2635_EK::GenPurp_Ctrlr_7(byte channel, byte val)
{//this is used to set the General Purpose Controller 7 value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x52, (valid127Data(0, val)));
}
void Dream2635_EK::GenPurp_Ctrlr_8(byte channel, byte val)
{//this is used to set the General Purpose Controller 8 value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x53, (valid127Data(0, val)));
}

//========Sound Controllers========
void Dream2635_EK::Sound_Ctrlr_1(byte channel, byte val)
{//this is used to set the Sound Controller 1 (typically Sound Variation) value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x46, (valid127Data(0, val)));
}
void Dream2635_EK::Sound_Ctrlr_2(byte channel, byte val)
{//this is used to set the Sound Controller 2 (typically Timbre/Harmonic Intensity) value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x47, (valid127Data(0, val)));
}
void Dream2635_EK::Sound_Ctrlr_3(byte channel, byte val)
{//this is used to set the Sound Controller 3 (typically Release Time) value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x48, (valid127Data(0, val)));
}
void Dream2635_EK::Sound_Ctrlr_4(byte channel, byte val)
{//this is used to set the Sound Controller 4 (typically Attack Time) value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x49, (valid127Data(0, val)));
}
void Dream2635_EK::Sound_Ctrlr_5(byte channel, byte val)
{//this is used to set the Sound Controller 5 (typically Brightness) value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x4A, (valid127Data(0, val)));
}
void Dream2635_EK::Sound_Ctrlr_6(byte channel, byte val)
{//this is used to set the Sound Controller 6 (typically Decay Time) value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x4B, (valid127Data(0, val)));
}
void Dream2635_EK::Sound_Ctrlr_7(byte channel, byte val)
{//this is used to set the Sound Controller 7 (typically Vibrato Rate) value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x4C, (valid127Data(0, val)));
}
void Dream2635_EK::Sound_Ctrlr_8(byte channel, byte val)
{//this is used to set the Sound Controller 8 (typically Vibrato Depth) value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x4D, (valid127Data(0, val)));
}
void Dream2635_EK::Sound_Ctrlr_9(byte channel, byte val)
{//this is used to set the Sound Controller 9 (typically Vibrato Delay) value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x4E, (valid127Data(0, val)));
}
void Dream2635_EK::Sound_Ctrlr_10(byte channel, byte val)
{//this is used to set the Sound Controller 10 (typically undefined) value of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x4F, (valid127Data(0, val)));
}


//========Effects and Effect Control========
void Dream2635_EK::Effect_Ctrl_1(byte channel, byte val)
{//this is used to set the expression of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x0C, (valid127Data(127, val)));
}
void Dream2635_EK::Effect_Ctrl_2(byte channel, byte val)
{//this is used to set the expression of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x0D, (valid127Data(127, val)));
}

void Dream2635_EK::reverb(byte channel, byte val)
{//this is used to set the Effects 1 (typically reverb) send level of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x5B, (valid127Data(0, val)));
}
void Dream2635_EK::effectsDepth_2(byte channel, byte val)
{//this is used to set the Effects 2 send level of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x5C, (valid127Data(0, val)));
}
void Dream2635_EK::chorus(byte channel, byte val)
{//this is used to set the Effects 3 (typically chorus) send level of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x5D, (valid127Data(0, val)));
}
void Dream2635_EK::effectsDepth_4(byte channel, byte val)
{//this is used to set the Effects 4 send level of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x5E, (valid127Data(0, val)));
}
void Dream2635_EK::effectsDepth_5(byte channel, byte val)
{//this is used to set the Effects 5 send level of a channel 1-16.  
//It subtracts one from the channel  because the documentation tables index them at 1, whereas the messages are indexed at 0
//allowable panning values are 0-127, default is 0-127
MIDI_Send3(0xB0 + (validChannel(0, channel-1)), 0x5F, (valid127Data(0, val)));
}





void Dream2635_EK::MIDIreset()
{
MIDI_Send1(0xFF); // send a reset message
}
void Dream2635_EK::timingClock()
{
MIDI_Send1(0xF8); // send a timing message
}
void Dream2635_EK::allChannels_allNotes_off()
{
	for (int i=1; i<17; i++)
	{
	allNotes_off(i);  delay (5); // all notes off on all channels 1-16, indexed at zero
	}
}

void Dream2635_EK::allChannels_allSounds_off()
{
	for (int i=1; i<17; i++)
	{
	allSounds_off(i);  delay (5); // all sounds off on all channels 1-16, indexed at zero
	}
}

void Dream2635_EK::allSounds_off(byte channel)
{
	MIDI_Send3 (0xB0 + (validChannel(0,channel)-1), 0x78, 0x00);  // all sounds off on channel 0xBn
}

void Dream2635_EK::allNotes_off(byte channel)
{
	MIDI_Send3 (0xB0 + (validChannel(0,channel)-1), 0x7B, 0x00);  // all notes off on channel 0xBn
}

void Dream2635_EK::reset_controllers(byte channel)
{
	MIDI_Send3 (0xB0 + (validChannel(0,channel)-1), 0x79, 0x00);  // reset all controllers on channel 0xBn
}

void Dream2635_EK::allChannels_reset_controllers()
{
	for (int i=1; i<17; i++)
	{
	reset_controllers(i-1);  // reset all controllers on channel 0xBn
	delay (5); // reset all controllers on all channels 1-16
	}
	
}
void Dream2635_EK::poly_on(byte channel)
{
MIDI_Send3 (0xB0 + (validChannel(0,channel)-1), 0x7F, 0x00);  // turn polyphony on channel 0xBn

}
void Dream2635_EK::allChannels_poly_on()
{for (int i=1; i<17; i++)
	{
	poly_on(i);  delay (5); // turn Polyphony on all channels 1-16, indexed at zero
	}}


//==============MIDI Command and Data Transmission methods================
void Dream2635_EK::MIDI_Send3(byte cmd, byte data1, byte data2)
{
  Serial.write(cmd);
  Serial.write(data1);
  Serial.write(data2);
}
void Dream2635_EK::MIDI_Send2(byte cmd, byte data1)
{
  Serial.write(cmd);
  Serial.write(data1);
}
void Dream2635_EK::MIDI_Send1(byte cmd)
{
  Serial.write(cmd);
}
//===========Private Internal-Only Functions================

byte Dream2635_EK::validChannel(byte invalid, byte channel)
{
if (channel>0 && channel <17) return channel;
else return invalid; // this simply returns the invalid value back if the value is out of limits, i.e. a default value for example
}

byte Dream2635_EK::valid127Data(byte invalid, byte data)
{
if (data>=0 && data <128) return data;
else return invalid;  // this simply returns the invalid value back if the value is out of limits, i.e. a default value for example
}