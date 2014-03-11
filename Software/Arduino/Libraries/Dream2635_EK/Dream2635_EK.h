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

#ifndef Dream2635_EK_h
#define Dream2635_EK_h

#include "Arduino.h"

class Dream2635_EK
{
  public:
    Dream2635_EK (int nul);
	
	void MIDI_Send1(byte cmd);
	void MIDI_Send2(byte cmd, byte data1);
	void MIDI_Send3(byte cmd, byte data1, byte data2);
	void MIDI_Send4(byte cmd, byte data1, byte data2, byte data3);
	void MIDI_Send5(byte cmd, byte data1, byte data2, byte data3, byte data4);
	void MIDI_Send6(byte cmd, byte data1, byte data2, byte data3, byte data4, byte data5);
	void MIDI_Send7(byte cmd, byte data1, byte data2, byte data3, byte data4, byte data5, byte data6);
	void MIDI_Send8(byte cmd, byte data1, byte data2, byte data3, byte data4, byte data5, byte data6, byte data7);
	
	void note_on(byte channel, byte note, byte vel);
	void note_off(byte channel, byte note, byte vel);
	void pitch_bend(byte channel, byte high, byte low);
	void prog_change(byte channel, byte program);
	void instr_change(byte channel, byte program, byte variation);
	void chan_pressure(byte channel, byte pressure);
	void key_aftertouch(byte channel, byte note, byte val);
	void mod_wheel(byte channel, byte val);
	void portamento_time(byte channel, byte val);
	void volume(byte channel, byte val);
	void pan(byte channel, byte val);
	void expression(byte channel, byte val);
	void sustain(byte channel, byte val);
	void portamento(byte channel, byte val);
	void balance(byte channel, byte val);
	
	void reverb(byte channel, byte val);
	void effectsDepth_2(byte channel, byte val);
	void chorus(byte channel, byte val);
	void effectsDepth_4(byte channel, byte val);
	void effectsDepth_5(byte channel, byte val);
	
	void Breath_Ctrlr(byte channel, byte val);
	void Foot_Ctrlr(byte channel, byte val);
	void bank_sel(byte channel, byte val);
	
	void GenPurp_Ctrlr_1(byte channel, byte val);
	void GenPurp_Ctrlr_2(byte channel, byte val);
	void GenPurp_Ctrlr_3(byte channel, byte val);
	void GenPurp_Ctrlr_4(byte channel, byte val);
	void GenPurp_Ctrlr_5(byte channel, byte val);
	void GenPurp_Ctrlr_6(byte channel, byte val);
	void GenPurp_Ctrlr_7(byte channel, byte val);
	void GenPurp_Ctrlr_8(byte channel, byte val);
	
	
	void Sound_Ctrlr_1(byte channel, byte val);
	void Sound_Ctrlr_2(byte channel, byte val);
	void Sound_Ctrlr_3(byte channel, byte val);
	void Sound_Ctrlr_4(byte channel, byte val);
	void Sound_Ctrlr_5(byte channel, byte val);
	void Sound_Ctrlr_6(byte channel, byte val);
	void Sound_Ctrlr_7(byte channel, byte val);
	void Sound_Ctrlr_8(byte channel, byte val);
	void Sound_Ctrlr_9(byte channel, byte val);
	void Sound_Ctrlr_10(byte channel, byte val);
	
	void Effect_Ctrl_1(byte channel, byte val);
	void Effect_Ctrl_2(byte channel, byte val);
	
	
	void MIDIreset();
	void poly_on(byte channel);
	void allChannels_poly_on();
	void allNotes_off(byte channel);
	void allSounds_off(byte channel);
	void allChannels_allNotes_off();
	void allChannels_allSounds_off();
	void reset_controllers(byte channel);
	void allChannels_reset_controllers();
	void timingClock();

	
	private:
	int i;  // used 4 loops
	byte validChannel(byte invalid, byte channel);
	byte valid127Data(byte invalid, byte data);
};

#endif
