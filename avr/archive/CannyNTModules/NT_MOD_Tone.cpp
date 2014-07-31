

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Tone Commands



#ifdef USE_TONE
#include <avr/pgmspace.h>

#include "pitches.h"
PROGMEM prog_uint16_t melody[] = { NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
PROGMEM prog_uint16_t noteDurations[] = { 4, 8, 8, 4,4,4,4,4 };

uint8_t tone_playDitty(int p1) {
 for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000/pgm_read_word_near(noteDurations+thisNote);
    tone(TONE_PIN, pgm_read_word_near(melody+thisNote),noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(TONE_PIN);
  }  
  return NT_STATUS_OK;
}


uint8_t tone_playTone(uint8_t id, int p1) {
    int noteDuration = 1000/id; // 4 or 8 - see ply ditty
    tone(TONE_PIN, p1, noteDuration);  // playDitt / pitces.h
  return NT_STATUS_OK;
}


uint8_t  tone_processCommand(int cmd, int id, int p1) {
  int status = NT_ERROR_CMD_INVALID;
  switch (cmd) {
    case NT_CMD_TONE_PLAY_DITTY:
      status = tone_playDitty(id);
      break;
    case NT_CMD_TONE_PLAY_TONE:
      status = tone_playTone(id, p1);
      break;
    default:
      DBG("XX:NT_ERROR_CMD_INVALID!");
      status = NT_ERROR_CMD_INVALID;
  }
  return status;
}

#endif 


