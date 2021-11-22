//////////////////////////////////////////////////////////
// Firmware for OTOKABE (Sketches for Arduino Leonardo) //
//////////////////////////////////////////////////////////
#define BOARD_NUMBER (0) 

#if (BOARD_NUMBER == 0)
#define ENABLE_MIDIUSB
#include "MIDIUSB.h"
#endif



#define NOTE_ON_VELOCITY         (100)
#define DEFAULT_MIDI_CH          (0)
#define ANTICHATTERING_WAIT_MSEC (100)
#define INVALID                  (0xFF)

typedef struct {
  int          analogThreshold;  // Analog threshold
  byte         value;            // HIGH or LOW
  unsigned int valueChangedTime; // msec
  byte         midiCh;           // 0 to 15
  byte         noteNumber;       // 0 to 127 (INVALID if there is no sensor)
} SENSOR_STATE;

SENSOR_STATE s_sensorStates[] = {
#if (BOARD_NUMBER == 0)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 60      }, // A0
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 61      }, // A1
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 62      }, // A2
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 63      }, // A3
  { 0, HIGH, 0, DEFAULT_MIDI_CH, INVALID }, // A4
  { 0, HIGH, 0, DEFAULT_MIDI_CH, INVALID }, // A5
  { 0, HIGH, 0, DEFAULT_MIDI_CH, INVALID }, // A6 (D4)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, INVALID }, // A7 (D6)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, INVALID }, // A8 (D8)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, INVALID }, // A9 (D9)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, INVALID }, // A10 (D10)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, INVALID }, // A11 (D12)
#elif (BOARD_NUMBER == 1)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 48      }, // A0
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 49      }, // A1
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 50      }, // A2
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 51      }, // A3
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 52      }, // A4
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 53      }, // A5
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 54      }, // A6 (D4)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 55      }, // A7 (D6)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 56      }, // A8 (D8)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 57      }, // A9 (D9)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 58      }, // A10 (D10)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 59      }, // A11 (D12)
#endif
};



void setup()
{
#if 1
  // Speed up analogRead()
  ADCSRA = ADCSRA & 0xf8;
  ADCSRA = ADCSRA | 0x04;
#endif

  // Determine thresholds from initial analog values
  for (byte analogPin = 0; analogPin < sizeof(s_sensorStates) / sizeof(SENSOR_STATE); analogPin++) {
    if (s_sensorStates[analogPin].noteNumber != INVALID) {
      int initialAnalogValue = analogRead(analogPin);
      s_sensorStates[analogPin].analogThreshold = initialAnalogValue * 0.5;
    }
  }
}

void loop()
{
  for (byte analogPin = 0; analogPin < sizeof(s_sensorStates) / sizeof(SENSOR_STATE); analogPin++) {
    if (s_sensorStates[analogPin].noteNumber != INVALID) {
      unsigned int currentTime = millis();

      if (currentTime - s_sensorStates[analogPin].valueChangedTime >= ANTICHATTERING_WAIT_MSEC) {
        byte sensorValue = readSensorValue(analogPin, s_sensorStates[analogPin].analogThreshold);

        if ((s_sensorStates[analogPin].value == HIGH) && (sensorValue == LOW)) {
          s_sensorStates[analogPin].value = LOW;
          s_sensorStates[analogPin].valueChangedTime = currentTime;
          sendMIDINoteOn(s_sensorStates[analogPin].midiCh, s_sensorStates[analogPin].noteNumber);
        } else if ((s_sensorStates[analogPin].value == LOW) && (sensorValue == HIGH)) {
          s_sensorStates[analogPin].value = HIGH;
          s_sensorStates[analogPin].valueChangedTime = currentTime;
          sendMIDINoteOff(s_sensorStates[analogPin].midiCh, s_sensorStates[analogPin].noteNumber);
        }
      }
    }
  }
}

byte readSensorValue(byte analogPin, int analogThreshold)
{
  int analogValue = analogRead(analogPin);

#if 0
  // Debug Output
  sendMIDIControlChange(analogPin, 0x50, analogValue >> 3);
#endif

  if (analogValue >= analogThreshold) {
    return HIGH;
  }

  return LOW;
}

void sendMIDINoteOn(byte midiCh, byte noteNumber)
{
#if defined(ENABLE_MIDIUSB)
  midiEventPacket_t event = {0x09, (uint8_t) (0x90 | midiCh), noteNumber, NOTE_ON_VELOCITY};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
#endif
}

void sendMIDINoteOff(byte midiCh, byte noteNumber)
{
#if defined(ENABLE_MIDIUSB)
  midiEventPacket_t event = {0x08, (uint8_t) (0x80 | midiCh), noteNumber, 0x00};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
#endif
}

void sendMIDIControlChange(byte midiCh, byte controlNumber, byte value)
{
#if defined(ENABLE_MIDIUSB)
  midiEventPacket_t event = {0x0B, (uint8_t) (0xB0 | midiCh), controlNumber, value};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
#endif
}
