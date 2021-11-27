//////////////////////////////////////////////////////////
// Firmware for OTOKABE (Sketches for Arduino Leonardo) //
//////////////////////////////////////////////////////////
#define BOARD_NUMBER (0)  // Board 1 (TX) -> (RX) Board 0 -> (USB Host)



#include <MIDI.h>

#if (BOARD_NUMBER == 0)
#define ENABLE_MIDIUSB
#include <MIDIUSB.h>
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
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 64      }, // A4
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 65      }, // A5
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 66      }, // A6 (D4)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 67      }, // A7 (D6)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 68      }, // A8 (D8)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 69      }, // A9 (D9)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 70      }, // A10 (D10)
  { 0, HIGH, 0, DEFAULT_MIDI_CH, 71      }, // A11 (D12)
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

MIDI_CREATE_DEFAULT_INSTANCE();



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

  MIDI.setHandleNoteOn(handlerNoteOn);
  MIDI.setHandleNoteOff(handlerNoteOff);
  MIDI.setHandleControlChange(handlerControlChange);
  MIDI.begin();
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
          sendMIDINoteOn(s_sensorStates[analogPin].midiCh, s_sensorStates[analogPin].noteNumber, NOTE_ON_VELOCITY);
        } else if ((s_sensorStates[analogPin].value == LOW) && (sensorValue == HIGH)) {
          s_sensorStates[analogPin].value = HIGH;
          s_sensorStates[analogPin].valueChangedTime = currentTime;
          sendMIDINoteOff(s_sensorStates[analogPin].midiCh, s_sensorStates[analogPin].noteNumber, 64);
        }
      }
    }
  }

#if (BOARD_NUMBER == 0)
  MIDI.read();
#endif
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

void sendMIDINoteOn(byte midiCh, byte noteNumber, byte velocity)
{
  MIDI.sendNoteOn(noteNumber, velocity, midiCh);

#if defined(ENABLE_MIDIUSB)
  midiEventPacket_t event = {0x09, (uint8_t) (0x90 | midiCh), noteNumber, NOTE_ON_VELOCITY};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
#endif
}

void sendMIDINoteOff(byte midiCh, byte noteNumber, byte velocity)
{
  MIDI.sendNoteOff(noteNumber, velocity, midiCh);

#if defined(ENABLE_MIDIUSB)
  midiEventPacket_t event = {0x08, (uint8_t) (0x80 | midiCh), noteNumber, velocity};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
#endif
}

void sendMIDIControlChange(byte midiCh, byte controlNumber, byte value)
{
  MIDI.sendControlChange(controlNumber, value, midiCh);

#if defined(ENABLE_MIDIUSB)
  midiEventPacket_t event = {0x0B, (uint8_t) (0xB0 | midiCh), controlNumber, value};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
#endif
}

void handlerNoteOn(byte channel, byte note, byte velocity)
{
  // Forwarding
  sendMIDINoteOn(channel, note, velocity);
}

void handlerNoteOff(byte channel, byte note, byte velocity)
{
  // Forwarding
  sendMIDINoteOff(channel, note, velocity);
}

void handlerControlChange(byte channel, byte note, byte value)
{
  // Forwarding
  sendMIDIControlChange(channel, note, value);
}
