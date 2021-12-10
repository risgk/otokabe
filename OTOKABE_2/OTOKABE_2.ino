//////////////////////////////////////////////////////////
// Firmware for OTOKABE (Sketches for Arduino Leonardo) //
//////////////////////////////////////////////////////////
#define CONFIG_BOARD_NUMBER (1)  // Board 1 (TX) -> (RX) Board 0 -> (USB Host)
                                 //                              -> (D13 LED Out)



#include <MIDI.h>

#if (CONFIG_BOARD_NUMBER == 0)
#define ENABLE_MIDIUSB
#include <MIDIUSB.h>
#endif

#define NOTE_ON_VELOCITY            (100)
#define NOTE_OFF_VELOCITY           (64)
#define ANTICHATTERING_WAIT_MSEC    (100)
#define INVALID                     (0xFF)
#define LED_LIGHTING_TIME_MSEC      (100)

typedef struct {
  int          analogThreshold;          // Analog threshold
  unsigned int digitalValueChangedTime;  // msec
  byte         digitalValue;             // HIGH or LOW
  byte         channelZeroOrigin;        // 0 to 15
  byte         noteNumber;               // 0 to 127, INVALID (0xFF) if there is no sensor
} SENSOR_STATE;

SENSOR_STATE s_sensorStates[] = {
#if (CONFIG_BOARD_NUMBER == 0)
  { 0, 0, HIGH, 0, 79      }, // A0         // Sensor 1
  { 0, 0, HIGH, 0, 81      }, // A1         // Sensor 2
  { 0, 0, HIGH, 0, 83      }, // A2         // Sensor 3
  { 0, 0, HIGH, 0, 74      }, // A3         // Sensor 4
  { 0, 0, HIGH, 0, 76      }, // A4         // Sensor 5
  { 0, 0, HIGH, 0, 77      }, // A5         // Sensor 6
  { 0, 0, HIGH, 0, 69      }, // A6 (D4)    // Sensor 7
  { 0, 0, HIGH, 0, 71      }, // A7 (D6)    // Sensor 8
  { 0, 0, HIGH, 0, 72      }, // A8 (D8)    // Sensor 9
  { 0, 0, HIGH, 0, 64      }, // A9 (D9)    // Sensor 10
  { 0, 0, HIGH, 0, 65      }, // A10 (D10)  // Sensor 11
  { 0, 0, HIGH, 0, 67      }, // A11 (D12)  // Sensor 12
#elif (CONFIG_BOARD_NUMBER == 1)
  { 0, 0, HIGH, 0, 60      }, // A0         // Sensor 13
  { 0, 0, HIGH, 0, 62      }, // A1         // Sensor 14
  { 0, 0, HIGH, 1, 49      }, // A2         // Sensor 15
  { 0, 0, HIGH, 1, 42      }, // A3         // Sensor 16
  { 0, 0, HIGH, 1, 38      }, // A4         // Sensor 17
  { 0, 0, HIGH, 1, 50      }, // A5         // Sensor 18
  { 0, 0, HIGH, 1, 45      }, // A6 (D4)    // Sensor 19
  { 0, 0, HIGH, 1, 35      }, // A7 (D6)    // Sensor 20
  { 0, 0, HIGH, 0, INVALID }, // A8 (D8)    // Sensor 21
  { 0, 0, HIGH, 0, INVALID }, // A9 (D9)    // Sensor 22
  { 0, 0, HIGH, 0, INVALID }, // A10 (D10)  // Sensor 23
  { 0, 0, HIGH, 0, INVALID }, // A11 (D12)  // Sensor 24
#endif
};

MIDI_CREATE_DEFAULT_INSTANCE();

boolean      s_LedLit     = false;
unsigned int s_LedlitTime = 0;  // msec



void setup()
{
  // Speed up analogRead()
  ADCSRA = ADCSRA & 0xf8;
  ADCSRA = ADCSRA | 0x04;

  // Determine thresholds from initial analog values
  for (byte analogPin = 0; analogPin < sizeof(s_sensorStates) / sizeof(SENSOR_STATE); analogPin++) {
    if (s_sensorStates[analogPin].noteNumber != INVALID) {
      int initialAnalogValue = analogRead(analogPin);
      s_sensorStates[analogPin].analogThreshold = initialAnalogValue * 0.7;
    }
  }

  MIDI.setHandleNoteOn(handlerNoteOn);
  MIDI.setHandleNoteOff(handlerNoteOff);
  MIDI.setHandleControlChange(handlerControlChange);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  for (byte analogPin = 0; analogPin < sizeof(s_sensorStates) / sizeof(SENSOR_STATE); analogPin++) {
    if (s_sensorStates[analogPin].noteNumber != INVALID) {
      unsigned int currentTime = millis();

      if ((currentTime - s_sensorStates[analogPin].digitalValueChangedTime) >= ANTICHATTERING_WAIT_MSEC) {
        byte sensorValue = readSensorDigitalValue(analogPin, s_sensorStates[analogPin].analogThreshold);

        if ((s_sensorStates[analogPin].digitalValue == HIGH) && (sensorValue == LOW)) {
          s_sensorStates[analogPin].digitalValue = LOW;
          s_sensorStates[analogPin].digitalValueChangedTime = currentTime;
          sendMIDINoteOn(s_sensorStates[analogPin].channelZeroOrigin, s_sensorStates[analogPin].noteNumber, NOTE_ON_VELOCITY);
        } else if ((s_sensorStates[analogPin].digitalValue == LOW) && (sensorValue == HIGH)) {
          s_sensorStates[analogPin].digitalValue = HIGH;
          s_sensorStates[analogPin].digitalValueChangedTime = currentTime;
          sendMIDINoteOff(s_sensorStates[analogPin].channelZeroOrigin, s_sensorStates[analogPin].noteNumber, NOTE_OFF_VELOCITY);
        }
      }
    }
  }

#if (CONFIG_BOARD_NUMBER == 0)
  MIDI.read();
#endif

  checkLed();
}

byte readSensorDigitalValue(byte analogPin, int analogThreshold)
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

void sendMIDINoteOn(byte channelZeroOrigin, byte noteNumber, byte velocity)
{
  MIDI.sendNoteOn(noteNumber, velocity, channelZeroOrigin + 1);

#if defined(ENABLE_MIDIUSB)
  midiEventPacket_t event = {0x09, (uint8_t) (0x90 | channelZeroOrigin), noteNumber, velocity};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
#endif

  lightLed();
}

void sendMIDINoteOff(byte channelZeroOrigin, byte noteNumber, byte velocity)
{
  MIDI.sendNoteOff(noteNumber, velocity, channelZeroOrigin + 1);

#if defined(ENABLE_MIDIUSB)
  midiEventPacket_t event = {0x08, (uint8_t) (0x80 | channelZeroOrigin), noteNumber, velocity};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
#endif
}

void sendMIDIControlChange(byte channelZeroOrigin, byte controlNumber, byte value)
{
  MIDI.sendControlChange(controlNumber, value, channelZeroOrigin + 1);

#if defined(ENABLE_MIDIUSB)
  midiEventPacket_t event = {0x0B, (uint8_t) (0xB0 | channelZeroOrigin), controlNumber, value};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
#endif
}

void handlerNoteOn(byte channelOneOrigin, byte noteNumber, byte velocity)
{
  // Forwarding
  sendMIDINoteOn(channelOneOrigin - 1, noteNumber, velocity);
}

void handlerNoteOff(byte channelOneOrigin, byte noteNumber, byte velocity)
{
  // Forwarding
  sendMIDINoteOff(channelOneOrigin - 1, noteNumber, velocity);
}

void handlerControlChange(byte channelOneOrigin, byte noteNumber, byte value)
{
  // Forwarding
  sendMIDIControlChange(channelOneOrigin - 1, noteNumber, value);
}

void lightLed()
{
  digitalWrite(LED_BUILTIN, HIGH);
  s_LedlitTime = millis();
  s_LedLit = true;
}

void checkLed()
{
  if (s_LedLit) {
    unsigned int currentTime = millis();

    if ((currentTime - s_LedlitTime) >= LED_LIGHTING_TIME_MSEC) {
      s_LedLit = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}
