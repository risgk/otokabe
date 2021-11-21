//////////////////////////////////////////////////////////
// Firmware for OTOKABE (Sketches for Arduino Leonardo) //
//////////////////////////////////////////////////////////

#include "MIDIUSB.h"

#if 1
#define SERIAL_SPEED        (38400) // Serial
#else
#define SERIAL_SPEED        (31250) // MIDI
#endif

#define NOTE_ON_VELOCITY    (100)
#define DEFAULT_CH          (0)     // Default MIDI channel
#define ANTICHATTERING_WAIT (100)   // msec
#define INVALID             (0xFF)

typedef struct {
  int          analogThreshold;  // Analog threshold
  byte         value;            // HIGH or LOW
  unsigned int valueChangedTime; // msec
  byte         midiCh;           // 0 to 15
  byte         noteNumber;       // 0 to 127 (INVALID if there is no sensor)
} SENSOR_STATE;

SENSOR_STATE s_sensorStates[] = {
  { 0, HIGH, 0, DEFAULT_CH, 60      }, // A0
  { 0, HIGH, 0, DEFAULT_CH, INVALID }, // A1
  { 0, HIGH, 0, DEFAULT_CH, INVALID }, // A2
  { 0, HIGH, 0, DEFAULT_CH, INVALID }, // A3
  { 0, HIGH, 0, DEFAULT_CH, INVALID }, // A4
  { 0, HIGH, 0, DEFAULT_CH, INVALID }, // A5
  { 0, HIGH, 0, DEFAULT_CH, INVALID }, // A6 (D4)
  { 0, HIGH, 0, DEFAULT_CH, INVALID }, // A7 (D6)
  { 0, HIGH, 0, DEFAULT_CH, INVALID }, // A8 (D8)
  { 0, HIGH, 0, DEFAULT_CH, INVALID }, // A9 (D9)
  { 0, HIGH, 0, DEFAULT_CH, INVALID }, // A10 (D10)
  { 0, HIGH, 0, DEFAULT_CH, 71      }, // A11 (D12)
};



void setup()
{
#if 1
  // Speed up analogRead()
  ADCSRA = ADCSRA & 0xf8;
  ADCSRA = ADCSRA | 0x04;
#endif

  Serial.begin(SERIAL_SPEED);
  while (!Serial) {
    ;
  }

  // Determine thresholds from initial analog values
  for (byte analogPin = 0; analogPin < sizeof(s_sensorStates) / sizeof(SENSOR_STATE); analogPin++) {
    if (s_sensorStates[analogPin].noteNumber != INVALID) {
      int initialAnalogValue = analogRead(analogPin);
      s_sensorStates[analogPin].analogThreshold = initialAnalogValue * 0.5;
#if 0
      Serial.print(analogPin);
      Serial.print("(Threshold):");
      Serial.println(s_sensorStates[analogPin].analogThreshold);
#endif
    }
  }
}

void loop()
{
  for (byte analogPin = 0; analogPin < sizeof(s_sensorStates) / sizeof(SENSOR_STATE); analogPin++) {
    if (s_sensorStates[analogPin].noteNumber != INVALID) {
      unsigned int currentTime = millis();

      if (currentTime - s_sensorStates[analogPin].valueChangedTime >= ANTICHATTERING_WAIT) {
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
  Serial.print(analogPin);
  Serial.print(':');
  Serial.println(analogValue);
#endif

  if (analogValue >= analogThreshold) {
    return HIGH;
  }

  return LOW;
}

void sendMIDINoteOn(byte midiCh, byte noteNumber)
{
  midiEventPacket_t event = {0x09, (uint8_t) (0x90 | midiCh), noteNumber, NOTE_ON_VELOCITY};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}

void sendMIDINoteOff(byte midiCh, byte noteNumber)
{
  midiEventPacket_t event = {0x08, (uint8_t) (0x80 | midiCh), noteNumber, 0x00};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}

void sendMIDIControlChange(byte midiCh, byte controlNumber, byte value)
{
  midiEventPacket_t event = {0x0B, (uint8_t) (0xB0 | midiCh), controlNumber, value};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}
