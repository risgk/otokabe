//////////////////////////////////////////////////////////
// Firmware for OTOKABE (Sketches for Arduino Leonardo) //
//////////////////////////////////////////////////////////

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
  byte         value;            // HIGH or LOW
  unsigned int valueChangedTime; // msec
  byte         midiCh;           // 0 to 15
  byte         noteNumber;       // 0 to 127 (INVALID if there is no sensor)
  int          analogThreshold;  // Analog threshold
} SENSOR_STATE;

SENSOR_STATE s_sensorStates[] = {
  { HIGH, 0, DEFAULT_CH, 60      }, // A0
  { HIGH, 0, DEFAULT_CH, INVALID }, // A1
  { HIGH, 0, DEFAULT_CH, INVALID }, // A2
  { HIGH, 0, DEFAULT_CH, INVALID }, // A3
  { HIGH, 0, DEFAULT_CH, INVALID }, // A4
  { HIGH, 0, DEFAULT_CH, INVALID }, // A5
  { HIGH, 0, DEFAULT_CH, INVALID }, // A6 (D4)
  { HIGH, 0, DEFAULT_CH, INVALID }, // A7 (D6)
  { HIGH, 0, DEFAULT_CH, INVALID }, // A8 (D8)
  { HIGH, 0, DEFAULT_CH, INVALID }, // A9 (D9)
  { HIGH, 0, DEFAULT_CH, INVALID }, // A10 (D10)
  { HIGH, 0, DEFAULT_CH, 71      }, // A11 (D12)
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
  Serial.write(0x90 | midiCh);
  Serial.write(noteNumber);
  Serial.write(NOTE_ON_VELOCITY);
  Serial.flush();
}

void sendMIDINoteOff(byte midiCh, byte noteNumber)
{
  Serial.write(0x80 | midiCh);
  Serial.write(noteNumber);
  Serial.write(static_cast<uint8_t>(0x00));
  Serial.flush();
}

void sendMIDIConstolChange(byte midiCh, byte value)
{
  Serial.write(0xE0 | midiCh);
  Serial.write(static_cast<uint8_t>(0x00));
  Serial.write(value);
  Serial.flush();
}
