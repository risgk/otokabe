////////////////////////////////////////////////////////////////
// Firmware for OTOKABE (Sketches for Arduino Mega 2560 Rev3) //
////////////////////////////////////////////////////////////////

#if 1
#define SERIAL_SPEED        (38400) // Serial
#else
#define SERIAL_SPEED        (31250) // MIDI
#endif

#define ANALOG_THRESHOLD    (100)   // 1 to 1023
#define NOTE_ON_VELOCITY    (100)
#define DEFAULT_CH          (0)     // Default MIDI channel
#define ANTICHATTERING_WAIT (100)   // msec
#define INVALID             (0xFF)

typedef struct {
  byte         value;            // HIGH or LOW
  unsigned int valueChangedTime; // msec
  byte         midiCh;           // 0 to 15
  byte         noteNumber;       // 0 to 127 (INVALID if there is no sensor)
} SENSOR_STATE;

SENSOR_STATE s_sensorStates[] = {
  { HIGH, 0, DEFAULT_CH, 60      }, // Pin 54 (A0)
#if 0
  { HIGH, 0, DEFAULT_CH, 61      }, // Pin 55 (A1)
  { HIGH, 0, DEFAULT_CH, 62      }, // Pin 56 (A2)
  { HIGH, 0, DEFAULT_CH, 63      }, // Pin 57 (A3)
  { HIGH, 0, DEFAULT_CH, 64      }, // Pin 58 (A4)
  { HIGH, 0, DEFAULT_CH, 65      }, // Pin 59 (A5)
  { HIGH, 0, DEFAULT_CH, 66      }, // Pin 60 (A6)
  { HIGH, 0, DEFAULT_CH, 67      }, // Pin 61 (A7)
  { HIGH, 0, DEFAULT_CH, 68      }, // Pin 62 (A8)
  { HIGH, 0, DEFAULT_CH, 69      }, // Pin 63 (A9)
  { HIGH, 0, DEFAULT_CH, 70      }, // Pin 64 (A10)
  { HIGH, 0, DEFAULT_CH, 71      }, // Pin 65 (A11)
  { HIGH, 0, DEFAULT_CH, 72      }, // Pin 66 (A12)
  { HIGH, 0, DEFAULT_CH, INVALID }, // Pin 67 (A13)
  { HIGH, 0, DEFAULT_CH, INVALID }, // Pin 68 (A14)
  { HIGH, 0, DEFAULT_CH, INVALID }, // Pin 69 (A15)
#endif
};



void setup()
{
#if 1
  // Speed up analogRead()
  ADCSRA = ADCSRA & 0xf8;
  ADCSRA = ADCSRA | 0x04;
#endif

  Serial.begin(SERIAL_SPEED);
}

void loop()
{
  for (byte i = 0; i < sizeof(s_sensorStates) / sizeof(SENSOR_STATE); i++) {
    if (s_sensorStates[i].noteNumber != INVALID) {
      unsigned int currentTime = millis();

      if (currentTime - s_sensorStates[i].valueChangedTime >= ANTICHATTERING_WAIT) {
        byte currentState = readState(i);

        if ((s_sensorStates[i].value == HIGH) && (currentState == LOW)) {
          s_sensorStates[i].value = LOW;
          s_sensorStates[i].valueChangedTime = currentTime;
          sendMIDINoteOn(s_sensorStates[i].midiCh, s_sensorStates[i].noteNumber);
        } else if ((s_sensorStates[i].value == LOW) && (currentState == HIGH)) {
          s_sensorStates[i].value = HIGH;
          s_sensorStates[i].valueChangedTime = currentTime;
          sendMIDINoteOff(s_sensorStates[i].midiCh, s_sensorStates[i].noteNumber);
        }
      }
    }
  }
}

byte readState(byte pin)
{
  int analogValue = analogRead(pin);

#if 0
  Serial.print(pin);
  Serial.print(':');
  Serial.println(analogValue);
#endif

  if (analogValue >= ANALOG_THRESHOLD) {
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
  Serial.write(0);
  Serial.flush();
}

void sendMIDIConstolChange(byte midiCh, byte value)
{
  Serial.write(0xE0 | midiCh);
  Serial.write(0x00);
  Serial.write(value);
  Serial.flush();
}
