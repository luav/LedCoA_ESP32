#include <Arduino.h>
// #include <stdio.h>  // printf (inlcuding the output to serial port)
// #include <Wire.h>  // I2C

// #define LED_BUILTIN to PA8
//! Analog reading pins for ADC0,1
// enum ARD_PINS {
//   ARD00 = PA0,
//   ARD01 = PA1,
//   ARD10 = PA6,  // PB0;  NOTE: PA4 is used by DAC0
//   ARD11 = PA7,  // NOTE: PA5 is used by DAC1
// };

//! Digital input pins from cameras
enum CAM_PINS {
  CAM1 = A13,  // P15
  CAM2 = A14,  // P13
  // CAM3 = A15,  // P12
  // CAM4 = A16,  // P14
};

//! Digital Trigger Pulse for the lighting strip activation
enum LED_PINS {
  LED1 = 16, // 32;  <- 5. O/P  Digital Trigger Pulse OFF:  0V  Pulse ON:  3.3V  Channel Top Strip L1
  LED2 = 17 // 27;  <- 8. O/P  Digital Trigger Pulse OFF:  0V  Pulse ON:  3.3V  Channel Bottom Strip L3U
};

// ! Input signals from the photo sensing diodes
enum PSD_PINS {
  PSD1 = 34, // 34 (VDET_1 = ADC1_6 = Arduino's ADC0)  <- 9. I/P  Analog 0…3.3V  FB Photo diode Channel Top Strip L1
  PSD2 = 35 // 35 (VDET_2 = ADC1_7 = Arduino's ADC1)  <- 12. I/P  Analog 0…3.3V  FB Photo diode Channel Bottom Strip L3U
};

uint16_t  dbgLedCycle = 1000;  // In ms for blinking
uint16_t  dbgLedGrain = 10;  // In ms for blinking


void setup() {
  // Setup internal LED for debuging perposese
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup DAC ports (2 native ports)
  pinMode(DAC1, OUTPUT);    // P25
  pinMode(DAC2, OUTPUT);    // P26

  // Setup lighting strip activation signals
  pinMode(LED1, OUTPUT);    // LED strip activation
  pinMode(LED2, OUTPUT);    // LED strip activation

  // Setup lighting strip sensing signals
  pinMode(PSD1, INPUT);    // LED strip activation
  pinMode(PSD2, INPUT);    // LED strip activation

  // Initialise I2C communication as Master
  // Wire.begin();

  // Setup input pins from the egrabber
  pinMode(CAM1, INPUT);
  pinMode(CAM2, INPUT);
  // pinMode(CAM3, INPUT);
  // pinMode(CAM4, INPUT);

  // Serial monitor setup
  Serial.begin(115200);

  // Perform lighting strips activation
  delay(20); // Wait 20 msec
  // Set max brightness for the LED strips
  dacWrite(DAC1, 255);  // 255= 3.3V 128=1.65V
  dacWrite(DAC2, 255);
  // Perform lighting strips activation
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  // Activate builtin LED
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  if(!Serial.availableForWrite())
    return;
  Serial.println("Input the lighting intensity (<ledstrip_id: uint2_t> <intensity: uint8_t>");

  // Identify the target LED strip id to be adjusted
  uint8_t  ledStripId = 0;  // 255
  while(!Serial.available())
    delay(100);  // Wait 100 ms
  Serial.read(&ledStripId, sizeof ledStripId);
  if(Serial.availableForWrite()) {
    Serial.print("Adjusting #LED: ");
    Serial.println(ledStripId);
  }
  // Blink the number of times equal to the selected LED ID
  for(uint16_t i = 0; i < ledStripId; ++i) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(20);  // Wait 20 ms
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // Identify the target intesity of the required LED strip
  uint8_t  intensity = -1;  // 255
  while(!Serial.available())
    delay(100);  // Wait 100 ms
  Serial.read(&intensity, sizeof intensity);
  if(Serial.availableForWrite()) {
    Serial.print("Set intensity: ");
    Serial.println(intensity);
  }
  // Use internal LED to visualize the target intensify
  digitalWrite(LED_BUILTIN, LOW);
  uint16_t  dbgLedIters = dbgLedCycle / dbgLedGrain;
  uint8_t intensHigh10 = 1 + (intensity / 256.f) * 10;
  for(uint16_t i = 0; i < dbgLedIters; ++i) {
    if ((i%10 + 1) * intensHigh10 >= 10)
      digitalWrite(LED_BUILTIN, HIGH);
    else digitalWrite(LED_BUILTIN, LOW);
    delay(dbgLedGrain);
  }

  switch (ledStripId) {
  case 1:
    dacWrite(DAC1, intensity);  // 255= 3.3V 128=1.65V
    break;
  case 2:
    dacWrite(DAC1, intensity);  // 255= 3.3V 128=1.65V
    break;
  case 3:
    // TODO: Transfer signal to the DAC1 in the Slave Board
    break;
  case 4:
    // TODO: Transfer signal to the DAC2 in the Slave Board
    break;
  default:
    break;
  }

  // Read signals from the grabber
  // int cam1 = digitalRead(CAM1);
  // int cam2 = digitalRead(CAM2);

  // // Activate respective lighting on camera signal
  // if(cam1)
  //   dacWrite(DAC1, bright1);
  // if(cam2)
  //   dacWrite(DAC2, bright2);

  // // // Demo of the internal DACs control
  // // analogWriteResolution(8); // 8 bit resolution
  // int imax = 256;
  // for(int i = 0; i < imax; ++i) {
  //   dacWrite(DAC1, i);  // 255= 3.3V 128=1.65V
  //   dacWrite(DAC2, imax - 1 - i);
  //   // analogWrite(DAC1, i); // output values of varible b at the Digial to analog converter
  //   delay(8); // Wait 8 ms; 8*255 != 2 sec
  //   // Serial.println (b);
  // }
}


// // External DACs via I2S (adafruit-i2s-stereo-decoder-uda1334a.pdf):
// // PIN mapping:
// RXD	[CLK]	  	SCK	(WSEL/LRCLK) (like in adafruit-i2s-stereo-decoder-uda1334a.pdf)
// TXD [TD0 TCK]	BCK	[NOTE: I mixed TXD/RXD when ask Amrit to connect wires]
// [SD0] IO23		  DIN	(IO23 according to sp32_technical_reference_manual_en.pdf)

// #include <I2S.h>

// #define FREQUENCY 440  // frequency of sine wave in Hz
// #define AMPLITUDE 10000  // amplitude of sine wave
// #define SAMPLERATE 44100  // sample rate in Hz

// int16_t sinetable[SAMPLERATE / FREQUENCY];
// uint32_t sample = 0;

// #define PI 3.14159265

// void setup() {
//   Serial.begin(115200);
//   Serial.println("I2S sine wave tone");
//   // start I2S at the sample rate with 16-bits per sample
//   if (!I2S.begin(I2S_PHILIPS_MODE, SAMPLERATE, 16)) {  // Philips Standard, MSB Alignment Standard, and PCM Standard are supported by ESP32: esp32_technical_reference_manual_en.pdf
//     Serial.println("Failed to initialize I2S!");
//     while (1); // do nothing
//   }
//   // fill in sine wave table
//   for (uint16_t s=0; s < (SAMPLERATE / FREQUENCY); s++) {
//    sinetable[s] = sin(2.0 * PI * s / (SAMPLERATE/FREQUENCY)) * AMPLITUDE;
//   }
// }

// void loop() {
//   if (sample == (SAMPLERATE / FREQUENCY)) {
//     sample = 0;
//   }
//   // write the same sample twice, once for left and once for the right channel
//   I2S.write((int16_t) sinetable[sample]); // We'll just have same tone on both!
//   I2S.write((int16_t) sinetable[sample]);
//   // increment the counter for the next sample in the sine wave table
//   sample++;
// }
