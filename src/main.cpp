#include <Arduino.h>
// #include <stdio.h>  // printf (inlcuding the output to serial port)
#include <Wire.h>  // I2C

// // ATTENTION: Should be commented when compiling the slave
// #define BOARD_MASTER
constexpr bool  dbgBlinking = true;  // Blink with the builtin LED for the debugging perposes
bool prompted = false;  // Whether the user input is prompted

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
uint16_t  dbgLedGrain = 4;  // In ms for blinking; 25 fps = 4 ms
bool led1On = false, led2On = false;  // Whether LEDN is on
uint16_t  vp1 = 0xACAC, vp2 = 0xACAC;  // Photodioudes sensing: some stable initial value, which is unlikely to occur

//! @brief Report LED Strips State Change to UART (Serial Port) and via the builtin LED
//! 
//! @param ledStripIdMask 
//! @param intensity 
void reportLedState(uint8_t ledStripIdMask, uint8_t intensity, bool wire)
{
  // Report ledStripIdMask
  if(Serial.availableForWrite()) {
    if(wire)
      Serial.print("Wire; ");
    Serial.printf("LED strips id (mask): %#X\r\n", ledStripIdMask);
  }
  // Blink the number of times equal to the selected LED ID
  if(dbgBlinking) {
    for(uint16_t i = 0; i < ledStripIdMask; ++i) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(dbgLedGrain * 2);  // Wait 20 ms
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  // Report intensity
  if(Serial.availableForWrite()) {
    if(wire)
      Serial.print("Wire; ");
    Serial.printf("Set intensity: %#X\r\n", intensity);
  }
  // Use internal LED to visualize the target intensify
  if(dbgBlinking) {
    digitalWrite(LED_BUILTIN, LOW);
    uint16_t  dbgLedIters = dbgLedCycle / dbgLedGrain;
    const uint8_t  dbgLedDimCycle = 10;
    uint8_t intensHighDim = intensity / 255.f * dbgLedDimCycle;
    for(uint16_t i = 0; i < dbgLedIters; ++i) {
      if ((i%dbgLedDimCycle + 1) * intensHighDim >= dbgLedDimCycle)
        digitalWrite(LED_BUILTIN, HIGH);
      else digitalWrite(LED_BUILTIN, LOW);
      delay(dbgLedGrain);
    }
  }
}

//! @brief Syncronization receiver callback
//! 
//! @param nbytes  - the number of bytes to be received
void getSyncData(int nbytes)
{
  assert(nbytes == 2 && "2 bytes are expected: ledStripId, intensity");
  const uint8_t ledStripIdMask = Wire.read();
  const uint8_t intensity = Wire.read();

  if(!(0b1100 & ledStripIdMask)) {
    if(Serial.availableForWrite())
      Serial.printf("WARNING: The led id mask is not controllable by the slave board: %#X\n", ledStripIdMask);
    return;
  }

  // Adjust LED strips lighting intensity
  if(0b0100 & ledStripIdMask)
    dacWrite(DAC1, intensity);  // 255= 3.3V 128=1.65V
  if(0b1000 & ledStripIdMask)
    dacWrite(DAC2, intensity);  // 255= 3.3V 128=1.65V

  reportLedState(ledStripIdMask, intensity, true);
  prompted = false;
}

//! @brief Set the required intensity for the DAC/LedPin, eliminating the residual lighting
//! 
//! @param dac  - target DAC
//! @param uint8_t  - LED intensity
void setIntensity(uint8_t dac, uint8_t intensity) {
  LED_PINS ledPin = dac == DAC1 ? LED1 : LED2;
  bool &ledOn = ledPin == LED1 ? led1On : led2On;
  if(intensity) {
    if(!ledOn) {
      digitalWrite(ledPin, HIGH);  // Activate lighting strip
      ledOn = true;
    }
    dacWrite(dac, intensity);  // 255= 3.3V 128=1.65V
  } else if(ledOn) {
    digitalWrite(ledPin, LOW);  // Deactivate lighting strip to prevent residual lighting
    ledOn = false;
  }
}

// ATTENTION: this Arduino callback is not defined in ESP32, so it is called manually
void serialEvent() {
  assert(Serial.available() == 2 && "2 bytes are expected: ledStripId, intensity");
  uint8_t  ledStripIdMask = 0;  // 255
  // while(!Serial.available())
  //   delay(100);  // Wait 100 ms
  Serial.read(&ledStripIdMask, sizeof ledStripIdMask);
  uint8_t intensity = -1;  // 255
  Serial.read(&intensity, sizeof intensity);

  // Adjust LED strips lighting intensity
  // Note: for the intensity 0 some signal is still present on the DAC, so the lighting should be turned of by the trigger signal
  if(0b0001 & ledStripIdMask)
    setIntensity(DAC1, intensity);

  if(0b0010 & ledStripIdMask)
    setIntensity(DAC2, intensity);

  const uint8_t  idMaskCut = 0b1100 & ledStripIdMask;
  if(idMaskCut) {
    // Transfer signal to the DAC1 in the Slave Board
    Wire.beginTransmission(0); // transmit to device #0
    Wire.write(idMaskCut);     // sends five bytes
    Wire.write(intensity);     // sends one byte  
    Wire.endTransmission();    // stop transmitting
    Serial.printf("Transferring to wire (idMask2, intensity): %#X %#X\r\n", idMaskCut, intensity);
  }

  const uint8_t  idMaskLoc = 0b11 & ledStripIdMask;
  if(idMaskLoc)
    reportLedState(idMaskLoc, intensity, false);
  prompted = false;
}

void setup()
{
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

  // I2C Setup
  Wire.begin(); // join i2c bus (address optional for master)
  //Wire.onReceive(getSyncData); // Register Wire receive event; NOTE: that is not implemented for ESP32

  // Perform lighting strips activation
  delay(20); // Wait 20 msec
  // Set max brightness for the LED strips
  dacWrite(DAC1, 255);  // 255= 3.3V 128=1.65V
  dacWrite(DAC2, 255);
  // Perform lighting strips activation
  digitalWrite(LED1, HIGH);
  led1On = true;
  digitalWrite(LED2, HIGH);
  led2On = true;
  // Activate builtin LED
  if(dbgBlinking)
    digitalWrite(LED_BUILTIN, HIGH);

  // Prompt user input
  Serial.println("\nInput the lighting intensity (<ledstrip_id: uint2_t> <intensity: uint8_t>");
  prompted = true;  // User
}

void loop()
{
  // TODO: consider master/slave initialization via serial port

  if(Serial.available())
    serialEvent();

  size_t  wireBytes = Wire.available();
  if(wireBytes)
    getSyncData(wireBytes);

  // Sense photodiodes
  uint16_t vp = analogRead(PSD1);
  if(vp != vp1 && Serial.availableForWrite()) {
    Serial.printf("\nSensed lighting intensity 1: %u\n", vp);
    vp1 = vp;
  }
  vp = analogRead(PSD2);
  if(vp != vp2 && Serial.availableForWrite()) {
    Serial.printf("\nSensed lighting intensity 1: %u\n", vp);
    vp2 = vp;
  }

  // Propmp user input
  if(!prompted && Serial.availableForWrite()) {
    Serial.println("\nInput the lighting intensity (<ledstrip_id: uint2_t> <intensity: uint8_t>");
    prompted = true;
  }

  while(!(Serial.available() || Wire.available()))
    delay(10); // Wait 10 msec = 100 fps

  // // Identify the target LED strip id to be adjusted
  // uint8_t  ledStripId = 0;  // 255
  // // while(!Serial.available())
  // //   delay(100);  // Wait 100 ms
  // Serial.read(&ledStripId, sizeof ledStripId);

  // // Identify the target intesity of the required LED strip
  // uint8_t  intensity = -1;  // 255
  // // while(!Serial.available())
  // //   delay(100);  // Wait 100 ms
  // Serial.read(&intensity, sizeof intensity);

  // // Report ledStripId
  // if(Serial.availableForWrite()) {
  //   Serial.print("Adjusting #LED: ");
  //   Serial.println(ledStripId);
  // }
  // // Blink the number of times equal to the selected LED ID
  // for(uint16_t i = 0; i < ledStripId; ++i) {
  //   digitalWrite(LED_BUILTIN, LOW);
  //   delay(20);  // Wait 20 ms
  //   digitalWrite(LED_BUILTIN, HIGH);
  // }

  // // Report ledStripId
  // if(Serial.availableForWrite()) {
  //   Serial.print("Set intensity: ");
  //   Serial.println(intensity);
  // }
  // // Use internal LED to visualize the target intensify
  // digitalWrite(LED_BUILTIN, LOW);
  // uint16_t  dbgLedIters = dbgLedCycle / dbgLedGrain;
  // uint8_t intensHigh10 = 1 + (intensity / 256.f) * 10;
  // for(uint16_t i = 0; i < dbgLedIters; ++i) {
  //   if ((i%10 + 1) * intensHigh10 >= 10)
  //     digitalWrite(LED_BUILTIN, HIGH);
  //   else digitalWrite(LED_BUILTIN, LOW);
  //   delay(dbgLedGrain);
  // }

  // switch (ledStripId) {
  // case 1:
  //   dacWrite(DAC1, intensity);  // 255= 3.3V 128=1.65V
  //   break;
  // case 2:
  //   dacWrite(DAC1, intensity);  // 255= 3.3V 128=1.65V
  //   break;
  // case 3:
  // case 4:
  //   // Transfer signal to the DAC1 in the Slave Board
  //   Wire.beginTransmission(0); // transmit to device #0
  //   Wire.write(ledStripId);        // sends five bytes
  //   Wire.write(intensity);              // sends one byte  
  //   Wire.endTransmission();    // stop transmitting
  //   break;
  // default:
  //   break;
  // }

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
