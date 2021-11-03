#include <Arduino.h>
// #include <stdio.h>  // printf (inlcuding the output to serial port)
#include <Wire.h>  // I2C

// ATTENTION: Should be commented when compiling the slave
#define BOARD_MASTER
constexpr bool  dbgBlinking = true;  // Blink with the builtin LED for the debugging perposes
bool prompted = false;  // Whether the user input is prompted
// I2C connectivity
constexpr uint8_t I2C_SLAVE_ADDR = 0x55;  // 0x0C;  // 0x04; 0x55

enum I2C_PINS: uint8_t {
  I2C_SDA = 21,  // Standard 21 or -1
  I2C_SCL = 22  // Standard 22 or -1
};

//! Digital input pins from cameras
enum CAM_PINS: uint8_t {
  CAM1 = A13,  // P15
  CAM2 = A14,  // P13
  // CAM3 = A15,  // P12
  // CAM4 = A16,  // P14
};

//! Digital Trigger Pulse for the lighting strip activation
enum LED_PINS: uint8_t {
  LED1 = 16, // 32;  <- 5. O/P  Digital Trigger Pulse OFF:  0V  Pulse ON:  3.3V  Channel Top Strip L1
  LED2 = 17 // 27;  <- 8. O/P  Digital Trigger Pulse OFF:  0V  Pulse ON:  3.3V  Channel Bottom Strip L3U
};

// ! Input signals from the photo sensing diodes
enum PSD_PINS: uint8_t {
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

//! @brief Syncronization receiver callback
//! 
//! @param nbytes  - the number of bytes to be received
void getSyncData(int nbytes)
{
  Serial.println("getSyncData() Slave");
  // assert(nbytes == 2 && "2 bytes are expected: ledStripId, intensity");
  if(Wire.available() < 2)
    return;
// #ifdef BOARD_MASTER
//   Serial.println("getSyncData() Master");
//   const uint8_t ledStripIdMask = Wire.read();
//     //const bool intencifyAnailable = Wire.available() >= 1;
//   const uint8_t intensity = Wire.read();
// #else
  const uint8_t ledStripIdMask = Wire.read();
  //const bool intencifyAnailable = WireSlave1.available() >= 1;
  const uint8_t intensity = Wire.read();
// #endif  // BOARD_MASTER

  if(!(0b1100 & ledStripIdMask)) {
    if(Serial.availableForWrite())
      Serial.printf("WARNING: The led id mask is not controllable by the slave board: %#X\r\n", ledStripIdMask);
    return;
  }

  // Adjust LED strips lighting intensity
  if(0b0100 & ledStripIdMask)
    setIntensity(DAC1, intensity);  // 255= 3.3V 128=1.65V
  if(0b1000 & ledStripIdMask)
    setIntensity(DAC2, intensity);  // 255= 3.3V 128=1.65V

  reportLedState(ledStripIdMask, intensity, true);
  prompted = false;
}

// ATTENTION: this Arduino callback is not defined in ESP32, so it is called manually
void serialEvent() {
  //assert(Serial.available() == 2 && "2 bytes are expected: ledStripId, intensity");
  if(Serial.available() < 2)
    return;
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
#ifdef BOARD_MASTER
  if(idMaskCut) {
    // Transfer signal to the DAC1 in the Slave Board
    Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device I2C_SLAVE_ADDR
    Wire.write(idMaskCut);     // sends one byte
    Wire.write(intensity);     // sends one byte  
    const uint8_t err = Wire.endTransmission();    // stop transmitting
    Serial.printf("Transferring to wire %#X (idMask2: %#X, intensity: %#X), errCode: %#X\r\n", I2C_SLAVE_ADDR, idMaskCut, intensity, err);
  }
#endif  // BOARD_MASTER

  const uint8_t  idMaskLoc = 0b11 & ledStripIdMask;
  if(idMaskLoc)
    reportLedState(idMaskLoc, intensity, false);
  prompted = false;
}

void i2cClientScan()
{
  uint8_t  ndevs = 0;
  for(uint8_t addr = 1; addr <= 127; addr++ ) {
    Wire.beginTransmission(addr);
    const uint8_t err = Wire.endTransmission();
    if (err == 0) {
      ++ndevs;
      Serial.printf("I2C device #%u found at %#X\r\n", ndevs, addr);
    }
    // else if (err != 2)
    //   Serial.printf("Error at the I2C device %#X: %#X\r\n", addr, err);
  }
  Serial.printf("I2C clients found: %u\r\n", ndevs);
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

  // Setup input pins from the egrabber
  pinMode(CAM1, INPUT);
  pinMode(CAM2, INPUT);
  // pinMode(CAM3, INPUT);
  // pinMode(CAM4, INPUT);

  // Serial monitor setup
  Serial.begin(115200);
  // Serial.setDebugOutput(true);

  // I2C Setup
  // // Enable pull-up resistors for I2C, which is performed automatically inside Wire.begin()
  // pinMode(I2C_SDA, INPUT_PULLUP);
  // pinMode(I2C_SCL, INPUT_PULLUP);
#ifdef BOARD_MASTER
    bool success = Wire.begin(I2C_SDA, I2C_SCL); // Join i2c bus (address optional for master)
    if(success) {
      Serial.println("I2C Master started");
      i2cClientScan();
    } else Serial.println("I2C Master failed");
#else
    bool success = Wire.begin(I2C_SDA, I2C_SCL, I2C_SLAVE_ADDR);
    if (!success) {
        Serial.println("I2C slave init failed");
        while(1) delay(100);
    } else Serial.printf("I2C Slave started at %#X\r\n", I2C_SLAVE_ADDR);
    Wire.onReceive(getSyncData);
#endif  // BOARD_MASTER
  //Wire.onReceive(getSyncData); // Register Wire receive event; NOTE: that is not implemented for ESP32

// #if CONFIG_IDF_TARGET_ESP32
//   char message[64];
//   snprintf(message, 64, "%u Packets.", i++);
//   Wire.slaveWrite((uint8_t *)message, strlen(message));
// #endif

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
  Serial.println("\r\nInput the lighting intensity (<ledstrip_id: uint2_t> <intensity: uint8_t>");
  prompted = true;  // User
}

void loop()
{
  // TODO: consider master/slave initialization via serial port
  constexpr uint16_t latencyMax = 10;  // Response larency, ms; 10 ms = 100 fps
  constexpr uint16_t dtSerOutMax = 2000;  // Serial port output interval, ms
  static ulong tresp = 0;  // Response time stamp
  static ulong tSerOut = 0;  // Serial port output time stamp

#ifdef BOARD_MASTER  
  // Note: serialEvent() should be called automatically between each loop() call when a new data comes in the hardware serial RX, but that does not happen on ESP32
  if(Serial.available())
    serialEvent();
// #else
//   // WireSlave1.update();  // Required by ESP32 I2C Slave library
//   // // size_t  wireBytes = WireSlave1.available();
//   // // if(wireBytes)
//   // //   getSyncData(wireBytes);
#endif

  // Sense photodiodes
  const ulong tstamp = millis();
  uint16_t vp = analogRead(PSD1);
  if(vp != vp1 && Serial.availableForWrite()) {
    if(tstamp - tSerOut >= dtSerOutMax)
      Serial.printf("\r\nSensed lighting intensity 1: %u\r\n", vp);
    vp1 = vp;
  }
  vp = analogRead(PSD2);
  if(vp != vp2 && Serial.availableForWrite()) {
    if(tstamp - tSerOut >= dtSerOutMax)
      Serial.printf("\r\nSensed lighting intensity 2: %u\r\n", vp);
    vp2 = vp;
  }
  if(tstamp - tSerOut >= dtSerOutMax)
    tSerOut = tstamp;

  // Propmp user input
  if(!prompted && Serial.availableForWrite()) {
    Serial.println("\r\nInput the lighting intensity (<ledstrip_id: uint2_t> <intensity: uint8_t>");
    prompted = true;
  }

  if(tstamp - tresp < latencyMax) {
    delay(latencyMax - tstamp);
    tresp = tstamp;
  }

//   while(!(Serial.available()
// // #ifndef BOARD_MASTER
// //     || WireSlave1.available()
// // #endif
//   ))
//     delay(10); // Wait 10 msec = 100 fps

  // --------------------------------------------------
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
