#include <Arduino.h>
#include <Wire.h>  // I2C
#include <stdio.h>  // sprintf
#include <stdarg.h>  // var args

//////////////////////////////////////////////////////////////
// These definitions should be adjusted before the compilation

// ATTENTION: BOARD_MASTER should be commented when compiling for the slave
#define BOARD_MASTER

constexpr bool  dbgBlinking = true;  // Blink with the builtin LED for the debugging purposes, which significantly delays the execution cycle
constexpr bool  prompting = true;  // Prompting the user input and tracing the command execution to the serial port, which slightly delays the execution cycle. Enabled only on logging level >= info
//////////////////////////////////////////////////////////////

// I2C connectivity -----------------------------------
constexpr uint8_t I2C_SLAVE_ADDR = 0x55;  // 0x04; 0x55

// ESP32 clock sources: MAX freq for SCL = 4 MHz (https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html#id4)
// The default I2C bus clock speed is 100 kHz (100000). At that rate, the ESP32-S2 will leave *10ms* gaps between I2C transactions, which can be solved Raising the I2C bus frequency to 125 kHz (125000) or higher. (https://learn.adafruit.com/adafruit-esp32-s2-feather/esp32-s2-bugs-and-limitations)
// TX I2C: The maximum clock frequency (fSCL (max)) is specified to be up to 400 kHz for I2C FM and up to 1000 kHz for FM+ spec (https://www.nxp.com/docs/en/brochure/75015687.pdf, https://www.ti.com/lit/an/slva695/slva695.pdf, )
// NXP I2C: The I2C-bus clock frequency is 0 to 1 MHz,
constexpr uint32_t I2C_FREQ_HZ = 400000;  // 400 KHz; 125000 is 125 KHz

// Pin definition -------------------------------------
enum I2C_PINS: uint8_t {
	I2C_SDA = 21,  // Standard 21 or -1
	I2C_SCL = 22  // Standard 22 or -1
};

// Note: the grabber does not initiate communication with the ESP32 boards, it only receives TTL1/2 signals
// //! Digital input pins from cameras
// enum CAM_PINS: uint8_t {
// 	CAM1 = A13,  // P15
// 	CAM2 = A14,  // P13
// 	// CAM3 = A15,  // P12
// 	// CAM4 = A16,  // P14
// };

//! Digital Trigger Pulse for the lighting strip activation
enum LED_PINS: uint8_t {
	LED1 = 16, // 32;  <- 5. O/P  Digital Trigger Pulse OFF:  0V  Pulse ON:  3.3V  Channel Top Strip L1
	LED2 = 17 // 27;  <- 8. O/P  Digital Trigger Pulse OFF:  0V  Pulse ON:  3.3V  Channel Bottom Strip L3U
};

//! Input signals from the photo sensing diodes
enum PSD_PINS: uint8_t {
	PSD1 = 34, // 34 (VDET_1 = ADC1_6 = Arduino's ADC0)  <- 9. I/P  Analog 0…3.3V  FB Photo diode Channel Top Strip L1
	PSD2 = 35 // 35 (VDET_2 = ADC1_7 = Arduino's ADC1)  <- 12. I/P  Analog 0…3.3V  FB Photo diode Channel Bottom Strip L3U
};

//! Pins for the grabber control via TTL signals
enum GRAB_PINS: uint8_t {
	// Related ports: TXD is GPIO1 (empty), RXD is GPIO3 (empty); IO27 (wired) and IO32 (wired) or IO17 (wired) and IO16 (wired) are used via or gate to trigger GTL1
	//! Grabber TTL1 is generated by the OR gate upon the LED shutting down
	GTL2 = 33  //! Grabber TTL2 is bound to the master board IO33
};

// Execution parameters and definitions ---------------
enum LOG_LEV: uint8_t {
	// NONE,
	ERR,
	WRN,
	INF,
	DBG
};

LOG_LEV  logLev = INFO;


uint16_t  dbgLedCycle = 1000;  // In ms for blinking
uint16_t  dbgLedGrain = 4;  // In ms for blinking; 25 fps = 4 ms
bool led1On = false, led2On = false;  // Whether LEDx (target lighting) is on
uint16_t  vp1 = 0xACAC, vp2 = 0xACAC;  // Photodioudes sensing: some stable initial value, which is unlikely to occur
uint16_t camTrigCycle = 200;  // Camera triggering cycle in ms; 200 ms = 5 FPS
uint16_t ledBlinkCycle = camTrigCycle;  // Camera triggering cycle in ms; 200 ms = 5 FPS
// uint16_t ledSigDur[] = {ledBlinkCycle, ledBlinkCycle};  // Camera triggering cycle in ms; 200 ms = 5 FPS
uint16_t durLed1 = ledBlinkCycle, durLed2 = ledBlinkCycle;  // Duration of the LED lighting (0 .. ledBlinkCycle)

bool promted = false;  // A flag, indicating prompt for the user input
// bool isMaster = true;  // Whether the board works as a master


// Accessory functions --------------------------------------------------------

//! @brief Logs messages with arguments, cutting log entries to 255 bytes
//! @param nonblocking  - whether the call is required to be non-blocking, then omit the blocking messages
//! @param provenance  - include the internal time counter and code line number as an output prefix
//! @param lev  - target log level
//! @param msg  - message without the terminating linefeed, which is appended automatically
//! @param  ...args  - message arguments
void log(bool nonblocking, bool provenance, LOG_LEV lev, const char* msg, ...)
{
	if(!Serial || lev > logLev)  // Whether the Serial connection is open
		return;

	// Capacity of the Serial port buffer to perform non-blocking writing
	uint serCap = Serial.availableForWrite();
	if(nonblocking && serCap <= 2) // Consider ending \r\n
		return;
	serCap -= 2;  // -2 to consider ending \r\n

	char buf[256] = "";
	char* rem = buf;  // Reminder
	uint16_t  msgLen = 0;  // Message length

	if(provenance && (!nonblocking || serCap >= 18)) {
		const char *levStr = "";
		switch(lev) {
			ERR:
				levStr = "ERR";
				break;
			WRN:
				levStr = "WRN";
				break;
			INF:
				levStr = "INF";
				break;
			DBG:
				levStr = "DBG";
				break;
		}

		const ulong tcur = millis();  // The number of milliseconds passed since the boot, ~< 50 days
		snprintf(buf, sizeof buf, "%s %c, %d ln.%d: ",
#ifdef BOARD_MASTER
			'M'
#else
			'S'
#endif
			levStr, tcur, __LINE__);
		msgLen = strlen(buf);
		if(serCap <= msgLen) {  // This is only the message prefix without the body
			if(nonblocking)
				return;
			serCap = 0;
		} else serCap -= msgLen;
		rem += msgLen;
	}

	va_list args;
	va_start(args, msg);
	vsnprintf(rem, sizeof buf - msgLen, msg, args);
	va_end(args);
	msgLen += strlen(rem);
	if(nonblocking && serCap < msgLen)
		return;
	Serial.println(buf);  // Appends \r\n to the outputting message
}


//! @brief Yields string value of a role (master/slave)
//! 
//! @return role: Master or Slave
const char* boardRole()
{
	return // isMaster ? "Master" : "Slave";
#ifdef BOARD_MASTER
		"Master"
#else
		"Slave"
#endif
		;
}

//! @brief Report LED Strips State Change to UART (Serial Port) and via the builtin LED
//! 
//! @param ledStripIdMask 
//! @param intensity 
//! @param wire  whether reporting for the direct (serial port) or wire (I2C) signal
void reportLedState(uint8_t ledStripIdMask, uint8_t intensity, bool wire)
{
	// Report ledStripIdMask
	// availableForWrite - Get the number of bytes (characters) available for writing in the serial buffer without blocking the write operation.
	if(Serial.availableForWrite()) {
		if(wire)
			Serial.print("Wire; ");
		Serial.printf("LED strips id (mask): %#X\r\n", ledStripIdMask);
	}
	// Blink the number of times equal to the selected LED ID
	if(dbgBlinking) {
		for(uint16_t i = 0; i < ledStripIdMask; ++i) {
			digitalWrite(LED_BUILTIN, LOW);
			delay(dbgLedGrain * 2);  // Wait 2x LedGrain duration
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
			digitalWrite(ledPin, HIGH);  // Activate lighting strip, activating OR gate => Grabber TTL1
			ledOn = true;
			// TODO: notify user abut the activated LED strip to map Grabber TTL1 to the appropriate camera
			// ? digitalWrite(ledGatePin, HIGH);
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
	if(prompting)
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
	//const bool intencifyAnailable = Wire.available() >= 1;
	const uint8_t intensity = Wire.read();
// #endif  // BOARD_MASTER

	if(!(0b1100 & ledStripIdMask)) {
		if(prompting && Serial.availableForWrite())
			Serial.printf("WARNING: The led id mask is not controllable by the slave board: %#X\r\n", ledStripIdMask);
		return;
	}

	// Adjust LED strips lighting intensity
	if(0b0100 & ledStripIdMask)
		setIntensity(DAC1, intensity);  // 255= 3.3V 128=1.65V
	if(0b1000 & ledStripIdMask)
		setIntensity(DAC2, intensity);  // 255= 3.3V 128=1.65V

	if(prompting) {
		reportLedState(ledStripIdMask, intensity, true);
		promted = false;
	}
}

//! @brief Handle control signals from the serial port
//! Note: serialEvent() is called automatically between each loop() call when a new data comes in the hardware serial RX
void serialEvent() {
	// TODO: ensure that this is a master
	//assert(Serial.available() == 2 && "2 bytes are expected: ledStripId, intensity");
	if(Serial.available() < 2)
		return;
	// First byte defines the command, where high 4 bits define the control type.
	uint8_t  ctlCmd = 0;  // 255
	// while(!Serial.available())
	//   delay(100);  // Wait 100 ms
	Serial.read(&ctlCmd, sizeof ctlCmd);
	uint8_t cmdVal = -1;  // 255
	Serial.read(&cmdVal, sizeof cmdVal);
#ifdef BOARD_MASTER
	if(ctlCmd & 0x80) {
		camTrigCycle = cmdVal + (uint16_t(ctlCmd & 0x7F) << 8);
		Serial.printf("Updated camTrigCycle: %d ms\r\n", camTrigCycle);
		return;
	}
#endif

	uint8_t &ledStripIdMask = ctlCmd;  // 255
	uint8_t &intensity = cmdVal;  // 255

	// Adjust LED strips lighting intensity
	// Note: for the intensity 0 some signal is still present on the DAC, so the lighting should be turned of by the trigger signal
	if(0b0001 & ledStripIdMask)
		setIntensity(DAC1, intensity);

	if(0b0010 & ledStripIdMask)
		setIntensity(DAC2, intensity);

	const uint8_t  idMaskCut = 0b1100 & ledStripIdMask;
#ifdef BOARD_MASTER
	if(idMaskCut) {
		// WirePacker packer;
		// packer.write(idMaskCut);
		// packer.write(intensity);
		// packer.end();
		uint8_t data[] = {idMaskCut, intensity};

		// Transfer signal to the DAC1 in the Slave Board
		Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device I2C_SLAVE_ADDR
		// while (packer.available())    // write every packet byte
		//   Wire.write(packer.read());
		Wire.write(data, sizeof data);
		// Wire.write(idMaskCut);     // sends one byte
		// Wire.write(intensity);     // sends one byte  
		const uint8_t err = Wire.endTransmission();    // stop transmitting
// 0: success.
// 1: data too long to fit in transmit buffer.
// 2: received NACK on transmit of address.
// 3: received NACK on transmit of data.
// 4: other error.
// 5: timeout

		Serial.printf("Transferring to wire %#X (idMask2: %#X, intensity: %#X), errCode: %#X\r\n", I2C_SLAVE_ADDR, idMaskCut, intensity, err);
	}
#endif  // BOARD_MASTER

	const uint8_t  idMaskLoc = 0b11 & ledStripIdMask;
	if(idMaskLoc)
		reportLedState(idMaskLoc, intensity, false);
	promted = false;
}


//! @brief Scan for the connected I2C devices
void i2cClientScan()
{
	for(uint8_t addr = 1; addr <= 127; addr++ ) {
		Wire.beginTransmission(addr);
		const uint8_t err = Wire.endTransmission();
		if (err == 0)
			Serial.printf("I2C device found at %#X\r\n", addr);
		else if (err != 2)
			Serial.printf("Error at the I2C device %#X: %#X\r\n", addr, err);
	}
}

void onWireReceive(int len) {
	if(!Serial)
		return;
	Serial.printf("%s onReceive[%d]: ", boardRole(), len);
	while(Wire.available()) {
		Serial.printf("%#X ", Wire.read());
		// Serial.write(Wire.read());
	}
	Serial.println();
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
	Serial.setDebugOutput(true);  // Required on ESP to enable output from printf() function

	// I2C Setup
	delay(3000);  // Required to delay initialization and check it on reflashing

	// // Dynamic identification of the boar role (master/slave), initializing as a slave if slaves are not present
	// // Start as a master and check whether a slave is connected, becoming the slave if necessary
	// bool success = Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ); // Join i2c bus (address optional for master)
	// if(success) {
	//   Serial.println("I2C Master initialized");
	//   // Ensure that the slave is connected, otherwise reinitialize as a slave
	//   Wire.beginTransmission(I2C_SLAVE_ADDR);
	//   const uint8_t err = Wire.endTransmission();
	//   // err values:
	//   // https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
	//   // endTransmission() returns:
	//   // 0: success.
	//   // 1: data too long to fit in transmit buffer.
	//   // 2: received NACK on transmit of address.
	//   // 3: received NACK on transmit of data.
	//   // 4: other error.
	//   // 5: timeout.
	//   Serial.printf("Test transfer code: %d\r\n", err);
	//   if(err == 5) {  // 2 NACK (slave exists but not responding), 5 timeout (no answer)
	//     // Slave initialization
	//     Serial.printf("There are no slaves connected, err: %d. Reinitializing as a slave\r\n", err);
	//     if(Wire.end())
	//       isMaster = false;
	//       // Master finalization is completed. perform slave initialization
	//     else Serial.println("Cannot finalize the master mode");
	//   } else {
	//     // Complete master initialization
	//     // Setup Grabber communication signals
	//     pinMode(GTL2, OUTPUT);    // Grabber TTL 2
	//     if (err == 0) {
	//       Serial.printf("I2C slave is found at %#X\r\n", I2C_SLAVE_ADDR);
	//     } else if (err != 2)  // 2: received NACK on transmit of address, 5: timeout
	//       Serial.printf("Error at the I2C device %#X: %d\r\n", I2C_SLAVE_ADDR, err);
	//   }
	// } else {
	//   isMaster = false;
	//   Serial.println("I2C Master initialization failed, switching to the slave mode");
	// }
	//
	// if(!isMaster) {
	//   // Perform slave initialization
	//   Serial.println("Starting I2C slave initialization");
	//   success = Wire.begin(I2C_SLAVE_ADDR, I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
	//   if (success) {
	//       // Wire.onReceive(getSyncData); // Register the Wire data receipt callback before starting the slave
	//       Wire.onReceive(onWireReceive);  // Register the Wire data receipt callback before starting the slave
	//       Serial.printf("I2C Slave started at %#X\r\n", I2C_SLAVE_ADDR);
	//   } else Serial.println("I2C slave init failed");
	//
	//   // Pre-write to the slave response buffer. This is used only for the ESP32 (not necessary for ESP32-S2 and ESP32-C3) in order to add the slave capability on the chip
	//   #if CONFIG_IDF_TARGET_ESP32
	//     // char message[64];
	//     // snprintf(message, 64, "%u Packets.", 0);
	//     Wire.slaveWrite((uint8_t *)"\0\0", 2);
	//   #endif
	// }

	// Static initialization of the board role (master/slave) based on the macrodefinition BOARD_MASTER
	//// Enable pull-up resistors for I2C, which is performed automatically inside Wire.begin()
	// pinMode(I2C_SDA, INPUT_PULLUP);
	// pinMode(I2C_SCL, INPUT_PULLUP);
#ifdef BOARD_MASTER
		bool success = Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ); // Join i2c bus (address optional for master)
		if(Serial)  // Whether the Serial connection is open
			if(success) {
				Serial.println("I2C Master started");
				// i2cClientScan();
			} else Serial.println("I2C Master initialization failed");
	// Setup Grabber communication signals
	pinMode(GTL2, OUTPUT);    // Grabber TTL 2
#else
		// Wire.onReceive(getSyncData); // Register the Wire data receipt callback before starting the slave
		Wire.onReceive(onWireReceive);  // Register the Wire data receipt callback before starting the slave
		bool success = Wire.begin(I2C_SLAVE_ADDR, I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
		if (!success) {
				Serial.println("I2C slave init failed");
		} else Serial.printf("I2C Slave started at %#X\r\n", I2C_SLAVE_ADDR);
#endif  // BOARD_MASTER

	// Perform lighting strips activation
	delay(1); // Wait 1 ms
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
	Serial.println("\r\nInput the control command (<ledstrip_idmask: uint4_t> <intensity: uint8_t>");

	Serial.println("\r\nInput the lighting intensity (<ledstrip_id: uint2_t> <intensity: uint8_t>");
	promted = true;  // User
}

constexpr uint16_t  lightSensCycle = 2000;  // Light sensing output trigger, ms
constexpr uint16_t  lsTrig = 2000;  // Light sensing output trigger, ms
uint16_t  lsTime = 0;  // Light sensing counter to reduce output cluttering

// void loop()
// {
//   Serial.println("Scanning for the connected devices\r\n");
//   i2cClientScan();
//   delay(1000); // Wait 1 sec
// }

bool sent = false;
void loop()
{
	// delay(1); // Wait 1 ms
	delayMicroseconds(100);  // 0.1 ms ~ half of a delay of the control signal transfer

	if(!sent) {
		delay(3000);
		// Note: beginTransmission works only for the Master
		if(isMaster) {
			Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device I2C_SLAVE_ADDR
			// while (packer.available())    // write every packet byte
			//   Wire.write(packer.read());
			Wire.write("hi");
			// Wire.write(idMaskCut);     // sends one byte
			// Wire.write(intensity);     // sends one byte  
			const uint8_t err = Wire.endTransmission();    // stop transmitting
			if(err) {
				Serial.printf("Wire transfer from %s to %#X failed: %d\r\n", boardRole(), I2C_SLAVE_ADDR, err);
			} else sent = true;
		} else Wire.slaveWrite((uint8_t *)"hi", 2);
	}

// 0: success.
// 1: data too long to fit in transmit buffer.
// 2: received NACK on transmit of address.
// 3: received NACK on transmit of data.
// 4: other error.
// 5: timeout


//   // TODO: consider master/slave initialization via serial port
//   static ulong  tstamp = millis();

// #ifndef BOARD_MASTER
//   delay(20); // Wait 20 ms
//   ulong  tcur = millis();
//   if(tcur - tstamp >= 1000) {
//     tstamp = tcur;
//     Serial.printf("Listening for the master commands (w av: %d, w1 av: %d)\r\n", Wire.available(), Wire.available());
//   }
//   return;
// #endif  // BOARD_MASTER

// //   // Note: serialEvent() should be called automatically between each loop() call when a new data comes in the hardware serial RX, but that does not happen on ESP32
// // #ifdef BOARD_MASTER  
// //   if(Serial.available())
// //     serialEvent();
// // #else
// //   Wire.update();  // Required by ESP32 I2C Slave library
// //   // size_t  wireBytes = Wire.available();
// //   // if(wireBytes)
// //   //   getSyncData(wireBytes);
// // #endif

//   // Sense photodiodes
//   lsTime = 1;
//   uint16_t vp = analogRead(PSD1);
//   if(vp != vp1 && Serial.availableForWrite()) {
//     if(lsCtr >= lsTrig)
//       Serial.printf("\r\nSensed lighting intensity 1: %u\r\n", vp);
//     vp1 = vp;
//   }
//   vp = analogRead(PSD2);
//   if(vp != vp2 && Serial.availableForWrite()) {
//     if(lsCtr >= lsTrig)
//       Serial.printf("\r\nSensed lighting intensity 2: %u\r\n", vp);
//     vp2 = vp;
//   }
//   if(lsCtr >= lsTrig)
//     lsCtr = 0;

//   // Propmp user input
//   if(prompting && !promted && Serial.availableForWrite()) {
//     Serial.println("\r\nInput the lighting intensity (<ledstrip_id: uint2_t> <intensity: uint8_t>");
//     promted = true;
//   }

//   while(!(Serial.available()
// // #ifndef BOARD_MASTER
// //     || Wire.available()
// // #endif
//   )) {
//     delay(10); // Wait 10 msec = 100 fps

// #ifdef BOARD_MASTER
//      // Emit triggering event each camTrigCycle ms
//     ulong tcur = millis();
//     const ulong dur = 1;  // Duration of the grabber TTL2 signal: 0-1 ms
//     if(tcur - tstamp >= (ulong)camTrigCycle) {  // ~ 33 fps;  25 fps = 1/25 = 40 ms
//       digitalWrite(GTL2, HIGH);
//       delay(dur);  // Delay 1 or 0 ms, which is the duration of the signal
//       digitalWrite(GTL2, LOW);
//       tstamp = tcur + dur;
//     }
// #endif
//   }

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
