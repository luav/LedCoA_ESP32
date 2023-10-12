# LedCoA_ESP32 - LED Strips Control Automation via 2x ESP32 D1Mini WeMos DevBoards
This application controls up to 4 LED strips from 2 synchronized ESP32 D1Mini WeMos DevBoards using 2 DACs on each board. In addition, it senses photodiodes corresponding to each of the LED strips and synchronizes LED strips blinking with the [Euresys Coaxlink Grabber (Coaxlink Quad G3, KQG13496)](https://www.euresys.com/en/Products/Frame-Grabbers/Coaxlink-series/Coaxlink-Quad-G3) that captures frames from the cameras, whose lighting is provisioned by the LED strips.

**Table of Contents**
- [Hardware documentation and drivers:](#hardware-documentation-and-drivers)
- [Build](#build)
- [Usage](#usage)
  - [Minicom for UART Reading](#minicom-for-uart-reading)
- [ESP32 Port Mapping](#esp32-port-mapping)
  - [#1 D1 Mini ESP32 WeMos](#1-d1-mini-esp32-wemos)
  - [#2 D1 Mini ESP32 WeMos](#2-d1-mini-esp32-wemos)
## Hardware documentation and drivers:
- [ESP32 tutorials](https://dronebotworkshop.com/esp32-2/)
- [Euresys Coaxlink Grabber (Coaxlink Quad G3) documentation and drivers](https://www.euresys.com/en/Support/Download-area?Series=105d06c5-6ad9-42ff-b7ce-622585ce607f)

ESP32 Pinout:
![ESP32 D1Mini WeMos DevBoard](MH-ET_LIVE_D1_mini_ESP32_pinout.jpg)

## Build

Depending on whether you build the firmware for the master or slave boards, it is necessary to (un)comment the respective macro definition at the top of `main.cpp`:
```c++
#define BOARD_MASTER  // Should be specified only for the master board build
```

Use the [PlatformIO IDE](https://platformio.org/install/ide) extension of the [Visual Studio Code IDE](https://code.visualstudio.com/) to build and flash the firmware to ESP32 D1Mini WeMos DevBoards.

## Usage

ESP32 Boards are controllable via the UART (Serial port) from the host PC, which typically features the Euresys Coaxlink Camera Grabber.  
After flashing an ESP32 board, connect their ports as specified in the sections [ESP32 Port Mapping](#esp32-port-mapping), then use minicom (see its [configuration below](#minicom-for-uart-reading)) or other application to read from the UART output of the master (or slave) board.

Connect to the master board (you can list available USB devices by `$ ls /dev/ttyUSB*`; we assume that the master board on `ttyUSB0`):
```sh
$ minicom -D /dev/ttyUSB0
Input the lighting intensity (<ledstrip_id: uint2_t> <intensity: uint8_t>
```
Open another terminal to send control signals to the master board:
```
echo -ne "\x5\xAA" > /dev/ttyUSB0
```
Then you should see in the first terminal via the minicom:
```
Transferring to wire (idMask2, intensity): 0X4 0XAA
LED strips id (mask): 0X1
Set intensity: 0XAA
```
To *exit from the minicom (which is mandatory before the reflashing)*, press `Ctrl+A X` and select `Yes`.

### Control Commands

Control commands are sent to the _ESP32 master board_ from the host PC via a serial port (USB UART), which automatically propagates them to the slave board.  
> The slave board also accepts commands via UART, but it cannot propagate them to the master board or initiate communication with the master.

Each command is represented by _2 bytes_ as follows.
```txt
Command bits: |7|6|5|4|3|2|1|0| |7|6|5|4|3|2|1|0|
```

#### Command Specification

<style>
.def td {
  text-align: center;
}

.def td:first-child {
  text-align: left;
  font-weight: bold;
}
</style>
<table id="cmd-spec" class="def">
    <thead>
        <tr>
            <th></th>
            <th colspan=8>1st byte</th>
            <!-- <th></th> -->
            <th colspan=8>2nd byte</th>
        </tr>
    </thead>
    <tbody>
        <tr style=":first-child {font-weight: bold}">
            <td>Bit</td>
            <td>7</td>
            <td>6</td>
            <td>5</td>
            <td>4</td>
            <td>3</td>
            <td>2</td>
            <td>1</td>
            <td>0</td>
            <!-- <td></td> -->
            <td>7</td>
            <td>6</td>
            <td>5</td>
            <td>4</td>
            <td>3</td>
            <td>2</td>
            <td>1</td>
            <td>0</td>
        </tr>
        <tr>
            <td>Notation</td>
            <td>-</td>
            <td colspan=3>-</td>
            <td colspan=4>mask</td>
            <!-- <td></td> -->
            <td colspan=8>value</td>
        </tr>
    </tbody>
</table>

Description:
```txt
mask (1.0-3):  required operation
  0001  - Master Board DAC1 (Top IR LED strip)
  0010  - Master Board DAC2 (Middle IR LED strip)
  0100  - Slave Board DAC2 (Middle Fluo LED strip)
  1000  - Slave Board DAC1 (Bottom Fluo LED strip)
value (2.0-7):  brightness intensity of the respective LED strip (0..0xFF -> 0.08..3.3V = 0..100%)
```

### Minicom for UART Reading
Minicom can be installed by the following command:
```sh
$ sudo apt install minicom
```
Then, select the target USB port and listen to it using minicom:
```sh
$ minicom -D /dev/ttyUSB0
```
Configure the opened connection and save it as a default configuration: press `Ctrl+A O`, select `Serial port setup` and ensure the following values:
```
	E - "115200 8N1"
	F - No
```

## ESP32 Port Mapping

\#1 D1 Mini is powered by USB (and, hence, can be externally controlled and re-flashed at any time).
\#2 D1 Mini can be powered either from your dedicated power supply (3.3V DC) or from 3.3V pin of #1 D1 Mini connected to 3.3V pin (note that VCC pin accepts 5V rather than 3.3V). The ground supply of #2 D1 Mini is GND pin located near its VCC pin.

D1 Mini boards synchronization can be performed either by I2C (which are IO22=SCL0 and IO21=SDA0 by default; it might be necessary to wire also the grounds of the boards to get I2C work), or by one of the available UARTs (Serial ports), or by any custom 3 pins (e.g., IO33,  IO27, IO32).
### #1 D1 Mini ESP32 WeMos (Master)
**2 internal DACs (IO25, IO26) for Analog O/P:**
IO25 (Master DAC1)  <-  1. O/P (Output)  Analog 0…3.3V (0.08 .. 3.3 V)  Channel Top IR Strip (L1)
IO26 (Master DAC2)  <-  2. O/P  Analog 0…3.3V (0.08 .. 3.3 V)  Channel Middle IR Strip (L2S)
**O/P  Digital Trigger Pulse OFF is performed from GPIO:**
IO16 (might be updated to IO32)  <- 5. O/P  Digital Trigger Pulse OFF:  0V  Pulse ON:  3.3V  Channel Top Strip L1
IO17 (might be updated to IO27)  <- 8. O/P  Digital Trigger Pulse OFF:  0V  Pulse ON:  3.3V  Channel Bottom Strip L3U
**I/P  Analog 0…3.3V:**
IO34 (PSD1 = VDET_1 = ADC1_6 = Arduino's ADC0)  <- 9. I/P (Input)  Analog 0…3.3V  FB Photo diode Channel Top Strip L1
IO35 (PSD1 = VDET_2 = ADC1_7 = Arduino's ADC1)  <- 12. I/P  Analog 0…3.3V  FB Photo diode Channel Bottom Strip L3U
Alternatively, IO27 (ADC2_7) can be used to read data from 2 distinct ADCs in parallel (ADC1_6 and ADC2_7 ). However, it will load CPU because only ADC1 supports DMA.

**Internal communication:**
IO21 - I2C_SDA IO/P (Input-Output) Data
IO22 - I2C_SCL IO/P Clock
GND - Mutual ground wiring for all I2C devices is required

IO1 - UART0 TXD Serial Port Transmit Data
IO3 - UART0 RXD Serial Port Receive Data
GND - Ground wiring is required for each serial device
VCC - Power (5V) wiring is required for each serial device

IO33 - GTL2 O/P firmware-yielding camera's grabber trigger pulse (0..5V, where 2.5..5V is HIGH)

### #2 D1 Mini ESP32 WeMos (Slave)
**2 internal DACs (IO25, IO26) for Analog O/P:**
IO25 (Slave DAC1)  <-  3. O/P  Analog 0…3.3V (0.08 .. 3.3 V)  Channel Bottom Fluo Strip (L3U)
IO26 (Slave DAC2)  <-  4. O/P  Analog 0…3.3V (0.08 .. 3.3 V)  Channel Middle Fluo Strip (L3S)
**O/P  Digital Trigger Pulse OFF is performed from GPIO:**
IO16 (updating to IO32)  <- 6. O/P  Digital Trigger Pulse OFF:  0V  Pulse ON:  3.3V  Channel Middle Strip L2S
IO17 (updating to IO27)  <- 7. O/P  Digital Trigger Pulse OFF:  0V  Pulse ON:  3.3V  Channel Middle Strip L3S
**I/P  Analog 0…3.3V:**
IO34 (VDET_1 = ADC1_6 = Arduino's ADC0)  <- 10. I/P  Analog 0…3.3V  FB Photo diode Channel Middle Strip L2S
IO35 (VDET_2 = ADC1_7 = Arduino's ADC1)  <- 11. I/P  Analog 0…3.3V  FB Photo diode Channel Middle Strip L3S
Alternatively, IO27 (ADC2_7) can be used to read data from 2 distinct ADCs in parallel (ADC1_6 and ADC2_7 ). However, it will load CPU because only ADC1 supports DMA.

**Internal communication:**
IO21 - I2C_SDA IO/P (Input-Output) Data
IO22 - I2C_SCL IO/P Clock
GND - Mutual ground wiring for all I2C devices is required

IO1 - UART0 TXD Serial Port Transmit Data
IO3 - UART0 RXD Serial Port Receive Data
GND - Ground wiring is required for each serial device
VCC - Power (5V) wiring is required for each serial device
