
# RTES2018 - Intrusion Detection System

This Github page contains the codebase and documentation for the project "Intrusion Detection System" which uses 
- a [PIR sensor board](http://wiki.seeedstudio.com/Grove-PIR_Motion_Sensor/), 
- an [MPU-9250](http://wiki.seeedstudio.com/Grove-IMU_10DOF/) sensor board with a 3-axis accelerometer, 
- an [Adafruit Feather M0 RFM95 LoRa Radio (900MHz)](https://www.adafruit.com/product/3178) board 

to detect motion and transmit an 'intrusion detected' packet to the [TTN cloud service](http://console.thethingsnetwork.org/). 
A small Python script is also available on this repository for you to fetch data from the TTN server to your computer.

Click [here](https://github.com/v2h/RTES2018/blob/master/Documentation/IntrusionDetectionSystem.md) for a complete report of this project. If you just want to view the tutorial, continue down below.

## Tutorial

- [1 Development Environment Setup](#1-development-environment-setup)
  * [1.1 Installing Visual Studio Code](#11-installing-visual-studio-code)
  * [1.2 Installing Visual Studio Code Extensions (IntelliSense, PlatformIO)](#12-installing-visual-studio-code-extensions--intellisense--platformio-)
- [2 TTN Application Setup](#2-ttn-application-setup)
  * [2.1 Setting up a new TTN Application](#21-setting-up-a-new-ttn-application)
  * [2.2 Checking the Payload Format and Writing a Custom Decoder](#22-checking-the-payload-format-and-writing-a-custom-decoder)
  * [2.3 Registering a Device](#23-registering-a-device)
- [3 Hardware Setup](#3-hardware-setup)
- [4 Loading the Sensor Node Embedded Software](#4-loading-the-sensor-node-embedded-software)
  * [4.1 LoRaWAN Frequency Band and TTN Authentication Setup](#41-lorawan-frequency-band-and-ttn-authentication-setup)
  * [4.2 Components of the Sensor Node Embedded Software](#42-components-of-the-sensor-node-embedded-software)
    + [4.2.1 Initialization](#421-initialization)
    + [4.2.2 Low Power Mode](#422-low-power-mode)
    + [4.2.3 Motion Detection and Data Transmission](#423-motion-detection-and-data-transmission)
- [5 Flashing the Embedded Software](#5-flashing-the-embedded-software)
- [6 Testing and Viewing Data](#6-testing-and-viewing-data)
  * [6.1 Simulating Motion for Testing](#61-simulating-motion-for-testing)
  * [6.2  Viewing Transmited Data on the TTN Console](#62--viewing-transmited-data-on-the-ttn-console)
  * [6.3 Fetching Data from TTN to a PC](#63-fetching-data-from-ttn-to-a-pc)

### 1 Development Environment Setup
For this project, Visual Studio Code with the extension PlatformIO was used as the development environment.

Visual Studio Code is a cross-platform Electron-based source code editor released by Microsoft that provides built-in IntelliSense code completion which could greatly improve the development process compared to more simple development environments such as the Arduino IDE. Visual Studio Code also supports debugging, but this functionality was not used within the scope of this project.

PlatformIO is an extension for Visual Studio Code that is capable of building code and managing libraries for various hardware platforms including Arduino-compatible microcontroller devices. More information on PlatformIO can be found [here](https://docs.platformio.org/en/latest/what-is-platformio.html).

#### 1.1 Installing Visual Studio Code
Visual Studio Code can be downloaded [here](https://code.visualstudio.com/). As mentioned above, Visual Studio Code is platform-agnostic and can be installed on either a Windows, Mac or Linux machine.
#### 1.2 Installing Visual Studio Code Extensions (IntelliSense, PlatformIO)

- The `Extensions` panel can be viewed by clicking on the *Extension button* on the left sidebar.

	|![extension button](https://github.com/v2h/RTES2018/blob/master/Figures/visual_studio_code/vs_extension_button.png)|
	|:--:|

- More options can be displayed by clicking on the `...` button at the top of the `Extension` panel.

	|![dot button](https://github.com/v2h/RTES2018/blob/master/Figures/visual_studio_code/vs_extension_threedot_button.png)|
	|:--:|

- If `C/C++ IntelliSense` does not show up in the list of Installed Extensions, search for it by typing `C/C++ IntelliSense` in the Extension Search Bar and install it.

- Type `PlatformIO` in the Extension Search Bar to search for PlatformIO and install the extension.

### 2 TTN Application Setup

#### 2.1 Setting up a new TTN Application

- Register for an account at https://www.thethingsnetwork.org/<br>
  Login and navigate to https://console.thethingsnetwork.org/ 
- Click on `APPLICATIONS` to go to the application panel.
![console](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_console.jpg)

- Click on the `add application` button to add a new application. 
![add application](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_add_app.jpg)

- Give the application a **_unique application ID_** and choose the proper handler with respect to the region where the application is to be deployed.
![application id](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_add_app_2.jpg)

- The website will redirect to the application's `overview` page right after it is created.
![app overview](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_add_app_3.jpg)
#### 2.2 Checking the Payload Format and Writing a Custom Decoder
- Click on the `Payload Formats` tab on the upper right corner of the `Application Overview` page. 
- Check that `Custom` has been chosen by default.
- Copy this code into the decoder box and click `Save`
	```javascript
	function Decoder(bytes, port) {
	  // Decode an uplink message from a buffer
	  // (array) of bytes to an object of fields.
	  var decoded = {pir: 0, imu: 1};
	  
	  if (port === 1) 
	  {
	    decoded.pir = bytes[0];
	    decoded.imu = bytes[1]
	  }

	  return decoded;
	}
	```
	![payload formats](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_payload_formats.jpg)

#### 2.3 Registering a Device
- From the `Application Overview` page, click on the `Devices` tab.
Click on the `register device` button.
![register device](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_register_devices.jpg)

- Give the device a *unique device name* (within the scope of the application, meaning there should be no other devices in the application with the same name).\
On the `Device EUI` section, click the `generate` button to automatically generate a Device EUI.\
- Click the `Register` button to finish registering the device.
![register device panel](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_register_devices_2.jpg)

- The website will redirect to the `Device Overview` page after registration is complete.
![device overview](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_device_overview.jpg)
### 3 Hardware Setup
The following pieces of hardware were used in this project:

- An Adafruit Feather M0 LoRa 900Mhz board which includes an ATSAMD21G18 ARM Cortex M0 processor and an RFM95 LoRa module. A more detailed description of the board is provided by Adafruit on [the board's product page](https://www.adafruit.com/product/3178).
- [An MPU-9250 sensor board from Grove](http://wiki.seeedstudio.com/Grove-IMU_10DOF/), of which only the 3-axis accelerometer was used (for acceleration-based motion detection). This sensor board communicates with the Feather M0 board via I2C.
- [An PIR sensor board from Grove](http://wiki.seeedstudio.com/Grove-PIR_Motion_Sensor/). This sensor board transmits its digital signal to a GPIO pin of the Feather M0 board.

The wiring of the above components must be done according to the figure below.
![wiring](https://github.com/v2h/RTES2018/blob/master/Figures/embedded_system/sensors_featherM0_connections.png)

Note that:

- The Adafruit Feather M0 LoRa board does not include an antenna. Instructions on how to used a coper wire as antenna is covered [here](https://learn.adafruit.com/assets/31585) by Adafruit.
- The IO1 pin must be connected to Pin 6, otherwise the LoRa module would not function. This will be reflected in the pin mappings definition within the embedded code.

### 4 Loading the Sensor Node Embedded Software

 - **Download or clone the whole project repository [here](https://github.com/v2h/RTES2018).**
 - In Visual Studio Code, Click on the PlatformIO icon on the left sidebar to bring up the PlatformIO homepage.  

|![pio home](https://github.com/v2h/RTES2018/blob/master/Figures/visual_studio_code/platformIO_homepage.png)|
|:--:|

 -  Within the PlatformIO homepage, click `Open Project` to load a the project into Visual Studio Code.
 - Navigate to `.../PlatformIO/FeatherMO_LoRa_IMU_PIR_TTN` and click `Open "FeatherM0_LoRa_IMU_PIR_TTN` to load the project
 - From the Explorer panel, click on `platformio.ini` and make sure the settings match the figure below. 

|![ini settings](https://github.com/v2h/RTES2018/blob/master/Figures/visual_studio_code/ini_settings.png)|
|:--:|

 - Make sure the following library names are present in the `.piolibdeps` folder: <br>
 [Boder Flight Systems MPU9250](https://github.com/bolderflight/MPU9250) by Brian Taylor<br>
 [MCCI LoRaWAN LMIC](https://github.com/mcci-catena/arduino-lmic) by Matthis Kooijman et. al.<br>
 [RTCZero](https://github.com/arduino-libraries/RTCZero) by Arduino.<br>
 [SparkFun MPU-9250 Digital Motion Processing (DMP) Arduino Library](https://github.com/sparkfun/MPU-9250) by SparkFun Electronics.<br>

|![libaries](https://github.com/v2h/RTES2018/blob/master/Figures/visual_studio_code/libraries.png)|
|:--:|

 This can also be checked via the PlatformIO page:
 
![libraries 2](https://github.com/v2h/RTES2018/blob/master/Figures/visual_studio_code/libraries2.png)

 As the board configuration and libraries are bundled into the PlatformIO project, there is no need to install them again. However, it should be noted that all board configurations and libraries are available on the PlatformIO homepage, which can be accessed by clicking on the `PlatformIO` icon on the left sidebar.
 
#### 4.1 LoRaWAN Frequency Band and TTN Authentication Setup

It must be noted that **the correct define for LoRaWAN frequency**   
   has to be set in the `.../FeatherM0_LoRa_IMU_PIR_TTN/.piolibdeps/MCCI
   LoRaWAN LMIC library_ID5774/project_configlmic_project_config.h`file.
   An overview of the LoRaWan frequency plans is provided by   
   TheThingsNetwork [here](https://www.thethingsnetwork.org/docs/lorawan/frequency-plans.html).

Each registered device from the TTN application has its unique **Application UI**, **Device EUI** and **Application Key** and can be found on the each [`Device Overview`](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_device_overview.jpg) page. These values have to be copied in to the following variables inside the code:

-   `static const u1_t PROGMEM APPEUI[8]`: little-endian format
-   `static const u1_t PROGMEM DEVEUI[8]`: little-endian format
-   `static const u1_t PROGMEM APPKEY[16]`: big-endian format

The correct endian formats are provided by clicking on the buttons right next to each key's values on the TTN [`Device Overview`](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_device_overview.jpg) page.

Note that the values of these three keys are stored in the _program memory_ section of the microcontroller, which means they will not be erased when the microcontroller is reset or when the microcontroller wakes up from sleep.

 #### 4.2 Components of the Sensor Node Embedded Software
The operating cycle of the sensor node's embedded software is described in the figure below <br>
<img src="https://github.com/v2h/RTES2018/blob/master/Figures/embedded_system/operating_cycle.png" width=80% height=80%>
##### 4.2.1 Initialization
The initialization phase is done by a part of the setup()` function where:

- The RTC module is initialized by calling `rtc.begin()`. The clock source for the RTC is set to be XOSC32K by the RTCZero library.
- The LMIC stack is initialized by calling `os_init()` and `LMIC_reset()`. The LMIC stack is essential for ensuring LoRaWAN compliance. Note that the pin mapping for the Adafruit Feather M0 Lora board is defined as
	 ```cpp
	// Pin mapping for Adafruit Feather M0 LoRa
	const lmic_pinmap lmic_pins = {
	.nss  =  8,
	.rxtx  = LMIC_UNUSED_PIN,
	.rst  =  4,
	.dio  = {3, 6, LMIC_UNUSED_PIN},
	.rxtx_rx_active  =  0,
	.rssi_cal  =  8,
	.spi_freq  =  8000000,
	};
	```

- A Join Request is sent to the TTN server to authenticate the device with the TTN application. This is done by calling:
	```cpp
	sendJobIsDone =  false;
	do_send(&sendjob);
	do
	{
	os_runloop_once();
	} while (sendJobIsDone ==  false);
	sendJobIsDone =  true;
	```
	the flag `sendJobIsDone` can only be set to true when a `EV_TXCOMPLETE` event is received by the `on_event()` function.
- The two sensor boards are setup with interrupts by calling the `attachInterrupt()` function. 
- Note that to enable edge-triggered interrupt while the microcontroller is sleeping, a clock source has to be fed to the microcontroller's external interrupt controller (EIC):
	```cpp
	// This has to be done after the first call to attachInterrupt()
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.bit.SYNCBUSY);
	```
##### 4.2.2 Low Power Mode
To reduce power consumption, all peripherals should be put into low-power mode while still ensuring interrupt functionality:

- The microcontroller is put to sleep by calling `rtc.standbyMode()`.
- The PIR motion sensor is a passive device which consumes very little current by design.
- The MPU-9250 is already put in low-power mode (and its wake-on-motion interrupt is enabled) by calling `imu.enableWakeOnMotion()` from the initialization phase.
- The low-power mode for the LoRa module has not been implemented in this project due to the limited amount of time.

##### 4.2.3 Motion Detection and Data Transmission
- The PIR motion sensor sends a logic HIGH signal to PIN 16 of the microcontroller when motion is detected, causing the microcontroller to wake up via the configured GPIO level interrupt.
- The MPU-9250's interrupt pin turns active (LOW) whenever a motion exceeds the predefined acceleration threshold in any direction, causing the microcontroller to wake up via the configured GPIO edge-triggered interrupt.
If motion is detected by either sensor, the microcontroller board will transmit a packet to the TTN cloud service by calling the `do_send()` function.

### 5 Flashing the Embedded Software
The embedded software can be uploaded by:

- clicking the `PlatformIO:Upload` button at the bottom bar of the Visual Studio Code IDE, or 
- by pressing `F1` and typing `PlatformIO:Upload`.
|![upload](https://github.com/v2h/RTES2018/blob/master/Figures/visual_studio_code/upload.png)|
|:--:|

In the same manner, the Serial Monitor can be viewed by click the `PlatformIO:SerialMonitor` button at the bottom bar of the Visual Studio Code IDE or by pressing `F1` and typing `PlatformIO:SerialMonitor`.

Note that:
 
 - If the code cannot be uploaded, the reset button has to be quickly double-pressed (the status LED will keep glowing) and then the `PlatformIO:Upload` can be used to flash the code into the microcontroller.
 - The USB connection from the microcontroller board to the computer is disconnected when the system switch to low-power mode.

### 6 Testing and Viewing Data
#### 6.1 Simulating Motion for Testing
- Waving of hand over the sensor was enough to trigger detection from the PIR sensor. Note that the PIR sensor worked more reliably when the surrounding environment was dark.
- A rapid force applied in any direction could be enough to trigger a detection from the MPU-9250.

#### 6.2  Viewing Transmited Data on the TTN Console
Data can be viewed on the TTN `Application Overview` page by clicking on the `Data` tab. 
The payload contains only two bytes: one PIR motion detection and one for acceleration-based motion detection. The custom-payload encoder ensures that the data is displayed correctly. A value of `1` from any sensor means that motion is detected. 

![data view](https://github.com/v2h/RTES2018/blob/master/Figures/view_data/device_data_ttn.jpg)
Note that the OTTA authentication via sending a Join Request only occurred once at the beginning. After the being joined into the TTN application, the end node was able to transmitting data without having to re-join.

#### 6.3 Fetching Data from TTN to a PC
[A Python program was written](https://github.com/v2h/RTES2018/blob/master/Python/MQTT_TTN.py) and can be run to fetch uplink messages transmitted from the sensor node to the TTN server and wrangle the data to get the payload as a JSON string. Note that the program is set to timeout after 60 seconds by default with `time.sleep(60)`.

|<img src="https://github.com/v2h/RTES2018/blob/master/Figures/view_data/json_data.jpg">|
|:--:|
