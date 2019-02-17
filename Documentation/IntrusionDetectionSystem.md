

#### Hochschule Rhein-Waal<br>RTES2018
# Intrusion Detection System Using Motion Sensing and LoRaWAN

## Table of Contents
- [Abstract](#abstract)
- [Abbreviations](#abbreviations)
- [1 Introduction](#1-introduction)
  * [1. Motivation](#1-motivation)
  * [1.2 Aim](#12-aim)
  * [1.3 Approach](#13-approach)
  * [1.4 Document Outline](#14-document-outline)
- [2 System Overview](#2-system-overview)
- [3 Technology Review](#3-technology-review)
  * [3.1 Motion Sensing](#31-motion-sensing)
    + [3.1.1 Motion Sensing Using PIR Sensors](#311-motion-sensing-using-pir-sensors)
    + [3.1.2 Motion Sensing Using MEMS-based Accelerometers](#312-motion-sensing-using-mems-based-accelerometers)
  * [3.2 LoRaWAN](#32-lorawan)
- [4 Methodology](#4-methodology)
  * [4.1 Development Environment Setup](#41-development-environment-setup)
    + [4.1.1 Installing Visual Studio Code](#411-installing-visual-studio-code)
    + [4.1.2 Installing Visual Studio Code Extensions (IntelliSense, PlatformIO)](#412-installing-visual-studio-code-extensions--intellisense--platformio-)
  * [4.2 TTN Application Setup](#42-ttn-application-setup)
    + [4.2.1 Setting up a new TTN Application](#421-setting-up-a-new-ttn-application)
    + [4.2.2 Checking the Payload Format and Writing a Custom Decoder](#422-checking-the-payload-format-and-writing-a-custom-decoder)
    + [4.2.3 Registering a Device](#423-registering-a-device)
  * [4.3 Hardware Setup](#43-hardware-setup)
  * [4.4 Loading the Sensor Node Embedded Software](#44-loading-the-sensor-node-embedded-software)
    + [4.4.1 LoRaWAN Frequency Band and TTN Authentication Setup](#441-lorawan-frequency-band-and-ttn-authentication-setup)
    + [4.4.2 Components of the Sensor Node Embedded Software](#442-components-of-the-sensor-node-embedded-software)
      - [4.4.2.1 Initialization](#4421-initialization)
      - [4.4.2.2 Low Power Mode](#4422-low-power-mode)
      - [4.4.2.3 Motion Detection and Data Transmission](#4423-motion-detection-and-data-transmission)
  * [4.5 Flashing the Embedded Software](#45-flashing-the-embedded-software)
- [5 Results and Discussion](#5-results-and-discussion)
  * [5.1 Simulating Motion for Testing](#51-simulating-motion-for-testing)
  * [5.2  Viewing Transmited Data on the TTN Console](#52--viewing-transmited-data-on-the-ttn-console)
  * [5.3 Fetching Data from TTN to a PC](#53-fetching-data-from-ttn-to-a-pc)
- [6 Conclusion and Outlook](#6-conclusion-and-outlook)
- [References](#references)

## Abstract
The city of Kamp-Lintfort has been expanding and developing rapidly since the opening of the Hochschule Rhein-Waal's new campus at the city center in 2014. This results in more state-funded and academic research projects to be carried out in the near future, some of which involves the installation and maintenance of equipment and devices on remote and open fields. Therefore, it is the goal of this proof-of-concept project to rapidly develop an intrusion detection system for remote monitoring of large field areas and preventing vandalism and to be used as a reusable tutorial for learning and further research.

This proof-of-concept was carried out by using off-the-shelf sensor boards that are capable of detecting motion based on infrared radiation and acceleration. An embedded software was written to build an interrupt-driven embedded system that capable of reliably reading sensor data and transmitting data using LoRaWAN to the TTN cloud service. The TTN service was configured to properly authenticate the sensor node and decode the data. Furthermore, a Python application was written to fetch data from the TTN service to a local computer.

All components of the proof-of-concept system was tested to function properly. The sensor boards and microcontroller was able to switch go into low-power mode to reduce power consumption while motion detection events could still trigger interrupts. The TTN cloud service was configured to properly receive and display data using a custom payload decoder. The Python application deployed on a local computer was able to fetch data from the TTN server and store the data as JSON strings.

Due to the time constraint of the course, the sleep mode for the LoRa radio module on the microcontroller board could not be implemented, and measurement of power consumption of the system was not carried out. This should be implemented in the future to improve this proof-of-concept by reusing the written code base and documentation available on this project's [Github page](https://github.com/v2h/RTES2018/).

## Abbreviations
|  |  |
|--|--|
|IC|Integrated Circuit|
|IDE|Integrated Development Environment|
|IoT| Internet of Things|
|MEMS|Micro-Electro-Mechanical Systems|
|PC|Personal Computer|
|PIR |Passive Infrared Radiation |
|TTN|The Things Network|


## 1 Introduction
### 1. Motivation
Originally established in the twelfth century as the “Kamp” area and merged with other surrounding areas in 1934 to become “Kamp-Lintfort”, the city currently has a population of more than 37000 (Landesdatenbank NRW 2018). Compared to the population of other neighboring cities such as Moers (103949) or Wesel (60496) (Landesdatenbank NRW 2018), Kamp-Lintfort is a relatively small town. It is, however, has been expanding and modernizing quickly, especially since the opening of the new Hochschule Rhein-Waal campus.

The development of Kamp-Lintfort together with the on-going academic research projects at Hochschule Rhein-Waal means that it is necessary to consider an intrusion detection mechanism for protecting university and public properties. This could be particularly beneficial for field projects such as the Landesgartenschau project (Kamp-Lintfort Hochschulstadt 2018) where a city garden is going to be built resulting in installing and maintaining many electronic devices (for lighting, timely watering, …) on the open field.

### 1.2 Aim
The project “Intrusion Detection System Using Motion Sensing and LoRaWan” aimed to rapidly develop a proof-of-concept of an intrusion detection system based on different motion-sensing techniques, microcontrollers and on LoRaWAN, an emerging wireless technology. Within the scope of the course “Real-time Embedded Systems”, this proof-of-concept including its related documentation would be used:
-	As a feasibility measure in determining whether developing this proof-of-concept into a deployable package would make sense.
-	As a reusable tutorial for a hands-on experience with sensors, microcontrollers and LoRaWAN.
-	For evaluating students’ theoretical and practical knowledge gained in the course.

### 1.3 Approach
To rapidly develop a proof-of-concept within the constrained timeline of the study course, this project made use of commercial off-the-shelf components including sensor boards, a microcontroller board with LoRa capability and a free cloud service with LoRaWAN support. Moreover, to ensure the reusability of the codebase, Visual Studio Code was decided to be the development environment for the embedded components of the project.

### 1.4 Document Outline
This document is divided into four chapters:

-	Chapter 1 provides a background on the project, states the aim and motivation of the project.
-	Chapter 2 provides an overview of the embedded system implemented in the project
-	Chapter 3 provides a review on the technologies used in the project including the motion sensors, the microcontroller and LoRaWAN.
-	Chapter 4 discusses the work carried out in the project including the setup of the development environment, the TTN cloud service, the communication between the sensors and the controller as well as the operating cycle of the implemented embedded system.
-	Chapter 5 discusses the results of the implementation.
-	Chapter 6 gives an outlook on possible future works that could improve the system.

## 2 System Overview
| <img src="https://github.com/v2h/RTES2018/blob/master/Figures/embedded_system/system_overview.png" width=100% height=50%>|
|:--:|

As seen in the figure above, the implemented system consists of three components:

- The sensor node (or end node): this includes the motion sensor boards and microcontroller board with LoRa module for detecting motion at a desired area and transmitting the signal to the TTN cloud service.
- The TTN cloud service: receives and stores data from the sensor node, provides a control dashboard for the configuration of devices and also options for connecting to a user application server.
- The Python application: uses the TTN API to fetch data from the cloud service to a local computer.

Note that what was not covered in this proof-of-concept is the TTN gateway whose job is to forward data from the sensor nodes to the TTN cloud service.

## 3 Technology Review
### 3.1 Motion Sensing
#### 3.1.1 Motion Sensing Using PIR Sensors
A typical PIR motion sensor board consists of the following main components:
-	A pyroelectric sensor
-	An IC for amplification and outputting of the sensed signal

PIR stands for passive-infrared. “Passive” means that the pyroelectric sensor does not need current to function. “Infrared” means that the sensor works by detecting heat in the form of infrared radiation which is invisible to the human eye. The human body radiates at a wavelength of about 9.4 micrometers (L. Viktor Tóth, Sarolta Zahorecz, Csaba Kiss 2013) which lies within the infrared range.

A pyroelectric sensor is made of a crystalline material that produces electric charges when a change in infrared radiation is detected (Glolab Corporation 2015) . This change can be caused by a warm body either approaching the sensor or moving away from the sensor which subsequently caused a charge displacement. This charge displacement can easily be realized into a voltage pulse by use of a gate resistor.

A pyroelectric sensor with two sensing elements connected in series with opposite polarities are less susceptible to noise as it produces two opposite pulses when there is a change in infrared radiation.

|![Dual PIR](https://github.com/v2h/RTES2018/blob/master/Figures/sensor/dual_pir.png)|
|:--:| 
| *Dual-element PIR Sensor Output Signal (Yun & Lee 2014)* |

The generated pulses from the PIR sensor is fed into an IC. The IC consists of an amplifier for amplification of the input signal and a comparator to properly produces an output a digital signal whose amplitude lies within a desired range (i.e. 0 for LOW and 3.3V for HIGH). This output signal from the IC can be fed into a microcontroller. Unlike the PIR sensor which is passive, the IC does consume current as it is an active component.

|![PIR Sensor](https://github.com/v2h/RTES2018/blob/master/Figures/sensor/pir_block_diagram.jpg) | 
|:--:| 
| *PIR Sensor Block Diagram (RobotsCraft 2016)* |

The PIR motion sensor board used in this project includes a BSS0001 PIR motion detector IC. A description of this IC is provided by Adafruit and can be accessed [here](http://www.ladyada.net/media/sensors/BISS0001.pdf). The model of the sensor element is unknown.

#### 3.1.2 Motion Sensing Using MEMS-based Accelerometers
A MEMS–based accelerometer is a capacitive-type sensor that consists of two capacitive plates: one plate is fixed onto to a solid plane on a substrate and the other plate is moveable with a known mass (proof mass) that is connected to a spring and can move in response to an applied acceleration (Excelitas Technologies 2015). When the movable capacitive plate moves, the capacitance between its fingers and the fixed plate’s fingers is changed and thus an amplifying circuit can be connected to these two plates to realize a voltage.

|![MEMS accelerator](https://github.com/v2h/RTES2018/blob/master/Figures/sensor/MEMS_accelerator.jpg)|
|:--:|
|MEMS Accelerator Structure (Rob O'Reilly, Alex Khenkin, Kieran Harney 2009)| 

The MPU-9250 board used in this project consists of a gyroscope, an accelerometer and a magnetometer. This project, however, only made used of the MPU-9250’s 3-axis accelerometer. As each axis requires one set of capacitive plates, a 3-axis accelerometer must have three sets of capacitive plates and thus can measure acceleration with respect to the three axes. When lying on a flat surface, the accelerometer will measure +1g on the Z-axis and 0g on the other two axes (InvenSense Inc 2016).
The wake-on motion feature of the MPU9250’s accelerometer was used in the project. When enabled, the sensor board will output a signal if an acceleration is applied to the sensor and the measured acceleration value exceeds the pre-configured threshold.

### 3.2 LoRaWAN
LoRaWAN is the data link layer on top of the physical LoRa layer. The RFM96 LoRa module on the Adafruit Feather M0 LoRa board is LoRa-capable; however, to ensure LoRaWAN compliance, additional embedded software components must be written and flashed into the microcontroller that controls the LoRa module. The LoRaWAN specifications ensures that all devices that support LoRaWAN are standardized on the quality of service, security, lifetime and the variety of applications supported (LoRa Alliance 2015). The physical layer, LoRa, is based on the chirp spread spectrum (CSS) modulation technique and the name "LoRa" is a trademark of Semtech Corporation.

LoRa and LoRaWAN are suitable for IoT applications where low-power consumption and long-range transmission (up to 20 km) are concerned (Mekki et al. 2018). On the other hand, the LoRaWAN specification does not allow for high-volume data transmission, which is not a concern within the scope of this project.
 
 |![LoRa and LoRaWAN](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/lora_lora_wan_layers.jpg)|
 |:--:|
 |*LoRa and LoRaWAN Layer (LoRa Alliance 2015)*|
## 4 Methodology
[Note: this chapter was written as a reusable hands-on tutorial]

### 4.1 Development Environment Setup
For this project, Visual Studio Code with the extension PlatformIO was used as the development environment.

Visual Studio Code is a cross-platform Electron-based source code editor released by Microsoft that provides built-in IntelliSense code completion which could greatly improve the development process compared to more simple development environments such as the Arduino IDE. Visual Studio Code also supports debugging, but this functionality was not used within the scope of this project.

PlatformIO is an extension for Visual Studio Code that is capable of building code and managing libraries for various hardware platforms including Arduino-compatible microcontroller devices. More information on PlatformIO can be found [here](https://docs.platformio.org/en/latest/what-is-platformio.html).

#### 4.1.1 Installing Visual Studio Code
Visual Studio Code can be downloaded [here](https://code.visualstudio.com/). As mentioned above, Visual Studio Code is platform-agnostic and can be installed on either a Windows, Mac or Linux machine.
#### 4.1.2 Installing Visual Studio Code Extensions (IntelliSense, PlatformIO)

- The `Extensions` panel can be viewed by clicking on the *Extension button* on the left sidebar.

	|![extension button](https://github.com/v2h/RTES2018/blob/master/Figures/visual_studio_code/vs_extension_button.png)|
	|:--:|

- More options can be displayed by clicking on the `...` button at the top of the `Extension` panel.

	|![dot button](https://github.com/v2h/RTES2018/blob/master/Figures/visual_studio_code/vs_extension_threedot_button.png)|
	|:--:|

- If `C/C++ IntelliSense` does not show up in the list of Installed Extensions, search for it by typing `C/C++ IntelliSense` in the Extension Search Bar and install it.

- Type `PlatformIO` in the Extension Search Bar to search for PlatformIO and install the extension.

### 4.2 TTN Application Setup

#### 4.2.1 Setting up a new TTN Application

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
#### 4.2.2 Checking the Payload Format and Writing a Custom Decoder
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

#### 4.2.3 Registering a Device
- From the `Application Overview` page, click on the `Devices` tab.
Click on the `register device` button.
![register device](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_register_devices.jpg)

- Give the device a *unique device name* (within the scope of the application, meaning there should be no other devices in the application with the same name).\
On the `Device EUI` section, click the `generate` button to automatically generate a Device EUI.\
- Click the `Register` button to finish registering the device.
![register device panel](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_register_devices_2.jpg)

- The website will redirect to the `Device Overview` page after registration is complete.
![device overview](https://github.com/v2h/RTES2018/blob/master/Figures/ttn_setup/ttn_device_overview.jpg)
### 4.3 Hardware Setup
The following pieces of hardware were used in this project:

- An Adafruit Feather M0 LoRa 900Mhz board which includes an ATSAMD21G18 ARM Cortex M0 processor and an RFM95 LoRa module. A more detailed description of the board is provided by Adafruit on [the board's product page](https://www.adafruit.com/product/3178).
- [An MPU-9250 sensor board from Grove](http://wiki.seeedstudio.com/Grove-IMU_10DOF/), of which only the 3-axis accelerometer was used (for acceleration-based motion detection). This sensor board communicates with the Feather M0 board via I2C.
- [An PIR sensor board from Grove](http://wiki.seeedstudio.com/Grove-PIR_Motion_Sensor/). This sensor board transmits its digital signal to a GPIO pin of the Feather M0 board.

The wiring of the above components must be done according to the figure below.
![wiring](https://github.com/v2h/RTES2018/blob/master/Figures/embedded_system/sensors_featherM0_connections.png)

Note that:

- The Adafruit Feather M0 LoRa board does not include an antenna. Instructions on how to used a coper wire as antenna is covered [here](https://learn.adafruit.com/assets/31585) by Adafruit.
- The IO1 pin must be connected to Pin 6, otherwise the LoRa module would not function. This will be reflected in the pin mappings definition within the embedded code.

### 4.4 Loading the Sensor Node Embedded Software

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
 
#### 4.4.1 LoRaWAN Frequency Band and TTN Authentication Setup

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

 #### 4.4.2 Components of the Sensor Node Embedded Software
The operating cycle of the sensor node's embedded software is described in the figure below <br>
<img src="https://github.com/v2h/RTES2018/blob/master/Figures/embedded_system/operating_cycle.png" width=80% height=80%>
##### 4.4.2.1 Initialization
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
##### 4.4.2.2 Low Power Mode
To reduce power consumption, all peripherals should be put into low-power mode while still ensuring interrupt functionality:

- The microcontroller is put to sleep by calling `rtc.standbyMode()`.
- The PIR motion sensor is a passive device which consumes very little current by design.
- The MPU-9250 is already put in low-power mode (and its wake-on-motion interrupt is enabled) by calling `imu.enableWakeOnMotion()` from the initialization phase.
- The low-power mode for the LoRa module has not been implemented in this project due to the limited amount of time.

##### 4.4.2.3 Motion Detection and Data Transmission
- The PIR motion sensor sends a logic HIGH signal to PIN 16 of the microcontroller when motion is detected, causing the microcontroller to wake up via the configured GPIO level interrupt.
- The MPU-9250's interrupt pin turns active (LOW) whenever a motion exceeds the predefined acceleration threshold in any direction, causing the microcontroller to wake up via the configured GPIO edge-triggered interrupt.
If motion is detected by either sensor, the microcontroller board will transmit a packet to the TTN cloud service by calling the `do_send()` function.

### 4.5 Flashing the Embedded Software
The embedded software can be uploaded by:

- clicking the `PlatformIO:Upload` button at the bottom bar of the Visual Studio Code IDE, or 
- by pressing `F1` and typing `PlatformIO:Upload`.
|![upload](https://github.com/v2h/RTES2018/blob/master/Figures/visual_studio_code/upload.png)|
|:--:|

In the same manner, the Serial Monitor can be viewed by click the `PlatformIO:SerialMonitor` button at the bottom bar of the Visual Studio Code IDE or by pressing `F1` and typing `PlatformIO:SerialMonitor`.

Note that:
 
 - If the code cannot be uploaded, the reset button has to be quickly double-pressed (the status LED will keep glowing) and then the `PlatformIO:Upload` can be used to flash the code into the microcontroller.
 - The USB connection from the microcontroller board to the computer is disconnected when the system switch to low-power mode.


## 5 Results and Discussion
### 5.1 Simulating Motion for Testing
- Waving of hand over the sensor was enough to trigger detection from the PIR sensor. Note that the PIR sensor worked more reliably when the surrounding environment was dark.
- A rapid force applied in any direction could be enough to trigger a detection from the MPU-9250.

### 5.2  Viewing Transmited Data on the TTN Console
Data can be viewed on the TTN `Application Overview` page by clicking on the `Data` tab. 
The payload contains only two bytes: one PIR motion detection and one for acceleration-based motion detection. The custom-payload encoder ensures that the data is displayed correctly. A value of `1` from any sensor means that motion is detected. 

![data view](https://github.com/v2h/RTES2018/blob/master/Figures/view_data/device_data_ttn.jpg)
Note that the OTTA authentication via sending a Join Request only occurred once at the beginning. After the being joined into the TTN application, the end node was able to transmitting data without having to re-join.

### 5.3 Fetching Data from TTN to a PC
[A Python program was written](https://github.com/v2h/RTES2018/blob/master/Python/MQTT_TTN.py) and can be run to fetch uplink messages transmitted from the sensor node to the TTN server and wrangle the data to get the payload as a JSON string. Note that the program is set to timeout after 60 seconds by default with `time.sleep(60)`.

|<img src="https://github.com/v2h/RTES2018/blob/master/Figures/view_data/json_data.jpg">|
|:--:|

## 6 Conclusion and Outlook
The proof-of-concept "# Intrusion Detection System Using Motion Sensing and LoRaWAN" was rapidly developed and was able to contain the following features:

- Infrared-radiation and acceleration-based motion detection
- Interrupt-driven embedded system for the end node
- Low-power modes for the sensor boards and the microcontroller
- LoRaWAN-base transmission of data to the TTN cloud service
- Retrieval of data to a computer

Due to the time constraint of the course, however, the following has not been done:

- Configuration of a low-power mode for the LoRa module
- Measurement of the end node's power consumption
- Implementation of an actuator to react when a 'motion detected' message is received from the computer or an application server
- Sending of periodic 'Keep-alive' messages from the sensor node to the TTN server to inform that the sensor node still functions properly

This project was also intended to be reusable as a tutorial, hence the use of Visual Studio with PlatformIO, Github and the documentation. This is to ensure that the not-yet implemented features could be easily integrated into the proof-of-concept in the future.

## References

L. Viktor Tóth, Sarolta Zahorecz, Csaba Kiss 2013, Infrared Astronomy, Eötvös Loránd University. Available from: http://elte.prompt.hu/sites/default/files/tananyagok/InfraredAstronomy/ [17 February 2019].

Landesdatenbank NRW 2018, Bevölkerungsstand Basis Zensus 2011 - Gemeinden - Stichtag, Landesdatenbank NRW. Available from: https://www.landesdatenbank.nrw.de/ldbnrw/online/data;jsessionid=83F19BA3D1A9966574F82915963BC679.ldb1?operation=abruftabelleBearbeiten&levelindex=2&levelid=1550382014540&auswahloperation=abruftabelleAuspraegungAuswaehlen&auswahlverzeichnis=ordnungsstruktur&auswahlziel=werteabruf&selectionname=12410-00ir&auswahltext=&nummer=2&variable=2&name=DLAND&werteabruf=Werteabruf [17 February 2019].

LoRa Alliance 2015, A technical overview of LoRa and LoRaWAN, LoRa Alliance. Available from: https://lora-alliance.org/sites/default/files/2018-04/what-is-lorawan.pdf [17 February 2019].

Mekki, K, Bajic, E, Chaxel, F & Meyer, F 2018, 'A comparative study of LPWAN technologies for large-scale IoT deployment', ICT Express.

Rob O'Reilly, Alex Khenkin, Kieran Harney 2009, Using MEMS Accelerometers as Acoustic Pickups in Musical Instruments. Available from: https://www.analog.com/en/analog-dialogue/articles/mems-accelerometers-as-acoustic-pickups.html [17 February 2019].

RobotsCraft 2016, Introduction to PIR Sensor and Integrating It With Arduino. Available from: https://cdn.instructables.com/FO1/75G7/IW6POC25/FO175G7IW6POC25.LARGE.jpg [17 February 2019].

Yun, J & Lee, S-S 2014, 'Human movement detection and identification using pyroelectric infrared sensors', Sensors (Basel, Switzerland), vol. 14, no. 5, pp. 8057–8081.

