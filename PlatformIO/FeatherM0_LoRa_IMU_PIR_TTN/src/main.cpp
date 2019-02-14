//#define Serial SerialUSB
#include <RTCZero.h>
#include "SparkFunMPU9250-DMP.h"

#define SerialPort Serial

void imuSetup(bool enableInterrupt);
void printIMUData(void);
void imuInterruptHandler(void);
void pirSetup(bool interruptEnable);
void pirInterruptHandler(void);
int8_t readPirValue(void);
void alarmMatch(void);
void flashLED(uint8_t ledPin, uint8_t numberOfToggles, uint16_t delayInMs);

struct imuData
{
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float magX;
  float magY;  
  float magZ;
};

enum 
{
  PIR_PIN           = 15,
  IMU_INTERRUPT_PIN = 16, //12, 
  STATUS_LED        = 13,

  PIR_LED    = 17,
  IMU_LED    = 18
};


static RTCZero rtc;

static MPU9250_DMP imu;
static struct imuData imuData;
static bool imuInterruptFlag = false;
static bool pirInterruptFlag = false;

void setup() 
{
  pinMode(STATUS_LED, OUTPUT);
  SerialPort.begin(115200);
  delay(5000);
  rtc.begin();

  imuSetup(true);
  //pirSetup(false);

  Serial.end();
  USBDevice.detach(); // Safely detach the USB prior to sleeping
  rtc.standbyMode();
}

void loop()
{
  if (pirInterruptFlag)
  {
    pirInterruptFlag = false;
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

    flashLED(STATUS_LED, 6, 100);
  }

  //if (imu.dataReady() )
  if (imuInterruptFlag)
  {
    imuInterruptFlag = false; 
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

    flashLED(STATUS_LED, 3, 200);
  }
}

void printIMUData(void)
{  
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  imuData.accelX = imu.calcAccel(imu.ax);
  imuData.accelY = imu.calcAccel(imu.ay);
  imuData.accelZ = imu.calcAccel(imu.az);
  imuData.gyroX = imu.calcGyro(imu.gx);
  imuData.gyroY = imu.calcGyro(imu.gy);
  imuData.gyroZ = imu.calcGyro(imu.gz);
  imuData.magX = imu.calcMag(imu.mx);
  imuData.magY = imu.calcMag(imu.my);
  imuData.magZ = imu.calcMag(imu.mz);
  
  SerialPort.println("Accel: " + String(imuData.accelX) + ", " +
              String(imuData.accelY) + ", " + String(imuData.accelZ) + " g");
  SerialPort.println("Gyro: " + String(imuData.gyroX) + ", " +
              String(imuData.gyroY) + ", " + String(imuData.gyroZ) + " dps");
  SerialPort.println("Mag: " + String(imuData.magX) + ", " +
              String(imuData.magY) + ", " + String(imuData.magZ) + " uT");
  SerialPort.println("Time: " + String(imu.time) + " ms");
  SerialPort.println();
}

// Set up the mpu9250 imu sensors
// including its gyro, accel and compass
void imuSetup(bool enableInterrupt)
{
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  // Any of the following defines can be combined:
  // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
  // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  // Enable all sensors:
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setSampleRate(1); // Set accel/gyro sample rate to 1Hz
  imu.setCompassSampleRate(1); // Set mag rate to 1Hz

  // Use setGyroFSR() and setAccelFSR() to configure the gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5 (values are in Hz).
  imu.setLPF(5); // Set LPF corner frequency to 5Hz

  if (enableInterrupt)
  {
    // Set interrupt pin with internal pullup (active low)
    pinMode(IMU_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuInterruptHandler, FALLING);

    // Configure EIC to use GCLK1 which uses XOSC32K 
    // This has to be done after the first call to attachInterrupt()
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY);
    flashLED(STATUS_LED, 5, 300);
    Serial.println("setup successful");

    // interrupt output as a "data ready" indicator.
    imu.enableInterrupt();

    // Configure as active-low, since we'll be using the pin's
    // internal pull-up resistor.
    // Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
    imu.setIntLevel(INT_ACTIVE_LOW);

    // The interrupt can be set to latch until data has been read, or to work as a 50us pulse.
    // Use latching method -- we'll read from the sensor
    // as soon as we see the pin go LOW.
    // Options are INT_LATCHED or INT_50US_PULSE
    imu.setIntLatched(INT_LATCHED);
  }
}

void imuInterruptHandler(void)
{
  imuInterruptFlag = true;
}

// Set up the pir motion sensor
void pirSetup(bool interruptEnable)
{
  // Data pin, the sensor is 1-wire digital, either HIGH or LOW
  pinMode(PIR_PIN, INPUT);
  if (interruptEnable)
  {
    attachInterrupt(PIR_PIN, pirInterruptHandler, HIGH); // trigger interrupt when data pin is HIGH
  }
}

void pirInterruptHandler(void)
{
  pirInterruptFlag = true;
}

void flashLED(uint8_t ledPin, uint8_t numberOfToggles, uint16_t delayInMs)
{
  for (uint8_t i = 0; i < numberOfToggles; i++)
  {
    digitalWrite(ledPin, HIGH);
    delay(delayInMs);              
    digitalWrite(ledPin, LOW); 
    delay(delayInMs);   
  }
}

// Read data from the PIR sensor
int8_t readPirValue(void)
{
  return digitalRead(PIR_PIN);
}