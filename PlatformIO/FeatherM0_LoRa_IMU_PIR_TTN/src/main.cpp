//#define Serial SerialUSB
#include <RTCZero.h>
#include "MPU9250.h"

#define SerialPort Serial

void imuSetup(bool enableInterrupt);
void setInterruptLevel(bool activeHigh);
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
  IMU_LED    = 18,
};


static RTCZero rtc;

static MPU9250 imu(Wire, 0x68);
static int8_t imu_status;

static bool imuInterruptFlag = false;
static bool pirInterruptFlag = false;

void setup() 
{
  pinMode(STATUS_LED, OUTPUT);
  SerialPort.begin(115200);
  delay(5000);
  rtc.begin();

  imuSetup(true);
  pirSetup(true);

  Serial.end();
  USBDevice.detach(); // Safely detach the USB prior to sleeping
  rtc.standbyMode();
}

void loop()
{
  if (pirInterruptFlag)
  {
    pirInterruptFlag = false;

    flashLED(STATUS_LED, 6, 100);
  }

  //if (imu.dataReady() )
  if (imuInterruptFlag)
  {
    imuInterruptFlag = false; 

    flashLED(STATUS_LED, 3, 200);
  }
}

void printIMUData(void)
{  

}

// Set up the mpu9250 imu sensors
// including its gyro, accel and compass
void imuSetup(bool enableInterrupt)
{
  imu_status = imu.begin();
  if (imu_status < 0) 
  {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(imu_status);
    while(1) {}   
  }

  if (enableInterrupt)
  { 
    // enabling wake on motion low power mode with a threshold of 400 mg and
    // an accelerometer data rate of 15.63 Hz. 
    imu.enableWakeOnMotion(400,MPU9250::LP_ACCEL_ODR_15_63HZ);

    setInterruptLevel(false);

    // Set interrupt pin with internal pullup (active low)
    pinMode(IMU_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuInterruptHandler, FALLING);

    // Configure EIC to use GCLK1 which uses XOSC32K 
    // This has to be done after the first call to attachInterrupt()
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY);
    flashLED(STATUS_LED, 5, 300);
    Serial.println("setup successful");
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
  { // trigger interrupt when data pin is HIGH
    attachInterrupt(digitalPinToInterrupt(PIR_PIN), pirInterruptHandler, HIGH); 
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

void setInterruptLevel(bool activeHigh)
{
  uint8_t buffer;
  uint8_t devAddress = 0x68; //Default I2C address
  uint8_t regAddress = 0x37; //INT_PIN_CFG 

  Wire.beginTransmission(devAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(devAddress, 1);
  buffer = Wire.read();

  Wire.beginTransmission(devAddress);
  Wire.write(regAddress);
  if (activeHigh)
  {
    buffer &= (~(1 << 7));
    Wire.write(buffer);
  }
  else
  {
    buffer = buffer | (1 << 7);
    Wire.write(buffer);
  }
  Wire.endTransmission();

  Wire.beginTransmission(devAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(devAddress, 1);
  buffer = Wire.read();
  Serial.print("buffer: "); Serial.println(buffer, BIN);
}