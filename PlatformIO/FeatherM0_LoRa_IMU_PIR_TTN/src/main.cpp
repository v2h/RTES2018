//#define Serial SerialUSB
#include <RTCZero.h>
#include "SparkFunMPU9250-DMP.h"
#define SerialPort Serial

void imuSetup(bool enableInterrupt);
void alarmMatch(void);
void printIMUData(void);
void imuInterruptHandler(void);
void pirSetup(bool interruptEnable);
void pirInterruptHandler(void);
int8_t readPirValue();
void flashLED(void);

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
  PIR_PIN = 15,
  IMU_INTERRUPT_PIN = 16, 
  LED_PIN = 13
};


static RTCZero rtc;
//static int AlarmTime;

static MPU9250_DMP imu;
static struct imuData imuData;
static bool imuInterruptFlag = false;
static bool pirInterruptFlag = false;

void setup() 
{
  SerialPort.begin(115200);
  imuSetup(false);
  pirSetup(true);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  rtc.begin();
}

void loop()
{
  flashLED();

  rtc.attachInterrupt(imuInterruptHandler);
  Serial.end();
  USBDevice.detach(); // Safely detach the USB prior to sleeping
  rtc.standbyMode();
  USBDevice.attach();   // Re-attach the USB, audible sound on windows machines

  if (pirInterruptFlag)
  {
    pirInterruptFlag = false;
    Serial.println("Interrupted: pir\n");
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMUData();
  }

  //if (imu.dataReady() )
  if (imuInterruptFlag)
  {
    imuInterruptFlag = false; 
    Serial.println("Interrupted: imu\n");
    // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
    // UPDATE_TEMPERATURE.
    // (The update function defaults to accel, gyro, compass,
    //  so you don't have to specify these values.)
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMUData();
  }

  /*
  // Simple indication of being awake
  digitalWrite(LED_PIN, HIGH);   // turn the LED on 
  delay(1000);              
  digitalWrite(LED_PIN, LOW);    // turn the LED off
  delay(1000);
  digitalWrite(LED_PIN, HIGH);   // turn the LED on 
  delay(1000);              
  digitalWrite(LED_PIN, LOW);    // turn the LED off

  delay(1000);  // Delay added to make serial more reliable
  
  Serial.begin(9600);
  while (! Serial); // Wait until Serial is ready
  Serial.println("Awake");  
  */
}

void alarmMatch(void) // Do something when interrupt called
{
  
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

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(5); // Set LPF corner frequency to 5Hz

  if (enableInterrupt)
  {
    // Set interrupt pin with internal pullup (active low)
    pinMode(IMU_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(IMU_INTERRUPT_PIN, imuInterruptHandler, FALLING);

    // interrupt output as a "data ready" indicator.
    imu.enableInterrupt();

    // Configure as active-low, since we'll be using the pin's
    // internal pull-up resistor.
    // Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
    imu.setIntLevel(INT_ACTIVE_LOW);

    // The interrupt can be set to latch until data has
    // been read, or to work as a 50us pulse.
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
  // Data pin, the sensor is 1-wire
  pinMode(PIR_PIN, INPUT);
  if (interruptEnable)
  {
    attachInterrupt(PIR_PIN, pirInterruptHandler, HIGH);
  }
}

void pirInterruptHandler(void)
{
  pirInterruptFlag = true;
}

void flashLED(void)
{
  // Simple indication of being awake
  digitalWrite(LED_PIN, HIGH);   // turn the LED on 
  delay(1000);              
  digitalWrite(LED_PIN, LOW);    // turn the LED off
  delay(1000);
  digitalWrite(LED_PIN, HIGH);   // turn the LED on 
  delay(1000);              
  digitalWrite(LED_PIN, LOW);    // turn the LED off
  digitalWrite(LED_PIN, HIGH);   // turn the LED on 
  delay(1000);              
  digitalWrite(LED_PIN, LOW);    // turn the LED off
}

int8_t readPirValue()
{
  return digitalRead(PIR_PIN);
}