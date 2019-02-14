#include <RTCZero.h>
#include "MPU9250.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define SerialPort Serial

void imuSetup(bool enableInterrupt);
void imu_setInterruptLevel(bool activeHigh);
void imuInterruptHandler(void);

void pirSetup(bool interruptEnable);
void pirInterruptHandler(void);
int8_t readPirValue(void);

void flashLED(uint8_t ledPin, uint8_t numberOfToggles, uint16_t delayInMs);
void do_send(osjob_t *j);
void onEvent(ev_t ev);

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

static bool imuInterruptFlag = false;
static bool pirInterruptFlag = false;
static bool sendJobIsDone    = false;

// This EUI must be in little-endian format. 
// For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
static const u1_t PROGMEM APPEUI[8]= { 0xDA, 0x7D, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= { 0x35, 0x23, 0x5B, 0x72, 0x32, 0xDF, 0x0A, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format
static const u1_t PROGMEM APPKEY[16] = { 0x6E, 0xC7, 0xAC, 0xF1, 0x94, 0x01, 0x1A, 0xC2, 0xF9, 0x16, 0x50, 0xE4, 0x3A, 0x03, 0x34, 0x64 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;
const unsigned TX_INTERVAL = 60;

// Pin mapping for Adafruit Feather M0 LoRa
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};

void setup() 
{
  pinMode(STATUS_LED, OUTPUT);
  SerialPort.begin(115200);
  delay(5000);
  rtc.begin();

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF7,14);

  sendJobIsDone = false;
  do_send(&sendjob);
  do
  {
    os_runloop_once();
  } while (sendJobIsDone == false);
  sendJobIsDone = true;

  imuSetup(true);
  pirSetup(true);

  Serial.println("setup successful");
  delay(5000);
  Serial.end();
  USBDevice.detach(); // Safely detach the USB prior to sleeping
  rtc.standbyMode();
}

void loop()
{
  if (pirInterruptFlag || imuInterruptFlag)
  {
    if (pirInterruptFlag)
    {
      //pirInterruptFlag = false;
      flashLED(STATUS_LED, 6, 100);
    }
    else if (imuInterruptFlag)
    {
      //imuInterruptFlag = false;
      flashLED(STATUS_LED, 3, 200);
    }    
    
    USBDevice.attach();
    Serial.begin(115200);
    delay(5000);
    sendJobIsDone = false;
    Serial.println("preparing to send");

    do_send(&sendjob);
    sendJobIsDone = false;
    do
    {
      os_runloop_once();
    } while (sendJobIsDone == false);
    sendJobIsDone = true;

    Serial.println("ending..");
    delay(1000);
    Serial.end();
    USBDevice.detach(); // Safely detach the USB prior to sleeping
    rtc.standbyMode();
  }
}

// Set up the mpu9250 imu sensors
// including its gyro, accel and compass
void imuSetup(bool enableInterrupt)
{
  int8_t imu_status;
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

    imu_setInterruptLevel(false);

    // Set interrupt pin with internal pullup (active low)
    pinMode(IMU_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuInterruptHandler, FALLING);

    // Configure EIC to use GCLK1 which uses XOSC32K 
    // This has to be done after the first call to attachInterrupt()
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY);

    flashLED(STATUS_LED, 5, 300);
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

void imu_setInterruptLevel(bool activeHigh)
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
}

void onEvent(ev_t ev)
{
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev)
  {
  case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    {
      u4_t netid = 0;
      devaddr_t devaddr = 0;
      u1_t nwkKey[16];
      u1_t artKey[16];
      LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      Serial.print("netid: ");
      Serial.println(netid, DEC);
      Serial.print("devaddr: ");
      Serial.println(devaddr, HEX);
      Serial.print("artKey: ");
      for (int i = 0; i < sizeof(artKey); ++i)
      {
        if (i != 0)
          Serial.print("-");
        Serial.print(artKey[i], HEX);
      }
      Serial.println("");
      Serial.print("nwkKey: ");
      for (int i = 0; i < sizeof(nwkKey); ++i)
      {
        if (i != 0)
          Serial.print("-");
        Serial.print(nwkKey[i], HEX);
      }
      Serial.println("");
    }
    // Disable link check validation (automatically enabled
    // during join, but because slow data rates change max TX
    // size, we don't use it in this example.
    LMIC_setLinkCheckMode(0);
    //sendJobIsDone = true;
    break;
  /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ack"));
    if (LMIC.dataLen)
    {
      Serial.println(F("Received "));
      Serial.println(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
    }
    // Schedule next transmission
    //os_setCallback(&sendjob, do_send);
    sendJobIsDone = true;
    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
  /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
  case EV_TXSTART:
    Serial.println(F("EV_TXSTART"));
    break;
  default:
    Serial.print(F("Unknown event: "));
    Serial.println((unsigned)ev);
    break;
  }
}

void do_send(osjob_t *j)
{
  static uint8_t mydata[2] = {0};

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  { 
    mydata[0] = pirInterruptFlag ? 1 : 0;
    mydata[1] = imuInterruptFlag ? 1 : 0;
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
  }
}