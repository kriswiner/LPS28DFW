/* Basic sketch for Katydid.v02a (aka Connected Motion Sensor.v02a)
 *  
 *  Demonstrate RTC time keeping and use of RTC alarm for serial output
 *  Demonstrate BMA400 wake-on-motion and sleep-on-no-motion
 *  
 *  Intended to run without any module installed.
 *  
 
   This example code is in the public domain.
*/
#include "Arduino.h"
#include "STM32WB.h"
#include "LSM6DSM.h"
#include "BMA400.h"
#include "LPS28DFW.h"
#include "I2Cdev.h"
#include "SFLASH.h"
#include "RTC.h"

#define I2C_BUS    Wire1              // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

float VBAT, VDDA, Temperature;
uint32_t UID[3] = {0, 0, 0};
volatile bool USBConnected = false; 
bool SerialDebug = true;

uint8_t mid;
uint16_t did;

uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
volatile bool alarmFlag = false; // for RTC alarm interrupt

const uint8_t myLed = 7;  // green led

//LSM6DSM definitions
#define LSM6DSM_intPin1 5  // interrupt1 pin definitions, significant motion

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_245DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
      AODR_12_5Hz, AODR_26Hz, AODR_52Hz, AODR_104Hz, AODR_208Hz, AODR_416Hz, AODR_833Hz, AODR_1660Hz, AODR_3330Hz, AODR_6660Hz
      GODR_12_5Hz, GODR_26Hz, GODR_52Hz, GODR_104Hz, GODR_208Hz, GODR_416Hz, GODR_833Hz, GODR_1660Hz, GODR_3330Hz, GODR_6660Hz
*/ 
uint8_t AscaleL = AFS_2G, GscaleL = GFS_245DPS, AODRL = AODR_208Hz, GODRL = GODR_416Hz;

float aResL, gResL;              // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {-0.00499, 0.01540, 0.02902}, gyroBias[3] = {-0.50, 0.14, 0.28}; // offset biases for the accel and gyro
int16_t LSM6DSMData[7];        // Stores the 16-bit signed sensor output
float   Gtemperature;          // Stores the real internal gyro temperature in degrees Celsius
float axL, ayL, azL, gxL, gyL, gzL;  // variables to hold latest accel/gyro data values 

bool newLSM6DSMData = false;

LSM6DSM LSM6DSM(&i2c_0); // instantiate LSM6DSM class


//BMA400 definitions
const uint8_t BMA400_intPin1 = 20;    // interrupt1 pin definitions, wake-up from STANDBY pin
const uint8_t BMA400_intPin2 = 21;    // interrupt2 pin definitions, sleep interrupt

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFSB_2G, AFSB_4G, AFSB_8G, AFSB_16G  
      SR_15_5Hz, SR_25Hz, SR_50Hz, SR_100Hz, SR_200Hz, SR_400Hz, SR_800Hz 
      sleep_Mode, lowpower_Mode, normal_Mode, sleep_Mode
      osr0 (lowest power, lowest oversampling,lowest accuracy), osr1, osr2, osr3 (highest power, highest oversampling, highest accuracy)
      acc_filt1 (variable filter), acc_filt2 (fixed 100 Hz filter), acc_filt_lp (fixed 100 Hz filter, 1 Hz bandwidth)
*/ 
uint8_t AscaleB = AFSB_2G, SR = SR_200Hz, power_Mode = lowpower_Mode, OSR = osr0, acc_filter = acc_filt2;

float aResB;            // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float axB, ayB, azB;    // variables to hold latest sensor data values 
float offset[3];        // accel bias offsets

// Logic flags to keep track of device states
bool BMA400_wake_flag = false;
bool BMA400_sleep_flag = false;
bool InMotion = false;

BMA400 BMA400(&i2c_0); // instantiate BMA400 class


// LPS28DFW definitions
const uint8_t LPS28DFW0_intPin = 8;    // interrupt for LPS28DFW on main board
const uint8_t LPS28DFW1_intPin = 9;    // interrupt for LPS28DFW on daughter board

// configure the pressure sensors, assuming both configured the same
const uint8_t odr = ODR_10Hz;  // choices are ODR_PwrDown, ODR_oneShot, ODR_1Hz, ODR_4Hz, ODR_10Hz, ODR_25Hz, ODR_50Hz, ODR_75Hz, ODR_100Hz,and ODR_200Hz
const uint8_t avg = AVG_4 ;   // choices are AVG_4, AVG_8, AVG_16, AVG_32, AVG_64, AVG_128, AVG_512 (more averaging, les noise, more current usage)
const uint8_t fs = FS_MODE1;  // choices are FS_MODE1 (up to 1260 hPa) or FS_MODE2 (up to 4060 hPa)
const uint8_t lpf = LPF_NONE; // Choices are LPF_ODR_4 (odr/4), LPF_ODR_9 (odr/9), or LPF_NONE (no lpf filter) 

uint8_t LPS28DFW_status = 0;
int32_t LPS28DFW0_rawpressure = 0, LPS28DFW1_rawpressure = 0;
int16_t LPS28DFW0_rawtemperature = 0, LPS28DFW1_rawtemperature = 0;
float LPS28DFW0_pressure = 0.0f, LPS28DFW1_pressure = 0.0f, LPS28DFW0_temperature = 0.0f, LPS28DFW1_temperature = 0.0f, pressScale = 0.0f;
float LPS28DFW0_avgFIFOPressure = 0.0f, LPS28DFW1_avgFIFOPressure = 0.0f;
volatile bool LPS28DFW0_flag = true, LPS28DFW1_flag = true;

const uint8_t fifoMode = FIFOMODE; // choices are BYPASS, FIFOMODE, CONTINUOUS
const uint8_t wtm = 10;  // set watermark
uint8_t fifoLevel = 0, fifoStatus = 0, FStatus[2] = {0, 0};;
int32_t fifoSum = 0;
   
LPS28DFW LPS28DFW(&i2c_0); // instantiate LPS28DFW class


void setup() 
{
  if(SerialDebug) Serial.begin(38400);
  while (!Serial) { }
  if(SerialDebug) Serial.println("Serial enabled!");

  STM32WB.getUID(UID);
  if(SerialDebug) {Serial.print("STM32L4 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX);} 

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);  // start with led on, since active HIGH
 
  // Set up the I2C sensors
  pinMode(LSM6DSM_intPin1, INPUT);   // define data ready interrupt for LSM6DSM
  pinMode(BMA400_intPin1, INPUT);    // define BMA400 wake and sleep interrupt pins as STM32WB inputs
  pinMode(BMA400_intPin2, INPUT);
  pinMode(LPS28DFW0_intPin, INPUT);  // define LPS28DFW0 interrupt
  pinMode(LPS28DFW1_intPin, INPUT);  // define LPS28DFW1 interrupt

  I2C_BUS.begin();                // Set master mode, default on SDA/SCL for STM32L4
  I2C_BUS.setClock(400000);       // I2C frequency at 400 kHz
  delay(1000);
  
  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                // should detect BMA400 at 0x14  
  delay(1000);

  // Check device IDs
  // Read the BMA400 Chip ID register, this is a good test of communication
  Serial.println("BMA400 accelerometer...");
  byte BMA400_ID = BMA400.getChipID();  // Read CHIP_ID register for BMA400
  Serial.print("BMA400 "); Serial.print("I AM "); Serial.print(BMA400_ID, HEX); Serial.print(" I should be "); Serial.println(0x90, HEX);
  Serial.println(" ");
  delay(1000);   
  
  // Read the LSM6DSM Chip ID register 
  Serial.println("LSM6DSM accel/gyro...");
  byte LSM6DSM_ID = LSM6DSM.getChipID();  // Read CHIP_ID register for LSM6DSM
  Serial.print("LSM6DSM "); Serial.print("I AM "); Serial.print(LSM6DSM_ID, HEX); Serial.print(" I should be "); Serial.println(0x6A, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the LPS28DW_0 WHO_AM_I register 
  Serial.println("LPS28DFW_0 barometer...");
  byte LPS28DFW_0_ID = LPS28DFW.getChipID(LPS28DFW_ADDRESS_0);  // Read CHIP_ID register for LPS28DFW_0
  Serial.print("LPS28DFW_0 "); Serial.print("I AM "); Serial.print(LPS28DFW_0_ID, HEX); Serial.print(" I should be "); Serial.println(0xB4, HEX);
  Serial.println(" ");
  delay(1000); 

  // Read the LPS28DW_1 WHO_AM_I register 
  Serial.println("LPS28DFW_1 barometer...");
  byte LPS28DFW_1_ID = LPS28DFW.getChipID(LPS28DFW_ADDRESS_1);  // Read CHIP_ID register for LPS28DFW_1
  Serial.print("LPS28DFW_1 "); Serial.print("I AM "); Serial.print(LPS28DFW_1_ID, HEX); Serial.print(" I should be "); Serial.println(0xB4, HEX);
  Serial.println(" ");
  delay(1000); 

  if(BMA400_ID == 0x90 && LSM6DSM_ID == 0x6A && LPS28DFW_0_ID == 0xB4 && LPS28DFW_1_ID == 0xB4) // check if BMA400 and LSM6DSM and two LPS28DFWs have acknowledged
  {
   Serial.println("LSM6DSM and BMA400 and both LPS28DFWs are online..."); Serial.println(" ");
   
   LSM6DSM.reset();  // software reset LSM6DSM to default registers

   // get sensor resolutions, only need to do this once
   aResL = LSM6DSM.getAres(AscaleL);
   gResL = LSM6DSM.getGres(GscaleL);

   LSM6DSM.init(AscaleL, GscaleL, AODRL, GODRL);

   Serial.println("Initialize the LSM6DSM");
   LSM6DSM.selfTest();

   LSM6DSM.offsetBias(gyroBias, accelBias);
   Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
   Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
   Serial.println(" ");
   delay(1000); 

  Serial.println("Initialize the BMA400");
  aResB = BMA400.getAres(AscaleB);                                              // get sensor resolutions, only need to do this once
  BMA400.resetBMA400();                                                         // software reset before initialization
  delay(100);      
  BMA400.selfTestBMA400();                                                      // perform sensor self test
  BMA400.resetBMA400();                                                         // software reset before initialization
  delay(100);                                                                   // give some time to read the screen
  BMA400.CompensationBMA400(AscaleB, SR, normal_Mode, OSR, acc_filter, offset); // quickly estimate offset bias in normal mode
  BMA400.initBMA400(AscaleB, SR, power_Mode, OSR, acc_filter);                  // Initialize sensor in desired mode for application                     

  LPS28DFW.reset(LPS28DFW_ADDRESS_0); // software reset LPS28DFWs to default registers
  delay(100);
  LPS28DFW.reset(LPS28DFW_ADDRESS_1);
  delay(100);
  if(fs == FS_MODE1) pressScale = 1.0f/4096.0f; // get pressure scale, depends on full-scale selection
  if(fs == FS_MODE2) pressScale = 1.0f/2048.0f;
  LPS28DFW.init(LPS28DFW_ADDRESS_0, odr, avg, fs, lpf); // configure LPS28DFW_0
  delay(100);
  LPS28DFW.init(LPS28DFW_ADDRESS_1, odr, avg, fs, lpf); // configure LPS28DFW_1
  delay(100);

  LPS28DFW.initFIFO(LPS28DFW_ADDRESS_0, fifoMode, wtm); // configure LPS28DFW_0 FIFO
  LPS28DFW.initFIFO(LPS28DFW_ADDRESS_1, fifoMode, wtm); // configure LPS28DFW_1 FIFO
  }
  else 
  {
  if(BMA400_ID     != 0x90) Serial.println(" BMA400     not functioning!");     
  if(LSM6DSM_ID    != 0x6A) Serial.println(" LSM6DSM    not functioning!");     
  if(LPS28DFW_0_ID != 0xB4) Serial.println(" LPS28DFW_0 not functioning!");     
  if(LPS28DFW_1_ID != 0xB4) Serial.println(" LPS28DFW_1 not functioning!");  
  while(1) {}; // Ruh-roh! Wait here indefinitely...
  }

  digitalWrite(myLed, LOW); // turn off led after sensors are configured

  // Test QSPI flash memory
  Serial.println("QSPI Flash Check");
  SFLASH.begin();
  SFLASH.identify(mid, did);
  Serial.print("MID = ");       Serial.println(mid, HEX); 
  Serial.print("DID = ");       Serial.println(did, HEX); 
  Serial.print("CAPACITY = ");  Serial.println(SFLASH.capacity());
  Serial.print("BLOCKSIZE = "); Serial.println(SFLASH.blockSize());
  Serial.print("PAGESIZE = ");  Serial.println(SFLASH.pageSize());
  Serial.print("LENGTH = ");    Serial.println(SFLASH.length()); Serial.println(" ");

  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC_MATCH_ANY); // alarm once a second  
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  attachInterrupt(LSM6DSM_intPin1,  myinthandler1, RISING);  // define interrupt for intPin1 output of LSM6DSM
  attachInterrupt(BMA400_intPin1,   myinthandler2, RISING);  // attach in-motion  interrupt for INT1 pin output of BMA400
  attachInterrupt(BMA400_intPin2,   myinthandler3, RISING);  // attach no-motion  interrupt for INT2 pin output of BMA400 
  attachInterrupt(LPS28DFW0_intPin, myinthandler4, RISING);  // attach data ready interrupt for LPS28DFW0
  attachInterrupt(LPS28DFW1_intPin, myinthandler5, RISING);  // attach data ready interrupt for LPS28DFW1

  BMA400.getStatus(); // read status of interrupts to clear
  LPS28DFW.getStatus(LPS28DFW_ADDRESS_0); // read status of interrupts to clear
  LPS28DFW.getStatus(LPS28DFW_ADDRESS_1); // read status of interrupts to clear
  } // end of setup

void loop() 
{
  /* BMA400 sleep/wake detect*/
  if(BMA400_wake_flag)
  {
   Serial.println("** BMA400 is awake! **");
   BMA400_wake_flag = false; // clear the wake flag
   InMotion = true;          // set motion state latch
   BMA400.activateNoMotionInterrupt();  
   attachInterrupt(BMA400_intPin2, myinthandler3, RISING);  // attach no-motion interrupt for INT2 pin output of BMA400 

   digitalWrite(myLed, HIGH); // turn on green led when motion detected
   }

  if(BMA400_sleep_flag)
  {
   Serial.println("** BMA400 is asleep! **");
   BMA400_sleep_flag = false;            // clear the sleep flag
   InMotion = false;                     // set motion state latch
   detachInterrupt(BMA400_intPin2);      // Detach the BMA400 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L4
   BMA400.deactivateNoMotionInterrupt(); // disable no-motion interrupt to save power 

   digitalWrite(myLed, LOW); // turn off green led when no motion, i.e, going to sleep
   }/* end of sleep/wake detect */


   // If intPin goes high, either all data registers have new data
   if(newLSM6DSMData) {   // On interrupt, read data
      newLSM6DSMData = false;     // reset newData flag

     LSM6DSM.readData(LSM6DSMData); // INT1 cleared on any read
   
   // Now we'll calculate the accleration value into actual g's
     axL = (float)LSM6DSMData[4]*aResL - accelBias[0];  // get actual g value, this depends on scale being set
     ayL = (float)LSM6DSMData[5]*aResL - accelBias[1];   
     azL = (float)LSM6DSMData[6]*aResL - accelBias[2];  

   // Calculate the gyro value into actual degrees per second
     gxL = (float)LSM6DSMData[1]*gResL - gyroBias[0];  // get actual gyro value, this depends on scale being set
     gyL = (float)LSM6DSMData[2]*gResL - gyroBias[1];  
     gzL = (float)LSM6DSMData[3]*gResL - gyroBias[2];     
   }
   

   // Handle LPS28DFW_0 interrupt
   if(LPS28DFW0_flag) {           // On interrupt, read data
      LPS28DFW0_flag = false;     // reset INT flag

      //Chaech for data ready interrupts
      LPS28DFW_status = LPS28DFW.getStatus(LPS28DFW_ADDRESS_0); // check which interrupt has triggered
      // Serial.print("LPS28DFW0 status = "); Serial.println(LPS28DFW_status, HEX);
      if(LPS28DFW_status & 0x20) Serial.println("LPS28DFW0 Temperature data overrun!"); // simple error handling
      if(LPS28DFW_status & 0x10) Serial.println("LPS28DFW0 Pressure data overrun!");
      
      if(LPS28DFW_status & 0x01) { // new pressure data ready
      LPS28DFW0_rawpressure = LPS28DFW.Pressure(LPS28DFW_ADDRESS_0);      
      LPS28DFW0_pressure = ((float) LPS28DFW0_rawpressure) * pressScale;
      }
      
      if(LPS28DFW_status & 0x02) { // new temperature data ready
      LPS28DFW0_rawtemperature = LPS28DFW.Temperature(LPS28DFW_ADDRESS_0);
      LPS28DFW0_temperature = ((float) LPS28DFW0_rawtemperature) / 100.0f;
      }

      // Check for FIFO interrupts
      LPS28DFW.FIFOStatus(LPS28DFW_ADDRESS_0, FStatus); // read FIFO status registers
      fifoLevel  = FStatus[0];
      fifoStatus = FStatus[1];
      // Serial.print("FIFO Status = "); Serial.println(fifoStatus);
      // Serial.print("FIFO Level = "); Serial.println(fifoLevel);
      if(fifoStatus & 0x40) Serial.println("FIFO is full and at least one sample in the FIFO has been overwritten!"); // SImple error handling
      if(fifoStatus & 0x20) Serial.println("FIFO is completely filled, no samples overwritten!");  
      if(fifoStatus & 0x80) {
          fifoSum = 0;
          for (uint8_t ii = 0; ii < fifoLevel; ii++) {
          fifoSum += LPS28DFW.FIFOPressure(LPS28DFW_ADDRESS_0);
          }
          if(fifoLevel != 0) LPS28DFW0_avgFIFOPressure = ((float) fifoSum / (float) fifoLevel) * pressScale;
          Serial.print("Average LPS28DFW0 FIFO Pressure (hPa) over "); Serial.print(fifoLevel); Serial.print(" samples = "); Serial.println(LPS28DFW0_avgFIFOPressure, 2);
          LPS28DFW.FIFOReset(LPS28DFW_ADDRESS_0);               // reset LPS28DFW_0 FIFO
          LPS28DFW.initFIFO(LPS28DFW_ADDRESS_0, fifoMode, wtm); // restart LPS28DFW_0 FIFO
      }
    }

   // Handle LPS28DFW_1 interrupt
   if(LPS28DFW1_flag) {           // On interrupt, read data
      LPS28DFW1_flag = false;     // reset int flag

      // Check for data ready interrupts
      LPS28DFW_status = LPS28DFW.getStatus(LPS28DFW_ADDRESS_1); // check which interrupt has triggered
      // Serial.print("LPS28DFW1 status = "); Serial.println(LPS28DFW_status, HEX);
      if(LPS28DFW_status & 0x20) Serial.println("LPS28DFW1 Temperature data overrun!"); // simple error handling
      if(LPS28DFW_status & 0x10) Serial.println("LPS28DFW1 Pressure data overrun!");
      
      if(LPS28DFW_status & 0x01) { // new pressure data ready
      LPS28DFW1_rawpressure = LPS28DFW.Pressure(LPS28DFW_ADDRESS_1);   
      LPS28DFW1_pressure = ((float) LPS28DFW1_rawpressure) * pressScale;
      }
      
      if(LPS28DFW_status & 0x02) { // new temperature data ready
      LPS28DFW1_rawtemperature = LPS28DFW.Temperature(LPS28DFW_ADDRESS_1);
      LPS28DFW1_temperature = ((float) LPS28DFW1_rawtemperature) / 100.0f;
      }   

      // Check for FIFO interrupts
      LPS28DFW.FIFOStatus(LPS28DFW_ADDRESS_1, FStatus); // read FIFO status registers
      fifoLevel  = FStatus[0];
      fifoStatus = FStatus[1];
      // Serial.print("FIFO Status = "); Serial.println(fifoStatus);
      // Serial.print("FIFO Level = "); Serial.println(fifoLevel);
      if(fifoStatus & 0x40) Serial.println("FIFO is full and at least one sample in the FIFO has been overwritten!"); // Simple error handling
      if(fifoStatus & 0x20) Serial.println("FIFO is completely filled, no samples overwritten!");  
      if(fifoStatus & 0x80) {
          fifoSum = 0;
          for (uint8_t ii = 0; ii < fifoLevel; ii++) {
          fifoSum += LPS28DFW.FIFOPressure(LPS28DFW_ADDRESS_1);
          }
          if(fifoLevel != 0) LPS28DFW1_avgFIFOPressure = ((float) fifoSum / (float) fifoLevel) * pressScale;
          Serial.print("Average LPS28DFW1 FIFO Pressure (hPa) over "); Serial.print(fifoLevel); Serial.print(" samples = "); Serial.println(LPS28DFW1_avgFIFOPressure, 2);
          LPS28DFW.FIFOReset(LPS28DFW_ADDRESS_1);               // reset LPS28DFW_1 FIFO
          LPS28DFW.initFIFO(LPS28DFW_ADDRESS_1, fifoMode, wtm); // restart LPS28DFW_1 FIFO
      }
   }
   // end sensor interrupt handling

  
  /*RTC Timer*/
  if (alarmFlag) { // update serial output whenever there is a timer alarm
      alarmFlag = false;

  if(odr == ODR_oneShot)  {
    LPS28DFW.oneShot(LPS28DFW_ADDRESS_0);  // use one shot mode for aperiodic barometer measurements
    LPS28DFW.oneShot(LPS28DFW_ADDRESS_1);
  }

  if(InMotion) { // output accel data when BMA400 is active
      
   BMA400.readBMA400AccelData(accelCount); // get 12-bit signed accel data

  // Now we'll calculate the acceleration value into actual g's
  axB = (float)accelCount[0]*aResB - offset[0];  // get actual g value, this depends on scale being set
  ayB = (float)accelCount[1]*aResB - offset[1];   
  azB = (float)accelCount[2]*aResB - offset[2]; 
     
  Serial.println(" ");
  Serial.println("BMA400:");
  Serial.print("ax = ");  Serial.print((int)1000*axB);  
  Serial.print(" ay = "); Serial.print((int)1000*ayB); 
  Serial.print(" az = "); Serial.print((int)1000*azB); Serial.println(" mg");
  Serial.println(" ");
  }

  Serial.println(" ");
  Serial.println("LSM6DSM:");
  Serial.print("ax = "); Serial.print((int)1000*axL);  
  Serial.print(" ay = "); Serial.print((int)1000*ayL); 
  Serial.print(" az = "); Serial.print((int)1000*azL); Serial.println(" mg");
  Serial.print("gx = "); Serial.print( gxL, 2); 
  Serial.print(" gy = "); Serial.print( gyL, 2); 
  Serial.print(" gz = "); Serial.print( gzL, 2); Serial.println(" deg/s");

  Gtemperature = ((float) LSM6DSMData[0]) / 256.0f + 25.0f; // Gyro chip temperature in degrees Centigrade
  // Print temperature in degrees Centigrade      
  Serial.print("Gyro temperature is ");  Serial.print(Gtemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C

  Serial.println(" ");
  Serial.print("Latest LPS28DFW0 pressure (hPa) = "); Serial.print(LPS28DFW0_pressure, 2); Serial.print(", Temperature (oC) = "); Serial.println(LPS28DFW0_temperature, 1);
  Serial.print("Latest LPS28DFW1 pressure (hPa) = "); Serial.print(LPS28DFW1_pressure, 2); Serial.print(", Temperature (oC) = "); Serial.println(LPS28DFW1_temperature, 1);
  Serial.println(" ");

  Serial.println("RTC:");
  Day = RTC.getDay();
  Month = RTC.getMonth();
  Year = RTC.getYear();
  Seconds = RTC.getSeconds();
  Minutes = RTC.getMinutes();
  Hours   = RTC.getHours();     
  if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
  Serial.print(":"); 
  if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
  Serial.print(":"); 
  if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

  Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
  Serial.println(" ");
  
  VDDA = STM32WB.readVDDA();
  Temperature = STM32WB.readTemperature();
  USBConnected = USBDevice.attached();
  VBAT = STM32WB.readBattery();

  if(SerialDebug)   Serial.print("VDDA = "); Serial.println(VDDA, 2); 
  if(SerialDebug)   Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);
  if(USBConnected && SerialDebug) Serial.println("USB connected!");
  if(SerialDebug)   Serial.print("VBAT = "); Serial.println(VBAT, 2); 
  Serial.println(" ");

  digitalWrite(myLed, HIGH); delay(10);  digitalWrite(myLed, LOW); // toggle blue led on
   
  } /* End of RTC Timer Handling */
  
//  STM32WB.stop();
  STM32WB.sleep();
} /* End of Main Loop */


// Useful functions

void myinthandler1()
{
  newLSM6DSMData = true;
  STM32WB.wakeup();
}


void myinthandler2()
{
  BMA400_wake_flag = true; 
  STM32WB.wakeup();
}


void myinthandler3()
{
  BMA400_sleep_flag = true;
  STM32WB.wakeup();
}


void myinthandler4()
{
  LPS28DFW0_flag = true;
  STM32WB.wakeup();
}


void myinthandler5()
{
  LPS28DFW1_flag = true;
  STM32WB.wakeup();
}


void alarmMatch()
{
  alarmFlag = true;
  STM32WB.wakeup();
}
