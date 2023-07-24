/* 07/22/2023 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  The LPS28DFW is an ultra-compact piezoresistive absolute pressure sensor which
*   functions as a digital output barometer
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#include "LPS28DFW.h"
#include "I2CDev.h"

LPS28DFW::LPS28DFW(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


uint8_t LPS28DFW::getChipID(uint8_t dev_address)
  {
  uint8_t temp = _i2c_bus->readByte(dev_address, LPS28DFW_WHO_AM_I);
  return temp;
  }


void LPS28DFW::reset(uint8_t dev_address)
  {
  uint8_t temp = _i2c_bus->readByte(dev_address, LPS28DFW_CTRL_REG2);  
  _i2c_bus->writeByte(dev_address, LPS28DFW_CTRL_REG2, temp | 0x04); // software reset the LPS28DFW
  }


void LPS28DFW::boot(uint8_t dev_address)
  {
  uint8_t temp = _i2c_bus->readByte(dev_address, LPS28DFW_CTRL_REG2);  
  _i2c_bus->writeByte(dev_address, LPS28DFW_CTRL_REG2, temp | 0x80); // re-boot the LPS28DFW
  uint8_t status = 0;
  while(!status) { // wait for boot process to complete
   status = _i2c_bus->readByte(dev_address, LPS28DFW_INT_SOURCE) & 0x80; 
  } 
  }


uint8_t LPS28DFW::getStatus(uint8_t dev_address)
{
  uint8_t temp = _i2c_bus->readByte(dev_address, LPS28DFW_STATUS);
  return temp;
}


void LPS28DFW::FIFOStatus(uint8_t dev_address, uint8_t * dest)
{
  uint8_t rawData[2]= {0, 0};
  _i2c_bus->readBytes(dev_address, LPS28DFW_FIFO_STATUS1, 2, &rawData[0]);
  dest[0] = rawData[0];
  dest[1] = rawData[1];
}


void LPS28DFW::FIFOReset(uint8_t dev_address)
{
 _i2c_bus->writeByte(dev_address, LPS28DFW_FIFO_CTRL,  0x00); // Disable water mark and enable BYPASS mode 
}


void LPS28DFW::init(uint8_t dev_address, uint8_t odr, uint8_t avg, uint8_t fs, uint8_t lpf)
{
   // set ODR (bits 3 - 6) and averaging (bits 0 - 2)
   _i2c_bus->writeByte(dev_address, LPS28DFW_CTRL_REG1,  odr << 3 | avg);

  // set full-scale (bit 6)  and low-pass filter (bit 5), enable BDU (bit 3) to avoid mis-matched data
   if(lpf == LPF_NONE) {
    _i2c_bus->writeByte(dev_address, LPS28DFW_CTRL_REG2,  fs << 6 | 0x08);
   }
  else {
    _i2c_bus->writeByte(dev_address, LPS28DFW_CTRL_REG2,  fs << 6 | lpf << 5 | 0x10 | 0x08);    // enable low-pass filter (bit 4)
  }
    
   // Interrupt configuration //
   // Default interrupt is push-pull, active HIGH with auto-address-increment enabled (see CTRL_REG3)
   _i2c_bus->writeByte(dev_address, LPS28DFW_CTRL_REG4, 0x07 << 4 | 0x07);   // route all possible interrupts to the INT pin  

   // Bit 6 is Data-ready pulsed on INT_DRDY pin.
   // Bit 5 is Date-ready signal on INT_DRDY pin.        
   // Bit 4 is Interrupt  enable on INT_DRDY pin.
   // Bit 2 is FIFO full flag on INT_DRDY pin        
   // Bit 1 is FIFO threshold (watermark) status on INT_DRDY pin. 
   // Bit 0 is FIFO overrun status on INT_DRDY pin.
   // We will check them all in the interrupt handler in the main code.
}        


  void LPS28DFW::initFIFO(uint8_t dev_address, uint8_t fmode, uint8_t wtm)
  {
     _i2c_bus->writeByte(dev_address, LPS28DFW_FIFO_WTM,  wtm); // define watermark
     _i2c_bus->writeByte(dev_address, LPS28DFW_FIFO_CTRL,  0x08 | fmode); // Enable water mark and enable FIFO mode 
  }

void LPS28DFW::oneShot(uint8_t dev_address)
{
  uint8_t temp = _i2c_bus->readByte(dev_address, LPS28DFW_CTRL_REG2);  
  _i2c_bus->writeByte(dev_address, LPS28DFW_CTRL_REG2, temp | 0x01); // set bit 0 to enable one shot mode, will reset to 0 after execution
}


int32_t LPS28DFW::Pressure(uint8_t dev_address)
{
    uint8_t rawData[3];  // 24-bit pressure register data stored here
    _i2c_bus->readBytes(dev_address, LPS28DFW_PRESSURE_OUT_XL, 3, &rawData[0]);  
    return (int32_t) (((int32_t) rawData[2] << 24 | (int32_t) rawData[1] << 16 | rawData[0] << 8 | 0x00) >> 8);
}


int16_t LPS28DFW::Temperature(uint8_t dev_address)
{
  uint8_t rawData[2];  // 16-bit pressure register data stored here
  _i2c_bus->readBytes(dev_address, LPS28DFW_TEMP_OUT_L, 2, &rawData[0]);    // Read the raw data register  
  int16_t temp = (int16_t) ( ((int16_t)rawData[1] << 8) )| rawData[0];       // Turn into signed 16-bit temperature value
  return temp;
  }


int32_t LPS28DFW::FIFOPressure(uint8_t dev_address)
{
    uint8_t rawData[3];  // 24-bit pressure register data stored here
    _i2c_bus->readBytes(dev_address, LPS28DFW_FIFO_DATA_OUT_PRESS_XL, 3, &rawData[0]);  
    return (int32_t) (((int32_t) rawData[2] << 24 | (int32_t) rawData[1] << 16 | rawData[0] << 8 | 0x00) >> 8);
}
