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
 
#ifndef LPS28DFW_h
#define LPS28DFW_h

#include "Arduino.h"
#include "I2CDev.h"
#include <Wire.h>

/* Register Map LPS28DFW
// https://www.st.com/resource/en/datasheet/lps28dfw.pdf
*/
#define LPS28DFW_WHO_AM_I                   0x0F  // should be 0xB4

#define LPS28DFW_INTERRUPT_CFG              0x0B
#define LPS28DFW_THS_P_L                    0x0C
#define LPS28DFW_THS_P_H                    0x0D
#define LPS28DFW_IF_CTRL                    0x0E

#define LPS28DFW_CTRL_REG1                  0x10
#define LPS28DFW_CTRL_REG2                  0x11
#define LPS28DFW_CTRL_REG3                  0x12
#define LPS28DFW_CTRL_REG4                  0x13
#define LPS28DFW_FIFO_CTRL                  0x14
#define LPS28DFW_FIFO_WTM                   0x15
#define LPS28DFW_REF_P_L                    0x16
#define LPS28DFW_REF_P_H                    0x17

#define LPS28DFW_I3C_IF_CTRL                0x19
#define LPS28DFW_RPDS_L                     0x1A
#define LPS28DFW_RPDS_H                     0x1B

#define LPS28DFW_INT_SOURCE                 0x24
#define LPS28DFW_FIFO_STATUS1               0x25
#define LPS28DFW_FIFO_STATUS2               0x26
#define LPS28DFW_STATUS                     0x27
#define LPS28DFW_PRESSURE_OUT_XL            0x28
#define LPS28DFW_PRESSURE_OUT_L             0x29
#define LPS28DFW_PRESSURE_OUT_H             0x2A
#define LPS28DFW_TEMP_OUT_L                 0x2B
#define LPS28DFW_TEMP_OUT_H                 0x2C

#define LPS28DFW_FIFO_DATA_OUT_PRESS_XL     0x78
#define LPS28DFW_FIFO_DATA_OUT_PRESS_L      0x79
#define LPS28DFW_FIFO_DATA_OUT_PRESS_H      0x7A
 
#define LPS28DFW_ADDRESS_0 0x5C  // if SDO is 0  
#define LPS28DFW_ADDRESS_1 0x5D  // if SDO is 1  


// ODR sample rate
#define ODR_PwrDown      0x00   // default is power down
#define ODR_oneShot      0x00    
#define ODR_1Hz          0x01
#define ODR_4Hz          0x02
#define ODR_10Hz         0x03
#define ODR_25Hz         0x04   
#define ODR_50Hz         0x05
#define ODR_75Hz         0x06
#define ODR_100Hz        0x07
#define ODR_200Hz        0x08  // 200 Hz sample rate  

// Avereging per data sample
#define AVG_4            0x00   // default, 2.5 uA current usage at 1 Hz
#define AVG_8            0x01
#define AVG_16           0x02
#define AVG_32           0x03
#define AVG_64           0x04   
#define AVG_128          0x05
#define AVG_512          0x06 // 32.8 uA current usage at 1 Hz

#define FS_MODE1         0x00  // full scale up to 1260 hPa
#define FS_MODE2         0x01  // full scale up to 4060 hPa

#define LPF_ODR_4        0x00 // Low-pass filter is ODR/4
#define LPF_ODR_9        0x01 // Low-pass filter is ODR/9
#define LPF_NONE         0x02 // No low pass filter

// define FIFO modes
#define BYPASS     0x00
#define FIFOMODE   0x01
#define CONTINUOUS 0x02
// When trigger mode is enabled
#define BYPASS_TO_FIFO     0x01
#define BYPASS_TO_CONTINUOUS   0x02
#define CONTINUOUS_TO_FIFO 0x03


class LPS28DFW
{
  public: 
  LPS28DFW(I2Cdev* i2c_bus);
  uint8_t  getChipID(uint8_t dev_address);
  void     reset(uint8_t dev_address);
  void     boot(uint8_t dev_address);
  uint8_t  getStatus(uint8_t dev_address);
  void     init(uint8_t dev_address, uint8_t odr, uint8_t avg, uint8_t fs, uint8_t lpf);
  void     oneShot(uint8_t dev_address);
  void     initFIFO(uint8_t dev_address, uint8_t fmode, uint8_t wtm);
  void     FIFOStatus(uint8_t dev_address, uint8_t * dest);
  void     FIFOReset(uint8_t dev_address);
  int32_t  Pressure(uint8_t dev_address);
  int16_t  Temperature(uint8_t dev_address);
  int32_t  FIFOPressure(uint8_t dev_address);

  private:
  I2Cdev* _i2c_bus;
};

#endif
