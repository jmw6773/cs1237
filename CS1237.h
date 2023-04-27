/*********************************************************** CS1237 ADC library
*
*    MIT License
*
*Copyright (c) 2020    GitHubName:SiBangkotan
*
*Permission is hereby granted, free of charge, to any person obtaining a copy
*of this software and associated documentation files (the "Software"), to deal
*in the Software without restriction, including without limitation the rights
*to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*copies of the Software, and to permit persons to whom the Software is
*furnished to do so, subject to the following conditions:
*
*The above copyright notice and this permission notice shall be included in all
*copies or substantial portions of the Software.
*
*THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*SOFTWARE.
*
*
*
*
******************************************************************************/

#ifndef CS1237_LIB_H
#define CS1237_LIB_H

#include <Arduino.h>
#include "esphome.h"

#ifdef __cplusplus
extern "C" {
#endif


/*

CS1237lib Status Register

  Bit 7 bit26 of last ADC response
  Bit 6 bit25 of last ADC response
  Bit 5 initialize sucessfull
  Bit 4 new data is ready at ADCread
  
  Bit 3 sleeping
  Bit 2 reserved
  Bit 1 config write cycle is pending
  Bit 0 config read cycle is pending
*/  
//bit position
#define SRB_BIT26_UPDATE2      7 
#define SRB_BIT25_UPDATE1      6
#define SRB_INIT_SUCCESS       5
#define SRB_NEW_DATA_READY     4
#define SRB_ADC_SLEEPING       3
//#define SRB_RESERVED         2
#define SRB_WRITE_CNF_PENDING  1
#define SRB_READ_CNF_PENDING   0

//mask
#define SRM_BIT26_UPDATE2      0x80  
#define SRM_BIT25_UPDATE1      0x40
#define SRM_INIT_SUCCESS       0x20
#define SRM_NEW_DATA_READY     0x10
#define SRM_ADC_SLEEPING       0x08
//#define SRM_RESERVED         0x04
#define SRM_WRITE_CNF_PENDING  0x02
#define SRM_READ_CNF_PENDING   0x01

  
/*

CS1237 ADC Config Register shadow

default value after reset 0Ch:
REFO_OFF  0 , REF Output active
SPEED_SEL 0 , Sampling rate 10 Hz
PGA_SEL   3 , Analog input Gain 128 x
CH_SEL    0 , Channel A


  Bit 7 reserved, must be 0
  Bit 6 REFO_OFF
  Bit 5-4 SPEED_SEL   30h 1280 Hz      :sample rate
                      20h  640 Hz
                      10h   40 Hz
                      00h   10 Hz
  Bit 3-2 PGA_SEL     0Ch  128 x       :analog gain
                      08h   64 x
                      04h    2 x
                      00h    1 x
  Bit 1-0 CH_SEL      03h Internal short :channel select
                      02h Temperature
                      01h Chip retention
                      00h Channel A (normal #define SPEED_SEL_1280        1280
opr)

*/
//Config Register Bit (CRB) position
#define CRB_REFO_OFF           6
#define CRB_SPEED_SEL          4 
#define CRB_PGA_SEL            2
#define CRB_CH_SEL             0

//Config Register Mask (CRM) pattren
#define CRM_REFO_OFF          0x40
#define CRM_SPEED_SEL         0x30
#define CRM_PGA_SEL           0x0C
#define CRM_CH_SEL            0x03

//Config Register parameter
#define SPEED_SEL_1280        0x30
#define SPEED_SEL_640         0x20
#define SPEED_SEL_40          0x10
#define SPEED_SEL_10          0x00

#define PGA_SEL_128           0x0C
#define PGA_SEL_64            0x08
#define PGA_SEL_2             0x04
#define PGA_SEL_1             0x00

#define CH_SEL_CHANNEL_A        0
//#define CH_SEL_Chip_retention   1
#define CH_SEL_TEMPERATURE      2
#define CH_SEL_INTERNAL_SHORT   3

#define REFO_OFF_FALSE        false
#define REFO_OFF_TRUE         true

/*
 * 
 * I/O addr
 * 
 * 
 */

 const uint32_t cstGPOS = 0x60000304 ;
 const uint32_t cstGPOC = 0x60000308 ;
 const uint32_t cstGPES = 0x60000310 ;
 const uint32_t cstGPEC = 0x60000314 ;
 const uint32_t cstGPI  = 0x60000318 ;
 

class CS1237{

  public:
     CS1237(int _datapinno, int _clkpinno);
     bool    begin(int _gain , int _samplerate , int _channel = CH_SEL_CHANNEL_A, bool _refo_off = 0);
     uint8_t statusReg;          // this instance status register
     uint8_t configReg;          // last read hardware config register shadow
     bool    ADCpool();          // test if new data available, read it to ADCdata
     int     ADCdata;            // last ADC data memory register
     void    sleep(bool _powerSaveMode);
     //uint32_t samples[256];    // interrupt data buffer
  
  
  private:
    uint32_t twi_sda;             // this instance data pin mask
    uint32_t twi_scl;             // this instance clk pin mask
    char     twi_sda_pin;         // this instance data pin
    char     twi_scl_pin;         // this instance clk pin
    uint8_t  configRegW;
    bool     ADCread();           // word level serdes
    bool     one_bit(bool bit=1); // bit level serdes 
    //void     InterruptServiceRoutinex();
    //uint32_t sampleWritePointer;

};

#ifdef __cplusplus
}
#endif

#endif // CS1237_LIB_H
