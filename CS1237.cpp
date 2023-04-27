#include "CS1237.h"

/**********************************************************************
 * 
 *   Constructor. Hardware related such as pin number, test pin functionality
 *   
 *********************************************************************/

CS1237::CS1237(int _datapinno, int _clkpinno)
{
    //save pin no and create handy bit mask
    twi_sda = (1<<_datapinno);
    twi_scl = (1<<_clkpinno) ;
    twi_sda_pin = _datapinno;   // this instance data pin
    twi_scl_pin = _clkpinno ;   // this instance clk pin

    //test and setup pins
    statusReg = SRM_INIT_SUCCESS;
    //statusReg2=0;

    //test clk stuck low
    pinMode(twi_scl_pin, INPUT_PULLUP);
    delay(3);
    if (digitalRead(twi_scl_pin) == LOW) 
    {statusReg &= ~SRM_INIT_SUCCESS;
    }

    //test clk stuck high
    digitalWrite(twi_scl_pin, HIGH);
    pinMode(twi_scl_pin, OUTPUT);
    digitalWrite(twi_scl_pin, LOW);
    uint32_t tmpReg=digitalRead(twi_scl_pin);
    digitalWrite(twi_scl_pin, HIGH);
    if (tmpReg) 
    {statusReg &= ~SRM_INIT_SUCCESS;
    }
    //exit as output HIGH in attemp to persuade ADC to sleep

    //test sda stuck high
    pinMode(twi_sda_pin, OUTPUT_OPEN_DRAIN);
    digitalWrite(twi_sda_pin, LOW);
    delay(1);
    if (digitalRead(twi_sda_pin) == HIGH) 
    {statusReg &= ~SRM_INIT_SUCCESS;
    }

    //test sda stuck low
    pinMode(twi_sda_pin, INPUT_PULLUP);
    digitalWrite(twi_sda_pin, HIGH);
    delay(3);
    if (digitalRead(twi_sda_pin) == LOW) 
    {statusReg &= ~SRM_INIT_SUCCESS;
    }

    //exit with: GPIO output block set to low open drain
    //           but overall "output block" then disabled.
    //           GPIO is set as input with pull up.
    //           to output a LOW just need to reenable "output block" 
    pinMode(twi_sda_pin, OUTPUT_OPEN_DRAIN);
    digitalWrite(twi_sda_pin, LOW);
    pinMode(twi_sda_pin, INPUT_PULLUP);
}
/*CS1237::~CS1237()
{
  //debugprint("Warning: CS1237 deconstruct not supported");
  //errorblink(NO_DECONSTRUCT);//the CS1237 object is hard wired at PCB, not software alterable
}*/


/*****************************************************************
 * 
 *  "begin()" function, handle user settable parameter.
 *  
 *****************************************************************/



bool CS1237::begin(int _gain , int _samplerate , int _channel , bool _refo_off )
{
  //check constructor
  if (!(statusReg & SRM_INIT_SUCCESS)) 
  {
  //Debug print
    ESP_LOGI("CS1237", "Error: begin: Constructor was not successfull:%h", statusREG);
    return 0;   
  }

  //Reset ADC via sleep
  sleep(1);                                        //put ADC on a nap
  delay(10);       
  sleep(0);                                        //wake it up, now the ADC is in a known clk state
  uint32_t dRdy=0;
  uint32_t startTime = micros();
  uint32_t nowTime = startTime;
  do {
    dRdy= (digitalRead(twi_sda_pin));              //should go low
    nowTime=micros();
  } while ((nowTime-startTime<8000) && dRdy);      //in 3,125 or 6,25ms++ (datasheet page 9)
  if (dRdy)                                        //if still high, have to use yield
  {
    do 
    {
      yield();
      dRdy= (digitalRead(twi_sda_pin));
      nowTime=micros();
    } while ((nowTime-startTime<320000) && dRdy);  //it may go up to 70 or 300ms++ (datasheet p9)
  }
  if (dRdy) {
    statusReg &= ~SRM_INIT_SUCCESS;                //error condition
  //Debug print
    ESP_LOGI("CS1237", "begin: Error: Time out waking up from sleep.");
    return 0;
  } else {
    ESP_LOGI("CS1237", "begin: Waking up time (us): %d", nowTime-startTime);
  }
  
 
  //
  //Configure
  //
  configRegW = 0x00;

  //PGA_SEL
  if      (_gain >=100) configRegW|=(PGA_SEL_128); //Gain=128
  else if (_gain >= 10) configRegW|=(PGA_SEL_64);  //Gain= 64
  else if (_gain >=  2) configRegW|=(PGA_SEL_2);   //Gain=  2
  //else                             PGA_SEL_1       Gain=  1
  
  
  //SPEED_SEL
  if      (_samplerate>=1000) configRegW|=(SPEED_SEL_1280); //Sampling 1280Hz, 753 us
  else if (_samplerate>= 200) configRegW|=(SPEED_SEL_640 ); //Sampling 640Hz, 1562 us
  else if (_samplerate>=  20) configRegW|=(SPEED_SEL_40  ); //Sampling 40Hz, 25ms
  //else                                   SPEED_SEL_10     //Sampling 10Hz, 100ms


  //CH_SEL  
  configRegW |= ((_channel & 3));    //CH_SEL two bit channel selector
  
  if (_refo_off) configRegW|=(CRM_REFO_OFF); //REFO_OFF


  //
  //Send out config
  //
  statusReg |= (SRM_WRITE_CNF_PENDING | SRM_READ_CNF_PENDING); //Initiate config write cycle 

  startTime = micros();
  while(!ADCpool() && (micros()-startTime< 120000))
  {
    yield(); //write config
  }
  uint32_t wrstartTime = micros()-startTime;

  startTime = micros();
  while(!ADCpool() && (micros()-startTime< 360000) ) //reread, allow settling more than 300ms
  {
    yield();
  }
  startTime = micros()-startTime;
  Serial.print("write cnf (us)\t");
  Serial.println(wrstartTime);
  Serial.print("reread cnf (us)\t");
  Serial.println(startTime);
  
  if ((statusReg & SRM_WRITE_CNF_PENDING) | (statusReg & SRM_READ_CNF_PENDING) | (configRegW != configReg))
  {
    Serial.print("begin: Error: ");
    if ((statusReg & SRM_WRITE_CNF_PENDING)) Serial.print("write ");
    if ((statusReg & SRM_READ_CNF_PENDING)) Serial.print("reread ");
    if ((configRegW != configReg)) Serial.print("compare");
    Serial.print(".\ncnfWr..cnfRd..ADC: ");
    Serial.print(configRegW,HEX);
    Serial.print("\t");
    Serial.print(configReg,HEX);
    Serial.print("\t");
    Serial.println(ADCdata,HEX);
    statusReg &= ~SRM_INIT_SUCCESS;
    sleep(1); //try to put ADC into sleeping
    return 0;
  }
  else
  {
    ESP_LOGI("CS1237", "ADC Initiated");
    statusReg |= SRM_INIT_SUCCESS;
    return 1;
  }

}


/********************************************************************************************
 *  Serdes
 * 
 * 
 *   Bit handler
 * 
 *******************************************************************************************/



bool IRAM_ATTR CS1237::one_bit(bool _bit) //contains time sensitive inline code, put in RAM
{
#define NOMINAL_SPEED_kbps 1100
#define delayTwiUp (F_CPU/NOMINAL_SPEED_kbps/1000/2-22)/4
#define delayTwiDn (F_CPU/NOMINAL_SPEED_kbps/1000/2-32)/4

    //SCL_HIGH();
    GPOS=twi_scl;

    //set SDA 
    if (_bit) {
      GPEC = (twi_sda);
    } else {
      GPES = (twi_sda);
    }
    
    uint32_t delayTwi = delayTwiUp;
    asm volatile (
      "100:;"
      " addi.n %[a2],%[a2],-1;"
      " bgei   %[a2],1,100b;"
      : [a2]"=r"(delayTwi) : "[a2]"(delayTwi) );

    
    //SCL_LOW();
    GPOC=twi_scl;

    
    delayTwi = delayTwiDn;
    asm volatile (
      "200:;"
      " addi.n %[a2],%[a2],-1;"
      " bgei   %[a2],1,200b;"
      : [a2]"=r"(delayTwi) : "[a2]"(delayTwi) );

    //get data from ADC
    _bit = (GPI & twi_sda);
    if(_bit) _bit=1;
    return _bit;
}


/********************************************************************
 * 
 *   word handler
 * 
 ********************************************************************/



bool IRAM_ATTR CS1237::ADCread()    //ToDo : test callable from ISR
{

  ADCdata = 0;
    int32_t tmp;
    for (tmp = 0; tmp < 24; tmp++)  //clk 1-24
    {
        ADCdata = (ADCdata << 1) | one_bit(1);
    }
#define SX_TMP 0x800000
  ADCdata = (ADCdata ^ SX_TMP) - SX_TMP;   //sign extend
  statusReg |= SRM_NEW_DATA_READY;             //new data ready
  
  statusReg &= ~(SRM_BIT25_UPDATE1 | SRM_BIT26_UPDATE2);
  statusReg |= (one_bit(1)<<SRB_BIT25_UPDATE1);    //clk 25
  statusReg |= (one_bit(1)<<SRB_BIT26_UPDATE2);    //clk 26

  one_bit(1);            //clk 27
  
  if (statusReg & SRM_WRITE_CNF_PENDING)
  {
    //11 110 0101 1 cccc cccc 1 
    //111 1111 11 
    //876 5432 1098 7654 3210
    //111 1001 011c cccc ccc1 = 7 9 6 0 1      B00F9601
    uint32_t writeout= (0x79601  |  ((configRegW & 0xFF)<<1));
    for (uint32_t bitpos=28 ; bitpos<=46 ; bitpos++)
    {
        uint32_t tempReg=(one_bit((writeout>>18) & 1)) ;
        writeout = (writeout<<1)|tempReg;
        
    }
//?/    statusReg2=writeout;
    statusReg &= ~SRM_WRITE_CNF_PENDING;
  } 
  
  else  //if and only if no config write cycle
  {
    if (statusReg & SRM_READ_CNF_PENDING)
    {
      //11 101 0110 1 rrrr rrrr 1 
      //
      //111 1111 11 
      //876 5432 1098 7654 3210
      //                                                      1098 7654 3210 9876 5432 1098 7654 3210
      //111 0101 1011 1111 1111 = 7 5 b f f          DFFF5A01 1101 1111 1111 1111 0101 1010 0000 0001
      //
      //223 3333 3333 3444 4444
      //890 1234 5678 9012 3456
      //
      //0123 4567 8901 2345 678
      //
      uint32_t writeout= 0x75bff;
      for (uint32_t bitpos=28 ; bitpos<=46 ; bitpos++)
      {
        uint32_t tempReg=(one_bit((writeout>>18) & 1)) ;
        writeout = (writeout<<1)|tempReg;
      }
      configReg = (writeout>>1) & 0xFF;
//?/    statusReg3=writeout;
      statusReg &= ~SRM_READ_CNF_PENDING;
    } 
  } 
  return 1;
}   //takes about 25us normally or 3%CPU at 1280 sampling rate, up to 50us if config is accessed

/************************************************************************************
 *   SerDes   serial deserialize
 * 
 *   Interrupt chaining              THIS SECTION IS STILL UNDER DEVELOPMET
 * 
 * **********************************************************************************

void CS1237::InterruptServiceRoutinex()
{
  //reenable all other interrupts //TODO check if interrupt nesting is allowed.
  ADCread();
  if (_zeroCrossing)                   //zc if not zero then find zeroCrossing
  {
    if (sampleWritePointer == 0)        //zc is this the first sampling attempt
    {
      if(ADCdata == threshold)      // found a zeroCrossing at first pass; lucky
      {
        _zeroCrossing=0;
      }
    }
    else                            //zc not first sampling
    {
      if (samples[0]>threshold)    // previous signal was above threshold
      {
        if (ADCdata<=threshold)  // found a zeroCrossing
        {
          _zeroCrossing=0;
        }
        else                     // still above threshold
        {
          sampleWritePointer=0;
        }
      }
      else             // previous signal was bellow threshold
      {
        if (ADCdata>=threshold)  // found a zeroCrossing
        {
          _zeroCrossing=0;
        }
        else           // still below threshold
        {
          sampleWritePointer=0;
        }
      }
    }
    _zeroCrossing--;                           //zc decrement until 0
  }
  
  samples[sampleWritePointer++] = ADCdata;

  if (sampleWritePointer >= numberOfSamples)
  {
    detachPinInterrupt(twi_sda_pin);
    statusReg |= NEW_DATA_READY;      //new data ready bit
    _zeroCrossing=zeroCrossing;
    
  }
}
*/


/************************************************************************
 *   SerDes
 * 
 *    Access to ADC via pooling
 * 
 ************************************************************************/
bool CS1237::ADCpool()
{
  statusReg &= ~SRM_NEW_DATA_READY;   //clear stale new data ready bit
  if (statusReg & SRM_INIT_SUCCESS)
  {
    if (statusReg & SRM_ADC_SLEEPING)
    {
      sleep(0); // wake it up
    }
    //SCL_LOW(twi_scl);               //just make sure CLK is low
    if (GPI & twi_sda)  return 0;
    if (ADCread())
    {   
      statusReg |= SRM_NEW_DATA_READY;             //new data ready
      return 1;
    }
  }
  return 0;
}
/*******************************************************************************
 *  SerDes
 * 
 *    Access to ADC via interrupt       THIS SECTION IS STILL UNDER DEVELOPMENT
 * 
 * ******************************************************************************
bool CS1237::get(uint32_t _numberOfSamples, uint32_t _maxNumberOf_zeroCrossing_search = 0)
{
  statusReg &= ~NEW_DATA_READY;                 //clear new data ready bit
  if (statusReg & SRM_INIT_SUCCESS)         //dont proceed if there is error
  {
    if (statusReg & SRM_ADC_SLEEPING)
    {
      sleep(0);                   // wake it up
    }
    
    numberOfSamples =_NumberOfSamples;
    if (numberOfSamples > MAX_NUMBER_OF_SAMPLES)
    {
      numberOfSamples = MAX_NUMBER_OF_SAMPLES;  //fool proofing
    }
    
    zeroCrossing = _maxNumberOf_zeroCrossing_search;
    if (zeroCrossing > MAX_NUMBER_OF_ZERO_CROSSING)
    {
      zeroCrossing = MAX_NUMBER_OF_ZERO_CROSSING; //fool proofing
    }
    
    sampleWritePointer = 0;
    
    //ADC will generate a falling edge (without clk) whenever new data is ready
    attachPinInterrupt(twi_sda,CS1237ISR,FALLING_EDGE);
    
    return 1
  }
  return 0;
}
*/


/*********************************************
 *  CS1237 util
 * 
 *    Sleep
 *********************************************/
void CS1237::sleep(bool _sleep)
{
  if (_sleep)
  {
    digitalWrite(twi_scl_pin,HIGH);
//    SCL_HIGH(twi_scl);  //TO DO set scl pin mode SLEEP_PULL_UP
    uint32_t startTime = micros();
    while (micros()-startTime < 120)  //at least 100us
    {
      yield();
    }
    statusReg |= SRM_ADC_SLEEPING;
  }
  else //wake it up
  {
    digitalWrite(twi_scl_pin,LOW);
    uint32_t startTime = micros();
    while (micros()-startTime < 15)     //at least 10 us
    {
      asm(" nop.n;"); //do nothing
    }
    statusReg &= ~SRM_ADC_SLEEPING;
  }
}
