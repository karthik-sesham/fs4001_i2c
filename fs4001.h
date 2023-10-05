#ifndef _FS4001_H_
#define _FS4001_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Wire.h>
/*----------------------------------------*/

//Hex Commands for FS4001

#define FS4001_DEFAULT_ADDR           0x01

//WRITE COMMANDS
#define FS4001_WRITE_ADDR             0x05  //Bit7 ~ Bit1 are the addr, Bit0 is R/W bit
#define FS4001_WRITE_GAS_FACTOR       0x0A  //Int16, default is 1000 for air
#define FS4001_WRITE_FILTER_DEPTH     0x0B  //Int8, 0~254
#define FS4001_WRITE_RESPONSE_TIME    0x0C  //Int16, unit is msec
#define FS4001_CALIBRATE_OFFSET       0x1C  //1 byte, ensure NO FLOW in the pipe

//READ COMMANDS
#define FS4001_READ_SERIAL            0x82  //ASCII, 12 * Int16_t
#define FS4001_READ_FLOWRATE          0x83  // Int32/1000slpm + CRC (see datasheet)
#define FS4001_READ_ADDR              0x85  //Bit7 ~ Bit1
#define FS4001_READ_GAS_FACTOR        0x8A  //Int16
#define FS4001_READ_FILTER_DEPTH      0x8B  //Int8, 0~254
#define FS4001_READ_RESPONSE_TIME     0x8C  //Int16, unit is msec
/*----------------------------------------*/

//I2C functions
static inline uint8_t i2cread();
static inline void i2cwrite(int8_t x);

//Functions for operation
class fs4001{
  public:
  
    fs4001(bool debug = false);
    bool begin(uint8_t addr = FS4001_DEFAULT_ADDR);
    void write_addr(uint8_t addr);
    void write_gas_factor(uint16_t factor);
    void write_filter_depth(uint8_t depth);
    void write_response_time(uint16_t time);
    void cal_offset();
    std::string read_sn();
    double read_flowrate();
    uint8_t read_addr();
    int16_t read_gas_factor();
    uint8_t read_filter_depth();
    int16_t read_response_time();

  private:

    int8_t read_register(uint8_t reg);
    void write_register(uint8_t reg, int8_t value);
    
    int8_t _i2caddr;
    bool _debug; //set to 1 if you want debug output to serial
};

#endif