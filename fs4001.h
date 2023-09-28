
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

//I2C functions
static inline uint8_t i2cread() {
  #if ARDUINO >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}
static inline void i2cwrite(int8_t x) {
  #if ARDUINO >= 100
  Wire.write(x);
  #else
  Wire.send(x);
  #endif
}

//Functions for operation
class fs4001{
  public:
  
    fs4001(bool debug = false) : _i2caddr(FS4001_DEFAULT_ADDR), _debug(debug){}
    
    //Starts the sensor with address = addr >> 1
    bool begin(uint8_t addr = FS4001_DEFAULT_ADDR){
      Wire.begin(4,5); //CHANGE TO 21, 22 if using pins 21 and 22 for SDA and SCL
  
      Serial.print("Started new sensor w/ addr: "); Serial.println(addr);

      cal_offset(); //Calibrate the offset
      write_filter_depth(0xFE); //Set filter depth to 254

      return true;
    }

    //NOT WORKING!!!!
    void write_addr(uint8_t addr){
      Serial.print("Writing addr as "); Serial.print(addr); 
      if(_debug) Serial.print(": ");
      else Serial.println();

      write_register(FS4001_WRITE_ADDR, addr); //CHANGE BACK IF NEEDED
      _i2caddr = addr; 
    }

    void write_gas_factor(uint16_t factor){
      Serial.print("Writing gas correction factor as "); Serial.print(factor);
      if(_debug) Serial.print(": ");
      else Serial.println();

      Wire.beginTransmission(_i2caddr);
      i2cwrite(FS4001_WRITE_GAS_FACTOR);
      i2cwrite(factor >> 8); i2cwrite((uint8_t)(factor & 0x00FF));
      int temp = Wire.endTransmission();

      //for debugging
      if(_debug){
        Serial.print("\twrite reg code: "); Serial.print(temp); Serial.println();
      }
    }

    //Writes the filter depth for the sensor
    void write_filter_depth(uint8_t depth){
      Serial.print("Writing filter depth as "); Serial.print(depth);
      if(_debug) Serial.print(": ");
      else Serial.println();

      write_register(FS4001_WRITE_FILTER_DEPTH, depth);
    }

    //NOT WORKING
    void write_response_time(uint16_t time){
      Serial.print("Writing response time as "); Serial.print(time); Serial.print(" ms");
      if(_debug) Serial.print(": ");
      else Serial.println();

      Wire.beginTransmission(_i2caddr);
      i2cwrite(FS4001_WRITE_RESPONSE_TIME);
      i2cwrite(time >> 8); i2cwrite((uint8_t)(time & 0x00FF));
      int temp = Wire.endTransmission();

      //for debugging
      if(_debug){
        Serial.print("\twrite reg code: "); Serial.print(temp); Serial.println();
      }
    }

    void cal_offset(){
      Serial.print("Calibrating offset");
      if(_debug) Serial.print(": ");
      else Serial.println();

      write_register(FS4001_CALIBRATE_OFFSET, 0x00);
    }

    //Reads serial number
    //Note: Add .c_str() at the end of this function if 
    //you want to print it to serial in Arduino
    std::string read_sn(){
      Serial.print("Reading serial number: ");

      Wire.beginTransmission(_i2caddr);
      i2cwrite(FS4001_READ_SERIAL);

      int temp = Wire.endTransmission(false); 
      int temp1 = Wire.requestFrom(_i2caddr, 12);

      //for debugging
      if(_debug){
        Serial.print("\tread reg code: "); Serial.print(temp);
        Serial.print("\tread bytes available: "); Serial.println(temp1); 
      }
      

      if (! Wire.available()){
        Serial.println("\tERROR in Register read"); //for debugging
        return "";
      }
    
      std::string result = "";
      for (int i = 0; i < 12; ++i){
        result += i2cread();
      }

      return result;
    }

    //Returns flowrate in sccm
    double read_flowrate(){
      Serial.print("Reading flowrate: ");

      Wire.beginTransmission(_i2caddr);
      i2cwrite(FS4001_READ_FLOWRATE);
      int temp = Wire.endTransmission(false); 
      int temp1 = Wire.requestFrom(_i2caddr, 5);

      //for debugging
      if(_debug){
        Serial.print("\tread reg code: "); Serial.print(temp);
        Serial.print("\tread bytes available: "); Serial.println(temp1);
      }
      
      int64_t flowrate = i2cread(); flowrate <<= 8;
      flowrate |= i2cread(); flowrate <<= 8;
      flowrate |= i2cread(); flowrate <<= 8;
      flowrate |= i2cread(); flowrate <<= 2;

      
      uint8_t crc = i2cread();  //crc needs to be processed in a different way

      if(_debug){
        Serial.print("Crc: "); Serial.println(crc);
      }

      return flowrate / 1000.0;
    }

    //Reads the address of the device
    uint8_t read_addr(){
      Serial.print("Reading addr: ");
      uint8_t addr = read_register(FS4001_READ_ADDR) >> 1;
      return addr;
    }

    int16_t read_gas_factor(){
      Serial.print("Reading gas correction factor: ");

      Wire.beginTransmission(_i2caddr);
      i2cwrite(FS4001_READ_RESPONSE_TIME);
      int temp = Wire.endTransmission(false); 
      int temp1 = Wire.requestFrom(_i2caddr, 2);

      //for debugging
      if(_debug){
        Serial.print("\tread reg code: "); Serial.print(temp);
        Serial.print("\tread bytes available: "); Serial.println(temp1);
      }
      
      int16_t factor = i2cread(); factor <<= 8;
      factor |= i2cread();

      return factor;
    }

    //Reads the filter depth
    uint8_t read_filter_depth(){
      Serial.print("Reading filter depth: "); 
      return read_register(FS4001_READ_FILTER_DEPTH);
    }

    //Reads response time in milliseconds
    int16_t read_response_time(){
      Serial.print("Reading response time: ");

      Wire.beginTransmission(_i2caddr);
      i2cwrite(FS4001_READ_RESPONSE_TIME);
      int temp = Wire.endTransmission(false); 
      int temp1 = Wire.requestFrom(_i2caddr, 2);

      //for debugging
      if(_debug){
        Serial.print("\tread reg code: "); Serial.print(temp);
        Serial.print("\tread bytes available: "); Serial.println(temp1);
      }
      

      int16_t time = i2cread(); time <<= 8;
      time |= i2cread();

      return time;
    }

  protected:

    //Reads 1 byte from device
    int8_t read_register(uint8_t reg){
      Wire.beginTransmission(_i2caddr);
      i2cwrite(reg);

      int temp = Wire.endTransmission(false); //repeated start sequence
      int temp1 = Wire.requestFrom(_i2caddr, 1); //requests 1 bytes from device register

      //for debugging
      if(_debug){
        Serial.print("\tread reg code: "); Serial.print(temp);
        Serial.print("\tread bytes available: "); Serial.println(temp1); 
      }


      if (! Wire.available()){
        Serial.println("\tERROR in Register read"); //for debugging
        return -1;
      }

      //Read the register and return
      int8_t result = i2cread();
      return result;
    }

    //Writes 1 bytes of data to register
    void write_register(uint8_t reg, int8_t value){
      Wire.beginTransmission(_i2caddr);
      i2cwrite(reg);

      //Sends the data
      i2cwrite(value);
      int temp = Wire.endTransmission();

      //for debugging
      if(_debug){
        Serial.print("\twrite reg code: "); Serial.print(temp); Serial.println();
      }
      
    }
    
  private:
    int8_t _i2caddr;
    bool _debug;
};


#endif