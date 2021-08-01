//#include <EEPROM.h>

/**
 * Library of feeprom.h
 * 
 * Developed for AURA Embedded System 2020
 * 
 * The library that works with EEPROM
 * @1kzpro && @vladimireganov
 * @version 09/11/2020
 * 
 * @param BackUpMemory Class that controls EEPROM
*/

class BackUpMemory {
  private:
    int eeAddress = 0;
  public:
  unsigned long size_of_int;
  unsigned long size_of_float;

  BackUpMemory(unsigned long size_of_int, unsigned long size_of_float) {
    this->size_of_int = size_of_int;
    this->size_of_float = size_of_float;
  }

  // запись в ЕЕПРОМ
  void EEPROM_int_write(int val) {  
    EEPROM.put(eeAddress, val);
  }

  // чтение из ЕЕПРОМ 
  int EEPROM_int_read(int address){    
    int f;
    EEPROM.get(address, f);
    return f;
  }

  // запись в ЕЕПРОМ float
  void EEPROM_float_write(float value) {  
    byte *x = (byte *)&value;
    for(byte i = 0; i < 4; i++) EEPROM.write(i+eeAddress, x[i]);
  }

  // чтение из ЕЕПРОМ float
  float EEPROM_float_read(int address) {    
    byte x[4];
    for(byte i = 0; i < 4; i++) x[i] = EEPROM.read(i+address);
    float *y = (float *)&x;
    return y[0];
  }
}