#include <RENESAS3224.h>

/* Commands definitions */
#define RENESAS3224_CMD_CHECKSUM        0x90
#define RENESAS3224_CMD_START_NOM       0xA8
#define RENESAS3224_CMD_START_CM        0xA9
#define RENESAS3224_CMD_MSR_FULL        0xAA
#define RENESAS3224_CMD_MSR_CYCLIC      0xAB
#define RENESAS3224_CMD_MSR_OVR2        0xAC
#define RENESAS3224_CMD_MSR_OVR4        0xAD
#define RENESAS3224_CMD_MSR_OVR8        0xAE
#define RENESAS3224_CMD_MSR_OVR16       0xAF
#define RENESAS3224_CMD_SLCT_SMCONFIG1  0xB0
#define RENESAS3224_CMD_SLCT_SMCONFIG2  0xB1
#define RENESAS3224_CMD_STOP_CYC        0xBF

#define RENESAS3224_READ_MEM_LOWER_THR 0x00
#define RENESAS3224_READ_MEM_UPPER_THR 0x39
#define RENESAS3224_WRITE_MEM_LOWER_THR 0x40
#define RENESAS3224_WRITE_MEM_UPPER_THR 0x79
#define RENESAS3224_RAW_CMD_LOWER_THR 0xA0
#define RENESAS3224_RAW_CMD_UPPER_THR 0xA7

#define RENESAS3224_CMD_SMCONFIG1 0x12
#define RENESAS3224_CMD_SMCONFIG2 0x16

RENESAS3224::RENESAS3224 (byte I2C_ADRESS) : _DEVICE_ADDRESS(I2C_ADRESS) {
    _connected = false;
}

bool RENESAS3224::begin(TwoWire &I2Chandler ) {
    renesas3224_status_t dev_status;

    pi2chandler = &I2Chandler;
    // if the port was not initialized, we start the hardware here.
    pi2chandler->begin();

    //Check connection //
    _connected = isConnected();
    if (!_connected) return false; //I2C comm failure

    //--- make default setup ---//
    // Set the device to command mode //
    if(!set_CommandMode()) return false; //error

    // Get status //
    dev_status.reg_value = getStatus();
    //if(dev_status.bits.MemError) calculate_CheckSum();
    //while(isBusy()) ;
    smconfig_selector = dev_status.bits.ConfigSetup;

    // Get Default SM_Config //
    //smconfig1.reg_value = read_MemoryAddress(RENESAS3224_CMD_SMCONFIG1);
    //smconfig2.reg_value = read_MemoryAddress(RENESAS3224_CMD_SMCONFIG2);

    // Set default config parameters //

    return _connected;
}

// Test to see if the device is responding //
bool RENESAS3224::isConnected(void) {
  pi2chandler->beginTransmission((uint8_t)_DEVICE_ADDRESS);
  return (pi2chandler->endTransmission() == 0);
}

// Command 0xA8 //
bool RENESAS3224::set_NormalMode(){
    renesas3224_status_t dev_status;

    if(_connected){
        if(writeCommand(RENESAS3224_CMD_START_NOM)){
            // Get status //
            dev_status.reg_value = getStatus();
            if(dev_status.bits.Mode == RENESAS3224_STATUS_MODE_NORMAL){
                return true;
            }
        }
        
    } 
    return false;

}

// Command 0xA9 //
bool RENESAS3224::set_CommandMode(){
    renesas3224_status_t dev_status;

    if(_connected){
        if(writeCommand(RENESAS3224_CMD_START_CM)){
            // Get status //
            dev_status.reg_value = getStatus();
            if(dev_status.bits.Mode == RENESAS3224_STATUS_MODE_COMMAND){
                return true;
            }
        }
        
    } 
    return false;

}

// --- //
uint8_t RENESAS3224::getStatus(void){

    if(_connected){
        pi2chandler->requestFrom(_DEVICE_ADDRESS, (uint8_t)1);
        if (pi2chandler->available()){
            return pi2chandler->read();
        }
    }
    return 0;

}

// --- //
bool RENESAS3224::isBusy(void){
    renesas3224_status_t dev_status;

    if(_connected){
        dev_status.reg_value = getStatus();
        return (dev_status.bits.Busy == 1);
    }

    return false;

}

//  //
bool RENESAS3224::select_SMconfig1(void){
    renesas3224_status_t dev_status;

    if(_connected){
        if(writeCommand(RENESAS3224_CMD_SLCT_SMCONFIG1)){
            // Get status //
            dev_status.reg_value = getStatus();
            if(dev_status.bits.ConfigSetup == RENESAS3224_STATUS_SMCONFIG_1){
                smconfig_selector = dev_status.bits.ConfigSetup;
                return true;
            }
        }
        
    } 
    return false;
}

//  //
bool RENESAS3224::select_SMconfig2(void){
    renesas3224_status_t dev_status;

    if(_connected){
        if(writeCommand(RENESAS3224_CMD_SLCT_SMCONFIG2)){
            // Get status //
            dev_status.reg_value = getStatus();
            if(dev_status.bits.ConfigSetup == RENESAS3224_STATUS_SMCONFIG_2){
                smconfig_selector = dev_status.bits.ConfigSetup;
                return true;
            }
        }
        
    } 
    return false;

}


bool RENESAS3224::calculate_CheckSum(void){

    if(_connected)
        return writeCommand(RENESAS3224_CMD_CHECKSUM);
    
    return false;

}

// --- //
uint16_t RENESAS3224::read_MemoryAddress(uint8_t memory_address){
    uint8_t mem_address=0;

    // check if the memory address is valid //
    if( (memory_address>= RENESAS3224_READ_MEM_LOWER_THR) && (memory_address <= RENESAS3224_READ_MEM_UPPER_THR) ){
        mem_address = memory_address;
    }else{ 
        return 0; // invalid memory address //
    }

    // check if connected //
    if(!_connected || !(writeCommand(mem_address)) ){
        return 0; // not connected to the device //
    }

    // check if device is busy //
    while(isBusy());

    // Read memory address //
    return readData();
}

// --- //
bool RENESAS3224::write_MemoryData(uint8_t memory_address, uint8_t data_msb, uint8_t data_lsb){
    return write_data_to_memory(memory_address, ((uint16_t)data_msb<<8 | data_lsb) );
}
bool RENESAS3224::write_MemoryData(uint8_t memory_address, uint16_t data){
    return write_data_to_memory(memory_address, (uint16_t)data);
}

bool RENESAS3224::write_data_to_memory(uint8_t memory_address, uint16_t data){
    uint8_t mem_address=0;

    Serial.print("[write] Writing -> ");
    Serial.print(memory_address,HEX);
    Serial.print("; ");
    Serial.println(data, HEX);

    // check if the memory address is valid //
    if( (memory_address >= RENESAS3224_WRITE_MEM_LOWER_THR) && (memory_address <= RENESAS3224_WRITE_MEM_UPPER_THR) ){
        mem_address = memory_address;
    }else{
        Serial.print("[write] INvalid memory");
        return false; // invalid memory address //
    }    

    // // memory_address must be different than SMconfig1 and SMCONFIG 2 //
    // // Direct methods are provided for these registers //
    // if( (memory_address == RENESAS3224_CMD_SMCONFIG1+RENESAS3224_WRITE_MEM_LOWER_THR) || 
    //     (memory_address == RENESAS3224_CMD_SMCONFIG2+RENESAS3224_WRITE_MEM_LOWER_THR) ){
    //     return false;
    // }

    // check if connected //
    if(!_connected || !(writeCommand(mem_address, data)) ){
        Serial.print("[write] not connected or could not write");
        return false; // not connected to the device //
    }

    // check if device is busy //
    while(isBusy());

    calculate_CheckSum();

    delay(10);


    uint16_t reg_value = read_MemoryAddress(mem_address-RENESAS3224_WRITE_MEM_LOWER_THR);

    Serial.print("[write] Checking Memory -> ");
    Serial.print(memory_address-RENESAS3224_WRITE_MEM_LOWER_THR, HEX);
    Serial.print("NewVAlue: ");
    Serial.println(reg_value, HEX);
    // verify that the data has been written correctly //
    
    //if(read_MemoryAddress(mem_address-RENESAS3224_WRITE_MEM_LOWER_THR) != data){
    if(reg_value != data){
        Serial.print("[write] returned data do not match");
        return false;
    }

    return true;
}


int RENESAS3224::get_RawData(uint8_t command, uint16_t config){
    uint8_t raw_cmd=0;

    // check if the memory address is valid //
    if( (command >= RENESAS3224_RAW_CMD_LOWER_THR) && (command <= RENESAS3224_RAW_CMD_UPPER_THR) ){
        raw_cmd = command;
    }else{ 
        return 0; // invalid memory address //
    }

    // check if connected //
    if(!_connected || !(writeCommand(raw_cmd, config)) ){
        Serial.print("[RAW] Not connected");
        return 0; // not connected to the device //
    }

    // check if device is busy //
    while(isBusy()) ;//Serial.print("[RAW] Busy");

    // Read memory address //
    return readRawMeasurement();

}


bool RENESAS3224::set_SMconfigRegister(uint8_t sm_selector, uint16_t register_value){
    uint8_t memory_address=0;

    Serial.print("+ Updating SMreg -> Cur VAl: 0x");

    if(sm_selector == 0){
        smconfig1.reg_value = read_MemoryAddress(RENESAS3224_CMD_SMCONFIG1);
        Serial.print(smconfig1.reg_value, HEX);

        if(smconfig1.reg_value != register_value){
            smconfig1.reg_value = register_value;
            memory_address = RENESAS3224_CMD_SMCONFIG1+RENESAS3224_WRITE_MEM_LOWER_THR;

            Serial.print("; New VAl: 0x");
            Serial.println(smconfig1.reg_value, HEX);
        }else{ 
            return true;
        }
    }else{
        smconfig2.reg_value = read_MemoryAddress(RENESAS3224_CMD_SMCONFIG1);
        if(smconfig2.reg_value != register_value){
            smconfig2.reg_value = register_value;
            memory_address = RENESAS3224_CMD_SMCONFIG2+RENESAS3224_WRITE_MEM_LOWER_THR;
        }else{
            return true;
        }
    }

    return write_data_to_memory(memory_address, register_value);
    // if(!_connected)
    //     return false;

    // return writeCommand(memory_address, register_value);

}

bool RENESAS3224::set_SMconfigGain1(uint8_t sm_selector, uint8_t gain1){
    uint8_t memory_address=0;
    uint16_t register_value=0;

    Serial.print("+ Updating GAIN1 -> Cur VAl: 0x");

    if(sm_selector == 0){
        smconfig1.reg_value = read_MemoryAddress(RENESAS3224_CMD_SMCONFIG1);
        Serial.print(smconfig1.reg_value, HEX);

        if(smconfig1.bits.gain1 != gain1){
            smconfig1.bits.gain1 = gain1;
            register_value = smconfig1.reg_value;
            memory_address = RENESAS3224_CMD_SMCONFIG1+RENESAS3224_WRITE_MEM_LOWER_THR;

            Serial.print("; New VAl: 0x");
            Serial.println(smconfig1.reg_value, HEX);
        }else return true;
        
    }else{
        smconfig2.reg_value = read_MemoryAddress(RENESAS3224_CMD_SMCONFIG2);
        if(smconfig2.bits.gain1 != gain1){
            smconfig2.bits.gain1 = gain1;
            register_value = smconfig2.reg_value;
            memory_address = RENESAS3224_CMD_SMCONFIG2+RENESAS3224_WRITE_MEM_LOWER_THR;
        }else return true;
    }

    return write_data_to_memory(memory_address, register_value);
}

bool RENESAS3224::set_SMconfigGain2(uint8_t sm_selector, uint8_t gain2){
    uint8_t memory_address=0;
    uint16_t register_value=0;

    Serial.print("+ Updating GAIN2 -> Cur VAl: 0x");

    if(sm_selector == 0){
        smconfig1.reg_value = read_MemoryAddress(RENESAS3224_CMD_SMCONFIG1);
        Serial.print(smconfig1.reg_value, HEX);

        if(smconfig1.bits.gain2 != gain2){
            smconfig1.bits.gain2 = gain2;
            register_value = smconfig1.reg_value;
            memory_address = RENESAS3224_CMD_SMCONFIG1+RENESAS3224_WRITE_MEM_LOWER_THR;

            Serial.print("; New VAl: 0x");
            Serial.println(smconfig1.reg_value, HEX);

        }else return true;
        
    }else{
        smconfig2.reg_value = read_MemoryAddress(RENESAS3224_CMD_SMCONFIG2);
        if(smconfig2.bits.gain2 != gain2){
            smconfig2.bits.gain2 = gain2;
            register_value = smconfig2.reg_value;
            memory_address = RENESAS3224_CMD_SMCONFIG2+RENESAS3224_WRITE_MEM_LOWER_THR;
        }else return true;
    }

    return write_data_to_memory(memory_address, register_value);
}

bool RENESAS3224::set_SMconfigADCbits(uint8_t sm_selector, uint8_t adc_bits){
    uint8_t memory_address=0;
    uint16_t register_value=0;

    Serial.print("+ Updating ADC -> Cur VAl: 0x");

    if(sm_selector == 0){
        smconfig1.reg_value = read_MemoryAddress(RENESAS3224_CMD_SMCONFIG1);
        Serial.print(smconfig1.reg_value, HEX);

        if(smconfig1.bits.adc_bits != adc_bits){
            smconfig1.bits.adc_bits = adc_bits;
            register_value = smconfig1.reg_value;
            memory_address = RENESAS3224_CMD_SMCONFIG1+RENESAS3224_WRITE_MEM_LOWER_THR;

            Serial.print("; New VAl: 0x");
            Serial.println(smconfig1.reg_value, HEX);

        }else return true;
        
    }else{
        smconfig2.reg_value = read_MemoryAddress(RENESAS3224_CMD_SMCONFIG2);
        if(smconfig2.bits.adc_bits != adc_bits){
            smconfig2.bits.adc_bits = adc_bits;
            register_value = smconfig2.reg_value;
            memory_address = RENESAS3224_CMD_SMCONFIG2+RENESAS3224_WRITE_MEM_LOWER_THR;
        }else return true;
    }

    return write_data_to_memory(memory_address, register_value);
}


/******/
uint16_t RENESAS3224::readData(void){

    pi2chandler->requestFrom(_DEVICE_ADDRESS, (uint8_t)2);
    if( pi2chandler->available() ){
        uint8_t status = pi2chandler->read();
        uint8_t msb    = pi2chandler->read();
        uint8_t lsb    = pi2chandler->read();
        return ((uint16_t)msb << 8 | lsb);
    }
    return (0);

}

/******/
int RENESAS3224::readRawMeasurement(void){
    int ddata=0;
    uint8_t _adc_bits;

    _adc_bits = smconfig1.bits.adc_bits;

    pi2chandler->requestFrom(_DEVICE_ADDRESS, (uint8_t)4);
    if( pi2chandler->available() ){
        uint8_t status = pi2chandler->read();
        uint8_t dat_23_16 = pi2chandler->read();
        uint8_t dat_15_8  = pi2chandler->read();
        uint8_t dat_7_0   = pi2chandler->read();

        Serial.print("[rRAW] Data: 0x");
        Serial.print(dat_23_16, HEX);
        Serial.print("; 0x");
        Serial.print(dat_15_8, HEX);
        Serial.print("; 0x");
        Serial.println(dat_7_0, HEX);

        // 2-Complement's calculation //
        if( (dat_23_16&0x80) > 0 ){
            //ddata = (((1<<24) - ((dat_23_16<<16) + (dat_15_8<<8) + dat_7_0))*-1) >> (24-_adc_bits);
            ddata = (((1<<24) - ((dat_23_16<<16) + (dat_15_8<<8) + dat_7_0))*-1);
        }else{
            //ddata = ((dat_23_16<<16) + (dat_15_8<<8) + dat_7_0) >> (24-_adc_bits);
            ddata = (((1<<24) - ((dat_23_16<<16) + (dat_15_8<<8) + dat_7_0))*-1);
        }

        Serial.print("[rRAW] Data Converted: ");
        Serial.println(ddata);

        return ddata;
    }

    return (0);
}

//uint8_t readFullMeasurement(int &sensorData, int &tempData){} //

/******/
//Write two bytes to a given command code location (8 bits) //
bool RENESAS3224::writeCommand(uint8_t commandCode) {

    pi2chandler->beginTransmission(_DEVICE_ADDRESS);
    pi2chandler->write(commandCode);
    if (pi2chandler->endTransmission() != 0) {
        return (false); //Sensor did not ACK
    }
    delay(10);
    while(isBusy()) ;

    calculate_CheckSum();
    delay(10);
    while(isBusy()) ;

    return (true);

}


bool RENESAS3224::writeCommand(uint8_t commandCode, uint16_t value) {

  pi2chandler->beginTransmission(_DEVICE_ADDRESS);
  pi2chandler->write(commandCode);
  pi2chandler->write((uint8_t)(value >> 8));   //MSB
  pi2chandler->write((uint8_t)(value & 0xFF)); //LSB
  if (pi2chandler->endTransmission() != 0) {
    return (false); //Sensor did not ACK
  }
  return (true);

}


bool RENESAS3224::writeCommand(uint8_t commandCode, uint8_t value_msb, uint8_t value_lsb) {

  pi2chandler->beginTransmission(_DEVICE_ADDRESS);
  pi2chandler->write(commandCode);
  pi2chandler->write(value_msb); // MSB
  pi2chandler->write(value_lsb); // LSB
  if (pi2chandler->endTransmission() != 0) {
    return (false); //Sensor did not ACK
  }
  return (true);

}
