#ifndef RENESAS3224_H
#define RENESAS3224_h

#include "Arduino.h"
#include <Wire.h>

#define MAXIMUM_RCV_SIZE 7
#define DEFAULT_I2C_ADDRESS 0x08
// STATUS //
#define RENESAS3224_STATUS_SMCONFIG_1 0
#define RENESAS3224_STATUS_SMCONFIG_2 1
#define RENESAS3224_STATUS_MODE_NORMAL 0
#define RENESAS3224_STATUS_MODE_COMMAND 1

// SM_CONFIG Parameters //
// GAIN 1 stage //
#define RENESAS3224_SMCONFIG_GAIN1_6 0
#define RENESAS3224_SMCONFIG_GAIN1_12 1
#define RENESAS3224_SMCONFIG_GAIN1_20 2
#define RENESAS3224_SMCONFIG_GAIN1_30 3
#define RENESAS3224_SMCONFIG_GAIN1_40 4
#define RENESAS3224_SMCONFIG_GAIN1_60 5
#define RENESAS3224_SMCONFIG_GAIN1_80 6
#define RENESAS3224_SMCONFIG_GAIN1_120 7
// GAIN 2 stage //
#define RENESAS3224_SMCONFIG_GAIN2_11 0
#define RENESAS3224_SMCONFIG_GAIN2_12 1
#define RENESAS3224_SMCONFIG_GAIN2_13 2
#define RENESAS3224_SMCONFIG_GAIN2_14 3
#define RENESAS3224_SMCONFIG_GAIN2_15 4
#define RENESAS3224_SMCONFIG_GAIN2_16 5
#define RENESAS3224_SMCONFIG_GAIN2_17 6
#define RENESAS3224_SMCONFIG_GAIN2_18 7
// GAIN polarity //
#define RENESAS3224_SMCONFIG_POLARITY_POS 0
#define RENESAS3224_SMCONFIG_POLARITY_NEG 1
// ADC bits //
#define RENESAS3224_SMCONFIG_ADC_12BIT 0
#define RENESAS3224_SMCONFIG_ADC_13BIT 1
#define RENESAS3224_SMCONFIG_ADC_14BIT 2
#define RENESAS3224_SMCONFIG_ADC_15BIT 3
#define RENESAS3224_SMCONFIG_ADC_16BIT 4
#define RENESAS3224_SMCONFIG_ADC_17BIT 5
#define RENESAS3224_SMCONFIG_ADC_18BIT 6
#define RENESAS3224_SMCONFIG_ADC_19BIT 7
#define RENESAS3224_SMCONFIG_ADC_20BIT 8
#define RENESAS3224_SMCONFIG_ADC_21BIT 9
#define RENESAS3224_SMCONFIG_ADC_22BIT 10
#define RENESAS3224_SMCONFIG_ADC_23BIT 11
#define RENESAS3224_SMCONFIG_ADC_24BIT 12
// ABSv Enable/Disable //
#define RENESAS3224_SMCONFIG_ABSV_DISABLE 0
#define RENESAS3224_SMCONFIG_ABSV_ENABLE 1
// OFFSET//
#define RENESAS3224_SMCONFIG_OFFSET_NON 0
#define RENESAS3224_SMCONFIG_OFFSET_675 1
#define RENESAS3224_SMCONFIG_OFFSET_125 2
#define RENESAS3224_SMCONFIG_OFFSET_1925 3
#define RENESAS3224_SMCONFIG_OFFSET_25 4
#define RENESAS3224_SMCONFIG_OFFSET_3175 5
#define RENESAS3224_SMCONFIG_OFFSET_385 6
#define RENESAS3224_SMCONFIG_OFFSET_4325 7
// SHIFT Method//
#define RENESAS3224_SMCONFIG_SHIFT_NON 0
#define RENESAS3224_SMCONFIG_SHIFT_ENABLE 1


typedef volatile union{
    struct {
        uint8_t address: 7;
        uint8_t int_setup: 2;
        uint8_t ss_polarity: 1;
        uint8_t ckp_cke: 2;
        uint8_t cyc_period: 3;
        uint8_t sot_curve: 1;
    }bits;
    uint16_t reg_value;
}renesas3224_intconfig_t;

typedef volatile union{
    struct {
        uint8_t gain1: 3;
        uint8_t gain2: 3;
        uint8_t gain_polarity: 1;
        uint8_t adc_bits: 4;
        uint8_t absv_enable: 1;
        uint8_t offset: 3;
        uint8_t shift_method: 1;
    }bits;
    uint16_t reg_value;
}renesas3224_smconfig_t;

typedef volatile union{
    struct {
        uint8_t ALU: 1;
        uint8_t ConfigSetup: 1;
        uint8_t MemError: 1;
        uint8_t Mode: 2;
        uint8_t Busy: 1;
        uint8_t Powered: 1;
        uint8_t NA: 1;
    }bits;
    uint8_t reg_value;
}renesas3224_status_t;

typedef struct{
    renesas3224_status_t status;
    int data;
}single_data_t;

typedef struct{
    renesas3224_status_t status;
    int sensor;
    int temperature;
}bundle_data_t;

class RENESAS3224
{
    public:
        RENESAS3224(uint8_t _DEVICE_ADDRESS=DEFAULT_I2C_ADDRESS);
        bool begin(TwoWire &I2Chandler);

        // Check if the device respond to the given I2C address //
        bool isConnected(void);

        // Set device in Normal state operation; Command 0xA8 //
        bool set_NormalMode(void);
        // Set device in Command state operation; Command 0xA9 //
        bool set_CommandMode(void);

        // Request device status register //
        uint8_t getStatus(void);
        bool isBusy(void);

        // Select 'SM_configX' (X: 1-2) register for any measurement using the memory contents //
        //    SMconfig1: (0x12) //
        //    SMconfig1: (0x16) //
        bool select_SMconfig1(void);
        bool select_SMconfig2(void);

        // Cyclic measure: continuous full measurement cycle (AZSM, SM, AZTM, and TM) //
        // return: 24-bit sensor measurement + 24-bit temperature data //
        //bool measure_cyclic(void);
        //bool stop_cyclic(void);
        
        // Calculate and write memory checksum (CRC), which is register address 39HEX) //
        bool calculate_CheckSum(void);

        // Read data in the user memory address (0x00 to 0x39) matching the command //
        // return: memory value //
        uint16_t read_MemoryAddress(uint8_t memory_address);

        // Write data to user memory at address specified by command minus 0x40 //
        bool write_MemoryData(uint8_t memory_address, uint8_t data_msb, uint8_t data_lsb);
        bool write_MemoryData(uint8_t memory_address, uint16_t data);

        // Get_RawData -> This command performs a measurement and write the raw ADC data into the output register //
        // Command range: 0xA0 to 0xA7
        // return:  24-bit formatted raw data //
        int get_RawData(uint8_t command, uint16_t config=0);

        // Set SMconfig register //
        bool set_SMconfigRegister(uint8_t sm_selector, uint16_t register_value);
        bool set_SMconfigGain1(uint8_t sm_selector, uint8_t gain1);
        bool set_SMconfigGain2(uint8_t sm_selector, uint8_t gain2);
        bool set_SMconfigADCbits(uint8_t sm_selector, uint8_t adc_bits);

    private:
        uint8_t _DEVICE_ADDRESS;
        TwoWire *pi2chandler;
        
        // Device status //
        bool _connected;

        // Device config registers //
        renesas3224_smconfig_t smconfig1;
        renesas3224_smconfig_t smconfig2;
        uint8_t smconfig_selector;

        // Read Data and read a full measurement //
        uint16_t readData(void);
        int readRawMeasurement(void);
        uint8_t readFullMeasurement(int &sensorData, int &tempData);
        
        // //
        bool write_data_to_memory(uint8_t memory_address, uint16_t data);
        bool writeCommand(uint8_t commandCode);
        bool writeCommand(uint8_t commandCode, uint16_t value);
        bool writeCommand(uint8_t commandCode, uint8_t value_msb, uint8_t value_lsb);

};

#endif

