// ---------------------------------------------------------------- /
// Arduino I2C Scanner
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using GY-87 module for the target
// Tested on 10 September 2019
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
// ---------------------------------------------------------------- /

#include <Arduino.h>
#include <Wire.h> //include Wire.h library
#include <RENESAS3224.h>

#define UPDATING_INTERVAL_MS 3000
#define HostCom Serial
#define HostCom_BaudRate 115200

RENESAS3224 ampSensor(0x08);

void setup() {
  renesas3224_status_t stat;
  HostCom.begin(HostCom_BaudRate); // The baudrate of Serial monitor is set in 9600
  while (!HostCom);                // Waiting for Serial Monitor
  HostCom.println("\n--- RENESAS3224 Library ---");

  Wire.begin(); // Wire communication begin
  ampSensor.begin(Wire);
  HostCom.print("- Device Connected: ");
  HostCom.print(ampSensor.isConnected());
  HostCom.print("; Status: ");
  stat.reg_value = ampSensor.getStatus();
  HostCom.print(stat.reg_value, HEX);
  HostCom.print("-> Setup: ");
  HostCom.print(stat.bits.ConfigSetup);
  HostCom.print("; Power: ");
  HostCom.print(stat.bits.Powered);
  HostCom.print("; Busy: ");
  HostCom.print(stat.bits.Busy);
  HostCom.print("; Mode: ");
  HostCom.print(stat.bits.Mode);
  HostCom.print("; Mem: ");
  HostCom.print(stat.bits.MemError);
  HostCom.println();

  // // Set inittial setup //
  // HostCom.print("- Interface1: 0x");
  // HostCom.println(ampSensor.read_MemoryAddress(0x02), HEX);
  // HostCom.print("- SMConfig1: 0x");
  // renesas3224_smconfig_t smconfig1;
  // smconfig1.reg_value = ampSensor.read_MemoryAddress(0x12);
  // HostCom.println(smconfig1.reg_value, HEX);
  // HostCom.print("- ADC: ");
  // HostCom.print(smconfig1.bits.adc_bits, HEX);
  // HostCom.print("; GAIN1: ");
  // HostCom.print (smconfig1.bits.gain1, HEX);
  // HostCom.print("; GAIN2: ");
  // HostCom.println(smconfig1.bits.gain1, HEX);

  // // Set default values //
  // smconfig1.bits.adc_bits = RENESAS3224_SMCONFIG_ADC_20BIT;
  // smconfig1.bits.gain1 = RENESAS3224_SMCONFIG_GAIN1_60;
  // smconfig1.bits.gain2 = RENESAS3224_SMCONFIG_GAIN2_12;
  // HostCom.print("- New SMConfig1: 0x");
  // HostCom.println(smconfig1.reg_value, HEX);

  // HostCom.print("* Set SMCONFIG REG: ");
  // HostCom.println(ampSensor.set_SMconfigRegister(0, smconfig1.reg_value)==true?"True":"False");
  // // HostCom.print("* Set ADC: ");
  // // HostCom.println(ampSensor.set_SMconfigADCbits(0, smconfig1.bits.adc_bits)==true?"True":"False");
  // // HostCom.print("* Set Gain1: ");
  // // HostCom.println(ampSensor.set_SMconfigGain1(0, smconfig1.bits.gain1)==true?"True":"False");
  // // HostCom.print("* Set Gain2: ");
  // // HostCom.println(ampSensor.set_SMconfigGain2(0, smconfig1.bits.gain2)==true?"True":"False");
  
  // HostCom.print("- Status: ");
  // HostCom.println(ampSensor.getStatus(), HEX);

}

void loop() {
  static unsigned long time_interval = millis();
  unsigned long mtime = millis();
  renesas3224_status_t dev_status = {0};

  // if(mtime >= time_interval){

  //   dev_status.reg_value = ampSensor.getStatus();

  //   HostCom.print("- Device Connected: ");
  //   HostCom.print(ampSensor.isConnected());
  //   HostCom.print("; Status: ");
  //   HostCom.print(dev_status.reg_value, HEX);
  //   HostCom.print("-> Setup: ");
  //   HostCom.print(dev_status.bits.ConfigSetup);
  //   HostCom.print("; Power: ");
  //   HostCom.print(dev_status.bits.Powered);
  //   HostCom.print("; Busy: ");
  //   HostCom.print(dev_status.bits.Busy);
  //   HostCom.print("; Mode: ");
  //   HostCom.print(dev_status.bits.Mode);
  //   HostCom.print("; Mem: ");
  //   HostCom.print(dev_status.bits.MemError);
  //   HostCom.println();

  //   HostCom.print("   * IsBusy: ");
  //   HostCom.print(ampSensor.isBusy());
  //   HostCom.println();

  //   HostCom.print("   * ReadMem: ");
  //   HostCom.print(ampSensor.read_MemoryAddress(0x12), HEX);
  //   HostCom.print("; ReadMem: ");
  //   HostCom.print(ampSensor.read_MemoryAddress(0x16), HEX);
  //   HostCom.println();

  //   HostCom.print("   * RawData: ");
  //   HostCom.print(ampSensor.get_RawData(0xA0));
  //   HostCom.print("; RawData: ");
  //   HostCom.print(ampSensor.get_RawData(0xA2));

  //   HostCom.println();
  //   HostCom.flush();

  //   time_interval += UPDATING_INTERVAL_MS;
  // }

}