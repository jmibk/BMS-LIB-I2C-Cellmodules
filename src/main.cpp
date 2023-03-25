/*
current request time: 
79ms @Speed = 1.000   => max 100 boards
87ms @Speed = 5.000   => max 90 boards
44ms @Speed = 10.000  => max 180 boards
15ms @Speed = 50.000  => max 530 boards
11ms @Speed = 100.000 => max 720 boards

2ms if no board is there
*/

#include <Arduino.h>
#include "i2c_cellmodules.h"

Cellmodules battery = Cellmodules();

void setup() {
  //init serial connection
  Serial.begin(115200);
  Serial.println("Demo - Cellmodules over I2C");
 
  //init i2c
  if (battery.init(13, 16, 5000))  //SDA, SCL
    Serial.println("cellmodules initialised");
  else
    Serial.println("cellmodules failed!");

  //set modes and so on
  //battery.set_batterymode(ALLSERIAL);     //ALLSERIAL, PARALLEL
  battery.set_numberofmodules(1, 2);      //set 16 cells on lane 1
  //battery.set_numberofmodules(2,  4);      //set 0 cells on lane 2
  //battery.set_lanenumber(0);

  //get initial values from modules - at first start
  battery.getDataFromModules();
  }

void loop() { 
  //battery
  battery.getDataFromModulesSingle();

  Serial.println("Index  : "+String(battery._modules_data.indexLane)+"/"+String(battery._modules_data.indexModule));

  //battery.set_locate(1, true);

  Serial.println("Reading data from cell modules");

  Serial.println("Modules available: "+String(battery.get_modulesavailable()));
  Serial.println("Modules failed: "+String(battery.get_modulesnotavailable()));
/*
  Serial.println("CRC Errors: "+String(battery.get_crcerrors()));
  Serial.println("lowest V: "+String(battery.get_lowestcellvoltage(),3));
  Serial.println("Number: "+String(battery.get_lowestcellvoltagenumber()));
  Serial.println("highest V: "+String(battery.get_highestcellvoltage(),3));
  Serial.println("Number: "+String(battery.get_highestcellvoltagenumber()));
  Serial.println("lowest Temp: "+String(battery.get_lowestcelltemperature(),1));
  Serial.println("Number: "+String(battery.get_lowestcelltemperaturenumber()));
  Serial.println("highest Temp: "+String(battery.get_highestcelltemperature(),1));
  Serial.println("Number: "+String(battery.get_highestcelltemperaturenumber()));

  Serial.println("Battery V: "+String(battery.get_batteryvoltage(),3));
  Serial.println("Mean Temperature: "+String(battery.get_meancelltemperature(),1));
*/
/*
  battery.scanForModules(); 
  for (uint8_t i = 0; i <= 127; i++){
      if (battery.get_moduleonline(i))
          Serial.println("found module "+String(i));
      }
*/
  for (uint8_t i = 1; i <= 16; i++){
    Serial.println("Module "+String(i)+": "+String(battery.get_moduleonline(i)));
    uint32_t temp = battery.get_cellerrorregister(i);
    if (temp)
      Serial.println("Error Register "+String(i)+": "+String(temp, BIN));
    }
/*
  //config test
  byte address = 16;
  //battery.calibratemodule(VOLTAGE, address, 0.019);    //ADDRESS, VOLTAGE, CURRENT, TEMPERATURE; cellmodule; value in SI
  //battery.calibratemodule(TEMPERATURE, address, 0);    //ADDRESS, VOLTAGE, CURRENT, TEMPERATURE; cellmodule; value in SI
  Serial.println("Calibration Data ADDRESS: "+String(battery.getcalibrationdata(ADDRESS, address),0));
  Serial.println("Calibration Data VOLTAGE: "+String(battery.getcalibrationdata(VOLTAGE, address),3)+" V, "+String(battery.get_cellvoltage(address),3)+" V");
  Serial.println("Calibration Data CURRENT: "+String(battery.getcalibrationdata(CURRENT, address),3)+" A, "+String(battery.get_cellbalancecurrent(address),3)+" A");
  Serial.println("Calibration Data TEMPERATURE: "+String(battery.getcalibrationdata(TEMPERATURE, address),1)+" degC, "+String(battery.get_celltemperature(address),1)+" degC");
*/
  delay(1000);
  Serial.println("");
  }