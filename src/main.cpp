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
  if (battery.init(13, 16, 100000))  //SDA, SCL
    Serial.println("cellmodules initialised");
  else
    Serial.println("cellmodules failed!");

  //set modes and so on
  battery.set_batterymode(ALLSERIAL);        //ALLSERIAL, PARALLEL
  battery.set_numberofmodules(3, 6);         //set 6 cells on lane 2 [1...8]
  battery.set_numberofmodules(7,  16);       //set 10 cells on lane 6 [1...8]

  //get initial values from modules - at first start
  battery.getDataFromModules();
  }

void loop() { 
      
#ifdef SCAN_I2C
  uint16_t modulecount = 0;

  for (uint8_t lane = 1; lane <= 8; lane++) {
    battery.set_lane_index(lane);  //set multiplexer to lane 1 [1...8]
    Serial.println("Lane number: "+String(battery.get_lane_index() ));

    battery.scanForModules(lane); 
    for (uint8_t i = 1; i <= 127; i++){
        if (battery.get_moduleonline(lane, i)) {
          Serial.println("found module "+String(i));
          modulecount++;
          }
        }
      }
   
    Serial.println("Modules found: "+String(modulecount));
    Serial.println();

#endif
  
#ifndef GET_MODULE_DATA

  battery.getDataFromModulesSingle();
  Serial.print("Module Lane: "+String(battery.get_lane_index())+" Nr: "+String(battery.get_module_index())+": ");
  if (battery.get_moduleonline(battery.get_lane_index(), battery.get_module_index()))
    Serial.println("ONLINE");
  else  
    Serial.println("OFFLINE");

#endif


//battery.set_locate(1, true);

/*
Serial.println("Modules available: "+String(battery.get_modulesavailable()));
Serial.println("Modules failed: "+String(battery.get_modulesnotavailable()));

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
/*


/*
  //Serial.println(battery.TCA_isready());
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
  }