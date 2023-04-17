/*
uses i2c channel 0
*/
#ifndef I2C_CELLMODULES_H
#define I2C_CELLMODULES_H
#include <Arduino.h>
#include <Wire.h>

#define USED_I2C_HARDWARE   1
#define I2C_SPEED           1000ul
#define MAX_CELL_MODULES    127

enum configValue {
    ADDRESS,
    REFERENCE,
    VOLTAGE,
    CURRENT,
    TEMPERATURE,
	CURRENTCOMPENSATION,
	CURRENT_MISSMATCH_TIME,
	CURRENT_REGULATION_STEP,
	CURRENT_DEVIATION,
	CURRENT_MAX
};

class Cellmodules {
    public:
        Cellmodules(void);
        bool init(int pinSDA, int pinSCL, uint32_t speed);
        bool init(int pinSDA, int pinSCL);
        bool init(void); 
        bool scanBusForModules(void);
        bool getDataFromModules(void);
        bool getDataFromModulesSingle(void);

        void set_numberofmodules(uint8_t value)                             {_modules_data.numberofmodules = value;}
        uint8_t get_numberofmodules(void)                                   {return _modules_data.numberofmodules;}

        //global module data
        uint8_t get_modulesavailable(void)                                  {return _modules_data.modulesavailable;}
        uint8_t get_modulesnotavailable(void)                               {return _modules_data.modulesnotavailable;}
        uint32_t get_crcerrors(void)                                        {return _modules_data.crcerrors;}

        float get_lowestcellvoltage(void)                                   {return _modules_data.lowestcellvoltage;}
        uint8_t get_lowestcellvoltagenumber(void)                           {return _modules_data.lowestcellvoltagenumber;}
        float get_highestcellvoltage(void)                                  {return _modules_data.highestcellvoltage;}
        uint8_t get_highestcellvoltagenumber(void)                          {return _modules_data.highestcellvoltagenumber;}
        float get_lowestcelltemperature(void)                               {return _modules_data.lowestcelltemperature;}
        uint8_t get_lowestcelltemperaturenumber(void)                       {return _modules_data.lowestcelltemperaturenumber;}
        float get_highestcelltemperature(void)                              {return _modules_data.highestcelltemperature;}
        uint8_t get_highestcelltemperaturenumber(void)                      {return _modules_data.highestcelltemperaturenumber;}
        float get_batteryvoltage(void)                                      {return _modules_data.batteryvoltage;}
        float get_meancelltemperature(void)                                 {return _modules_data.meancelltemperature;}
	    float get_batterydeltavoltage(void)                                 {return _modules_data.batterydeltavoltage;}

        //module data
        void set_cellbalancecurrentsetpoint(float value);                   //set balacing current to all modules
        void set_cellbalancecurrentsetpointsingle(uint8_t address, float value)   {if(address < MAX_CELL_MODULES) _modules_data.cellbalancecurrentsetpoint[address] = value;}
        void set_cellbalanceenabled(uint8_t address, bool value)            {if(address < MAX_CELL_MODULES) _modules_data.cellbalanceenable[address] = value;}

        bool get_moduleonline(uint8_t address)                              {if(address < MAX_CELL_MODULES) return _modules_data.moduleonline[address]; return false;}
        float get_cellvoltage(uint8_t address)                              {if(address < MAX_CELL_MODULES) return _modules_data.cellvoltage[address]; return false;}
        float get_celltemperature(uint8_t address)                          {if(address < MAX_CELL_MODULES) return _modules_data.celltemperature[address]; return false;}
        bool get_cellbalanceenabled(uint8_t address)                        {if(address < MAX_CELL_MODULES) return _modules_data.cellbalanceenabled[address]; return false;}
        float get_cellbalancecurrent(uint8_t address)                       {if(address < MAX_CELL_MODULES) return _modules_data.cellbalancecurrent[address]; return false;}
        uint16_t get_pwmvalue(uint16_t address)                             {if(address < MAX_CELL_MODULES) return _modules_data.discharge_pwm_value[address]; return false;}  
        uint32_t get_cellerrorregister(uint8_t address)                     {if(address < MAX_CELL_MODULES) return _modules_data.cellerrorregister[address]; return false;}
        uint32_t get_cellcrcerrors(uint8_t address)                         {if(address < MAX_CELL_MODULES) return _modules_data.cellcrcerrors[address]; return false;}
	    void set_cellcrcerrors(uint8_t address, uint32_t value)             {if(address < MAX_CELL_MODULES)  _modules_data.cellcrcerrors[address] = value;}
        
		//calibration
        bool calibratemodule(configValue config, uint8_t address, float value);
        float getcalibrationdata(configValue config, uint8_t address);

        //locate function
        bool setLocate(uint8_t address, bool state);
        bool getLocate(uint8_t address)                                     {if(address < MAX_CELL_MODULES) return _modules_data.locate_module[address]; return false;}

    private:
        TwoWire _i2c = TwoWire(USED_I2C_HARDWARE);
        bool _checkModule(byte i2cAddress);
        bool _readCellModule(uint8_t address, uint8_t &modulesavailable, uint8_t &modulesnotavailable);
        void _calculateCellStateValues(void);
        bool _writedata(int i2cAddress, byte i2cRegister, uint16_t data);
        uint16_t _readdata(int i2cAddress, byte i2cRegister);

    struct modules_data_struct { 
        uint8_t         numberofmodules = 16;                           //number of cell modules

        //data on whole battery
        float           lowestcellvoltage = 9999;
        uint8_t         lowestcellvoltagenumber = 0;
        float           highestcellvoltage = 0;
        uint8_t         highestcellvoltagenumber = 0;
        float           lowestcelltemperature = 9999;
        uint8_t         lowestcelltemperaturenumber = 0;
        float           highestcelltemperature = -100;
        uint8_t         highestcelltemperaturenumber = 0; 
        float           batteryvoltage = 51.2;
        float           meancelltemperature = 24;
	    float           batterydeltavoltage = 0;
        uint8_t         modulesavailable = 16;                          //cellmodules that can be communicated with
        uint8_t         modulesnotavailable = 0;                        //cellmodules that are missing for communication
        uint32_t        crcerrors = 0;                                  //crc communication errors since system start
        uint8_t         moduleindex = 0;
        
        //individual cell data to set
        float           cellbalancecurrentsetpoint[MAX_CELL_MODULES+1]; //for setting the desired balancing current
        bool            cellbalanceenable[MAX_CELL_MODULES+1];          //for enabling the cell balance procedure

        //individual cell data
        bool            moduleonline[MAX_CELL_MODULES+1];               //cell module is online (true/false)
        float           cellvoltage[MAX_CELL_MODULES+1];                //cell voltage in volts
        float           celltemperature[MAX_CELL_MODULES+1];            //cell temperature in degC
        bool            cellbalanceenabled[MAX_CELL_MODULES+1];         //cell balancing enabled (true/false)
        float           cellbalancecurrent[MAX_CELL_MODULES+1];         //current cell balance current
        uint32_t        cellerrorregister[MAX_CELL_MODULES+1];          //cell error register: <mastererrors> <notimplemented> <notimplemented> <value errors>
        uint16_t        discharge_pwm_value[MAX_CELL_MODULES+1];        //raw pwm value of discharge system
        byte            moduleerrorregister[MAX_CELL_MODULES+1];        //module error register: master errors
        uint32_t        cellcrcerrors[MAX_CELL_MODULES+1];              //crc communication errors since last counter reset

        //calibration data and locate
        float           calibration_reference[MAX_CELL_MODULES+1];      			//reference voltage
        float           calibration_voltage[MAX_CELL_MODULES+1];        			//voltage to calibrate
        float           calibration_current[MAX_CELL_MODULES+1];        			//current to calibrate
        float           calibration_temperature[MAX_CELL_MODULES+1];    			//temperature to calibrate
		uint16_t        calibration_current_compensation[MAX_CELL_MODULES+1];   	//discharge current compensation to calibrate
		uint16_t        calibration_current_missmatch_time[MAX_CELL_MODULES+1]; 	//discharge current missmatch timer
		uint8_t        	calibration_current_regulation_step[MAX_CELL_MODULES+1]; 	//discharge current regulation step
		float       	calibration_current_deviation[MAX_CELL_MODULES+1]; 			//max allowable current deviation while discharging
		float       	calibration_current_maximum[MAX_CELL_MODULES+1]; 			//max allowable current for discharge current
        bool            locate_module[MAX_CELL_MODULES+1];              			//locate enabled?
    };
    modules_data_struct _modules_data;

};
#endif