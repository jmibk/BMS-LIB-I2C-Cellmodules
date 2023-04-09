/*
uses i2c channel 0
*/
#ifndef I2C_CELLMODULES_H
#define I2C_CELLMODULES_H
#include <Arduino.h>
#include <Wire.h>

#define TCA_ADDRESS         112
#define USED_I2C_HARDWARE   1
#define I2C_SPEED           1000ul
#define MAX_CELL_MODULES    127
#define MAX_LANES           8

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

enum batteryConfig {
    ALLSERIAL,
    PARALLEL
};

class Cellmodules {
    public:
        Cellmodules(void);
        bool init(int pinSDA, int pinSCL, uint32_t speed);
        bool init(int pinSDA, int pinSCL);
        bool init(void); 

        //new
        bool scanForModules(uint8_t lane);
        bool getDataFromModules(void);
        bool getDataFromModulesSingle(void)                                     {return getDataFromModulesSingle(true);}            //catches also calibration data
        bool getDataFromModulesSingle(boolean fullData);                                                                        //catches calibration data true/false

/*DEPRICATED*/
        //old
        bool scanForModules(void)                                               {return scanForModules(1);}
/*DEPRICATED END*/

        uint16_t get_numberofmodules_total(void);				//returns the number of modules in the whole system
		uint16_t get_numberofmodules_stack(void);				//returns the maximum number of cells in series regaring configuration PARALLEL or ALLSERIAL
        void set_batterymode(batteryConfig config)                              {_modules_data.battery_config = config;} 
		batteryConfig get_batterymode(void)										{return _modules_data.battery_config;}
 
        //new      
        void set_numberofmodules(uint8_t lane, uint8_t value)                   {_modules_data.numberofmodules[lane] = value;}                                                                                       
        uint8_t get_numberofmodules(uint8_t lane)                               {return _modules_data.numberofmodules[lane];}

/*DEPRICATED*/
        //old
        void set_numberofmodules(uint8_t value)                                 {set_numberofmodules(1, value);}                                                                               
        uint8_t get_numberofmodules(void)                                       {return get_numberofmodules(1);}
/*DEPRICATED END*/

        //global module data
        uint16_t get_modulesavailable(void)                                     {return _modules_data.modulesavailable;}
        uint16_t get_modulesnotavailable(void)                                  {return _modules_data.modulesnotavailable;}
        uint32_t get_crcerrors(void)                                            {return _modules_data.crcerrors;}

        float get_lowestcellvoltage(void)                                       {return _modules_data.lowestcellvoltage;}
        uint8_t get_lowestcellvoltagenumber(void)                               {return _modules_data.lowestcellvoltagenumber;}
        float get_highestcellvoltage(void)                                      {return _modules_data.highestcellvoltage;}
        uint8_t get_highestcellvoltagenumber(void)                              {return _modules_data.highestcellvoltagenumber;}
        float get_lowestcelltemperature(void)                                   {return _modules_data.lowestcelltemperature;}
        uint8_t get_lowestcelltemperaturenumber(void)                           {return _modules_data.lowestcelltemperaturenumber;}
        float get_highestcelltemperature(void)                                  {return _modules_data.highestcelltemperature;}
        uint8_t get_highestcelltemperaturenumber(void)                          {return _modules_data.highestcelltemperaturenumber;}
        float get_batteryvoltage(void)                                          {return _modules_data.batteryvoltage;}
        float get_batteryvoltage(uint8_t lane)                                  {return _modules_data.lane_voltage[lane];}
        float get_meancelltemperature(void)                                     {return _modules_data.meancelltemperature;}
	float get_batterydeltavoltage(void)                                     {return _modules_data.batterydeltavoltage;}

        //module data
        void set_lane_index(uint8_t lane)                                       {_modules_data.indexLane = constrain(lane,1,8);}
        uint8_t get_lane_index(void)                                            {return _modules_data.indexLane;}
        uint8_t get_module_index(void)                                          {return _modules_data.indexModule;}

        //new
        void set_cellbalancecurrentsetpoint(uint8_t lane, float value);                   //set balacing current to all modules
        void set_cellbalancecurrentsetpointsingle(uint8_t lane, uint8_t address, float value)   {if(address < MAX_CELL_MODULES) _modules_data.cellbalancecurrentsetpoint[lane][address] = value;}
        void set_cellbalanceenabled(uint8_t lane, uint8_t address, bool value)              {if(address < MAX_CELL_MODULES) _modules_data.cellbalanceenable[lane][address] = value;}

        bool get_moduleonline(uint8_t lane, uint8_t address)                                {if(address < MAX_CELL_MODULES) return _modules_data.moduleonline[lane][address]; return false;}
        float get_cellvoltage(uint8_t lane, uint8_t address)                                {if(address < MAX_CELL_MODULES) return _modules_data.cellvoltage[lane][address]; return false;}
        float get_celltemperature(uint8_t lane, uint8_t address)                            {if(address < MAX_CELL_MODULES) return _modules_data.celltemperature[lane][address]; return false;}
        bool get_cellbalanceenabled(uint8_t lane, uint8_t address)                          {if(address < MAX_CELL_MODULES) return _modules_data.cellbalanceenabled[lane][address]; return false;}
        float get_cellbalancecurrent(uint8_t lane, uint8_t address)                         {if(address < MAX_CELL_MODULES) return _modules_data.cellbalancecurrent[lane][address]; return false;}
        uint16_t get_pwmvalue(uint8_t lane, uint16_t address)                               {if(address < MAX_CELL_MODULES) return _modules_data.discharge_pwm_value[lane][address]; return false;}  
        uint32_t get_cellerrorregister(uint8_t lane, uint8_t address)                       {if(address < MAX_CELL_MODULES) return _modules_data.cellerrorregister[lane][address]; return false;}
        uint32_t get_cellcrcerrors(uint8_t lane, uint8_t address)                           {if(address < MAX_CELL_MODULES) return _modules_data.cellcrcerrors[lane][address]; return false;}
	void set_cellcrcerrors(uint8_t lane, uint8_t address, uint32_t value)               {if(address < MAX_CELL_MODULES)  _modules_data.cellcrcerrors[lane][address] = value;}

/*DEPRICATED*/
        //old
        void set_cellbalancecurrentsetpoint(float value)                                    {set_cellbalancecurrentsetpoint(1, value);}
        void set_cellbalancecurrentsetpointsingle(uint8_t address, float value)             {set_cellbalancecurrentsetpointsingle(1, address, value);}
        void set_cellbalanceenabled(uint8_t address, bool value)                            {set_cellbalanceenabled(1, address, value);}

        bool get_moduleonline(uint8_t address)                                              {return get_moduleonline(1, address);}
        float get_cellvoltage(uint8_t address)                                              {return get_cellvoltage(1, address);}
        float get_celltemperature(uint8_t address)                                          {return get_celltemperature(1, address);}
        bool get_cellbalanceenabled(uint8_t address)                                        {return get_cellbalanceenabled(1, address);}
        float get_cellbalancecurrent(uint8_t address)                                       {return get_cellbalancecurrent(1, address);}
        uint16_t get_pwmvalue(uint16_t address)                                             {return get_pwmvalue(1, address);} 
        uint32_t get_cellerrorregister(uint8_t address)                                     {return get_cellerrorregister(1, address);}
        uint32_t get_cellcrcerrors(uint8_t address)                                         {return get_cellcrcerrors(1, address);}
	void set_cellcrcerrors(uint8_t address, uint32_t value)                             {set_cellcrcerrors(1, address, value);}
/*DEPRICATED END*/
     
		//calibration
        //new
        bool calibratemodule(configValue config, uint8_t lane, uint8_t address, float value);
        float get_calibrationdata(configValue config, uint8_t lane, uint8_t address);

/*DEPRICATED*/
        //old
        bool calibratemodule(configValue config, uint8_t address, float value)              {return calibratemodule(config, 1, address, value);}
        float get_calibrationdata(configValue config, uint8_t address)                      {return get_calibrationdata(config, 1, address);}
/*DEPRICATED END*/

        //locate function
        //new
        bool set_locate(uint8_t lane, uint8_t address, bool state);
        bool get_locate(uint8_t lane, uint8_t address)                                       {if(address < MAX_CELL_MODULES) return _modules_data.locate_module[lane][address]; return false;}

/*DEPRICATED*/
        //old
        bool set_locate(uint8_t address, bool state)                                         {return set_locate(1, address, state);}
        bool get_locate(uint8_t address)                                                     {return get_locate(1, address);}
/*DEPRICATED END*/

/*TCA9548A STUFF*/
        bool TCA_isready(void);  
        bool TCA_setlane(uint8_t lane);
        uint8_t TCA_getlane(void);

bool _checkModule(uint8_t i2cAddress);
    private:
        TwoWire _i2c = TwoWire(USED_I2C_HARDWARE);
        
        bool _readCellModule(uint8_t lane, uint8_t address, uint16_t &modulesavailable, uint16_t &modulesnotavailable);         //reads regular values from module
        bool _readCellModuleCalibration(uint8_t lane, uint8_t address);             //reads calibration data from module
        void _calculateCellStateValues(void);
        bool _writedata(uint8_t lane, int i2cAddress, byte i2cRegister, uint16_t data);
        uint16_t _readdata(uint8_t lane, int i2cAddress, byte i2cRegister);

    struct modules_data_struct { 
        uint8_t         numberofmodules[MAX_LANES+1];                                                   //number of cell modules
        batteryConfig   battery_config = PARALLEL;                                                      //battery configuration SERAL or PARALLEL

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
        uint16_t        modulesavailable = 16;                                                          //cellmodules that can be communicated with
        uint16_t        modulesnotavailable = 0;                                                        //cellmodules that are missing for communication
        uint32_t        crcerrors = 0;                                                                  //crc communication errors since system start
        uint8_t         indexModule = 0;                                                                //index of cellmodules of one lane <NOT TO MODIFYING FROM ANY SCRIPT>
        uint8_t         indexLane = 1;                                                                  //index of cell module lane <NOT TO MODIFYING FROM ANY SCRIPT>

        //lane dependend things
        float           lane_voltage[MAX_LANES+1];                                                      //voltage per lane in Volts
        
        //individual cell data to set
        float           cellbalancecurrentsetpoint[MAX_LANES+1][MAX_CELL_MODULES+1];                      //for setting the desired balancing current
        bool            cellbalanceenable[MAX_LANES+1][MAX_CELL_MODULES+1];                               //for enabling the cell balance procedure

        //individual cell data
        bool            moduleonline[MAX_LANES+1][MAX_CELL_MODULES+1];                                    //cell module is online (true/false)
        float           cellvoltage[MAX_LANES+1][MAX_CELL_MODULES+1];                                     //cell voltage in volts
        float           celltemperature[MAX_LANES+1][MAX_CELL_MODULES+1];                                 //cell temperature in degC
        bool            cellbalanceenabled[MAX_LANES+1][MAX_CELL_MODULES+1];                              //cell balancing enabled (true/false)
        bool            cellbalancestate[MAX_LANES+1][MAX_CELL_MODULES+1];                                //cell balance state (true/false) for speed up of communication
        float           cellbalancecurrent[MAX_LANES+1][MAX_CELL_MODULES+1];                              //current cell balance current
        uint32_t        cellerrorregister[MAX_LANES+1][MAX_CELL_MODULES+1];                               //cell error register: <mastererrors> <notimplemented> <notimplemented> <value errors>
        uint16_t        discharge_pwm_value[MAX_LANES+1][MAX_CELL_MODULES+1];                             //raw pwm value of discharge system
        byte            moduleerrorregister[MAX_LANES+1][MAX_CELL_MODULES+1];                             //module error register: master errors
        uint32_t        cellcrcerrors[MAX_LANES+1][MAX_CELL_MODULES+1];                                   //crc communication errors since last counter reset

        //calibration data and locate
        float           calibration_reference[MAX_LANES+1][MAX_CELL_MODULES+1];      			          //reference voltage
        float           calibration_voltage[MAX_LANES+1][MAX_CELL_MODULES+1];        			          //voltage to calibrate
        float           calibration_current[MAX_LANES+1][MAX_CELL_MODULES+1];        			          //current to calibrate
        float           calibration_temperature[MAX_LANES+1][MAX_CELL_MODULES+1];    			          //temperature to calibrate
	uint16_t        calibration_current_compensation[MAX_LANES+1][MAX_CELL_MODULES+1];   	          //discharge current compensation to calibrate
	uint16_t        calibration_current_missmatch_time[MAX_LANES+1][MAX_CELL_MODULES+1]; 	          //discharge current missmatch timer
	uint8_t        	calibration_current_regulation_step[MAX_LANES+1][MAX_CELL_MODULES+1]; 	          //discharge current regulation step
	float       	calibration_current_deviation[MAX_LANES+1][MAX_CELL_MODULES+1]; 			      //max allowable current deviation while discharging
	float       	calibration_current_maximum[MAX_LANES+1][MAX_CELL_MODULES+1]; 			          //max allowable current for discharge current
        bool            locate_module[MAX_LANES+1][MAX_CELL_MODULES+1];              			          //locate enabled?
    };
    modules_data_struct _modules_data;

};
#endif