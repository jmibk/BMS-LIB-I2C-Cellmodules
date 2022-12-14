#include "i2c_cellmodules.h"

//constructor - does nothing
Cellmodules::Cellmodules(void) {
    }

bool Cellmodules::init(int pinSDA, int pinSCL, uint32_t speed) {
    _i2c.begin(pinSDA, pinSCL, speed);      
    return true;
    }

bool Cellmodules::init(int pinSDA, int pinSCL) {
    //_i2c.begin(13, 16, I2C_SPEED);  //ESP32: I2C_SDA, I2C_SCL - ESP32-EVB
    _i2c.begin(pinSDA, pinSCL, I2C_SPEED);      //ESP32: I2C_SDA, I2C_SCL - ESP32-EVB
    return true;
    }

bool Cellmodules::init(void) {
    return init(13, 16);
    }

bool Cellmodules::scanBusForModules(void){
    for (uint8_t module = 1; module <= MAX_CELL_MODULES; module++){
        if(_checkModule(module)){
            _modules_data.moduleonline[module] = true;          //module is online
			}
        else {
            _modules_data.moduleonline[module] = false;         //module is offline
			}
        }
    return true;
    }

bool Cellmodules::getDataFromModules(void) {
    //reset values: cellmodule communication states
    _modules_data.modulesavailable = 0;              
    _modules_data.modulesnotavailable = 0;

    //gather all data from all cell modules first
    for (uint8_t address = 1; address <= _modules_data.numberofmodules; address++){ 
        _readCellModule(address, _modules_data.modulesavailable, _modules_data.modulesnotavailable); 
        }
    
    //calculating some data such as mean temperature or voltage delta or min/max values
    _calculateCellStateValues();

    return true;
    }

bool Cellmodules::getDataFromModulesSingle(void) {
    //temporary variables to count modules available. Copy it after last module into storage to read
    static uint8_t modulesavailable;
    static uint8_t modulesnotavailable;

    //if moidule index is ZERO, correct it to 1
    if (_modules_data.moduleindex == 0)
       _modules_data.moduleindex = 1;

    //reset values: cellmodule communication states if module index  is 1 (starting)
    if (_modules_data.moduleindex == 1){
        _modules_data.modulesavailable = modulesavailable;              
        _modules_data.modulesnotavailable = modulesnotavailable;
        modulesavailable = 0;
        modulesnotavailable = 0;
        }

    //read one module (modulesavailable and modulesnotavailable will be updated there)
    _readCellModule(_modules_data.moduleindex, modulesavailable, modulesnotavailable); 

    //calculating some data such as mean temperature or voltage delta or min/max values
    _calculateCellStateValues();

    //check if module index is too high. If yes, restart it at 1
    _modules_data.moduleindex++;
    if (_modules_data.moduleindex > _modules_data.numberofmodules)
       _modules_data.moduleindex = 1;

    return true;
    }

//set balancing current to all modules
void Cellmodules::set_cellbalancecurrentsetpoint(float value){
    //step throug every cell module to set discharge current
    for (uint8_t address = 1; address <= _modules_data.numberofmodules; address++){
        set_cellbalancecurrentsetpointsingle(address, value);
        }
    }

//read single cell module
bool Cellmodules::_readCellModule(uint8_t address, uint8_t &modulesavailable, uint8_t &modulesnotavailable) {
    //reset values: cell module and master error
    _modules_data.cellerrorregister[address] = 0x000000;
    _modules_data.moduleerrorregister[address] = 0x00; 

    //if address is greather than maximum cell module count, return false
    if (address > MAX_CELL_MODULES)    
        return false;

    //try to communicate with cell module. if an error occours, set all cell data to zero and continue with next module
    if (!_checkModule(address)) {
        _modules_data.moduleonline[address] = false;
        _modules_data.cellvoltage[address] = 0;                   
        _modules_data.celltemperature[address] = 0;               
        _modules_data.cellbalanceenabled[address] = false;            
        _modules_data.cellbalancecurrent[address] = 0;                           
        _modules_data.discharge_pwm_value[address] = 0;
        _modules_data.moduleerrorregister[address] |= 0b00000001;     //bit0 = no cell module available   
        _modules_data.cellerrorregister[address] = 0x00000000;   
        _modules_data.cellerrorregister[address] |= _modules_data.moduleerrorregister[address] << 24;    

        _modules_data.calibration_reference[address] = 0; 
        _modules_data.calibration_voltage[address] = 0;           
        _modules_data.calibration_current[address] = 0;           
        _modules_data.calibration_temperature[address] = 0;       
		_modules_data.calibration_current_compensation[address] = 0;  
		_modules_data.calibration_current_missmatch_time[address] = 0;  
		_modules_data.calibration_current_regulation_step[address] = 0;  
		_modules_data.calibration_current_deviation[address] = 0;  
		_modules_data.calibration_current_maximum[address] = 0;  
        _modules_data.locate_module[address] = 0;                 

        modulesnotavailable++;        
        return false;
        }
    _modules_data.moduleonline[address] = true;    
    modulesavailable++;  

    //now there is no error, continue with reading some data

    //read the cell board address, if it is 0xFFFF there is an error
    uint16_t moduleaddress = _readdata(address, 0x02);                                                  //module i2c address
    if (moduleaddress == 0xFFFF) {
        _modules_data.moduleonline[address] = false;
        _modules_data.moduleerrorregister[address] |= 0b00000010;     //bit1 = cell module communication error 
        return false;
        }
        
    //everything looks ok read all other values from the cell modules
    _modules_data.cellvoltage[address] = _readdata(address, 0x05) / 1000.0;                     //cell voltage
    _modules_data.celltemperature[address] = (_readdata(address, 0x06) - 1000)/10.0;            //temperature
    _modules_data.cellbalanceenabled[address] = _readdata(address, 0x0C);                       //balancing active
    _modules_data.cellbalancecurrent[address] = _readdata(address, 0x0B) / 1000.0;              //current balancing current
    _modules_data.discharge_pwm_value[address] = _readdata(address, 0x0F);                      //pwm value
    _modules_data.cellerrorregister[address] = (_readdata(address, 0x09) << 16) + (_readdata(address, 0x08) << 8) + _readdata(address, 0x07);           //errors: 0b00000000 <0x09> <0x08> <0x07>
    _modules_data.cellerrorregister[address] |= _modules_data.moduleerrorregister[address] << 24;

    _modules_data.calibration_reference[address] = _readdata(address, 0x13)/1000.0;  
    _modules_data.calibration_voltage[address] = (_readdata(address, 0x10)-1000)/1000.0;           
    _modules_data.calibration_current[address] = (_readdata(address, 0x11)-1000)/1000.0;           
    _modules_data.calibration_temperature[address] = (_readdata(address, 0x12)-1000)/10.0;
	_modules_data.calibration_current_compensation[address] = _readdata(address, 0x14);  
	_modules_data.calibration_current_missmatch_time[address] = _readdata(address, 0x15);  
	_modules_data.calibration_current_regulation_step[address] = _readdata(address, 0x16);  
	_modules_data.calibration_current_deviation[address] = _readdata(address, 0x17)/1000.0;  
	_modules_data.calibration_current_maximum[address] = _readdata(address, 0x18)/1000.0;  
		
    _modules_data.locate_module[address] = _readdata(address, 0x04);  

    //set values to cell modules
    //enable balancing
    if (_modules_data.cellbalanceenable[address]){
        _writedata(address, 0x0A, _modules_data.cellbalancecurrentsetpoint[address]*1000.0);    //set balancing current
        _writedata(address, 0x0C, 1);                                                           //enable balancing
        }
    else {
        _writedata(address, 0x0C, 0);                                                           //disable balancing
        }

    //Serial.println("Cell Module "+String(address)+": "+String(_modules_data.cellbalancecurrentsetpoint[address]*1000.0) );

    return true;
    }

//calculates some values such as cell voltage, min/max cell voltages, min/max temperatures
void Cellmodules::_calculateCellStateValues(void) {
   //first "reset" all values
    float           lowestcellvoltage = 9999;
    uint8_t         lowestcellvoltagenumber = 0;
    float           highestcellvoltage = 0;
    uint8_t         highestcellvoltagenumber = 0;
    float           lowestcelltemperature = 9999;
    uint8_t         lowestcelltemperaturenumber = 0;
    float           highestcelltemperature = -100;
    uint8_t         highestcelltemperaturenumber = 0; 

    float           batteryvoltage = 0;
    float           meancelltemperature = 0;

    //step throug every cell voltage and temperature and compare / gather your data
    for (uint8_t address = 1; address <= _modules_data.numberofmodules; address++){ 
        //if module is offline, go to the next module
        if (!_modules_data.moduleonline[address])
            continue;

        //cell minimum voltage
        if (_modules_data.cellvoltage[address] <= lowestcellvoltage) {
            lowestcellvoltage = _modules_data.cellvoltage[address];
            lowestcellvoltagenumber = address;
            }
        //cell maximum voltage
        if (_modules_data.cellvoltage[address] >= highestcellvoltage) {
            highestcellvoltage = _modules_data.cellvoltage[address];
            highestcellvoltagenumber = address;
            }               
        //cell minimum temperature
        if (_modules_data.celltemperature[address] <= lowestcelltemperature) {
            lowestcelltemperature = _modules_data.celltemperature[address];
            lowestcelltemperaturenumber = address;
            } 
        //cell maximum temperature
        if (_modules_data.celltemperature[address] >= highestcelltemperature) {
            highestcelltemperature = _modules_data.celltemperature[address];
            highestcelltemperaturenumber = address;
            }

        //put data into struct (to avoid short outbreaks of values whren reading them)
        _modules_data.lowestcellvoltage = lowestcellvoltage;
        _modules_data.lowestcellvoltagenumber = lowestcellvoltagenumber;
        _modules_data.highestcellvoltage = highestcellvoltage;
        _modules_data.highestcellvoltagenumber = highestcellvoltagenumber;
        _modules_data.lowestcelltemperature = lowestcelltemperature;
        _modules_data.lowestcelltemperaturenumber = lowestcelltemperaturenumber;
        _modules_data.highestcelltemperature = highestcelltemperature;
        _modules_data.highestcelltemperaturenumber = highestcelltemperaturenumber; 

        //battery voltage
        batteryvoltage += _modules_data.cellvoltage[address];

        //celltemperature
        meancelltemperature += _modules_data.celltemperature[address];
        }

    //cellsdeltavoltage
    _modules_data.batterydeltavoltage = highestcellvoltage - lowestcellvoltage;

    //put data into struct (to avoid short outbreaks of values whren reading them)
    _modules_data.batteryvoltage = batteryvoltage;
    if (_modules_data.numberofmodules)      //avoid division /0
        _modules_data.meancelltemperature = meancelltemperature / _modules_data.modulesavailable;
    }

//check if a i2c device is available on the bus. Returns true if board is there, otherwise false.
bool Cellmodules::_checkModule(byte i2cAddress) {
    _i2c.beginTransmission(i2cAddress);
    byte state = _i2c.endTransmission();
    if (!state) 
        return true;    
    return false;
    }

bool Cellmodules::_writedata(int i2cAddress, byte i2cRegister, uint16_t data) {
    //cehck if address is in range 
    if(i2cAddress > MAX_CELL_MODULES) 
        return false;

    uint16_t crc = ( i2cRegister + (data >> 8) + (data >> 0) ) % 256;
    _i2c.beginTransmission(i2cAddress);                   //queuing the slave address
    _i2c.write(i2cRegister);                              //queuing the register address/pointing regsiter
    _i2c.write(data >> 8);                                //higher byte first
    _i2c.write(data >> 0);
    _i2c.write(crc);                                      //crc checksum
    byte busStatus = _i2c.endTransmission();              //transmit all queued data and bring STOP condition on I2C Bus
    if(busStatus != 0x00) {
        _modules_data.cellcrcerrors[i2cAddress]++;          //add up cell based crc counter, that is deleted after viewing on web interface
        _modules_data.crcerrors++;                          //add up global crc error counter
        return false;
        }  
    return true;
    }

//calibration procedures
bool Cellmodules::calibratemodule(configValue config, uint8_t address, float value){
    //cehck if address is in range 
    if(address > MAX_CELL_MODULES) 
        return false;

    //check if module is there
    if (!_checkModule(address))
        return false;

    //if it is there, populate register and value
    byte configregister;      //0x02 = address 0x10 = voltage, 0x11=current, 0x12=temperature
    switch (config) {
        case ADDRESS:
            configregister = 0x02;
            value = (int)value;                     //make it integer
            break;
        case REFERENCE:
            configregister = 0x13;
            _modules_data.calibration_reference[address] = value;
            value = (int)(value*1000);              //volts to mVolts
            break;
        case VOLTAGE:
            configregister = 0x10;
            _modules_data.calibration_voltage[address] = value;
            value = (int)(value*1000) + 1000;       //volts to mVolts and offset of 1000mV
            break;
        case CURRENT:
            configregister = 0x11;
            _modules_data.calibration_current[address] = value;
            value = (int)(value*1000) + 1000;       //amps to mAmps and offset of 1000mA
            break;
        case TEMPERATURE:
            configregister = 0x12;
            _modules_data.calibration_temperature[address] = value;
            value = (int)(value*10) + 1000;       //deg to ddeg and offset of 1000ddeg
            break;
        case CURRENTCOMPENSATION:
            configregister = 0x14;
            _modules_data.calibration_current_compensation[address] = value;
            value = (int)(value);      
            break;
        case CURRENT_MISSMATCH_TIME:
            configregister = 0x15;
            _modules_data.calibration_current_missmatch_time[address] = value;
            value = (int)(value);     
            break;
        case CURRENT_REGULATION_STEP:
            configregister = 0x16;
            _modules_data.calibration_current_regulation_step[address] = value;
            value = (int)(value); 
            break;
        case CURRENT_DEVIATION:
            configregister = 0x17;
            _modules_data.calibration_current_deviation[address] = value;
            value = (int)(value*1000);       //Volts to mV
            break;
        case CURRENT_MAX:
            configregister = 0x18;
            _modules_data.calibration_current_maximum[address] = value;
            value = (int)(value*1000);       //volts to mV
            break;
        default:
            return false;
            break; 
        }

    uint16_t module_calibration = _readdata(address, configregister);
    if (module_calibration != value) {
        _writedata(address, 0x03, 1);               //enable config mode
        _writedata(address, configregister, value);
        _writedata(address, 0x03, 0);               //disable config mode
        }

    //read back config value, if not the same, return false
    if (_readdata(address, configregister) != value){
        return false;
        }

    return true;
    }

float Cellmodules::getcalibrationdata(configValue config, uint8_t address) {
    //cehck if address is in range 
    if(address > MAX_CELL_MODULES) 
        return false;

    //check if module is there
    if (!_checkModule(address))
        return 0xFFFF;

    //if it is there, populate register and value
    switch (config) {
        case ADDRESS:
            return address;
            break;
        case REFERENCE:
            return _modules_data.calibration_reference[address];
            break;
        case VOLTAGE:
            return _modules_data.calibration_voltage[address];
            break;
        case CURRENT:
            return _modules_data.calibration_current[address];
            break;
        case TEMPERATURE:
            return _modules_data.calibration_temperature[address];
            break;
        case CURRENTCOMPENSATION:
            return _modules_data.calibration_current_compensation[address];
            break;
        case CURRENT_MISSMATCH_TIME:
            return _modules_data.calibration_current_missmatch_time[address];
            break;
        case CURRENT_REGULATION_STEP:
            return _modules_data.calibration_current_regulation_step[address];
            break;
        case CURRENT_DEVIATION:
            return _modules_data.calibration_current_deviation[address];
            break;
        case CURRENT_MAX:
            return _modules_data.calibration_current_maximum[address];
            break;		
        default:
            return 0xFFFF;
            break; 
        }
    
    return false;
    }

bool Cellmodules::setLocate(uint8_t address, bool state){
    //cehck if address is in range 
    if(address > MAX_CELL_MODULES) 
        return false;

    _modules_data.locate_module[address] = state;

    _writedata(address, 0x04, state);   //set locate
     
    //read back locate and return false if not the same
    if(_readdata(address, 0x04) != state)
        return false;

    //return
    return true;
    }

uint16_t Cellmodules::_readdata(int i2cAddress, byte i2cRegister) {
    //first tell the module which data has to be read from it
    _i2c.beginTransmission(i2cAddress);                   //queuing the slave address
    _i2c.write(i2cRegister);                              //queuing the register address/pointing regsiter   
    byte busStatus = _i2c.endTransmission();              //transmit all queued data and bring STOP condition on I2C Bus
    if(busStatus != 0x00) {
        return(0xFFFF);
        }   

    //now get the data containing of 2bytes of data and a CRC byte
    _i2c.requestFrom(i2cAddress, 3);                      //this is looping code; when 2-byte has arrived, the loop terminates
    byte byte1 = _i2c.read();                             //read the first byte from FIFO Buffer, higher
    byte byte2 = _i2c.read();
    uint16_t data = (byte1 << 8) | byte2;                 //16-bit data is formed
    uint8_t checksum = _i2c.read();

    //calculate and check the checksum, return the highest value (0xFFFF) if an error occours
    uint16_t crc = (byte1 + byte2) % 256;
    if (checksum != crc){
        //SDwriteLogFile(filenameLog, "I2C communication CRC error");
        Serial.println("receiving CRC comparison failed: "+String(checksum)+" > "+crc);
        _modules_data.cellcrcerrors[i2cAddress]++;        //add up cell based crc counter, that is deleted after viewing on web interface
        _modules_data.crcerrors++;                        //add up global crc error counter
        return(0xFFFF);
        }

    //return the data if everything is ok
    return(data);                                         
    }