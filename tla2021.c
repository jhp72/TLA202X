/*
 * tla2021.c
 *
 *  Created on: 2023.04.17.
 *      Author: jaehong park (smilemacho@gmail.com)
 */

/*
MIT License

Copyright (c) [2023.04.17] [jaehong park]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include PLATFORM_HEADER
#include "rtos_err.h"
#include "os.h"
#include "stack/include/ember-types.h"
#include "stack/include/event.h"
#include "hal/hal.h"
#include "hal/plugin/i2c-driver/i2c-driver.h"
#include "hal/micro/micro.h"
#include "zigbee_common.h"
#include "i2cspm.h"
#include "tla2021.h"
#include "app/framework/include/af.h"


/*
    Slave address:

    GND: 0x48
    VDD: 0x49
    SCL: 0x4B
*/
#define TLA202x_I2CADDR_DEFAULT (0x48 << 1)//TLA202x default i2c address

#define TLA202x_DATA_REG 0x00   //Data Register
#define TLA202x_CONFIG_REG 0x01 //Configuration Register

#define TLA202x_START_CONV_MSK  (0x8000) //When writing, start conversion
#define TLA202x_WRITE_WO_START_CONV (~0x8000) //When just writing without starting conversion
#define TLA202x_PERFORM_CONV_MSK    (0x8000) //When reading, status bit during conversion

/**
 * @brief Options for `readVoltage` to choose the single channel to read
 *
 */
typedef enum {
  TLA202x_CHANNEL_0, //Channel 0
  TLA202x_CHANNEL_1, //Channel 1
  TLA202x_CHANNEL_2, //Channel 2
  TLA202x_CHANNEL_3, //Channel 3
} tla202x_channel_t;

/**
 * @brief
 *
 * Allowed values for `setDataRate`.
 */
typedef enum {
  TLA202x_RATE_128_SPS,  //128 Samples per Second //000:DR = 128 SPS
  TLA202x_RATE_250_SPS,  //250 Samples per Second //001:DR = 250 SPS
  TLA202x_RATE_490_SPS,  //490 Samples per Second //010:DR
  TLA202x_RATE_920_SPS,  //920 Samples per Second //011:DR
  TLA202x_RATE_1600_SPS, //1600 Samples per Second//100:DR
  TLA202x_RATE_2400_SPS, //2400 Samples per Second//101:DR
  TLA202x_RATE_3300_SPS, //3300 Samples per Second//111:DR
} tla202x_rate_t;

/**
 * @brief Options for `setRate`
 *
 */
typedef enum {
  TLA202x_MODE_CONTINUOUS, // Take a new measurement as soon as the previous
                           // measurement is finished
  TLA202x_MODE_ONE_SHOT    // Take a single measurement then go into a low-power
                           // mode
} tla202x_mode_t;

/**
 * @brief Options for `setMux`
 *
 * Selects which inputs will be used for the positive (AINp) and negative (AINn)
 * inputs
 *
 */
typedef enum {
  TLA202x_MUX_AIN0_AIN1 = 0x0, // AINp = AIN 0, AINn = AIN 1
  TLA202x_MUX_AIN0_AIN3, // AINp = AIN 0, AINn = AIN 3
  TLA202x_MUX_AIN1_AIN3, // AINp = AIN 1, AINn = AIN 3
  TLA202x_MUX_AIN2_AIN3, // AINp = AIN 2, AINn = AIN 3
  TLA202x_MUX_AIN0_GND = 0x4,  // AINp = AIN 0, AINn = GND
  TLA202x_MUX_AIN1_GND,  // AINp = AIN 1, AINn = GND
  TLA202x_MUX_AIN2_GND,  // AINp = AIN 2, AINn = GND
  TLA202x_MUX_AIN3_GND,  // AINp = AIN 3, AINn = GND
} tla202x_mux_t;

/**
 * @brief Options for `setRange`
 *
 */
typedef enum {
  TLA202x_RANGE_6_144_V, // Measurements range from +6.144 V to -6.144 V //000:FSR
  TLA202x_RANGE_4_096_V, // Measurements range from +4.096 V to -4.096 V //001:FSR
  TLA202x_RANGE_2_048_V, // Measurements range from +2.048 V to -2.048 V //010:FSR
  TLA202x_RANGE_1_024_V, // Measurements range from +1.024 V to -1.024 V //011:FSR
  TLA202x_RANGE_0_512_V, // Measurements range from +0.512 V to -0.512 V //100:FSR
  TLA202x_RANGE_0_256_V  // Measurements range from +0.256 V to -0.256 V //101:FSR
} tla202x_range_t;

/**
 * @brief Possible states to be returned by `getOperationalState`
 *
 */
typedef enum {
  TLA202x_STATE_NO_READ, // Single-shot read in progress
  TLA202x_STATE_READ,    // Single-shot read available to read or start read
} tla202x_state_t;


uint8_t addr = TLA202x_I2CADDR_DEFAULT;//0x48
uint8_t convReg_ = TLA202x_DATA_REG;//0x00
uint8_t confReg_ = TLA202x_CONFIG_REG;//0x01

/*
    This is default config.

    (0x8583) => 1000 0101 1000 0011
    1000 //OS bit:1, Mux: 000
    0101 //PGA:010, Mode:1
    1000 //DR: 100, RVD:0
    0011 //RVD:0011

    Our custom config.
    OS bit:1, Mux: 100(0ch) //1100
    PGA: 000, Mode: 1       //0001  //TLA2021: 0101 (2.048V fixed)
    DR: 000(128SPS), RVD:0  //0000  //TLA2021: 1000 (1600SPS default)
    RVD:0011                //0011
    1100 0001 0000 0011 => (0xC103) //1100 0101 0000 0011 =>(0xC503)
*/
uint16_t initConf_ = 0xC103; //0xC503;//0xC103; //0x8583;
uint16_t savedConf_ = 0xC103; //0xC503;//0xC103; //0x8583;


tla202x_mode_t currentMode_ = TLA202x_MODE_ONE_SHOT;
tla202x_range_t currentFSR_val_ = TLA202x_RANGE_6_144_V;

union I2C_data {
    uint8_t packet[2];
    uint16_t value;
} data;


/*
    Initializes I2C bus
    returns:
        true - adc responds with correct default conf
        false - otherwise
*/
bool TLA2024_begin();

// begin with slave address setted
bool TLA2024_addrBegin(uint8_t address);

// Resets device to default configuration
int TLA2024_reset();

// Restores device to last config
void TLA2024_restore();

// Get analog reading for current multiplexer setting
int16_t TLA2024_analogRead();

//  Convenient method to get analog reading of a channel compared to GND
int16_t TLA2024_analogReadChannel(tla202x_channel_t channel);

// Sets the Full Scale Range of the ADC
void TLA2024_setFullScaleRange(tla202x_range_t range);

// Set multiplexer configuration
void TLA2024_setMuxConfig(tla202x_mux_t option);


// Continous conversion or single shot
void TLA2024_setOperatingMode(tla202x_mode_t mode);

// Set data rate setting
void TLA2024_setDataRate(tla202x_rate_t rate);

// Convenient method to read actual voltage (in volt) respecting the full scale range
int32_t TLA2024_readVoltage(tla202x_channel_t channel);
//float TLA2024_voltageRead(tla202x_channel_t channel);

uint16_t TLA2024_getCurrentFullRangeVoltage();

//  A generic read from mem_addr.
uint16_t TLA2024_read(uint8_t mem_addr);

/*
    We only write to the configuration register.
    out_data is the 16 bits of conf_regs

    Should always return 2
    Saves data to current_conf
*/
int TLA2024_write(uint16_t data);

// I2C_TransferReturn_TypeDef
// Return codes for read/write transaction error handling
/*
#define I2C_DRIVER_ERR_NONE         0x00
#define I2C_DRIVER_ERR_TIMEOUT      0x01
#define I2C_DRIVER_ERR_ADDR_NAK     0x02
#define I2C_DRIVER_ERR_DATA_NAK     0x03
#define I2C_DRIVER_ERR_ARB_LOST     0x04
#define I2C_DRIVER_ERR_USAGE_FAULT  0x05
#define I2C_DRIVER_ERR_SW_FAULT     0x06
#define I2C_DRIVER_ERR_UNKOWN       0xFF
*/
bool TLA2021Status = false;

char _Buffer[3] = {0,};


bool generalCallReset(void) {
    int32_t ack = -1;
    uint8_t const byte = 0x06;

    //_i2c->lock();
    ack = halI2cWriteBytes(0x00, &byte, 1);
    //_i2c->unlock();

	if (ack != I2C_DRIVER_ERR_NONE) {
		emberAfAppPrintln("Failed to generalCallReset...ack: %d", ack);
    } else {
		emberAfAppPrintln("Succeeded to generalCallReset...ack: %d", ack);
    }

    return (ack == 0);
}

bool generalCallLatch(void) {
    int32_t ack = -1;
    uint8_t const byte = 0x04;

    //_i2c->lock();
    ack = halI2cWriteBytes(0x00, &byte, 1);
    //_i2c->unlock();

	if (ack != I2C_DRIVER_ERR_NONE) {
		emberAfAppPrintln("Failed to generalCallLatch...ack: %d", ack);
    } else {
		emberAfAppPrintln("Succeeded to generalCallLatch...ack: %d", ack);
    }

    return (ack == 0);
}

bool generalCallConvert(void) {
    int32_t ack = -1;
    uint8_t const byte = 0x08;

    //_i2c->lock();
    ack = halI2cWriteBytes(0x00, &byte, 1);
    //_i2c->unlock();

	if (ack != I2C_DRIVER_ERR_NONE) {
		emberAfAppPrintln("Failed to generalCallConvert...ack: %d", ack);
    } else {
		emberAfAppPrintln("Succeeded to generalCallConvert...ack: %d", ack);
    }

    return (ack == 0);
}

void TLA2021_i2cSpmInit(void)
{
	I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;

  	I2CSPM_Init(&i2cInit);
	emberAfAppPrintln("Init I2CSPM_Init() ...");
}

void TLA2021_i2cInit(void) {
    // _address = slave_adr;
	// _freq = freq;
    TLA2021_i2cSpmInit();
}

bool TLA2024_begin() {
    int32_t ack = -1;
    if(TLA2024_reset()){
        ack = 0;
        TLA2021Status = true;
		emberAfAppPrintln("Succeeded in TLA2024_reset...ack: %d", ack);

    }else{//if 0, fails
        ack = -1;
        TLA2021Status = false;
		emberAfAppPrintln("Failed in TLA2024_reset...ack: %d", ack);
        return false;
    }

    halCommonDelayMilliseconds(10);

    uint16_t init = TLA2024_read(confReg_);
    emberAfAppPrintln("Done init. 0x%X", init);

    // make sure communication with device is working and that it is OK
    return (init == initConf_) ? true : false;
}

bool TLA2024_addrBegin(uint8_t address){
    addr = address;
    return TLA2024_begin();
}

uint32_t TLA2024_rateDelayTime(tla202x_rate_t rate){
  switch (rate) {//TLA202x_RATE_128_SPS
    case TLA202x_RATE_128_SPS:
        return 1000/128; // 128 SamplesPerSecond
    case TLA202x_RATE_250_SPS:
        return 1000/250; // 250 SPS
    case TLA202x_RATE_490_SPS:
        return 1000/490; // 490 SPS
    case TLA202x_RATE_920_SPS:
        return 1000/920; // 920 SPS
    case TLA202x_RATE_1600_SPS:
        return 1000/1600; // 1600 SPS
    case TLA202x_RATE_2400_SPS:
        return 1000/2400; // 2400 SPS
    case TLA202x_RATE_3300_SPS:
        return 1000/3300; // 3300 SPS
  }
  return 0; // Shouldn't happen
}

/*
#include <Wire.h>

void halI2cWriteBytes(uint8_t addr, uint8_t *data, uint16_t len) {
    Wire.beginTransmission(addr);
    for (uint16_t i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    Wire.endTransmission();
}

void halI2cReadBytes(uint8_t addr, uint8_t *data, uint16_t len) {
    Wire.beginTransmission(addr);
    Wire.write(data[0]);
    Wire.endTransmission();
    
    Wire.requestFrom(addr, len);
    for (uint16_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
}
*/

uint16_t TLA2024_read(uint8_t mem_addr) {
    int32_t ack = -1;

    memset(_Buffer, 0, sizeof(_Buffer));
    _Buffer[0] = mem_addr;
    //_Buffer[1] = data.packet[1];
    //_Buffer[2] = data.packet[0];
    ack = halI2cWriteBytes(addr, _Buffer, 1);
    if (ack != I2C_DRIVER_ERR_NONE) {
    	TLA2021Status = false;
		//emberAfAppPrintln("Failed in halI2cWriteBytes...ack: %d", ack);
    } else {
    	TLA2021Status = true;
		//emberAfAppPrintln("Succeeded in halI2cWriteBytes...ack: %d", ack);
    }

    halCommonDelayMilliseconds(5);
    //halCommonDelayMilliseconds(TLA2024_rateDelayTime(TLA202x_RATE_128_SPS));

    memset(_Buffer, 0, sizeof(_Buffer));
    ack = halI2cReadBytes(addr, _Buffer, 2);
    if (ack != I2C_DRIVER_ERR_NONE) {
    	TLA2021Status = false;
		//emberAfAppPrintln("Failed in halI2cReadBytes...ack: %d", ack);
    } else {
    	TLA2021Status = true;
		//emberAfAppPrintln("Succeeded in halI2cReadBytes...ack: %d", ack);
    }

    if(ack == 0){
      data.packet[1] = _Buffer[0];
      data.packet[0] = _Buffer[1];
#if (1)
      uint16_t ret = data.value;
      data.value = 0;
#else
      uint16_t ret = data.packet[1] << 8;
      ret |= data.packet[0];
#endif
      emberAfAppPrintln("Succeeded in TLA2024_read...ack: %d", ack);
      return ret;
    }

    emberAfAppPrintln("Failed in TLA2024_read...ack: %d", ack);
    return 0;
}

int TLA2024_write(uint16_t out_data) {
    int32_t ack = -1;
    int written = 0;
    
    // save conf
    savedConf_ = out_data;    

#if (1)
    data.value = out_data;
#else// put our out_data into the I2C data union so we can send MSB and LSB
    data.packet[1] = (out_data & 0xFF00) >> 8;
    data.packet[0] = out_data & 0x00FF;
#endif

    memset(_Buffer, 0, sizeof(_Buffer));
    _Buffer[0] = confReg_;
    _Buffer[1] = data.packet[1];
    _Buffer[2] = data.packet[0];
    ack = halI2cWriteBytes(addr, _Buffer, 3);
  
    if (ack != I2C_DRIVER_ERR_NONE) {
    	TLA2021Status = false;
		emberAfAppPrintln("Failed in TLA2024_write...ack: %d", ack);
    } else {
    	TLA2021Status = true;
		emberAfAppPrintln("Succeeded in TLA2024_write...ack: %d", ack);
        written = 2;
    }

    return written;
}

int TLA2024_reset() {
    return TLA2024_write(initConf_);
}

void TLA2024_restore() {
    uint16_t restore_conf = savedConf_ & ~0x8000; //
    TLA2024_write(restore_conf);
    emberAfAppPrintln("restore_conf: %d", restore_conf);
}

int16_t TLA2024_analogRead() {
    // this only needs to run when in single shot.
    if (currentMode_ == TLA202x_MODE_ONE_SHOT) {
        
        uint16_t current_conf = TLA2024_read(confReg_);

        // write 1 to OS bit to start conv
        current_conf |= TLA202x_START_CONV_MSK;//0x8000
        
        TLA2024_write(current_conf);

        // OS bit will be 0 until conv is done.
        do {
            halCommonDelayMilliseconds(5);
        } while ((TLA2024_read(confReg_) & TLA202x_PERFORM_CONV_MSK) == 0);//0x8000
    }

    // get data from conv_reg
    uint16_t in_data = TLA2024_read(convReg_);

    // shift out unused bits
    in_data >>= 4;

    // get sign and mask accordingly
    if (in_data & (1 << 11)) {
        // 11th bit is sign bit. if its set, set bits 15-12
        in_data |= 0xF000;
    } else {
        // not set, clear bits 15-12
        in_data &= ~0xF000;
    }

    // now store it as a signed 16 bit int.
    int16_t ret = in_data;

    // default Full Scale Range is -2.048V to 2.047V.
    // our 12bit 2's complement goes from -2048 to 2047
    // return ret /1000.0;

    // return raw adc data
    return ret;
}

int16_t TLA2024_analogReadChannel(tla202x_channel_t channel) {
    tla202x_mux_t muxCfg = (tla202x_mux_t)(channel + 4);
    TLA2024_setMuxConfig(muxCfg);
    return TLA2024_analogRead();
}

void TLA2024_setFullScaleRange(tla202x_range_t range) {
    currentFSR_val_ = range;

    // bring in conf reg
    uint16_t conf = TLA2024_read(confReg_);

    // clear the PGA bits:
    conf &= ~0x0E00;

    // shift
    conf |= range << 9;

    TLA2024_write(conf);
}

/*
void TLA2024_setFullScaleRange(tla202x_range_t range) {
    // bring in conf reg
    uint16_t conf = TLA2024_read(confReg_);
    // clear the PGA bits:
    conf &= ~0x0E00;

    switch (range) {
        case TLA202x_RANGE_6_144_V:
            conf |= 0x0000;
            break;

        case TLA202x_RANGE_4_096_V:
            conf |= 0x0200;
            break;

        case TLA202x_RANGE_2_048_V:
            conf |= 0x0400;
            break;

        case TLA202x_RANGE_1_024_V:
            conf |= 0x0600;
            break;

        case TLA202x_RANGE_0_512_V:
            conf |= 0x0800;
            break;

        case TLA202x_RANGE_0_256_V:
            conf |= 0x0A00;
            break;
    }
    TLA2024_write(conf);
}
*/

void TLA2024_setMuxConfig(tla202x_mux_t option) {
    // bring in conf reg
    uint16_t conf = TLA2024_read(confReg_);

    // clear MUX bits
    conf &= ~0x7000;

    // shift
    conf |= option << 12;
    TLA2024_write(conf);
}

/*
    Configures the input signals to the ADC

    option:
      1 -> P = 0, N = 1 (TP1 - TP2)
      2 -> P = 0, N = 3 (TP1 - VGND)
      3 -> P = 1, N = 3 (TP2 - VGND)
      4 -> P = 0, N = GND (channel: 0)
      5 -> P = 1, N = GND (channel: 1)
      6 -> P = 2, N = GND (channel: 2)
      7 -> P = 3, N = GND (channel: 3)

void TLA2024_setMuxConfig(uint8_t ch) {
    // bring in conf reg
    uint16_t conf = TLA2024_read(confReg_);
    // clear MUX bits
    conf &= ~0x7000;

    switch (ch) {
        case 0:
            conf |= 0x4000;
            break;

        case 1:
            conf |= 0x5000;
            break;

        case 2:
            conf |= 0x6000;
            break;

        case 3:
            conf |= 0x7000;
            break;

        default:
            break;
    }
    TLA2024_write(conf);
}
*/

void TLA2024_setOperatingMode(tla202x_mode_t mode) {
    // bring in conf reg
    currentMode_ = mode;
    uint16_t conf = TLA2024_read(confReg_);

    // clear MODE bit (8) (continous conversion)
    conf &= ~(1 << 8);
    if (mode == TLA202x_MODE_ONE_SHOT) {
        // single shot conversion
        conf |= (1 << 8);
    }

    TLA2024_write(conf);
}

void TLA2024_setDataRate(tla202x_rate_t rate) {
    // bring in conf reg
    uint16_t conf = TLA2024_read(confReg_);

    // set bits 7:5
    conf |= rate << 5;

    TLA2024_write(conf);
}

uint16_t TLA2024_getCurrentFullRangeVoltage() {
    uint16_t shifted = 8192 >> currentFSR_val_;
    
    // Special case
    if (currentFSR_val_ == 0) {
        shifted = 6144;
    }

    emberAfAppPrintln("shifted fsr Voltage: %dmV",shifted);// unit: mV
    return shifted;// unit: mV
}

/*
 * adc_voltage = (Vdd * adc_data) / pga / num_codes:
 * :=> adc_voltage = (FSR * adc_data) / pga / Resolution
 * :=> 1 LSB voltage = (FSR * 1 data) / pga / Resolution
 * 
 * reference datasheet:
 *    => https://drive.google.com/file/d/1WDigXUslDeJ42uHhQnPQ7amxNcIcR4Nl/view
 *       8.3.3 Full-Scale Range (FSR) and LSB size
 * 
    +--------------------------------+-------------------+---------------------------+
    | enum           | FSR               | LSB size   | Res(fixed)      | pga(fixed???)
    +================================+===================+===========================+
    | `RANGE_6_144V` | ±6.144 V          | 3 mV       | 4096 (2^12bit)  | 0.5 (1/2) 
    +--------------------------------+-------------------+---------------------------+
    | `RANGE_4_096V` | ±4.096 V          | 2 mV       | 4096            | 0.5
    +--------------------------------+-------------------+---------------------------+
    | `RANGE_2_048V` | ±2.048 V          | 1 mV       | 4096            | 0.5
    +--------------------------------+-------------------+---------------------------+
    | `RANGE_1_024V` | ±1.024 V          | 0.5 mV     | 4096            | 0.5
    +--------------------------------+-------------------+---------------------------+
    | `RANGE_0_512V` | ±0.512 V          | 0.25 mV    | 4096            | 0.5
    +--------------------------------+-------------------+---------------------------+
    | `RANGE_0_256V` | ±0.256 V          | 0.125 mV   | 4096            | 0.5
    ---------------------------------------------------------------------------------+

    Range.values(
    (
        ("RANGE_6_144V", 0x0, 6.144, 3),
        ("RANGE_4_096V", 0x1, 4.096, 2),
        ("RANGE_2_048V", 0x2, 2.048, 1),
        ("RANGE_1_024V", 0x3, 1.024, 0.5),
        ("RANGE_0_512V", 0x4, 0.512, 0.25),
        ("RANGE_0_256V", 0x5, 0.256, 0.125),
    )
)
*/
int32_t TLA2024_readVoltage(tla202x_channel_t channel) {
    int32_t voltage;//mV
    int16_t rawVal = TLA2024_analogReadChannel(channel);
    uint16_t fsrV = TLA2024_getCurrentFullRangeVoltage();// unit: V

    switch (currentFSR_val_) {
        case TLA202x_RANGE_6_144_V:
            voltage = (int32_t)(rawVal * 3);
            break;
        case TLA202x_RANGE_4_096_V:
            voltage = (int32_t)(rawVal * 2);
            break;
        case TLA202x_RANGE_2_048_V:
            voltage = (int32_t)(rawVal * 1);
            break;
        case TLA202x_RANGE_1_024_V:
            voltage = (int32_t)(rawVal * 0.5);
            break;
        case TLA202x_RANGE_0_512_V:
            voltage = (int32_t)(rawVal * 0.25);
            break;
        case TLA202x_RANGE_0_256_V:
            voltage = (int32_t)(rawVal * 0.125);
            break;
    }
    //long converted = (long)(voltage / 1000);// unit: V
    emberAfAppPrintln("fsrV: %ld mV, ADC CH%d data: %ld,  voltage: %ld mV", fsrV, channel, rawVal, voltage);
    int32_t convF = voltage;

    return convF;
}

/*
 * adc_voltage = (Vdd * adc_data) / pga / num_codes: 
 * :=> adc_voltage = converted, Vdd = 6144mV, adc_data = rawVal, pga = 0.5(gain), num_codes = 4096 (2^12) or 256 (2^8).
 * 
float TLA2024_voltageRead(tla202x_channel_t channel) {
    float rawVal = TLA2024_analogReadChannel(channel);
    float fsrV = 6144/1000; // default Voltage(V)  
 
    //adc_voltage = (Vdd * adc_data) / num_codes:
    //=> adc_voltage = converted, Vdd = 6144mV, adc_data = rawVal, num_codes = 4096 (2^12) or 256 (2^8). 
    long converted = (6144 * rawVal) / 4096;
    //long converted = map((long)rawVal, -6147, 6147, 0, fsrV*1000 );
    //long converted = map((long)rawVal, 0, 6147, 0, fsrV*1000 );
 
    emberAfAppPrintln("fsrV: %.4fV, raw ADC[%d] data: %lu,  converted: %lu", fsrV, channel, rawVal, converted);
    float convF = converted * 1.0f / 1000;
    return convF;
}
*/

bool TLA2021_initConfig(void) {
    int32_t ack = -1;

    if (TLA2024_begin()) {
        emberAfAppPrintln("Device is inited");
        ack = 0;
    }
    else {
        emberAfAppPrintln("Device is not inited. Continue anyway...");
        ack = 1;
    }

    TLA2024_setDataRate(TLA202x_RATE_128_SPS);
    TLA2024_setOperatingMode(TLA202x_MODE_ONE_SHOT);
    TLA2024_setMuxConfig(TLA202x_MUX_AIN0_GND);
    TLA2024_setFullScaleRange(TLA202x_RANGE_6_144_V);

    return (ack == 0);
}

/*
int main() {

	TLA2021_i2cInit();

    if(TLA2021_initConfig()){
        while (1) {
            emberAfAppPrintln("ADC0 voltage: %.3fV", TLA2024_readVoltage(0)); // read channel 0
            //emberAfAppPrintln("ADC0 value: %.3f", TLA2024_analogReadChannel(0));
            halCommonDelayMilliseconds(500);//500ms
        }
    }

    emberAfAppPrintln("failed to init...");
}
*/

void tla2021_init(void)
{
	TLA2021_i2cInit();
	halCommonDelayMilliseconds(100);//OSTimeDly(500, OS_OPT_TIME_DLY, &err);

	if(TLA2021_initConfig()) {
		emberAfAppPrintln("Succeeded in tla2024_init()...");
		return;
	}
	emberAfAppPrintln("Failed in tla2024_init()...");
}

uint32_t voltage2Current(uint32_t voltage) 
{
    uint32_t current = 0;
/*
    switch(voltage)
    {
        case 0://0
            break;

        case 0.5://0.516V
            break;

        case 1.0://1.006
            break;

        case 1.5://1.518
            break;

        case 2.0://2.028
            break;

        case 2.5://2.504
            break;

        case 3.0://3.009
            break;

        case 3.5://3.528
            break;

        case 4.0://4.04
            break;

        case 4.5://4.5
            break;

        case 5.0://4.98V
            break;
    }
*/

/*
    if(voltage < 590) {//516mV
        current = 4000;//3.86mA
    }else if((voltage >= 500) && (voltage < 1000)) {//1006mV
        current = 5480;//5.48mA
    }else if((voltage >= 1000) && (voltage < 1500)) {//1518mV
        current = 7010;//7.01
    }else if((voltage >= 1500) && (voltage < 2000)) {//2028mV
        current = 8590;//8.59
    }else if((voltage >= 2000) && (voltage < 2500)) {//2504mV
        current = 10150;//10.15
    }else if((voltage >= 2500) && (voltage < 3000)) {//3009mV
        current = 11590;//11.59
    }else if((voltage >= 3000) && (voltage < 3500)) {//3528mV
        current = 13100;//13.10
    }else if((voltage >= 3500) && (voltage < 4000)) {//4040mV
        current = 14650;//14.65
    }else if((voltage >= 4000) && (voltage < 4500)) {//4500mV
        current = 16160;//16.16
    }else if((voltage >= 4500) && (voltage < 5000)) {//4980mV
        current = 17490;//17.49
    }else if((voltage >= 5000)) {
        current = 20000;//18.91mA
    }
*/

    current = 3200 * voltage + 4000;
    if(voltage < 610){//jhp
        current = 4000;//uA
    }else if(voltage >= 5000){
        current = 20000;//uA
    }

    return current;//unit: uA
}


void tla2021_process(void)
{
    RTOS_ERR    err;
    uint32_t ch1_voltage = 0;//Unit: mV
    uint32_t ch2_voltage = 0;
    uint32_t ch3_voltage = 0;
    uint32_t ch4_voltage = 0;

    uint32_t ch1_current= 0;//Unit: uA
    uint32_t ch2_current= 0;
    uint32_t ch3_current= 0;
    uint32_t ch4_current= 0;

    emberAfAppPrintln("Invoked tla2024_process()...");

	if(!TLA2021Status){
#if (1)
        tla2021_init();
#else//test code
        {
            if(TLA2024_begin()) {
                emberAfAppPrintln("Succeeded in tla2021_init()...");
            }else{
                emberAfAppPrintln("Failed in tla2021_init()...");
            }
        }
#endif
        return;
    }

	while (1) {
        ch1_voltage = (uint32_t) TLA2024_readVoltage(TLA202x_CHANNEL_0);
		emberAfAppPrintln("ADC0 voltage: %ld mV", ch1_voltage);
        //emberAfAppPrintln("ADC0 voltage: %ld mV", TLA2024_readVoltage(TLA202x_CHANNEL_0));// read voltage at channel 0
        //emberAfAppPrintln("ADC0 value: %.3f", TLA2024_analogReadChannel(0));// read adc value at channel 0
        OSTimeDly(500, OS_OPT_TIME_DLY, &err);

        ch2_voltage = (uint32_t) TLA2024_readVoltage(TLA202x_CHANNEL_1);
		emberAfAppPrintln("ADC1 voltage: %ld mV", ch2_voltage);
        //emberAfAppPrintln("ADC1 value: %.3f", TLA2024_analogReadChannel(1));
        OSTimeDly(500, OS_OPT_TIME_DLY, &err);
    
        ch3_voltage = (uint32_t) TLA2024_readVoltage(TLA202x_CHANNEL_2);
		emberAfAppPrintln("ADC2 voltage: %ld mV", ch3_voltage);
        //emberAfAppPrintln("ADC2 value: %.3f", TLA2024_analogReadChannel(2));
        OSTimeDly(500, OS_OPT_TIME_DLY, &err);

        ch4_voltage = (uint32_t) TLA2024_readVoltage(TLA202x_CHANNEL_3);
		emberAfAppPrintln("ADC3 voltage: %ld mV", ch4_voltage);
        //emberAfAppPrintln("ADC3 value: %.3f", TLA2024_analogReadChannel(3);
        OSTimeDly(500, OS_OPT_TIME_DLY, &err);
    	
		break;
	}

	/*
	switch(channel)
	{
		case 0:
			break;
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;

		default :
			break;
	}
	*/

    /*
     * apply average filter to voltage values accoding to each channel.
     *
    */

    /*
     * convert using "voltage to current" table.
     *   
    */
   ch1_current = voltage2Current(ch1_voltage);
   ch2_current = voltage2Current(ch2_voltage);
   ch3_current = voltage2Current(ch3_voltage);
   ch4_current = voltage2Current(ch4_voltage);

   emberAfAppPrintln("CH1: %ldmV/%lduA \tCH2: %ldmV/%lduA \tCH3: %ldmV/%lduA \tCH4: %ldmV/%lduA", ch1_voltage, ch1_current, ch2_voltage, ch2_current, ch3_voltage, ch3_current, ch4_voltage, ch4_current);
   
    /*
    * fill in the g_adc_info_t.adc.ch1 and g_adc_info_t.adc.ch2. 
    *
    */
    // g_adc_info_t.adc.ch1 = TLA2024_readVoltage(0);
    // g_adc_info_t.adc.ch2 = TLA2024_readVoltage(1);
    // g_adc_info_t.adc.ch3 = TLA2024_readVoltage(2);
    // g_adc_info_t.adc.ch4 = TLA2024_readVoltage(3);

    //mutexLock();
    g_adc_info_t.adc.ch1 = ch1_current;//uA
    g_adc_info_t.adc.ch2 = ch2_current;
    g_adc_info_t.adc.ch3 = ch3_current;
    g_adc_info_t.adc.ch4 = ch4_current;
    //mutexUnlock();

    // g_adc_info_t.adc.ch1 = 5;
    // g_adc_info_t.adc.ch2 = 14;
    // g_adc_info_t.adc.ch3 = 0;
    // g_adc_info_t.adc.ch4 = 0;
}


void cli_dev_tla2024_rst(void)
{
	emberAfAppPrintln("Reset tla2024....");
}


