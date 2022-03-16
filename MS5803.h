/*
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

  * Library for Pressure Sensors of type MS5803-x of MEAS Switzerland     (www.meas-spec.com).
  * The driver uses I2C mode (sensor's Protocol Select (PS) pin pulled to high). 
  * MS5803-01BA (Barometer Sensor (Altimeter)) was successfully tested by Raig Kaufer.
  * MS5803-14BA (Underwater Pressure Sensor 14 bar) was successfully tested by Robert Katzschmann
  * Other types of MEAS are compatible but not tested 
  * Written by Raig Kaufer, distribute freely!
  * Revised by Robert Katzschmann
 */
#include "mbed.h"
#include <memory>

#ifndef MS5803_H
#define MS5803_H

#define MS5803_RX_DEPTH 3 //
#define MS5803_TX_DEPTH 2 //

// choose your connection here
#define ms5803_addrCL 0x77 //0b1110111  CSB Pin is low
#define ms5803_addrCH 0x76 //0b1110110  CSB Pin is high 

#define ms5803_reset       0x1E // Sensor Reset

#define ms5803_convD1_256  0x40 // Convert D1 OSR 256
#define ms5803_convD1_512  0x42 // Convert D1 OSR  512
#define ms5803_convD1_1024 0x44 // Convert D1 OSR 1024
#define ms5803_convD1_2048 0x46 // Convert D1 OSR 2048
#define ms5803_convD1_4096 0x48 // Convert D1 OSR 2048

#define ms5803_convD1 ms5803_convD1_4096 // choose your sampling rate here

#define ms5803_convD2_256  0x50 // Convert D2 OSR  256
#define ms5803_convD2_512  0x52 // Convert D2 OSR  512
#define ms5803_convD2_1024 0x54 // Convert D2 OSR 1024
#define ms5803_convD2_2048 0x56 // Convert D2 OSR 2048
#define ms5803_convD2_4096 0x58 // Convert D2 OSR 2048

#define ms5803_convD2 ms5803_convD2_4096 // choose your sampling rate here

#define ms5803_ADCread     0x00 // read ADC command
#define ms5803_PROMread    0xA0 // read PROM command base address

class MS5803{
private:
    uint64_t D1, D2, Temp, C[8];
    float T_MS5803, P_MS5803;
    /* Data buffers */
    char ms5803_rx_data[MS5803_RX_DEPTH + 1];
    char ms5803_tx_data[MS5803_TX_DEPTH];

public:
    MS5803 (std::shared_ptr<SPI> _spi, DigitalOut _cs,
            char ms5803_addr = ms5803_addrCH  )
            : spi( _spi ), cs(_cs), device_address( ms5803_addr << 1 ) 
    {
        cs = 0;
    }
    MS5803 (std::shared_ptr<SPI> _spi, std::shared_ptr<BusOut> _csBus, uint8_t _csAddr,
            char ms5803_addr = ms5803_addrCH  )
            : spi( _spi )
            , csBus(_csBus)
            , csAddr(_csAddr)
            , device_address( ms5803_addr << 1 )
            , cs(NC)
    {
    }
    void MS5803Init(void);
    void MS5803Reset(void);
    void MS5803ReadProm(void);
    void MS5803ConvertD1(void);
    void MS5803ConvertD2(void);
    uint64_t MS5803ReadADC(void);
    float MS5803_Pressure (void);
    float MS5803_Temperature (void);
    void Barometer_MS5803(void);


private:
    void activateCS();
    void deactivateCS();
    
    std::shared_ptr<SPI>     spi;
    DigitalOut cs;
    char    device_address;
    std::shared_ptr<BusOut> csBus;
    uint8_t csAddr;

};

inline void MS5803::activateCS()
{
    if(cs.is_connected())
    {
        cs = 0;
    }
    else
    {
        csBus->write(csAddr);
    }
}

inline void MS5803::deactivateCS()
{
    if(cs.is_connected())
    {
        cs = 1;
    }
    else
    {
        csBus->write(0x00);
    }
}

#endif
