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
  * MS5803-01BA was successfully tested by Raig Kaufer.
  * MS5803-14BA (Underwater Pressure Sensor 14 bar) was successfully tested by Robert Katzschmann
  * Other types of MEAS are compatible but not tested
  * Written by Raig Kaufer, distribute freely!
  * Revised by Robert Katzschmann
 */

#include <cstdint>
#include <stdlib.h>
#include "MS5803.h"


/*
 * Sensor operating function according data sheet
 */

void MS5803::MS5803Init(void)
{
    MS5803Reset();
    MS5803ReadProm();
    return;
}

/* Send soft reset to the sensor */
void MS5803::MS5803Reset(void)
{
    /* transmit out 1 byte reset command */
    ms5803_tx_data[0] = ms5803_reset;
    cs = 1;
    // wait_us(1000);
    if ( spi->write( ms5803_tx_data, 1, ms5803_rx_data, 0 ) );
    cs = 0;
    // wait_us(1000);
    //printf("send soft reset");
    wait_us(20000);
}

/* read the sensor calibration data from rom */
void MS5803::MS5803ReadProm(void)
{
    uint8_t i,j;
    for (i=0; i<8; i++) {
        j = i;
        ms5803_tx_data[0] = ms5803_PROMread + (j<<1);
        cs = 1;
        // wait_us(1000);
        if ( spi->write( ms5803_tx_data, 1, ms5803_rx_data, 3 ) );
        cs = 0;
        // wait_us(1000);
        C[i]   = ms5803_rx_data[2] + (ms5803_rx_data[1]<<8);
    }
}

/* Start the sensor pressure conversion */
void MS5803::MS5803ConvertD1(void)
{
    ms5803_tx_data[0] = ms5803_convD1;
    cs = 1;
    // wait_us(1000);
    if ( spi->write( ms5803_tx_data, 1, ms5803_rx_data, 0 ) );
    cs = 0;
    // wait_us(1000);
}

/* Start the sensor temperature conversion */
void MS5803:: MS5803ConvertD2(void)
{
    ms5803_tx_data[0] = ms5803_convD2;
    cs = 1;
    // wait_us(1000);
    if ( spi->write( ms5803_tx_data, 1, ms5803_rx_data, 0 ) );
    cs = 0;
    // wait_us(1000);
}

/* Read the previous started conversion results */
uint64_t MS5803::MS5803ReadADC(void)
{
    int32_t adc;
    wait_us(150000);
    ms5803_tx_data[0] = ms5803_ADCread;
    cs = 1;
    // wait_us(1000);
    if ( spi->write( ms5803_tx_data, 1, ms5803_rx_data, 4 ) );
    cs = 0;
    // wait_us(1000);
    adc = ms5803_rx_data[3] + (ms5803_rx_data[2]<<8) + (ms5803_rx_data[1]<<16);
    return (adc);
}

/* return the results */
float MS5803::MS5803_Pressure (void)
{
    return P_MS5803;
}
float MS5803::MS5803_Temperature (void)
{
    return T_MS5803;
}

/* Sensor reading and calculation procedure */
void MS5803::Barometer_MS5803(void)
{
    uint64_t dT, temp;
    uint64_t OFF, SENS, press;

    //no need to do this everytime!
    // MS5803Reset();                 // reset the sensor
    // MS5803ReadProm();             // read the calibration values
    
    
    MS5803ConvertD1();             // start pressure conversion
    D1 = MS5803ReadADC();        // read the pressure value
    MS5803ConvertD2();             // start temperature conversion
    D2 = MS5803ReadADC();         // read the temperature value

    /* calculation according MS5803-01BA data sheet DA5803-01BA_006 */
    dT       = D2 - (C[5]* 256);
    // OFF      = (int64_t)C[2] * (1<<18) + ((int64_t)dT * (int64_t)C[4]) / (1<<5);
    // SENS     = (int64_t)C[1] * (1<<17) + ((int64_t)dT * (int64_t)C[3]) / (1<<6);
    // OFF      = (int64_t)C[2] * (1<<16) + ((int64_t)dT * (int64_t)C[4]) / (1<<7);
    // SENS     = (int64_t)C[1] * (1<<15) + ((int64_t)dT * (int64_t)C[3]) / (1<<8);
    OFF = (int64_t)C[2] * (pow(2, 17)) + ((int64_t)C[4] * (int64_t)dT) / pow(2, 6);
    SENS = (int64_t)C[1] * (pow(2, 16)) + ((int64_t)C[3] * (int64_t)dT) / pow(2, 7);

    temp     = 2000 + (dT * C[6]) / (1<<23);
    T_MS5803 = (float) temp / 100.0f;                 // result of temperature in deg C in this var
    press    = (((int64_t)D1 * SENS) / (1<<21) - OFF) / (1<<15);
    P_MS5803 = (float) press / 100.0f;                 // result of pressure in mBar in this var

    // printf("C0: %llu\r\n", C[0]);
    // printf("C1: %llu\r\n", C[1]);
    // printf("C2: %llu\r\n", C[2]);
    // printf("C3: %llu\r\n", C[3]);
    // printf("C4: %llu\r\n", C[4]);
    // printf("C5: %llu\r\n", C[5]);
    // printf("C6: %llu\r\n", C[6]);
    // printf("D1: %llu\r\n", D1);
    // printf("D2: %llu\r\n", D2);
    // printf("dT: %llu\r\n", dT);
    // printf("OFF: %lli\r\n", OFF);
    // printf("SENS: %lli\r\n", SENS);
    // printf("temp: %llu\r\n", temp);
    // printf("press: %lli\r\n", press);
}
