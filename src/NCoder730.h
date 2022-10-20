/*!
 * @file NCoder730.h
 * @brief Define the infrastructure for the NMotion NCoder730 class
 * @n This is a NCoder730 (MA730 sensor breakout board) that supports SPI communication. The functions are as follows:
 * @n Measures absolute angle 
 * @n Configures Registers
 * @copyright Copyright (c) 2022 Nawe Robotics Pvt. Ltd. (https://www.nawerobotics.com/)
 * @licence The MIT License (MIT)
 * @author [jubin mathew]<jubin@nawerobotics.com>
 * @version V1.0
 * @date 2022-10-20
 * @url  https://github.com/JUBIN0407/NCoder
 */
#ifndef _NCoder730_H_
#define _NCoder730_H_

#include <Arduino.h>

#if (ARDUINO >= 100)
     #include "Arduino.h"
#else
     #include "WProgram.h"
#endif
#include <SPI.h>

 //SPI Mode: MA730 IC support SPI mode 3 and 0 [SPI_MODE3, SPI_MODE0]
#define NCoder730_SPI_MODE_0       SPI_MODE0
#define NCoder730_SPI_MODE_3       SPI_MODE3


#define ZERO_SETTING0_REG 0x0
#define ZERO_SETTING1_REG 0x1
#define BCT_REG 0x2
#define TRIMMING_REG 0x3
#define PPT0_REG 0x4
#define ILIP_REG 0x4
#define PPT1_REG 0x5
#define MAG_FIELD_THRESHOLD_REG 0x6
#define ROT_DIR_REG 0x9

class NCoder730 {
public:
    NCoder730();
    void  beginSPI(uint8_t spiChipSelectPin);
    void  beginSPI(int32_t spiSclkFrequency, uint8_t spiMode, uint8_t spiChipSelectPin);
    void setSpiClockFrequency(uint32_t speedMaximum);
    void setSpiDataMode(uint8_t spiMode);
    void setSpiChipSelectPin(uint8_t spiChipSelectPin);
    void setZeroPosition(float angle);
    float getZeroPosition();
    void setPulsePerTurn(uint16_t ppr);
    uint16_t getPulsePerTurn();
    void setRotationDirection(bool dir);
    bool getRotationDirection();
    void endSPI();
    double readAngle();
    uint16_t readAngleRaw();
    uint16_t readAngleRaw(bool* error);
    uint8_t readAngleRaw8();
    double convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle);
private:
    uint32_t _speedMaximum;
    uint8_t _spiMode;
    uint8_t _spiChipSelectPin;
    uint16_t readAngleRaw16();
    uint8_t readRegister(uint8_t address);
    uint8_t writeRegister(uint8_t address, uint8_t value);
};

#endif // _NCoder730_H_
