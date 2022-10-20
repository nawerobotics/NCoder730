/*!
 * @file NCoder730.cpp
 * @brief Implementation of NCoder730 class
 * @copyright Copyright (c) 2022 Nawe Robotics Pvt. Ltd (https://www.nawerobotics.com/)
 * @licence The MIT License (MIT)
 * @author [jubin_mathew]<jubin@nawerobotics.com>
 * @version V1.0
 * @date 2022-10-20
 * @url  https://github.com/JUBIN0407/NCoder
 */
#include "NCoder730.h"
#include <math.h>

//NCoder730 Read/Write Register Command
#define READ_REG_COMMAND    (0b010 << 13)
#define WRITE_REG_COMMAND   (0b100 << 13)

#define SSI_MODE            SPI_MODE1

NCoder730::NCoder730(){
}

void  NCoder730::beginSPI(uint8_t spiChipSelectPin){
    setSpiChipSelectPin(spiChipSelectPin);
    _speedMaximum = 10000000;
    _spiMode = NCoder730_SPI_MODE_3;
    SPI.begin();
    SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, _spiMode));
}

void  NCoder730::beginSPI(int32_t spiSclkFrequency, uint8_t spiMode, uint8_t spiChipSelectPin){
    setSpiChipSelectPin(spiChipSelectPin);
    _speedMaximum = spiSclkFrequency;
    _spiMode = spiMode;
    SPI.begin();
    SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, _spiMode));
}

void NCoder730::setSpiClockFrequency(uint32_t speedMaximum){
    _speedMaximum = speedMaximum;
    SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, _spiMode));
}

void NCoder730::setSpiDataMode(uint8_t spiMode){
    _spiMode = spiMode;
    SPI.setDataMode(_spiMode);
}

void NCoder730::setSpiChipSelectPin(uint8_t spiChipSelectPin){
    _spiChipSelectPin = spiChipSelectPin;
    pinMode(_spiChipSelectPin, OUTPUT);
    digitalWrite(_spiChipSelectPin, HIGH);
}

double NCoder730::convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle){
    double angleInDegree;
    angleInDegree = (rawAngle*360.0)/((double)pow(2, rawAngleDataBitLength));
    return angleInDegree;
}

void NCoder730::endSPI(){
    SPI.end();
}

double NCoder730::readAngle(){
  uint16_t angle;
  double angleInDegree;
  angle = readAngleRaw16();
  angleInDegree = (angle*360.0)/65536.0;
  return angleInDegree;
}

uint16_t NCoder730::readAngleRaw(){
    return readAngleRaw16();
}

uint16_t NCoder730::readAngleRaw16(){
    uint16_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = SPI.transfer16(0x0000); //Read 16-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint8_t NCoder730::readAngleRaw8(){
    uint8_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = SPI.transfer(0x00);     //Read 8-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint16_t NCoder730::readAngleRaw(bool* error){
    uint16_t angle;
    uint8_t parity;
    uint8_t highStateCount = 0;

    digitalWrite(_spiChipSelectPin, LOW);
    angle = SPI.transfer16(0x0000);
    parity = SPI.transfer(0x00);
    digitalWrite(_spiChipSelectPin, HIGH);

    parity = ((parity & 0x80) >> 7);
    //Count the number of 1 in the angle binary value
    for (int i=0;i<16;++i){
        if ((angle & (1 << i)) != 0){
            highStateCount++;
        }
    }
    //check if parity bit is correct
    if ((highStateCount % 2) == 0){
        if (parity == 0){
            *error = false;
        }
        else{
            *error = true;
        }
    }
    else{
        if (parity == 1){
            *error = false;
        }
        else{
            *error = true;
        }
    }
    return angle;
}

uint8_t NCoder730::readRegister(uint8_t address){
  uint8_t readbackRegisterValue;
  digitalWrite(_spiChipSelectPin, LOW);
  SPI.transfer16(READ_REG_COMMAND | ((address & 0x1F) << 8) | 0x00);
  digitalWrite(_spiChipSelectPin, HIGH);
  delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns before register readout
  digitalWrite(_spiChipSelectPin, LOW);
  readbackRegisterValue = ((SPI.transfer16(0x0000) & 0xFF00) >> 8);
  digitalWrite(_spiChipSelectPin, HIGH);
  delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns after register readout
  return readbackRegisterValue;
}

uint8_t NCoder730::writeRegister(uint8_t address, uint8_t value){
  uint8_t readbackRegisterValue;
  digitalWrite(_spiChipSelectPin, LOW);
  SPI.transfer16(WRITE_REG_COMMAND | ((address & 0x1F) << 8) | value);
  digitalWrite(_spiChipSelectPin, HIGH);
  delay(20);                      //Wait for 20ms
  digitalWrite(_spiChipSelectPin, LOW);
  readbackRegisterValue = ((SPI.transfer16(0x0000) & 0xFF00) >> 8);
  digitalWrite(_spiChipSelectPin, HIGH);
  //readbackRegisterValue should be equal to the written value
  return readbackRegisterValue;
}
