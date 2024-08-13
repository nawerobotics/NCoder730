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

NCoder730::~NCoder730(){
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

void NCoder730::writeDefaultConfigurations(){
    writeRegister(ZERO_SETTING0_REG, 0x00);
    writeRegister(ZERO_SETTING1_REG, 0x00);
    writeRegister(BCT_REG, 0x00);
    writeRegister(TRIMMING_REG, 0x00);
    writeRegister(PPT0_REG,0xC0);
    writeRegister(PPT1_REG,0xFF);
    writeRegister(ROT_DIR_REG, 0x00);
}

double NCoder730::readAbsoluteAngle(){
  uint16_t angle;
  double angleInDegree;
  angle = readAbsoluteAngleRaw16();
  angleInDegree = (angle*360.0)/65536.0;
  return angleInDegree;
}

uint16_t NCoder730::readAbsoluteAngleRaw(){
    return readAbsoluteAngleRaw16();
}

uint16_t NCoder730::readAbsoluteAngleRaw16(){
    uint16_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = SPI.transfer16(0x0000); //Read 16-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint8_t NCoder730::readAbsoluteAngleRaw8(){
    uint8_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = SPI.transfer(0x00);     //Read 8-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint16_t NCoder730::readAbsoluteAngleRaw(bool* error){
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

void NCoder730::setZeroPosition(float angle){
    uint16_t zero_pos = pow(2,16) * (1 - (angle / 360.0f));
    writeRegister(ZERO_SETTING0_REG,uint8_t(zero_pos));
    writeRegister(ZERO_SETTING1_REG,uint8_t(zero_pos>>8));
}

float NCoder730::getZeroPosition(){
    uint16_t zero_pos = readRegister(ZERO_SETTING1_REG) << 8 | readRegister(ZERO_SETTING0_REG);
    float angle = convertRawAngleToDegree(16 ,pow(2,16) - zero_pos);
    return angle;
}

void NCoder730::setBCTValue(uint8_t bct_value){
    writeRegister(BCT_REG, bct_value);
}

uint8_t NCoder730::getBCTValue(){
    return readRegister(BCT_REG);
}

void NCoder730::setETX(bool val){
    bool temp = getETY();
    writeRegister(TRIMMING_REG, temp << 1 | val);
}

void NCoder730::setETY(bool val){
    bool temp = getETX();
    writeRegister(TRIMMING_REG, val << 1 | temp);
}

bool NCoder730::getETX(){
    return readRegister(TRIMMING_REG) &0x1;
}

bool NCoder730::getETY(){
    return (readRegister(TRIMMING_REG) >> 1) & 0x1;
}

void NCoder730::setPulsePerTurn(uint16_t ppr){
    uint16_t val = ppr - 1;
    uint8_t reg_val = readRegister(PPT0_REG);
    writeRegister(PPT0_REG,uint8_t(uint8_t(val & 0x03) << 6) | (reg_val & 0x3F));
    writeRegister(PPT1_REG,uint8_t(uint8_t(val >> 2)));
}

uint16_t NCoder730::getPulsePerTurn(){
    uint16_t val = (readRegister(PPT1_REG) << 2) | ((readRegister(PPT0_REG) >> 6) & 0x03);
    return (val + 1);
}

void NCoder730::setRotationDirection(bool dir){
    writeRegister(ROT_DIR_REG,uint8_t(dir) << 7);
}

bool NCoder730::getRotationDirection(){
    return readRegister(ROT_DIR_REG);
}

void NCoder730::setMagneticFieldLowThreshold(uint8_t MGLT){
    uint8_t reg_val = readRegister(MAG_FIELD_THRESHOLD_REG);
    uint8_t MGLT_val = MGLT;
    writeRegister(MAG_FIELD_THRESHOLD_REG, uint8_t((reg_val & 0x1F) | (MGLT_val  << 5)));
}

void NCoder730::setMagneticFieldHighThreshold(uint8_t MGHT){
    uint8_t reg_val = readRegister(MAG_FIELD_THRESHOLD_REG);
    uint8_t MGHT_val = MGHT;
    writeRegister(MAG_FIELD_THRESHOLD_REG, uint8_t((reg_val & 0xE3) | (MGHT_val  << 2)));
}

uint8_t NCoder730::getMagneticFieldLowThreshold(){
    return (readRegister(MAG_FIELD_THRESHOLD_REG) >> 5) & 0x07;
}

uint8_t NCoder730::getMagneticFieldHighThreshold(){
    return (readRegister(MAG_FIELD_THRESHOLD_REG) >> 2) & 0x07;
}

bool NCoder730::getMagneticFieldLowLevelStatus(){
    return ((readRegister(MAG_FIELD_LEVEL_REG)>>6) & 0x1);
}

bool NCoder730::getMagneticFieldHighLevelStatus(){
    return ((readRegister(MAG_FIELD_LEVEL_REG)>>7) & 0x1);
}

void NCoder730::setIndexLength(float length){
    uint8_t val = length * 2 - 1;
    if(val < 0 && val > 3)
        val = 0;
    uint8_t reg_val = readRegister(ILIP_REG);
    writeRegister(ILIP_REG, (reg_val  & 0xCF) | (val << 4));
}

float NCoder730::getIndexLength(){
    return (((readRegister(ILIP_REG) >> 4) & 0x3) + 1.0f) * 0.50f;
}

void NCoder730::setIndexPosition(uint8_t position){
    uint8_t val = -1;
    uint8_t reg_val = readRegister(ILIP_REG);
    val = (((reg_val >> 4) & 0x03) + position) &0x03;
    writeRegister(ILIP_REG, (reg_val & 0xF3) | (val << 2 ));
}

uint8_t NCoder730::getIndexPosition(){
    uint8_t reg_val = readRegister(ILIP_REG);
    uint8_t length_reg_val = (reg_val >> 4) & 0x3;
    uint8_t pos_reg_val = (reg_val >> 2) & 0x3;
    if(pos_reg_val < length_reg_val)
        pos_reg_val = pos_reg_val | 0x4;
    return (pos_reg_val - length_reg_val) & 0x3;
}

void NCoder730::setFilterWindow(uint8_t filter_window){
    writeRegister(FW_REG, filter_window);
}

uint8_t NCoder730::getFilterWindow(){
    uint8_t reg_val = readRegister(FW_REG);
    uint8_t filter_window = reg_val;
    return filter_window;
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