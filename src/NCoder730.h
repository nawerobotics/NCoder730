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

// Registers of MA730
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
private:
    /**
     * @brief variable 
     * 
     */
    uint32_t _speedMaximum;
    uint8_t _spiMode;
    uint8_t _spiChipSelectPin;
    /**
     * @brief Reads the 16 bit Absolute Raw Angle Value
     * 
     * @return returns 16 bit raw absolute angle value 
     */
    uint16_t readAbsoluteAngleRaw16();
    /**
     * @brief Reads the SPI registers of NCoder730
     * 
     * @param address address whose value is to be read
     * @return returns the value written to the address
     */
    uint8_t readRegister(uint8_t address);
    /**
     * @brief Writes the SPI registors of NCoder730
     * 
     * @param address address into which value is to be written
     * @param value value that is to be written
     * @return returns the value in the registor
     */
    uint8_t writeRegister(uint8_t address, uint8_t value);

public:
    /**
     * @brief Construct a new NCoder730 object
     * 
     */
    NCoder730();
    /**
     * @brief Destroy the NCoder730 object
     * 
     */
    ~NCoder730();
    /**
     * @brief Initialization function for SPI Interface
     * 
     * @param spiChipSelectPin Chip Select pin for SPI Communication
     */
    void  beginSPI(uint8_t spiChipSelectPin);
    /**
     * @brief Initialization function for SPI Communication with SPI Settings
     * 
     * @param spiSclkFrequency SPI Clock Frequency for NCoder730
     * @param spiMode  SPI Mode for NCoder730
     * @param spiChipSelectPin SPI Chip Select Pin
     */
    void  beginSPI(int32_t spiSclkFrequency, uint8_t spiMode, uint8_t spiChipSelectPin);
    /**
     * @brief Set the Spi Clock Frequency
     * 
     * @param speedMaximum Clock frequency for SPI Communication
     */
    void setSpiClockFrequency(uint32_t speedMaximum);
    /**
     * @brief Set the Spi Data Mode for SPI Communication
     * 
     * @param spiMode Mode for SPI Communication
     * @n MA730 IC support SPI mode 3 and 0 [SPI_MODE3, SPI_MODE0]
     * @n NCoder730_SPI_MODE_0
     * @n NCoder730_SPI_MODE_3
     */
    void setSpiDataMode(uint8_t spiMode);
    /**
     * @brief Set the Spi Chip Select Pin for SPI Communication
     * 
     * @param spiChipSelectPin Digital Pin for Chip Select
     */
    void setSpiChipSelectPin(uint8_t spiChipSelectPin);
    /**
     * @brief Function to end SPI Transaction/Communication
     * 
     */
    void endSPI();
    /**
     * @brief Function to write default configurations to the registors
     * 
     */
    void writeDefaultConfigurations();
    /**
     * @brief Reads absolute encoder angle
     * 
     * @return return absolute_angle in degrees 
     */
    double readAbsoluteAngle();
    /**
     * @brief Reads raw value of absolute angle
     * 
     * @return return raw absolute angle 
     */
    uint16_t readAbsoluteAngleRaw();
    /**
     * @brief Reads raw value of absolute angle with error check
     * 
     * @param error retreive error
     * @return return raw absolute value
     */
    uint16_t readAbsoluteAngleRaw(bool* error);
    /**
     * @brief Reads raw value of absolute angle with 8bit resolution
     * 
     * @return returns 8 bit raw absolute value
     */
    uint8_t readAbsoluteAngleRaw8();
    /**
     * @brief Converts raw absolute angle to degrees
     * 
     * @param rawAngleDataBitLength data length of raw angle
     * @param rawAngle 16-bit / 8bit raw absolute angle value
     * @return returns absolute angle in degrees
     */
    double convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle);
    /**
     * @brief Set the Zero Position of the NCoder730
     * 
     * @param angle offset angle in degrees for zero positioning
     */
    void setZeroPosition(float angle);
    /**
     * @brief Get the Zero Position Offset Angle of the NCoder730
     * 
     * @return returns zero position offset angle in degrees
     */
    float getZeroPosition();
    /**
     * @brief Set the Pulse Per Turn for NCoder730
     * 
     * @param ppr Pulse Per Revolution value of the Encoder in Incremental Mode
     * @n CPR is 4 times PPR
     */
    void setPulsePerTurn(uint16_t ppr);
    /**
     * @brief Get the Pulse Per Turn value of Incremental Encoder for NCoder730
     * 
     * @return returns the PPR value
     */
    uint16_t getPulsePerTurn();
    /**
     * @brief Set the Rotation Direction for the NCoder730
     * 
     * @param dir direction value
     * @n true -> clockwise
     * @n false-> anticlockwise
     */
    void setRotationDirection(bool dir);
    /**
     * @brief Get the Rotation Direction of the NCoder730
     * 
     * @return returns true if the rotation direction is clockwise
     * @return returns false if the rotation direction is anticlockwise
     */
    bool getRotationDirection();
    /**
     * @brief Set the Magnetic Field Low Threshold value for NCoder730
     * 
     */
    void setMagneticFieldLowThreshold(uint8_t MAGLT);
    /**
     * @brief Set the Magnetic Field High Threshold value for NCoder730
     * 
     */
    void setMagneticFieldHighThreshold(uint8_t MAGHT);
    /**
     * @brief Get the Magnetic Field Low Threshold of NCoder730
     * 
     * @return returns the MGLT value (refer datasheet for more details) 
     */
    uint8_t getMagneticFieldLowThreshold();
    /**
     * @brief Get the Magnetic Field High Threshold of NCoder730
     * 
     * @return returns the MGHT value (refer datasheet for more details) 
     */
    uint8_t getMagneticFieldHighThreshold();

};

#endif // _NCoder730_H_
