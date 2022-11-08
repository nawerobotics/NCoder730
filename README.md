# NMotion NCoder730

<img align="left" width="50%" style="margin-right: 12px" src="resources/images/IsometricView.png"/>

NMotion<sup>TM</sup> NCoder730 is a Breakout Board for MPS MagAlpha MA730 Magnetic Encoder. The MA730 detects the absolute angular position of a permanent magnet, typically a diametrically magnetized cylinder on a rotating shaft. Fast data acquisition and processing provide accurate angle measurements at speeds from 0 to 60,000 rpm. The MA730 supports a wide range of magnetic field strengths and spatial configurations. This can be used for both end-of-shaft and off-axis (side-shaft mounting) configurations.


<div align="center" style="padding: 12px;">
<a style="padding-right: 12px" href="https://www.amazon.in/gp/product/B0BC9TYGGC"><img src="https://img.shields.io/badge/-Purchase%20Product-green">
</a>
<a href="https://drive.google.com/file/d/1at6gYZ4LiHeWzJLjMkfs-I9OMhACQv6A/view"><img src="https://img.shields.io/badge/-Download%20Datahseet-blue">
</a>
</div>

## Table of Contents

* [Features](#features)
* [Applications](#applications)
* [Specifications](#specifications)
* [Breakout Board Datasheet](#breakout-board-datasheet)
* [Library Installation](#installation)
* [API Functions for SPI Interface](#api-functions)
* [Compatibility](#compatibility)


## Features
* 14-Bit Resolution Absolute Angle Encoder
* Contactless Sensing for Long Life
* SPI Serial Interface for Digital Angle Readout and Chip Configuration
* Incremental 12-Bit ABZ Quadrature Encoder Interface with Programmable Pulses Per Turn from 1 - 1024
* PWM Output 14-Bit

## Applications
*   General Purpose Angle Measurements
*   High-Resolution Angle Encoders
*   Automotive Angle
*   Robotics

## Specifications
*   Power Supply Voltage: 3.3V
*   Operating Current: < 12mA
*   Incremental Encoder Resolution: 1-1024 PPR
*   Absolute Encoder Resolution: 14-bits
*   PWM Output Resolution: 14-bits
*   Operating Temperature: -40<sup>o</sup>C to 125<sup>o</sup>C
*   Product Dimension: 20x20mm(LxW)
*   Mounting Hole: 2mm (4 holes) 


## Library Installation
To use this library, download the library file first, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## API Functions for SPI Interface with Chip

```C++
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
     * Following values can be set: 26mT, 41mT, 56mT, 70mT ,84mT ,98mT ,112mT ,126mT.
     */
    void setMagneticFieldLowThreshold(uint8_t MAGLT);
    /**
     * @brief Set the Magnetic Field High Threshold value for NCoder730
     * Following values can be set: 20mT, 35mT, 50mT, 64mT ,78mT ,92mT ,106mT ,120mT.
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
    /**
     * @brief Get the Magnetic Field Low Level Status of NCoder730
     * 
     * @return returns true if the magnetic field is below the low threshold. 
     * @return returns false if the magnetic field is above the low threshold
     */
    bool getMagneticFieldLowLevelStatus();
    /**
     * @brief Get the Magnetic Field High Level Status of NCoder730
     * 
     * @return returns true if the magnetic field is above the high threshold. 
     * @return returns false if the magnetic field is below the high threshold
     */
    bool getMagneticFieldHighLevelStatus();
    /**
     * @brief Set the Index Length 
     * 
     * @param length is the length of index pulse 
     * Four Possible values accepted: Index lenght is 
     * 0.5 times the A or B pulse length
     * 1 times the A or B pulse length
     * 1.5 times the A or B pulse length
     * 2 times the A or B pulse length
     * Refer Fig 26 Page No. 23 of MA730 IC datasheet for more details
     */
    void setIndexLength(float length);
    /**
     * @brief Get the Index Length 
     * 
     * @return returns the length of index pulse with respect to A or B pulse length
     */
    float getIndexLength();
    /**
     * @brief Set the Index Position with respect to channel A or B
     * 
     * @param pos is value for setting rising edge of Index pulse with respect to A or B pulse
     * 0 : index rising edge is aligned with the channel B falling edge
     * 1 : index rising edge is aligned with the channel A rising edge
     * 2 : index rising edge is aligned with the channel B rising edge
     * 3 : index rising edge is aligned with the channel A falling edge
     */
    void setIndexPosition(uint8_t pos);
    /**
     * @brief Get the Index Position with respect to channel A or B
     * 
     * @return returns the value of pos which is as follows:
     * 0 : index rising edge is aligned with the channel B falling edge
     * 1 : index rising edge is aligned with the channel A rising edge
     * 2 : index rising edge is aligned with the channel B rising edge
     * 3 : index rising edge is aligned with the channel A falling edge
     */
    uint8_t getIndexPosition();

```

## Compatibility

| MCU             | Work Well | Work Wrong | Untested | Remarks |
| --------------- | :-------: | :--------: | :------: | ------- |
| Arduino Uno     |     √     |            |          |         |
| Teensy 4.1      |     √     |            |          |         |
| Arduino MEGA2560|     √     |            |          |         |
| ESP32-S         |     √     |            |          |         |
