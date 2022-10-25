#include<NCoder730.h>

#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)
#define SPI_SCLK_FREQUENCY  10000000      //SPI SCLK Clock frequency in Hz
#define SPI_CS_PIN          10             //SPI CS pin
//Check https://www.arduino.cc/en/reference/SPI for SPI signals connections

NCoder730 enc;  // Creates NCoder730 object

void setup() {
  // put your setup code here, to run once:
  /**
   * @brief Initialize Serial Monitor
   * 
   */
  Serial.begin(UART_BAUDRATE);
  delay(1000);
  /**
   * @brief Initialize SPI with custom SPI_MODE3
   * 
   */
  enc.beginSPI(SPI_SCLK_FREQUENCY, NCoder730_SPI_MODE_3, SPI_CS_PIN);
  /**
   * @brief Writes default configuration to the registors
   * Zero Position -> 0 degrees
   * PPR -> 1024
   * Rotaion Direction -> clockwise
   * 
   */
  enc.writeDefaultConfigurations();
  /**
   * @brief Reading the configurations
   * 
   */
  Serial.println("Default Configuration:");
  Serial.print("Zero Position of Encoder is: "); Serial.println(enc.getZeroPosition());
  Serial.print("PPR of Encoder is: "); Serial.println(enc.getPulsePerTurn());
  Serial.print("Rotation Direction of Encoder is: "); Serial.println(enc.getRotationDirection());
}

void loop() {
  // put your main code here, to run repeatedly:
}