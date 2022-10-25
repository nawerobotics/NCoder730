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
   * @brief Initialize SPI with default settings
   * 
   */
  enc.beginSPI(SPI_CS_PIN);
  /**
   * @brief Writing configurations for NCoder730
   * 
   */
}

void loop() {
  // put your main code here, to run repeatedly:
  /**
   * @brief Reads Absolute Angle from the Encoder
   * 
   */
  float angle = enc.readAbsoluteAngle();
  Serial.print("Absolute Angle: ");Serial.println(angle);
  delay(100);
}