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
  /**
   * @brief Initialize SPI with custom SPI_MODE3
   * 
   */
  enc.beginSPI(SPI_SCLK_FREQUENCY, NCoder730_SPI_MODE_3, SPI_CS_PIN);
  /**
   * @brief Writing configurations for NCoder730
   * 
   */
  enc.setZeroPosition(30);  //  Sets zero angle offset by 30 degrees
  enc.setPulsePerTurn(1000);  // Sets PPR of incremental encoder to 1000
  enc.setRotationDirection(true); //Sets the direction of rotation as anti-clockwise
  
  /**
   * @brief Reading configurations of NCoder730
   * 
   */
  Serial.print("Zero Position of Encoder is: ");Serial.println(enc.getZeroPosition());
  Serial.print("PPR of Encoder is: ");Serial.println(enc.getPulsePerTurn());
  Serial.print("Rotation Direction of Encoder is: ");Serial.println(enc.getRotationDirection());
}

void loop() {
  // put your main code here, to run repeatedly:
}
