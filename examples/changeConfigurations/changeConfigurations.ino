#include<NCoder730.h>

#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)
#define SPI_SCLK_FREQUENCY  10000000      //SPI SCLK Clock frequency in Hz
#define SPI_CS_PIN          10             //SPI CS pin
//Check https://www.arduino.cc/en/reference/SPI for SPI signals connections

NCoder730 enc;  // Creates NCoder730 object

void setup() {
  // put your setup code here, to run once:
  /**
     @brief Initialize Serial Monitor

  */
  Serial.begin(UART_BAUDRATE);
  delay(1000);
  /**
     @brief Initialize SPI with custom SPI_MODE3

  */
  enc.beginSPI(SPI_SCLK_FREQUENCY, NCoder730_SPI_MODE_3, SPI_CS_PIN);
  /**
     @brief Writing configurations for NCoder730

  */
  enc.setZeroPosition(30);  //  Sets zero angle offset by 30 degrees
  enc.setPulsePerTurn(1000);  // Sets PPR of incremental encoder to 1000
  enc.setIndexLength(2.0f);  // Set index length to twice the pulse width of A or B
  enc.setIndexPosition(1);  // Make index rising edge is align with the channel A rising edge
  enc.setMagneticFieldLowThreshold(41); // Set Magnetic Field Low threshold to 41mT
  enc.setMagneticFieldHighThreshold(92); // Set Magnetic Field High threshold to 92mT
  enc.setRotationDirection(true); //Sets the direction of rotation as anti-clockwise

  /**
     @brief Reading configurations of NCoder730

  */
  Serial.print("Zero Position of Encoder is: "); Serial.println(enc.getZeroPosition());
  Serial.print("PPR of Encoder is: "); Serial.println(enc.getPulsePerTurn());
  Serial.print("Index Lenght is "); Serial.print(enc.getIndexLength()); Serial.println(" times the length of A or B pulse");
  Serial.print("Index pos is "); Serial.print(enc.getIndexPosition()); Serial.println(" which means");
  switch (enc.getIndexPosition()) {
    case 0:
      Serial.println("Index rising edge is aligned with the channel B falling edge");
      break;
    case 1:
      Serial.println("Index rising edge is aligned with the channel A rising edge");
      break;
    case 2:
      Serial.println("Index rising edge is aligned with the channel B rising edge");
      break;
    case 3:
      Serial.println("Index rising edge is aligned with the channel A falling edge");
      break;
  }
  Serial.print("Magnetic Field Low Threshold is "); Serial.print(enc.getMagneticFieldLowThreshold()); Serial.println("mT.");
  Serial.print("Magnetic Field High Threshold is "); Serial.print(enc.getMagneticFieldHighThreshold()); Serial.println("mT.");
  Serial.print("Rotation Direction of Encoder is: "); Serial.println(enc.getRotationDirection());

}

void loop() {
  // put your main code here, to run repeatedly:
}