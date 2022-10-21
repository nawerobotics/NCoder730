#include <NCoder730.h>
#include <math.h>
//Check https://www.arduino.cc/en/reference/SPI for SPI signals connections

#define NCoder730_CPR 4096          // CPR[Counts Per Revolution] of the Encoder

#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)

NCoder730 enc;

const int Encoder_ChannelA_Pin = 2;   // Connect Channel A to Interrupt Pin of the controller
const int Encoder_ChannelB_Pin = 3;   // Connect Channel B to Interrupt Pin of the controller
/**
   Interrput Pins
   UNO - 2, 3
   MEGA - 2, 3, 18, 19, 20, 21
   Due - all digital pins
   For more details refer https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
*/

volatile int64_t encoderCount = 0;  // variable to store encoder count

/**
   Interrupt Service Routine for Channel A of Encoder
   Trigger when Change in Pulse
*/
void EncoderChannelA_ISR()
{
  int check = digitalRead(Encoder_ChannelB_Pin);
  if ((check == digitalRead(Encoder_ChannelA_Pin)))
    encoderCount--;
  else
    encoderCount++;
}

/**
   Interrupt Service Routine for Channel B of Encoder
   Trigger when Change in Pulse
*/
void EncoderChannelB_ISR()
{
  int check = digitalRead(Encoder_ChannelA_Pin);
  if ((check == digitalRead(Encoder_ChannelB_Pin)))
    encoderCount++;
  else
    encoderCount--;
}

double calculate_angle(int encoder_count)
{
  double encoder_position = float(encoder_count) / NCoder730_CPR * 360.0;
  return encoder_position;
}


void setup() {
  // put your setup code here, to run once:
  //Set the Serial Communication used to report the angle
  Serial.begin(UART_BAUDRATE);
  /**
   * @brief Initialize Incremental Encoder Pins as Interrupts
   * 
   */
  pinMode(Encoder_ChannelA_Pin, INPUT_PULLUP);
  pinMode(Encoder_ChannelB_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Encoder_ChannelA_Pin), EncoderChannelA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_ChannelB_Pin), EncoderChannelB_ISR, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  double angle;  
  /**
   * @brief Calculates angle based on incremental encoder count
   * 
   */
  angle = calculate_angle(encoderCount);
  Serial.print("Encoder Count: ");Serial.print((int)encoderCount);
  Serial.print("\tAngle: ");Serial.println(angle);
  delay(100);
}
