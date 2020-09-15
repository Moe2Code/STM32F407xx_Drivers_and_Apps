/*
 * Project: I2CSlaveRxString
 * Description: Arduino Uno is used as I2C slave to receive a message from ST Discovery board (master). 
 *              The following used:
 *				+ Arduino Uno
 *				+ I2C SDA Pin: A4
 *				+ I2C SCL Pin: A5
 * Author: niekiran 
 * https://github.com/niekiran/MasteringMCU/tree/master/Resources/Arduino
 */

// Includes
#include <Wire.h>

// Defines
#define MY_ADDR   0x68

int LED = 13;
char rx_buffer[32] ;
uint32_t cnt =0;
uint8_t message[50];

void setup() {

  Serial.begin(9600);
  // Define the LED pin as Output
  pinMode (LED, OUTPUT);
  
 // Start the I2C Bus as Slave on address 0X69
  Wire.begin(MY_ADDR); 
  
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);

  sprintf(message,"Slave is ready : Address 0x%x",MY_ADDR);
  Serial.println((char*)message );  
  Serial.println("Waiting for data from master");  
}

void loop(void)
{
  
}

void receiveEvent(int bytes) 
{
 while( Wire.available() )
 {
   rx_buffer[cnt++] = Wire.read();
 }
  rx_buffer[cnt] = '\0';
  cnt=0;
  Serial.print("Received:");  
  Serial.println((char*)rx_buffer);  
}
