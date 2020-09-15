/*
 * Project: I2CSlaveTxString
 * Description: Arduino Uno is used as I2C slave transmitter and receiver. ST Discovery board (master) will
 *				transmit a command. Arduino Uno will receive the command and will send  one of two messages
 *				depending on the command. The following used:
 *				+ Arduino Uno
 *				+ I2C SDA Pin: A4
 *				+ I2C SCL Pin: A5
 * Author: niekiran 
 * https://github.com/niekiran/MasteringMCU/tree/master/Resources/Arduino
 */

// Includes
#include <Bridge.h>
#include <BridgeClient.h>
#include <BridgeServer.h>
#include <BridgeSSLClient.h>
#include <BridgeUdp.h>
#include <Console.h>
#include <FileIO.h>
#include <HttpClient.h>
#include <Mailbox.h>
#include <Process.h>
#include <YunClient.h>
#include <YunServer.h>

// Include the required Wire library for I2C
#include <Wire.h>

int LED = 13;
uint8_t active_command = 0xff,led_status=0;
char name_msg[32] = "I2C com with interrupts\n";

uint16_t device_id = 0xFF45;

#define SLAVE_ADDR 0x68

uint8_t get_len_of_data(void)
{
  return (uint8_t)strlen(name_msg);
}

void setup() {
  // Define the LED pin as Output
  pinMode (LED, OUTPUT);
  
  // Start the I2C Bus as Slave on address 9
  Wire.begin(SLAVE_ADDR); 
  
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);


}

//write
void receiveEvent(int bytes) {
  active_command = Wire.read();    // read one character from the I2C 
}

//read
void requestEvent() {

  if(active_command == 0X51)
  {
    uint8_t len = get_len_of_data();
    Wire.write(&len,1);
    active_command = 0xff;
  }
  
  if(active_command == 0x52)
  {
   // Wire.write(strlen(name));
    Wire.write(name_msg,get_len_of_data());
   // Wire.write((uint8_t*)&name_msg[32],18);
    active_command = 0xff;
  }
  //Wire.write("hello "); // respond with message of 6 bytes
  // as expected by master
}

void loop() 
{
  
}
