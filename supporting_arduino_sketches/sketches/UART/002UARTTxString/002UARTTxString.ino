/*
 * Project: UARTTxString
 * Description: Sketch to demo receiving and transmitting a message via UART to ST Discovery board.
 *				Discovery will send a string. Arduino Uno will receive the sting and change the case
 *				for each letter. Finally Arduino Uno will send the string back to Discovery via UART.
 *				The following was used:
 *				+ Arduino Uno
 *				+ UART Tx: Digital Pin 1
 *				+ UART Rx: Digital Pin 0
 * Author: niekiran 
 * https://github.com/niekiran/MasteringMCU/tree/master/Resources/Arduino
 */

void setup() {
  Serial.begin(115200);
  
  // Define the LED pin as Output
  pinMode (13, OUTPUT);
  
 // Serial.println("Arduino Case Converter program running");
 // Serial.println("-------------------------------------");
  
    
}

char changeCase(char ch)
{
  if (ch >= 'A' && ch <= 'Z')
  ch = ch + 32;
    else if (ch >= 'a' && ch <= 'z')
  ch = ch - 32;  

  return ch;
}
void loop() {

  digitalWrite(13, LOW); 
  //wait until something is received
  while(! Serial.available());
  digitalWrite(13, HIGH); 
  //read the data
  char in_read=Serial.read();
  //print the data
  Serial.print(changeCase(in_read));
}
