/*
 * Project: UARTRxString
 * Description: Sketch to demo receiving a message via UART from ST Discovery board
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
  
  Serial.println("Arduino UART Receiver");
  Serial.println("-----------------------------");
    
}


void loop() {

  digitalWrite(13, LOW); 
  //wait until something is received
  while(! Serial.available());
  digitalWrite(13, HIGH); 
  //read the data
  char in_read=Serial.read();
  //print the data
  Serial.print(in_read);

}
