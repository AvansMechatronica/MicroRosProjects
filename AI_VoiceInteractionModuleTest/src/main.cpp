#include <Arduino.h>


#include <HardwareSerial.h>
#include "vic_defines.h"

HardwareSerial *VIC_serial;


void setup() {
  // Configure serial transport

  Serial.begin(115200);
  delay(2000);


  VIC_serial = new HardwareSerial(2);
  VIC_serial->begin(115200, SERIAL_8N1, VIC_RX_PIN, VIC_TX_PIN);

}

uint32_t timeout_ms = 1000; // 1 second timeout for receiving data

uint16_t byte_swap(uint16_t val){
    return (val << 8) | (val >> 8);
}

void receive(){
  int frame_size = sizeof(vic_message_frame_t);
  vic_message_frame_t frame;
  uint8_t *frame_ptr = (uint8_t*)&frame;
  unsigned long startTime = millis();

  uint8_t buffer[255];
  memset(buffer, 0, sizeof(buffer));
  int number_of_bytes = VIC_serial->available();
  if(number_of_bytes == 0){
      Serial.printf("No data available to read\n");
      delay(100);
      return;
  }
  VIC_serial->readBytes(buffer, number_of_bytes);
  Serial.printf("Received %d bytes: ", number_of_bytes);  
  for(int i=0; i<number_of_bytes; i++){
      frame_ptr[i] = buffer[i]; // Just an example of processing
      Serial.printf("%02X ", buffer[i]);
  }
  Serial.println();
  frame.header = byte_swap(frame.header);
  frame.message = byte_swap(frame.message);
  Serial.printf("Received VIC frame - Header: %04X, Message: %04X, Footer: %02X\n", frame.header, frame.message, frame.footer);
  for(int i=0; i<sizeof(vic_commands)/sizeof(vic_command_t); i++){
      if(frame.message == vic_commands[i].command){
          Serial.printf("Matched command: %s\n", vic_commands[i].command_name);
      } 
  }



}

void loop() {



#if 0

  receive();
#else
  //uint8_t send[] = {0xAA, 0x55, 0x00, 0x06, 0xFB}; // Example command to request data
  //VIC_serial->write(send, sizeof(send)); // Send a test byte

  vic_message_frame_t command_to_vic;
  command_to_vic.header = 0x55AA;
  command_to_vic.message = 0x04; 
  command_to_vic.footer = 0xFB;
  VIC_serial->write((uint8_t*)&command_to_vic, sizeof(command_to_vic));

  delay(5000); // Wait for a second before next iteration
#endif
}