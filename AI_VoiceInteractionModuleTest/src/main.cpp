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
  Serial.printf("Received VIC frame - Header: %04X, Message: %04X, Footer: %02X\n", frame.header, frame.message, frame.footer);
  for(int i=0; i<sizeof(vic_commands)/sizeof(vic_command_t); i++){
      if(frame.message == vic_commands[i].command){
          Serial.printf("Matched command: %s\n", vic_commands[i].command_name);
      } 
  }


#if 0
  while (VIC_serial->available() < frame_size){
    if (millis() - startTime > timeout_ms){
        Serial.printf("Timeout: geen scan data ontvangen\n");
        //VIC_serial->flush();
        //return false;
        return;
    }
  }

  Serial.printf("Data beschikbaar, bytes: %d\n", VIC_serial->available());
  


  if(VIC_serial->readBytes(((uint8_t*)&frame), frame_size) != frame_size) {
      Serial.printf("Fout bij lezen volledige scan frame\n");
      //return false;
      return;
  }
  Serial.printf("Received VIC frame - Header: %04X, Message: %04X, Footer: %02X\n", frame.header, frame.message, frame.footer);
#else
#if 0
  String buffer;
  buffer = VIC_serial->readString(); // Clear buffer before reading new frame
  Serial.printf("Received raw data: %s\n", buffer.c_str());
  for(int i = 0; i<buffer.length()/2; i++){
      String byteString = buffer.substring(i*2, i*2+2);
      uint8_t byteValue = strtoul(byteString.c_str(), NULL, 16);
      Serial.printf("Byte %d: %02X\n", i, byteValue);
  }
  for(int i=0; i<frame_size; i++){
      if(buffer.length() < frame_size*2){
          Serial.printf("Timeout: geen volledige scan data ontvangen\n");
          return;
      }
      String byteString = buffer.substring(i*2, i*2+2);
      frame.header = (i<2) ? (frame.header << 8) | strtoul(byteString.c_str(), NULL, 16) : frame.header;
      frame.message = (i>=2 && i<4) ? (frame.message << 8) | strtoul(byteString.c_str(), NULL, 16) : frame.message;
      frame.footer = (i==4) ? strtoul(byteString.c_str(), NULL, 16) : frame.footer;
  }
#endif
#endif

}

void loop() {



#if 1

  receive();
#else
  uint8_t send[] = {0xAA, 0x55, 0x00, 0x06, 0xFB}; // Example command to request data
  VIC_serial->write(send, sizeof(send)); // Send a test byte
  delay(5000); // Wait for a second before next iteration
#endif
}