/**
  ******************************************************************************
  * @file    main.cpp
  * @author  JiapengLi
  * @adapted Gerard Harkema
  * @brief   This file provides a ROS 2-adapted demonstration of
              how to control an LED using VoiceRecognitionModule.
  ******************************************************************************
  * @note:
        voice control led
  ******************************************************************************
  * @section  HISTORY
    
    2013/06/13    Initial version.
  */
  
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"
#include "micro_ros.h"

MicroROS micro_ros;

/**        
 * Connection to VR module
 */
VR myVR(VR_RX_PIN, VR_TX_PIN);    // Defined in platformio.ini, default RX pin is 2, TX pin is 3

uint8_t records[7]; // save record
uint8_t buf[64];



#define onRecord    (0)
#define offRecord   (1) 


#if defined(DEBUG)
/**
  @brief   Print signature, if the character is invisible, 
           print hexible value instead.
  @param   buf     --> command length
           len     --> number of parameters
*/
void printSignature(uint8_t *buf, int len)
{
  int i;
  for(i=0; i<len; i++){
    if(buf[i]>0x19 && buf[i]<0x7F){
      Serial.write(buf[i]);
    }
    else{
      Serial.print("[");
      Serial.print(buf[i], HEX);
      Serial.print("]");
    }
  }
}

/**
  @brief   Print signature, if the character is invisible, 
           print hexible value instead.
  @param   buf  -->  VR module return value when voice is recognized.
             buf[0]  -->  Group mode(FF: None Group, 0x8n: User, 0x0n:System
             buf[1]  -->  number of record which is recognized. 
             buf[2]  -->  Recognizer index(position) value of the recognized record.
             buf[3]  -->  Signature length
             buf[4]~buf[n] --> Signature
*/
void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if(buf[0] == 0xFF){
    Serial.print("NONE");
  }
  else if(buf[0]&0x80){
    Serial.print("UG ");
    Serial.print(buf[0]&(~0x80), DEC);
  }
  else{
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if(buf[3]>0){
    printSignature(buf+4, buf[3]);
  }
  else{
    Serial.print("NONE");
  }
  Serial.println("\r\n");
}
#endif

void setup()
{
  
  Serial.begin(115200);
  delay(1000);
#if defined(DEBUG)
  Serial.println("Debug mode is on, the recognized signature will be printed in serial monitor.");
  Serial.println("Elechouse Voice Recognition V3 Module\r\nControl LED sample");
#endif
  delay(1000);
  /** initialize */
  myVR.begin(9600);

  pinMode(ONBOARD_LED_PIN, OUTPUT);

  // Initialize micro-ROS after the application task is created to keep enough
  // contiguous internal RAM available for task stack allocation.
  micro_ros.init();

#if defined(DEBUG)
  if(myVR.clear() == 0){
    Serial.println("Recognizer cleared.");
  }else{
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    micro_ros.error_loop(__LINE__); 
  }
#else
  if(myVR.clear()){
    micro_ros.error_loop(__LINE__);   
  }
#endif
  
  if(myVR.load((uint8_t)onRecord) >= 0){
#if defined(DEBUG)
    Serial.println("onRecord loaded");
#endif
  }
  
  if(myVR.load((uint8_t)offRecord) >= 0){
#if defined(DEBUG)
    Serial.println("offRecord loaded");
#endif

  }

}

void loop()
{
  int ret;
  ret = myVR.recognize(buf, 50);
  if(ret>0){
    micro_ros.publish_detected_word(buf[1]);
    switch(buf[1]){
      case onRecord:
        /** turn on LED */
        digitalWrite(ONBOARD_LED_PIN, LOW);
        break;
      case offRecord:
        /** turn off LED*/
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        break;
      default:
#if defined(DEBUG)
        Serial.println("Record function undefined");
#endif
        break;
    }
    /** voice recognized */
#if defined(DEBUG)
    printVR(buf);
#endif
  }
}


