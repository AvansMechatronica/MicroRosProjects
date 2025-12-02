// Filename: src/main.cpp
// Voice Interaction Module for micro-ROS on ESP32
// This demonstrates communication with a Voice Interaction Module (VIM)
// via UART using micro-ROS. It sets up publishers and subscribers to handle
// commands and responses between the ESP32 and the VIM.
// Make sure to configure the VIM_TX_PIN and VIM_RX_PIN according to your wiring.
// Required Libraries: micro_ros_platformio, HardwareSerial
// Author: Gerard Harkema
// Date: November 2025
// License: CC BY-NC-SA 4.0

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // Header for string assignment functions
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>


#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int16.h>

#include <HardwareSerial.h>
#include "vim_defines.h"
#include <std_msgs/msg/string.h>


std_msgs__msg__Int16 command_from_vim_msg;
std_msgs__msg__String command_from_vim_string_msg;

std_msgs__msg__Int16 response_to_vim_msg;
std_msgs__msg__String response_to_vim_string_msg;

rcl_publisher_t command_from_vim_publisher;
rcl_publisher_t command_from_vim_string_publisher;

rcl_subscription_t response_to_vim_subscriber;
rcl_publisher_t response_to_vim_string_publisher;


HardwareSerial *VIM_serial;

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#if defined(MULTI_COLOR_LED)
  #include <WS2812FX.h>
  WS2812FX ws2812fxStatus = WS2812FX(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);
  #define RGB_BRIGHTNESS 10 // Change white brightness (max 255)
#endif


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define NODE_NAME "voice_interaction_module_node"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

bool errorLedState = false;


void error_loop(){
  Serial.printf("Voice Interaction Module\nError\nSystem halted");
  while(1){
      

#if defined(MULTI_COLOR_LED)
        ws2812fxStatus.service();
#endif
        if(errorLedState){
#if defined(MULTI_COLOR_LED)
            ws2812fxStatus.setColor(0,0,0);
#else
#if defined(STATUS_LED_PIN)
            digitalWrite(STATUS_LED_PIN, HIGH);
#endif
#endif
            errorLedState = false;
        }
        else{
            //neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0, 0);
#if defined(MULTI_COLOR_LED)
            ws2812fxStatus.setColor(RGB_BRIGHTNESS,0,0);
#else
#if defined(STATUS_LED_PIN)
            digitalWrite(STATUS_LED_PIN, LOW);
#endif
#endif
            errorLedState = true;
        }
    delay(100);
    ESP.restart();
  }
}

uint16_t byte_swap(uint16_t val){
    return (val << 8) | (val >> 8);
}

bool toggle = false;
void response_to_vim_callback(const void* msgin) {
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  // Process the received command
  // Check for valid command index

#if defined(MULTI_COLOR_LED)
  if(toggle){
      ws2812fxStatus.setColor(0, 0, RGB_BRIGHTNESS);
  }
  else{
      ws2812fxStatus.setColor(0, RGB_BRIGHTNESS, 0);
  }
  toggle = !toggle;
  ws2812fxStatus.service();
#endif
  bool break_flag = false;
  for(int i=0; i<sizeof(vim_commands)/sizeof(vim_command_t); i++){
    if(msg->data == vim_commands[i].command){

        rosidl_runtime_c__String__init(&response_to_vim_string_msg.data);
        response_to_vim_string_msg.data = micro_ros_string_utilities_set(response_to_vim_string_msg.data, vim_response[i].command_name);
        break_flag = true;
        break;
    }
  }
  if(!break_flag){
      char* message;
      sprintf(message, "UNKNOWN RESPONSE TO VIM: 0x%04X", msg->data);
      rosidl_runtime_c__String__init(&response_to_vim_string_msg.data);
      response_to_vim_string_msg.data = micro_ros_string_utilities_set(response_to_vim_string_msg.data, message);
      RCSOFTCHECK(rcl_publish(&response_to_vim_string_publisher, &response_to_vim_string_msg, NULL));
      return;
  }
  
  RCSOFTCHECK(rcl_publish(&response_to_vim_string_publisher, &response_to_vim_string_msg, NULL));

  vim_message_frame_t response_to_vim;
  response_to_vim.header = byte_swap(0xAA55);
  response_to_vim.message = byte_swap(msg->data); 
  response_to_vim.footer = 0xFB;
  VIM_serial->write((uint8_t*)&response_to_vim, sizeof(response_to_vim));
}


bool receive(vim_message_frame_t *frame){
  int frame_size = sizeof(vim_message_frame_t);
  
  uint8_t *frame_ptr = (uint8_t*)frame;
  unsigned long startTime = millis();

  uint8_t buffer[255];
  memset(buffer, 0, sizeof(buffer));
  int number_of_bytes = VIM_serial->available();
  if(number_of_bytes == 0){
      return false;
  }
  VIM_serial->readBytes(buffer, number_of_bytes);
 
  for(int i=0; i<number_of_bytes; i++){
      frame_ptr[i] = buffer[i]; // Just an example of processing
  }
  Serial.println();
  frame->header = byte_swap(frame->header);
  frame->message = byte_swap(frame->message);
  return true;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  
  RCLC_UNUSED(last_call_time);
  int frame_size = sizeof(vim_message_frame_t);
  vim_message_frame_t frame;

  if (timer != NULL) {
    if(!receive(&frame)){
      return;
    }
    response_to_vim_msg.data = frame.message; // Example: echo back the message field
    RCSOFTCHECK(rcl_publish(&command_from_vim_publisher, &response_to_vim_msg, NULL));
    for(int i=0; i<sizeof(vim_commands)/sizeof(vim_command_t); i++){
      if(frame.message == vim_commands[i].command){
        // Publish string message
        //rosidl_runtime_c__String__assign(&command_from_vim_string_msg.data, vim_commands[i].command_name);
        rosidl_runtime_c__String__init(&command_from_vim_string_msg.data);
        command_from_vim_string_msg.data = micro_ros_string_utilities_set(command_from_vim_string_msg.data, vim_commands[i].command_name);
        RCSOFTCHECK(rcl_publish(&command_from_vim_string_publisher, &command_from_vim_string_msg, NULL));

        //rosidl_runtime_c__String__assign(&response_to_vim_string_msg.data, vim_response[i].command_name);
        rosidl_runtime_c__String__init(&response_to_vim_string_msg.data);
        response_to_vim_string_msg.data = micro_ros_string_utilities_set(response_to_vim_string_msg.data, vim_response[i].command_name);
        RCSOFTCHECK(rcl_publish(&response_to_vim_string_publisher, &response_to_vim_string_msg, NULL));

        break;
      }
    } 
  }
}

void setup() {
  // Configure serial transport
#if defined(MULTI_COLOR_LED)
  ws2812fxStatus.init();
  ws2812fxStatus.setMode(FX_MODE_STATIC);
  ws2812fxStatus.setColor(RGB_BRIGHTNESS,0,0);
  ws2812fxStatus.setBrightness(100);
  ws2812fxStatus.setSpeed(200);
  ws2812fxStatus.start();
  ws2812fxStatus.service();
#else
#if defined(STATUS_LED_PIN)
  pinMode(STATUS_LED_PIN, OUTPUT); 
  digitalWrite(STATUS_LED_PIN, HIGH);
#endif
#endif


  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

#if defined(MULTI_COLOR_LED)
  ws2812fxStatus.setColor(0, 0, RGB_BRIGHTNESS);
  ws2812fxStatus.service();
#endif

  VIM_serial = new HardwareSerial(2);
  VIM_serial->begin(115200, SERIAL_8N1, VIM_RX_PIN, VIM_TX_PIN);


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));


  // create sensor_information_publisher
  RCCHECK(rclc_publisher_init_default(
    &command_from_vim_publisher,  
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/vim/command_from_vim"));

  RCCHECK(rclc_publisher_init_default(
    &command_from_vim_string_publisher,  
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/vim/command_from_vim_string"));


  RCCHECK(rclc_publisher_init_default(
    &response_to_vim_string_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/vim/response_to_vim_string"));


  // create response_to_vim_subscriber
  RCCHECK(rclc_subscription_init_default(
    &response_to_vim_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/vim/response_to_vim"));


  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &response_to_vim_subscriber,
    &response_to_vim_msg,
    &response_to_vim_callback,
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

#if defined(MULTI_COLOR_LED)
  ws2812fxStatus.setColor(0, RGB_BRIGHTNESS,0);
#else
#if defined(STATUS_LED_PIN)
  digitalWrite(STATUS_LED_PIN, LOW);
#endif
#endif

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

#if defined(MULTI_COLOR_LED)
    ws2812fxStatus.service();
#endif
}