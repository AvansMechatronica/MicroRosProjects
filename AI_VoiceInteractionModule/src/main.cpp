  #include <Arduino.h>
  #include <micro_ros_platformio.h>

  #include <rcl/rcl.h>
  #include <rclc/rclc.h>
  #include <rclc/executor.h>
  #include "rosidl_runtime_c/string_functions.h"  // Header for string assignment functions

  #include <std_msgs/msg/bool.h>
  #include <std_msgs/msg/int16.h>

  #include <HardwareSerial.h>
  #include "vic_defines.h"


  std_msgs__msg__Int16 recieve_command_msg;
  std_msgs__msg__Int16 send_response_msg;

  rcl_subscription_t command_subscription;
  rcl_publisher_t response_publisher;


  HardwareSerial *VIC_serial;

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

  #define NODE_NAME "voice_interaction_node"

  #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
  #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

  bool errorLedState = false;


  void error_loop(){
    Serial.printf("Voice Interaction\nError\nSystem halted");
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
    }
  }


  void cmd_received_command_callback(const void* msgin) {
    const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
    Serial.printf("Received command: %d\n", msg->data);

  }

  const size_t size_of_frame = 4; // Example frame size
  const uint32_t timeout_ms = 100; // Timeout for serial read
  void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    
    RCLC_UNUSED(last_call_time);
    int frame_size = sizeof(vic_message_frame_t);
    vic_message_frame_t frame;


    if (timer != NULL) {
          uint32_t startTime = millis();
          while (VIC_serial ->available() < frame_size){
          if (millis() - startTime > timeout_ms){
              //DEBUG_PRINT("Timeout: geen scan data ontvangen\n");
              //return false;
              return;
          }
          //vTaskDelay(1/ portTICK_PERIOD_MS);
      }

      if(VIC_serial->readBytes(((uint8_t*)&frame), frame_size) != size_of_frame) {
            //DEBUG_PRINT("Fout bij lezen volledige scan frame\n");
          //return false;
          return;
      }
      Serial.printf("Received VIC frame - Header: %04X, Message: %04X, Footer: %04X\n", frame.header, frame.message, frame.footer);
      send_response_msg.data = frame.message; // Example: echo back the message field
      RCSOFTCHECK(rcl_publish(&response_publisher, &send_response_msg, NULL));
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

    VIC_serial = new HardwareSerial(2);
    VIC_serial->begin(115200, SERIAL_8N1, VIC_RX_PIN, VIC_TX_PIN);


    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));


    // create sensor_information_publisher
    RCCHECK(rclc_publisher_init_default(
      &response_publisher,  
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "response_topic"));

    // create command_subscription
    RCCHECK(rclc_subscription_init_default(
      &command_subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "command_topic"));

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
      &command_subscription,
      &recieve_command_msg,
      &cmd_received_command_callback,
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