#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_VL6180X.h>

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <vl6180_msgs/msg/vl6180.h>
// #include <vl6180_msgs/msg/detail/vl6180__functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>


/* microROS setup */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)){}}

rcl_publisher_t publisher;
vl6180_msgs__msg__Vl6180 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// microROS system state
enum SystemState {
    AGENT_WAIT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} system_state;


// VL6180 setup
#define VL0_ADDRESS 0X30
#define VL1_ADDRESS 0X31

Adafruit_VL6180X tof0 = Adafruit_VL6180X();
Adafruit_VL6180X tof1 = Adafruit_VL6180X();

typedef struct {
    Adafruit_VL6180X* psensor; // pointer to sensor object
    TwoWire* pwire;
    int id;     // sensor id
    int shutdown_pin;
    int interrupt_pin;
    int16_t range;
    uint8_t lux;
} sensor_t;

// Adafruit_VL6180X* sensors[] = {&tof0, &tof1};

sensor_t sensors[] = {
  {&tof0, &Wire, VL0_ADDRESS, 11, 9, 0, 0},
  {&tof1, &Wire1, VL1_ADDRESS, 12, 10, 0, 0},
};

const uint8_t NUM_SENSORS = sizeof(sensors) / sizeof(sensors[0]);
int32_t _data[NUM_SENSORS];

uint32_t led_blink_start_time = millis();

void error_loop() {
  Serial.println(F("Error loop..."));
  while (true) {
    if (millis() - led_blink_start_time > 100) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      led_blink_start_time = millis();
    }
  }
}

void reset_sensor(Adafruit_VL6180X* psensor, int _i2c_addr) {
  // TODO: shutdown pins, assign address

}

void reset_sensors() {
  /* Get the I2C sensors ready to read data. Disable all sensors by driving XSHUT pins low. */
  // Enable sensors one by one with respective addresses
  for (uint8_t i=0; i<NUM_SENSORS; ++i) {
  //   digitalWrite(sensors[i].shutdown_pin, HIGH);
  //   delay(100);
    if (!sensors[i].psensor->begin(sensors[i].pwire)) {
      error_loop();
    }
  //   sensors[i].psensor->setAddress(sensors[i].id);
  }
}

void read_sensors() {
  for (uint8_t i=0; i<NUM_SENSORS; ++i) {
    if (sensors[i].psensor->isRangeComplete()) { // check if read from continuous ranging is ready
      // if (sensors[i].psensor->readRangeStatus()) {}
      // sensors[i].range = sensors[i].psensor->readRangeResult();
      msg.data.data[i] = sensors[i].psensor->readRangeResult();
    }
  }
}

bool create_rcl_entities() {
  /* Create the microROS entities */
  // Init allocator
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // create node
  const char* _namespace = "microROS";
  const char* _node_name = "teensy";
  RCCHECK(rclc_node_init_default(&node, _node_name, _namespace, &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(vl6180_msgs, msg, Vl6180),
      "vl6180/data"
  ));
  
  return true;
}

rcl_ret_t destroy_rcl_entities() {
  /* Destroy the microROS-related entities */
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  // rcl_timer_fini(&timer);
  // rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  return RCL_RET_OK;
}

void execute_every_n_ms(int64_t ms, SystemState system_state) {
  /* Method for periodically pinging the micro_ros_agent */
  do {
      static volatile int64_t init = -1;
      if (init == -1) {
          init = uxr_millis();
      }
      if (uxr_millis() - init > ms) {
          system_state;
          init = uxr_millis();
      }
  }
  while (0);
}


void setup() {
  Wire.setClock(400000);
  Wire1.setClock(400000);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  reset_sensors();

  for (uint8_t i=0; i<NUM_SENSORS; ++i) {
    sensors[i].psensor->startRangeContinuous(50);
    // Serial.println(msg.data.data[i]); 
  }

  if (!vl6180_msgs__msg__Vl6180__init(&msg)) {
    error_loop();
  }
  msg.data.data = _data;
  msg.data.capacity = NUM_SENSORS;
  msg.data.size = NUM_SENSORS;
  // if (!rosidl_runtime_c__int32__Sequence__init(&msg.data, msg.data.size)) {
  //   error_loop();
  // }

}


void loop() {

  read_sensors();

  // Check system state for connection status to micro_ros_agent
  switch (system_state) {
    case AGENT_WAIT:
      execute_every_n_ms(500, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : AGENT_WAIT);
        break;
    case AGENT_AVAILABLE:
      system_state = (true == create_rcl_entities()) ? AGENT_CONNECTED : AGENT_WAIT;
      if (system_state == AGENT_WAIT) {
        destroy_rcl_entities();
      }
      break;
    case AGENT_CONNECTED:
      // Publish message
      execute_every_n_ms(10, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
      if (system_state == AGENT_CONNECTED) {
        // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
        rcl_publish(&publisher, &msg, NULL);
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_rcl_entities();
      system_state = AGENT_WAIT;
      break;
    default:
      break;
  }

  // Serial.println("hello world");


  // if (millis() - led_blink_start_time > 1000) {
  //   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //   led_blink_start_time = millis();
  // }

  // LED control
  if (system_state == AGENT_CONNECTED) {
    unsigned long now = millis();
    if (now - led_blink_start_time > 9900) {
      digitalWrite(LED_BUILTIN, HIGH); 
    }
    if (now - led_blink_start_time > 10000) {
      digitalWrite(LED_BUILTIN, LOW); 
      led_blink_start_time = now;
    }
  }  
}