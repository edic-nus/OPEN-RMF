#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmf_door_msgs/msg/door_state.h>
#include <rmf_door_msgs/msg/door_request.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int32.h>
#include <builtin_interfaces/msg/time.h>

#include "micro_ros_config.h"

// Open-RMF Door Simulation States
enum DoorState {
  DOOR_CLOSED = 0,
  DOOR_OPEN = 2,
  DOOR_MOVING = 1,
  DOOR_ERROR = 3
};

// RMF DoorMode constants
enum DoorMode {
  MODE_CLOSED = 0,
  MODE_OPEN = 2
};

// Door configuration
#define DOOR_NAME "door1"
#define DOOR_MOVE_TIME_MS 3000  // 3 seconds to open/close
#define STATUS_PUBLISH_RATE_MS 1000  // 1Hz status updates

// Door state variables
DoorState current_door_state = DOOR_CLOSED;
DoorMode requested_door_mode = MODE_CLOSED;
unsigned long door_movement_start_time = 0;
bool door_is_moving = false;
uint32_t door_request_id = 0;
String last_requester = "";

// Physical simulation pins
#define DOOR_MOTOR_PIN 4      // Simulate door motor
#define DOOR_SENSOR_OPEN 5    // Simulate door open sensor
#define DOOR_SENSOR_CLOSED 6  // Simulate door closed sensor  
#define STATUS_LED_PIN 2      // Status LED
#define DOOR_LOCK_PIN 7       // Simulate door lock

// Simple display initialization - focusing on backlight control for now
bool displayInitialized = false;

void initDisplay() {
    Serial.println("Initializing RMF Door Display...");
    
    // Setup backlight
    pinMode(38, OUTPUT);
    digitalWrite(38, HIGH); // Turn on backlight
    
    displayInitialized = true;
    Serial.println("RMF Door Display initialized");
}

// Initialize door hardware simulation
void initDoorHardware() {
    pinMode(DOOR_MOTOR_PIN, OUTPUT);
    pinMode(DOOR_SENSOR_OPEN, INPUT_PULLUP);
    pinMode(DOOR_SENSOR_CLOSED, INPUT_PULLUP);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(DOOR_LOCK_PIN, OUTPUT);
    
    // Initial state: door closed and locked
    digitalWrite(DOOR_MOTOR_PIN, LOW);
    digitalWrite(DOOR_LOCK_PIN, HIGH);  // Locked
    digitalWrite(STATUS_LED_PIN, LOW);
    
    Serial.println("Door hardware initialized - CLOSED and LOCKED");
}

// Door state management
void updateDoorState() {
    if (door_is_moving) {
        unsigned long elapsed = millis() - door_movement_start_time;
        
        if (elapsed >= DOOR_MOVE_TIME_MS) {
            // Movement complete
            door_is_moving = false;
            digitalWrite(DOOR_MOTOR_PIN, LOW);  // Stop motor
            
            if (requested_door_mode == MODE_OPEN) {
                current_door_state = DOOR_OPEN;
                digitalWrite(DOOR_LOCK_PIN, LOW);  // Unlock
                Serial.println("Door movement complete: OPEN");
            } else {
                current_door_state = DOOR_CLOSED;
                digitalWrite(DOOR_LOCK_PIN, HIGH); // Lock
                Serial.println("Door movement complete: CLOSED");
            }
        } else {
            current_door_state = DOOR_MOVING;
            // Simulate motor running
            digitalWrite(DOOR_MOTOR_PIN, (millis() / 100) % 2);
        }
    }
    
    // Update status LED
    switch (current_door_state) {
        case DOOR_OPEN:
            digitalWrite(STATUS_LED_PIN, HIGH);
            break;
        case DOOR_CLOSED:
            digitalWrite(STATUS_LED_PIN, LOW);
            break;
        case DOOR_MOVING:
            digitalWrite(STATUS_LED_PIN, (millis() / 250) % 2);  // Fast blink
            break;
        case DOOR_ERROR:
            digitalWrite(STATUS_LED_PIN, (millis() / 500) % 2);  // Slow blink
            break;
    }
}

const char* getDoorStateString(DoorState state) {
    switch (state) {
        case DOOR_CLOSED: return "CLOSED";
        case DOOR_OPEN: return "OPEN";
        case DOOR_MOVING: return "MOVING";
        case DOOR_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

// Log buffer for future display integration
String logBuffer[12]; // Store last 12 log lines
int logIndex = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastStatusPublish = 0;

// micro-ROS objects for RMF Door Node
rcl_publisher_t door_states_publisher;
rcl_subscription_t door_requests_subscriber;

rmf_door_msgs__msg__DoorState door_states_msg;
rmf_door_msgs__msg__DoorRequest door_requests_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t status_timer;

#define LED_PIN 2
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Function to add log message to buffer
void addLogMessage(String message) {
  logBuffer[logIndex] = message;
  logIndex = (logIndex + 1) % 12;
  Serial.println(message);
}

// Function to update display (placeholder for now)
void updateDisplay() {
  if (millis() - lastDisplayUpdate < 100) return; // Limit update rate
  
  // Simple activity indicator with backlight
  if (displayInitialized) {
    static bool activityBlink = false;
    if (millis() % 2000 < 100) { // Brief blink every 2 seconds
      digitalWrite(38, activityBlink);
      activityBlink = !activityBlink;
    } else {
      digitalWrite(38, HIGH);
    }
  }
  
  lastDisplayUpdate = millis();
}

// Publish door states (RMF DoorState format)
void publishDoorStates() {
  if (millis() - lastStatusPublish < STATUS_PUBLISH_RATE_MS) return;
  
  // Get current time
  unsigned long current_time_sec = millis() / 1000;
  unsigned long current_time_nanosec = (millis() % 1000) * 1000000;
  
  // Fill RMF DoorState message
  door_states_msg.door_time.sec = current_time_sec;
  door_states_msg.door_time.nanosec = current_time_nanosec;
  
  // Set door name
  door_states_msg.door_name.data = (char*)DOOR_NAME;
  door_states_msg.door_name.size = strlen(DOOR_NAME);
  door_states_msg.door_name.capacity = strlen(DOOR_NAME) + 1;
  
  // Set current mode based on door state
  if (current_door_state == DOOR_OPEN || (current_door_state == DOOR_MOVING && requested_door_mode == MODE_OPEN)) {
    door_states_msg.current_mode.value = MODE_OPEN;
  } else {
    door_states_msg.current_mode.value = MODE_CLOSED;
  }
  
  RCSOFTCHECK(rcl_publish(&door_states_publisher, &door_states_msg, NULL));
  
  const char* stateStr = getDoorStateString(current_door_state);
  const char* modeStr = (door_states_msg.current_mode.value == MODE_OPEN) ? "OPEN" : "CLOSED";
  addLogMessage("States: " + String(stateStr) + " | Mode: " + String(modeStr));
  lastStatusPublish = millis();
}

// Error handling
void error_loop(){
  addLogMessage("ERROR: RMF Door Node failed!");
  current_door_state = DOOR_ERROR;
  updateDisplay();
  while(1){
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    delay(100);
  }
}

// Timer callback for status publishing
void status_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    publishDoorStates();
  }
}

// Door request subscription callback
void door_requests_callback(const void * msgin)
{
  const rmf_door_msgs__msg__DoorRequest * request_msg = (const rmf_door_msgs__msg__DoorRequest *)msgin;
  
  // Extract information from RMF DoorRequest message
  String door_name = String(request_msg->door_name.data);
  String requester_id = String(request_msg->requester_id.data);
  uint32_t requested_mode = request_msg->requested_mode.value;
  
  addLogMessage("Request received from: " + requester_id + " for door: " + door_name);
  addLogMessage("Requested mode: " + String(requested_mode));
  
  // Check if this request is for our door
  if (door_name != String(DOOR_NAME)) {
    addLogMessage("Request not for this door (" + String(DOOR_NAME) + "), ignoring");
    return;
  }
  
  if (requested_mode == MODE_OPEN) {
    if (current_door_state == DOOR_CLOSED) {
      requested_door_mode = MODE_OPEN;
      door_is_moving = true;
      door_movement_start_time = millis();
      door_request_id++;
      last_requester = requester_id;
      
      digitalWrite(DOOR_MOTOR_PIN, HIGH);  // Start motor
      addLogMessage("Door opening... Requester: " + last_requester + " | ID: " + String(door_request_id));
    } else {
      addLogMessage("Door already open or moving");
    }
  }
  else if (requested_mode == MODE_CLOSED) {
    if (current_door_state == DOOR_OPEN) {
      requested_door_mode = MODE_CLOSED;
      door_is_moving = true;
      door_movement_start_time = millis();
      door_request_id++;
      last_requester = requester_id;
      
      digitalWrite(DOOR_MOTOR_PIN, HIGH);  // Start motor
      addLogMessage("Door closing... Requester: " + last_requester + " | ID: " + String(door_request_id));
    } else {
      addLogMessage("Door already closed or moving");
    }
  }
  else {
    addLogMessage("Unknown requested mode: " + String(requested_mode));
  }
}

void setup() {
  // Initialize serial communication first
  Serial.begin(115200);
  delay(2000); // Give serial time to initialize
  
  Serial.println("=== Open-RMF Door Node Starting ===");
  Serial.println("Door: " + String(DOOR_NAME));
  Serial.println("Initializing hardware...");
  
  // Initialize door hardware
  initDoorHardware();
  
  // Initialize display
  initDisplay();
  
  // Initialize log buffer
  for (int i = 0; i < 12; i++) {
    logBuffer[i] = "";
  }
  
  addLogMessage("RMF Door Node initializing...");
  addLogMessage("Door: " + String(DOOR_NAME));
  updateDisplay();
  
  // Configure micro-ROS transport
  set_microros_serial_transports(Serial);
  
  allocator = rcl_get_default_allocator();

  // Create init_options with domain ID
  addLogMessage("Initializing RCL support...");
  addLogMessage("Using ROS Domain ID: " + String(ROS_DOMAIN_ID));
  updateDisplay();
  
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
  
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  
  // Clean up init options
  RCCHECK(rcl_init_options_fini(&init_options));

  // Create node
  addLogMessage("Creating RMF door node...");
  updateDisplay();
  RCCHECK(rclc_node_init_default(&node, "rmf_door_node", "", &support));

  // Create door states publisher
  addLogMessage("Creating states publisher...");
  updateDisplay();
  RCCHECK(rclc_publisher_init_default(
    &door_states_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rmf_door_msgs, msg, DoorState),
    "door_states"));

  // Create door requests subscriber
  addLogMessage("Creating requests subscriber...");
  updateDisplay();
  RCCHECK(rclc_subscription_init_default(
    &door_requests_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rmf_door_msgs, msg, DoorRequest),
    "door_requests"));

  // Create status timer (1Hz)
  addLogMessage("Creating status timer...");
  updateDisplay();
  const unsigned int timer_timeout = STATUS_PUBLISH_RATE_MS;
  RCCHECK(rclc_timer_init_default2(
    &status_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    status_timer_callback,
    true));

  // Create executor
  addLogMessage("Creating executor...");
  updateDisplay();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &status_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &door_requests_subscriber, &door_requests_msg, &door_requests_callback, ON_NEW_DATA));

  // Initialize messages
  // Initialize door name string
  door_states_msg.door_name.data = (char*)malloc(50);
  door_states_msg.door_name.capacity = 50;
  strcpy(door_states_msg.door_name.data, DOOR_NAME);
  door_states_msg.door_name.size = strlen(DOOR_NAME);
  
  // Initialize door request message strings
  door_requests_msg.door_name.data = (char*)malloc(50);
  door_requests_msg.door_name.capacity = 50;
  door_requests_msg.requester_id.data = (char*)malloc(100);
  door_requests_msg.requester_id.capacity = 100;
  
  addLogMessage("RMF Door Node ready!");
  addLogMessage("State: " + String(getDoorStateString(current_door_state)));
  addLogMessage("Listening on /door_requests");
  addLogMessage("Publishing to /door_states");
  updateDisplay();
  
  Serial.println("=== RMF Door Node Ready ===");
  Serial.println("ROS Domain ID: " + String(ROS_DOMAIN_ID));
  Serial.println("Topics:");
  Serial.println("  Subscribe: /door_requests (rmf_door_msgs/DoorRequest)");
  Serial.println("  Publish:   /door_states (rmf_door_msgs/DoorState)");
  Serial.println("Native RMF message types enabled");
}

void loop() {
  // Update door state machine
  updateDoorState();
  
  // Spin the executor to handle ROS callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  
  // Update display regularly
  updateDisplay();
  
  delay(10); // Small delay to prevent watchdog issues
}