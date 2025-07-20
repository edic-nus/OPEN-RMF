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
#define NUM_DOORS 2
#define DOOR_MOVE_TIME_MS 3000  // 3 seconds to open/close
#define STATUS_PUBLISH_RATE_MS 1000  // 1Hz status updates

// Door information structure
struct DoorInfo {
  const char* name;
  DoorState current_state;
  DoorMode requested_mode;
  unsigned long movement_start_time;
  bool is_moving;
  uint32_t request_id;
  String last_requester;
  // Hardware pins
  int motor_pin;
  int sensor_open_pin;
  int sensor_closed_pin;
  int lock_pin;
};

// Initialize door information
DoorInfo doors[NUM_DOORS] = {
  {"door1", DOOR_CLOSED, MODE_CLOSED, 0, false, 0, "", 4, 5, 6, 7},    // Door 1
  {"door2", DOOR_CLOSED, MODE_CLOSED, 0, false, 0, "", 8, 9, 10, 11}   // Door 2
};

// Physical simulation pins - these will be overridden by door-specific pins
#define STATUS_LED_PIN 2      // Status LED for overall system

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
    // Initialize status LED for overall system
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    
    // Initialize each door's hardware
    for (int i = 0; i < NUM_DOORS; i++) {
        pinMode(doors[i].motor_pin, OUTPUT);
        pinMode(doors[i].sensor_open_pin, INPUT_PULLUP);
        pinMode(doors[i].sensor_closed_pin, INPUT_PULLUP);
        pinMode(doors[i].lock_pin, OUTPUT);
        
        // Initial state: door closed and locked
        digitalWrite(doors[i].motor_pin, LOW);
        digitalWrite(doors[i].lock_pin, HIGH);  // Locked
        
        Serial.println("Door " + String(doors[i].name) + " hardware initialized - CLOSED and LOCKED");
    }
}

// Door state management for all doors
void updateDoorState() {
    for (int i = 0; i < NUM_DOORS; i++) {
        DoorInfo& door = doors[i];
        
        if (door.is_moving) {
            unsigned long elapsed = millis() - door.movement_start_time;
            
            if (elapsed >= DOOR_MOVE_TIME_MS) {
                // Movement complete
                door.is_moving = false;
                digitalWrite(door.motor_pin, LOW);  // Stop motor
                
                if (door.requested_mode == MODE_OPEN) {
                    door.current_state = DOOR_OPEN;
                    digitalWrite(door.lock_pin, LOW);  // Unlock
                    Serial.println("Door " + String(door.name) + " movement complete: OPEN");
                } else {
                    door.current_state = DOOR_CLOSED;
                    digitalWrite(door.lock_pin, HIGH); // Lock
                    Serial.println("Door " + String(door.name) + " movement complete: CLOSED");
                }
            } else {
                door.current_state = DOOR_MOVING;
                // Simulate motor running
                digitalWrite(door.motor_pin, (millis() / 100) % 2);
            }
        }
    }
    
    // Update status LED based on any door activity
    bool anyDoorMoving = false;
    bool anyDoorOpen = false;
    for (int i = 0; i < NUM_DOORS; i++) {
        if (doors[i].current_state == DOOR_MOVING) anyDoorMoving = true;
        if (doors[i].current_state == DOOR_OPEN) anyDoorOpen = true;
    }
    
    if (anyDoorMoving) {
        digitalWrite(STATUS_LED_PIN, (millis() / 250) % 2);  // Fast blink when any door moving
    } else if (anyDoorOpen) {
        digitalWrite(STATUS_LED_PIN, HIGH);  // Solid when any door open
    } else {
        digitalWrite(STATUS_LED_PIN, LOW);   // Off when all doors closed
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
rcl_publisher_t door_states_publisher;  // Single publisher for all doors
rcl_subscription_t door_requests_subscriber;  // Single subscriber for all door requests

rmf_door_msgs__msg__DoorState door_states_msg;  // Single message reused for all doors
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

// Publish door states for all doors (RMF DoorState format)
void publishDoorStates() {
  if (millis() - lastStatusPublish < STATUS_PUBLISH_RATE_MS) return;
  
  // Get current time
  unsigned long current_time_sec = millis() / 1000;
  unsigned long current_time_nanosec = (millis() % 1000) * 1000000;
  
  // Publish state for each door using the same publisher
  for (int i = 0; i < NUM_DOORS; i++) {
    DoorInfo& door = doors[i];
    
    // Fill RMF DoorState message
    door_states_msg.door_time.sec = current_time_sec;
    door_states_msg.door_time.nanosec = current_time_nanosec;
    
    // Set door name dynamically
    strcpy(door_states_msg.door_name.data, door.name);
    door_states_msg.door_name.size = strlen(door.name);
    
    // Set current mode based on door state
    if (door.current_state == DOOR_OPEN || (door.current_state == DOOR_MOVING && door.requested_mode == MODE_OPEN)) {
      door_states_msg.current_mode.value = MODE_OPEN;
    } else {
      door_states_msg.current_mode.value = MODE_CLOSED;
    }
    
    RCSOFTCHECK(rcl_publish(&door_states_publisher, &door_states_msg, NULL));
    
    const char* stateStr = getDoorStateString(door.current_state);
    const char* modeStr = (door_states_msg.current_mode.value == MODE_OPEN) ? "OPEN" : "CLOSED";
    addLogMessage(String(door.name) + " - State: " + String(stateStr) + " | Mode: " + String(modeStr));
  }
  
  lastStatusPublish = millis();
}

// Error handling
void error_loop(){
  addLogMessage("ERROR: RMF Door Node failed!");
  // Set all doors to error state
  for (int i = 0; i < NUM_DOORS; i++) {
    doors[i].current_state = DOOR_ERROR;
  }
  updateDisplay();
  while(1){
    digitalWrite(STATUS_LED_PIN, (millis() / 500) % 2);  // Slow blink for error
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
  
  // Find the door with matching name
  int door_index = -1;
  for (int i = 0; i < NUM_DOORS; i++) {
    if (door_name == String(doors[i].name)) {
      door_index = i;
      break;
    }
  }
  
  if (door_index == -1) {
    addLogMessage("Request for unknown door: " + door_name);
    return;
  }
  
  DoorInfo& door = doors[door_index];
  
  if (requested_mode == MODE_OPEN) {
    if (door.current_state == DOOR_CLOSED) {
      door.requested_mode = MODE_OPEN;
      door.is_moving = true;
      door.movement_start_time = millis();
      door.request_id++;
      door.last_requester = requester_id;
      
      digitalWrite(door.motor_pin, HIGH);  // Start motor
      addLogMessage(String(door.name) + " opening... Requester: " + door.last_requester + " | ID: " + String(door.request_id));
    } else {
      addLogMessage(String(door.name) + " already open or moving");
    }
  }
  else if (requested_mode == MODE_CLOSED) {
    if (door.current_state == DOOR_OPEN) {
      door.requested_mode = MODE_CLOSED;
      door.is_moving = true;
      door.movement_start_time = millis();
      door.request_id++;
      door.last_requester = requester_id;
      
      digitalWrite(door.motor_pin, HIGH);  // Start motor
      addLogMessage(String(door.name) + " closing... Requester: " + door.last_requester + " | ID: " + String(door.request_id));
    } else {
      addLogMessage(String(door.name) + " already closed or moving");
    }
  }
  else {
    addLogMessage("Unknown requested mode: " + String(requested_mode) + " for door: " + door_name);
  }
}

void setup() {
  // Initialize serial communication first
  Serial.begin(115200);
  delay(2000); // Give serial time to initialize
  
  Serial.println("=== Open-RMF Multi-Door Node Starting ===");
  Serial.println("Doors: door1, door2");
  Serial.println("Initializing hardware...");
  
  // Initialize door hardware
  initDoorHardware();
  
  // Initialize display
  initDisplay();
  
  // Initialize log buffer
  for (int i = 0; i < 12; i++) {
    logBuffer[i] = "";
  }
  
  addLogMessage("RMF Multi-Door Node initializing...");
  addLogMessage("Doors: door1, door2");
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

  // Create single door states publisher for all doors
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

  // Initialize single door states message
  door_states_msg.door_name.data = (char*)malloc(50);
  door_states_msg.door_name.capacity = 50;
  
  // Initialize door request message strings
  door_requests_msg.door_name.data = (char*)malloc(50);
  door_requests_msg.door_name.capacity = 50;
  door_requests_msg.requester_id.data = (char*)malloc(100);
  door_requests_msg.requester_id.capacity = 100;
  
  addLogMessage("RMF Multi-Door Node ready!");
  for (int i = 0; i < NUM_DOORS; i++) {
    addLogMessage(String(doors[i].name) + " State: " + String(getDoorStateString(doors[i].current_state)));
  }
  addLogMessage("Listening on /door_requests");
  addLogMessage("Publishing to /door_states");
  updateDisplay();
  
  Serial.println("=== RMF Multi-Door Node Ready ===");
  Serial.println("ROS Domain ID: " + String(ROS_DOMAIN_ID));
  Serial.println("Doors: door1, door2");
  Serial.println("Topics:");
  Serial.println("  Subscribe: /door_requests (rmf_door_msgs/DoorRequest)");
  Serial.println("  Publish:   /door_states (rmf_door_msgs/DoorState)");
  Serial.println("Native RMF message types enabled");
  Serial.println("Note: Both doors publish to the same /door_states topic");
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