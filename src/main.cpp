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

// Display libraries
#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>

// I2C driver for manual initialization
#include "driver/i2c.h"

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

// LVGL porting configurations
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (500)
#define LVGL_TASK_MIN_DELAY_MS  (1)
#define LVGL_TASK_STACK_SIZE    (4 * 1024)
#define LVGL_TASK_PRIORITY      (2)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 20)

// Extend IO Pin define for Waveshare ESP32-S3-Touch-LCD-4.3
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

// I2C Pin define
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9

ESP_Panel *panel = NULL;
SemaphoreHandle_t lvgl_mux = NULL;

// LVGL display objects for door status
lv_obj_t *screen_main;
lv_obj_t *label_title;
lv_obj_t *label_door1_name;
lv_obj_t *label_door1_status;
lv_obj_t *label_door2_name;
lv_obj_t *label_door2_status;
lv_obj_t *label_timestamp;

bool displayInitialized = false;

// LVGL porting functions
#if ESP_PANEL_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_RGB
/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    lv_disp_flush_ready(disp);
}
#else
/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
}

bool notify_lvgl_flush_ready(void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}
#endif

#if ESP_PANEL_USE_LCD_TOUCH
/* Read the touchpad */
void lvgl_port_tp_read(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
    panel->getLcdTouch()->readData();

    bool touched = panel->getLcdTouch()->getTouchState();
    if(!touched) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        TouchPoint point = panel->getLcdTouch()->getPoint();
        data->state = LV_INDEV_STATE_PR;
        data->point.x = point.x;
        data->point.y = point.y;
    }
}
#endif

void lvgl_port_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks);
}

void lvgl_port_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

void lvgl_port_task(void *arg)
{
    Serial.println("Starting LVGL task");

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        lvgl_port_lock(-1);
        task_delay_ms = lv_timer_handler();
        lvgl_port_unlock();
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void createDoorStatusUI() {
    lvgl_port_lock(-1);
    
    // Create main screen
    screen_main = lv_obj_create(NULL);
    lv_obj_clear_flag(screen_main, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(screen_main, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    
    // Title
    label_title = lv_label_create(screen_main);
    lv_label_set_text(label_title, "Open-RMF Door Monitor");
    lv_obj_set_style_text_color(label_title, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label_title, LV_FONT_DEFAULT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 20);
    
    // Door 1 Status
    label_door1_name = lv_label_create(screen_main);
    lv_label_set_text(label_door1_name, "Door 1:");
    lv_obj_set_style_text_color(label_door1_name, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label_door1_name, LV_FONT_DEFAULT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(label_door1_name, LV_ALIGN_LEFT_MID, 50, -60);
    
    label_door1_status = lv_label_create(screen_main);
    lv_label_set_text(label_door1_status, "CLOSED");
    lv_obj_set_style_text_color(label_door1_status, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label_door1_status, LV_FONT_DEFAULT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(label_door1_status, LV_ALIGN_LEFT_MID, 150, -60);
    
    // Door 2 Status
    label_door2_name = lv_label_create(screen_main);
    lv_label_set_text(label_door2_name, "Door 2:");
    lv_obj_set_style_text_color(label_door2_name, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label_door2_name, LV_FONT_DEFAULT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(label_door2_name, LV_ALIGN_LEFT_MID, 50, -20);
    
    label_door2_status = lv_label_create(screen_main);
    lv_label_set_text(label_door2_status, "CLOSED");
    lv_obj_set_style_text_color(label_door2_status, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label_door2_status, LV_FONT_DEFAULT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(label_door2_status, LV_ALIGN_LEFT_MID, 150, -20);
    
    // Timestamp
    label_timestamp = lv_label_create(screen_main);
    lv_label_set_text(label_timestamp, "System Online");
    lv_obj_set_style_text_color(label_timestamp, lv_color_hex(0x888888), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label_timestamp, LV_FONT_DEFAULT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(label_timestamp, LV_ALIGN_BOTTOM_MID, 0, -20);
    
    // Load the main screen
    lv_disp_load_scr(screen_main);
    
    lvgl_port_unlock();
}

void initDisplay() {
    Serial.println("Initializing RMF Door Display with LVGL...");
    
    panel = new ESP_Panel();

    // Initialize LVGL core
    lv_init();

    // Initialize LVGL buffers
    static lv_disp_draw_buf_t draw_buf;
    uint8_t *buf = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    assert(buf);
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_BUF_SIZE);

    // Initialize the display device
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = ESP_PANEL_LCD_H_RES;
    disp_drv.ver_res = ESP_PANEL_LCD_V_RES;
    disp_drv.flush_cb = lvgl_port_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

#if ESP_PANEL_USE_LCD_TOUCH
    // Initialize the input device
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_port_tp_read;
    lv_indev_drv_register(&indev_drv);
#endif

    // Initialize bus and device of panel
    panel->init();
#if ESP_PANEL_LCD_BUS_TYPE != ESP_PANEL_BUS_TYPE_RGB
    panel->getLcd()->setCallback(notify_lvgl_flush_ready, &disp_drv);
#endif

    // Initialize IO expander for Waveshare ESP32-S3-Touch-LCD-4.3
    Serial.println("Initialize IO expander");
    
    // Initialize I2C driver first
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 400000,
        },
    };
    
    esp_err_t i2c_result = i2c_param_config((i2c_port_t)I2C_MASTER_NUM, &i2c_conf);
    if (i2c_result == ESP_OK) {
        i2c_result = i2c_driver_install((i2c_port_t)I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
        if (i2c_result == ESP_OK) {
            Serial.println("I2C driver initialized successfully");
        } else {
            Serial.printf("I2C driver install failed: %s\n", esp_err_to_name(i2c_result));
        }
    } else {
        Serial.printf("I2C config failed: %s\n", esp_err_to_name(i2c_result));
    }
    
    // Now create the IO expander
    ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000);
    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
    expander->multiDigitalWrite(TP_RST | LCD_BL | LCD_RST | SD_CS, HIGH);
    expander->digitalWrite(USB_SEL, LOW);
    panel->addIOExpander(expander);

    // Start panel
    panel->begin();

    // Create a task to run the LVGL task periodically
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    // Create door status UI
    createDoorStatusUI();
    
    displayInitialized = true;
    Serial.println("RMF Door Display with LVGL initialized");
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
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc, #fn);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println("Soft error in " #fn ": " + String(temp_rc));}}

// Function to add log message to buffer
void addLogMessage(String message) {
  logBuffer[logIndex] = message;
  logIndex = (logIndex + 1) % 12;
  Serial.println(message);
}

// Function to display current door status summary
void printDoorStatusSummary() {
  static unsigned long lastSummary = 0;
  if (millis() - lastSummary > 5000) { // Every 5 seconds
    Serial.println("=== RMF Door Status Summary ===");
    for (int i = 0; i < NUM_DOORS; i++) {
      Serial.printf("Door %s: %s", doors[i].name, getDoorStateString(doors[i].current_state));
      if (doors[i].is_moving) {
        unsigned long elapsed = millis() - doors[i].movement_start_time;
        Serial.printf(" (Moving: %lu ms)", elapsed);
      }
      if (doors[i].last_requester.length() > 0) {
        Serial.printf(" [Last req: %s]", doors[i].last_requester.c_str());
      }
      Serial.println();
    }
    Serial.println("==============================");
    lastSummary = millis();
  }
}

// Function to update display with door statuses
void updateDisplay() {
  if (millis() - lastDisplayUpdate < 500) return; // Limit update rate to 2Hz
  
  if (displayInitialized) {
    lvgl_port_lock(-1);
    
    // Update Door 1 status
    const char* door1_state = getDoorStateString(doors[0].current_state);
    lv_label_set_text(label_door1_status, door1_state);
    
    // Set color based on door state
    if (doors[0].current_state == DOOR_OPEN) {
      lv_obj_set_style_text_color(label_door1_status, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT); // Green
    } else if (doors[0].current_state == DOOR_MOVING) {
      lv_obj_set_style_text_color(label_door1_status, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT); // Yellow
    } else if (doors[0].current_state == DOOR_ERROR) {
      lv_obj_set_style_text_color(label_door1_status, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT); // Red
    } else {
      lv_obj_set_style_text_color(label_door1_status, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT); // White
    }
    
    // Update Door 2 status
    const char* door2_state = getDoorStateString(doors[1].current_state);
    lv_label_set_text(label_door2_status, door2_state);
    
    // Set color based on door state
    if (doors[1].current_state == DOOR_OPEN) {
      lv_obj_set_style_text_color(label_door2_status, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT); // Green
    } else if (doors[1].current_state == DOOR_MOVING) {
      lv_obj_set_style_text_color(label_door2_status, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT); // Yellow
    } else if (doors[1].current_state == DOOR_ERROR) {
      lv_obj_set_style_text_color(label_door2_status, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT); // Red
    } else {
      lv_obj_set_style_text_color(label_door2_status, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT); // White
    }
    
    // Update timestamp with current system uptime
    char timestamp_text[50];
    unsigned long uptime_minutes = millis() / 60000;
    unsigned long uptime_seconds = (millis() / 1000) % 60;
    snprintf(timestamp_text, sizeof(timestamp_text), "Uptime: %lu:%02lu", uptime_minutes, uptime_seconds);
    lv_label_set_text(label_timestamp, timestamp_text);
    
    lvgl_port_unlock();
    
    // Simple activity indicator - no direct backlight control needed
    // The ESP32_Display_Panel handles backlight through IO expander
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
void error_loop(rcl_ret_t error_code, const char* function_name){
  String error_msg = "ERROR: RMF Door Node failed in " + String(function_name) + " with code: " + String(error_code);
  Serial.println(error_msg);
  addLogMessage(error_msg);
  
  // Print more detailed error information
  switch(error_code) {
    case RCL_RET_OK:
      Serial.println("Error code: RCL_RET_OK (0) - This shouldn't happen!");
      break;
    case RCL_RET_ERROR:
      Serial.println("Error code: RCL_RET_ERROR (1) - Generic error");
      break;
    case RCL_RET_TIMEOUT:
      Serial.println("Error code: RCL_RET_TIMEOUT (2) - Timeout occurred");
      break;
    case RCL_RET_BAD_ALLOC:
      Serial.println("Error code: RCL_RET_BAD_ALLOC (10) - Memory allocation failed");
      break;
    case RCL_RET_INVALID_ARGUMENT:
      Serial.println("Error code: RCL_RET_INVALID_ARGUMENT (11) - Invalid argument");
      break;
    case RCL_RET_NOT_INIT:
      Serial.println("Error code: RCL_RET_NOT_INIT (20) - Not initialized");
      break;
    case RCL_RET_ALREADY_INIT:
      Serial.println("Error code: RCL_RET_ALREADY_INIT (21) - Already initialized");
      break;
    default:
      Serial.println("Error code: " + String(error_code) + " - Unknown error code");
      break;
  }
  
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
  Serial.println("Setting micro-ROS serial transport...");
  set_microros_serial_transports(Serial);
  
  Serial.println("Getting default allocator...");
  allocator = rcl_get_default_allocator();

  // Create init_options with domain ID
  addLogMessage("Initializing RCL support...");
  addLogMessage("Using ROS Domain ID: " + String(ROS_DOMAIN_ID));
  updateDisplay();
  
  Serial.println("Initializing init options...");
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  
  Serial.println("Setting domain ID to: " + String(ROS_DOMAIN_ID));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
  
  Serial.println("Initializing rclc support with options...");
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  
  Serial.println("Finalizing init options...");
  // Clean up init options
  RCCHECK(rcl_init_options_fini(&init_options));

  // Create node
  addLogMessage("Creating RMF door node...");
  updateDisplay();
  Serial.println("Creating node 'rmf_door_node'...");
  RCCHECK(rclc_node_init_default(&node, "rmf_door_node", "", &support));

  // Create single door states publisher for all doors
  addLogMessage("Creating states publisher...");
  updateDisplay();
  Serial.println("Creating publisher for /door_states...");
  RCCHECK(rclc_publisher_init_default(
    &door_states_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rmf_door_msgs, msg, DoorState),
    "door_states"));

  // Create door requests subscriber
  addLogMessage("Creating requests subscriber...");
  updateDisplay();
  Serial.println("Creating subscriber for /door_requests...");
  RCCHECK(rclc_subscription_init_default(
    &door_requests_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rmf_door_msgs, msg, DoorRequest),
    "door_requests"));

  // Create status timer (1Hz)
  addLogMessage("Creating status timer...");
  updateDisplay();
  Serial.println("Creating status timer...");
  Serial.println("Creating status timer...");
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
  Serial.println("Creating executor with 2 handlers...");
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  
  Serial.println("Adding timer to executor...");
  RCCHECK(rclc_executor_add_timer(&executor, &status_timer));
  
  Serial.println("Adding subscription to executor...");
  RCCHECK(rclc_executor_add_subscription(&executor, &door_requests_subscriber, &door_requests_msg, &door_requests_callback, ON_NEW_DATA));

  Serial.println("Initializing message memory...");
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
  
  // Print door status summary periodically
  printDoorStatusSummary();
  
  delay(10); // Small delay to prevent watchdog issues
}