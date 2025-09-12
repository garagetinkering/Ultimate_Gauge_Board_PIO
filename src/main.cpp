/* Basic Driver setup for the ESP32-S3 2.1 inch LCD Driver board  */
/* Author: Andy Valentine - Garage Tinkering                      */
/*                                                                */

#include <Arduino.h>
#include "CANBus_Driver.h"
#include "LVGL_Driver.h"
#include "I2C_Driver.h"
#include <vector>
#include "esp_task_wdt.h"

// IMAGES
#include "images/tabby_needle.h"
#include "images/tabby_tick.h"

LV_IMG_DECLARE(tabby_needle);
LV_IMG_DECLARE(tabby_tick);

// DATA STORE
typedef struct struct_gauge_data {
  int coolant_temp;
}struct_gauge_data;

struct_gauge_data GaugeData;
#define TAG "TWAI"

// CONTROL CONSTANTS
const int AVERAGE_VALUES      = 20;
const int SCALE_MIN           = -20;
const int SCALE_MAX           = 140;
const int SCALE_TICKS_COUNT   = 9;

const bool TESTING            = false; // set to true for needle sweep testing

// CONTROL VARIABLE INIT
bool initial_load             = false; // has the first data been received
volatile bool data_ready      = false; // new incoming data
bool init_anim_complete       = false; // needle sweep completed
bool status_led               = false; // flashing LED for CAN activity

// ROLLING AVERAGE FOR SMOOTHING
uint8_t average_coolant_temp  = 0;

// GLOBAL COMPONENTS
lv_obj_t *main_scr;
lv_obj_t *scale;
lv_obj_t *needle_img;

lv_obj_t *scale_ticks[SCALE_TICKS_COUNT];

void drivers_init(void) {
  i2c_init();

  Serial.println("Scanning for TCA9554...");
  bool found = false;
  for (int attempt = 0; attempt < 10; attempt++) {
    if (i2c_scan_address(0x20)) { // 0x20 is default for TCA9554
      found = true;
      break;
    }
    delay(50); // wait a bit before retrying
  }

  if (!found) {
    Serial.println("TCA9554 not detected! Skipping expander init.");
  } else {
    tca9554pwr_init(0x00);
  }
  lcd_init();
  canbus_init();
  lvgl_init();
}

// HELPER FUNCTIONS

// create moving average for smoothing
uint8_t get_moving_average(int new_value) {
  static int values[AVERAGE_VALUES] = {0};
  static int index = 0;
  static int count = 0;
  static double sum = 0;

  // Subtract the value being replaced
  sum -= values[index];

  // Insert the new value
  values[index] = new_value;
  sum += new_value;

  // Update index and count
  index = (index + 1) % AVERAGE_VALUES;
  if (count < AVERAGE_VALUES) count++;

  return std::round(sum / count);
}

void update_coolant_temp(void) {
  // use a moving average of the last x values for smoothing
  average_coolant_temp = get_moving_average(GaugeData.coolant_temp);
  if (init_anim_complete) {
    lv_scale_set_image_needle_value(scale, needle_img, average_coolant_temp);
  }
}

// update parts with incoming values
void update_values(void) {
  update_coolant_temp();
}

// update the UI with the latest value
static void set_needle_img_value(void * obj, int32_t v) {
  lv_scale_set_image_needle_value(scale, needle_img, v);
}

// mark the loader as complete
void needle_sweep_complete(lv_anim_t *a) {
  init_anim_complete = true;
}

// scroll the needle from 0 to the current value
void needle_to_current(lv_anim_t *a) {
  lv_anim_t anim_scale_img;
  lv_anim_init(&anim_scale_img);
  lv_anim_set_var(&anim_scale_img, scale);
  lv_anim_set_exec_cb(&anim_scale_img, set_needle_img_value);
  lv_anim_set_duration(&anim_scale_img, 1000);  
  lv_anim_path_ease_out(&anim_scale_img);
  lv_anim_set_values(&anim_scale_img, -20, average_coolant_temp);
  lv_anim_set_ready_cb(&anim_scale_img, needle_sweep_complete);
  lv_anim_start(&anim_scale_img);
}

// 0 to max to 0 sweep on load
void needle_sweep() {
  if (TESTING) {
    // back and forth sweep for testing
    lv_anim_t anim_scale_img;
    lv_anim_init(&anim_scale_img);
    lv_anim_set_var(&anim_scale_img, scale);
    lv_anim_set_exec_cb(&anim_scale_img, set_needle_img_value);
    lv_anim_set_duration(&anim_scale_img, 10000);
    lv_anim_set_repeat_count(&anim_scale_img, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_playback_duration(&anim_scale_img, 10000);
    lv_anim_set_values(&anim_scale_img, -20, 140);
    lv_anim_start(&anim_scale_img);
  } else {
    lv_anim_t anim_scale_img;
    lv_anim_init(&anim_scale_img);
    lv_anim_set_var(&anim_scale_img, scale);
    lv_anim_set_exec_cb(&anim_scale_img, set_needle_img_value);
    lv_anim_set_duration(&anim_scale_img, 2000);
    lv_anim_set_repeat_count(&anim_scale_img, 1);
    lv_anim_set_playback_duration(&anim_scale_img, 1000);
    lv_anim_set_values(&anim_scale_img, SCALE_MIN, SCALE_MAX);
    lv_anim_set_ready_cb(&anim_scale_img, needle_to_current);
    lv_anim_start(&anim_scale_img);
  }    
}

void make_scale_ticks(void) {
  for (int i = 0; i < SCALE_TICKS_COUNT; i++) {
        scale_ticks[i] = lv_image_create(main_scr);
        lv_image_set_src(scale_ticks[i], &tabby_tick);
        lv_obj_align(scale_ticks[i], LV_ALIGN_CENTER, 0, 200);
        lv_image_set_pivot(scale_ticks[i], 12, -188);

        lv_obj_set_style_image_recolor_opa(scale_ticks[i], 255, 0);
        lv_obj_set_style_image_recolor(scale_ticks[i], lv_color_make(255,255,255), 0); // TO DO - replace with color from CAN 

        int rotation_angle = (((i) * (240 / (SCALE_TICKS_COUNT - 1))) * 10); // angle calculation

        lv_image_set_rotation(scale_ticks[i], rotation_angle);
    }
}

// create the elements on the main scr
void main_scr_ui(void) {
  // scale used for needle
  scale = lv_scale_create(main_scr);
  lv_obj_set_size(scale, 480, 480);
  lv_scale_set_mode(scale, LV_SCALE_MODE_ROUND_INNER);
  lv_obj_set_style_bg_opa(scale, LV_OPA_0, 0);
  lv_scale_set_total_tick_count(scale, 0);
  lv_scale_set_label_show(scale, false);
  lv_obj_center(scale);
  lv_scale_set_range(scale, SCALE_MIN, SCALE_MAX);
  lv_scale_set_angle_range(scale, 240);
  lv_scale_set_rotation(scale, 90);

  lv_obj_t *lower_arc = lv_arc_create(main_scr);
  lv_obj_set_size(lower_arc, 412, 412);
  lv_arc_set_bg_angles(lower_arc, 90, 90 + (240 / (SCALE_TICKS_COUNT - 1)));
  lv_arc_set_value(lower_arc, 0);
  lv_obj_center(lower_arc);
  lv_obj_set_style_opa(lower_arc, 0, LV_PART_KNOB);
  lv_obj_set_style_arc_color(lower_arc, lv_color_make(87,10,1), LV_PART_MAIN);
  lv_obj_set_style_arc_width(lower_arc, 16, LV_PART_MAIN);
  lv_obj_set_style_arc_rounded(lower_arc, false, LV_PART_MAIN);

  lv_obj_t *upper_arc = lv_arc_create(main_scr);
  lv_obj_set_size(upper_arc, 412, 412);
  lv_arc_set_bg_angles(upper_arc, 330 - (240 / (SCALE_TICKS_COUNT - 1)), 330);
  lv_arc_set_value(upper_arc, 0);
  lv_obj_center(upper_arc);
  lv_obj_set_style_opa(upper_arc, 0, LV_PART_KNOB);
  lv_obj_set_style_arc_color(upper_arc, lv_color_make(87,10,1), LV_PART_MAIN);
  lv_obj_set_style_arc_width(upper_arc, 16, LV_PART_MAIN);
  lv_obj_set_style_arc_rounded(upper_arc, false, LV_PART_MAIN);
    
    
  make_scale_ticks();
    
  // needle image
  int needle_center_shift = 40; // how far the center of the needle is shifted from the left edge
   
  needle_img = lv_image_create(scale);
  lv_image_set_src(needle_img, &tabby_needle);
  lv_obj_align(needle_img, LV_ALIGN_CENTER, 108 - needle_center_shift, 0);
  lv_image_set_pivot(needle_img, needle_center_shift, 36);

    lv_obj_set_style_image_recolor_opa(needle_img, 255, 0);
    lv_obj_set_style_image_recolor(needle_img, lv_color_make(255,255,255), 0); // TO DO - replace with color from CAN 
}

// build the screens
void screens_init(void) {
  main_scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(main_scr, lv_color_make(0,0,0), 0);
  lv_screen_load(main_scr);

  main_scr_ui();
}

// process incoming CAN coolante temp message
void process_coolant_temp(uint8_t *byte_data) {
    int byte_pos = 0;
    int final_temp = byte_data[byte_pos] - 40; // reduce value by 40 (Nissan specific)

    Serial.println(final_temp);

    GaugeData.coolant_temp = final_temp;
}

// task to avoid blocking LVGL with CAN receive
void receive_can_task(void *arg) {
  // Register this task with the watchdog
  esp_task_wdt_add(NULL);

  while (1) {
    twai_message_t message;
    // Use shorter wait time so we can periodically reset WDT
    esp_err_t err = twai_receive(&message, pdMS_TO_TICKS(100));

    if (err == ESP_OK) {
      
        Serial.print("Received CAN message: ID=0x");
        Serial.print(message.identifier, HEX);
        Serial.print(" DLC=");
        Serial.print(message.data_length_code);
        Serial.print(" Data=");
        for (int i = 0; i < message.data_length_code; i++) {
          Serial.printf("%02X ", message.data[i]);
        }
        Serial.println();

        // process CAN IDs
        switch (message.identifier) {
          case 0x551:
            process_coolant_temp(message.data);
            data_ready = true;
            break;
          default:
            break;
        }

  set_exio(EXIO_PIN4, High);
    } else if (err == ESP_ERR_TIMEOUT) {
        // No message within 100ms — that’s fine
        ESP_LOGW(TAG, "Reception timed out");
    } else {
        ESP_LOGE(TAG, "Message reception failed: %s", esp_err_to_name(err));
    }

    // Reset watchdog and yield
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(10)); // let scheduler breathe
  }
}

void setup(void) {
  Serial.begin(115200);
  Serial.println("begin");
  drivers_init();
  set_backlight(100);
  screens_init();
  needle_sweep();
  set_exio(EXIO_PIN4, Low);
  xTaskCreate(receive_can_task, "Receive_CAN_Task", 8192, NULL, 1, NULL);
  Serial.print("setup complete");
}

void loop(void) {
  lv_timer_handler();

  // only update the display if new data has arrived
  if (data_ready) {
    data_ready = false;
  update_values();
  }

  delay(5);
}