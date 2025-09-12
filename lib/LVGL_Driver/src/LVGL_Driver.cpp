/*****************************************************************************
  | File        :   LVGL_Driver.c
  
  | help        : 
    The provided LVGL library file must be installed first
******************************************************************************/
#include "LVGL_Driver.h"

static lv_color_t* buf1 = (lv_color_t*)heap_caps_aligned_alloc(32, (LCD_WIDTH * LCD_HEIGHT * 2) / BUFFER_FACTOR, MALLOC_CAP_DMA);
static lv_color_t* buf2 = (lv_color_t*)heap_caps_aligned_alloc(32, (LCD_WIDTH * LCD_HEIGHT * 2) / BUFFER_FACTOR, MALLOC_CAP_DMA);

/* Flush callback: Transfers LVGL-rendered area to the actual LCD */
void lvgl_flush_callback(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p) {
  lcd_add_window(area->x1, area->x2, area->y1, area->y2, color_p);
  lv_display_flush_ready(disp);
}

/* Initialize LVGL with double buffering and display flushing */
void lvgl_init(void) {
  lv_init();
  lv_tick_set_cb(xTaskGetTickCount);
  
  if (!buf1) {
    printf("LVGL buffer allocation failed!\n");
    return;
  }

  lv_display_t *disp_drv = lv_display_create(LCD_WIDTH, LCD_HEIGHT);

  /* Initialize the draw buffer */
  lv_display_set_buffers(disp_drv, buf1, NULL, (LCD_WIDTH * LCD_HEIGHT) / BUFFER_FACTOR, LV_DISPLAY_RENDER_MODE_PARTIAL);

  /* Set the display resolution */
  lv_display_set_resolution(disp_drv, LCD_WIDTH, LCD_HEIGHT);
  lv_display_set_physical_resolution(disp_drv, LCD_WIDTH, LCD_HEIGHT);

  /* Set flush callback */
  lv_display_set_flush_cb(disp_drv, lvgl_flush_callback);
}