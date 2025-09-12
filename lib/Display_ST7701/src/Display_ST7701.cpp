#include "Display_ST7701.h"  
#include "driver/gpio.h"
      
spi_device_handle_t SPI_handle = NULL;     
esp_lcd_panel_handle_t panel_handle = NULL;            
void st7701_write_command(uint8_t cmd) {
  spi_transaction_t spi_tran = {
    .cmd = 0,
    .addr = cmd,
    .length = 0,
    .rxlength = 0,
  };
  spi_device_transmit(SPI_handle, &spi_tran);
}

void st7701_write_data(uint8_t data) {
  spi_transaction_t spi_tran = {
    .cmd = 1,
    .addr = data,
    .length = 0,
    .rxlength = 0,
  };
  spi_device_transmit(SPI_handle, &spi_tran);
}

void st7701_cs_en(){
  set_exio(EXIO_PIN3, Low);

  vTaskDelay(pdMS_TO_TICKS(10));
}

void st7701_cs_dis(){
  set_exio(EXIO_PIN3, High);

  vTaskDelay(pdMS_TO_TICKS(10));
}

void st7701_reset(){
  set_exio(EXIO_PIN1, Low);
  vTaskDelay(pdMS_TO_TICKS(10));
  set_exio(EXIO_PIN1, High);
  vTaskDelay(pdMS_TO_TICKS(50));
}

void st7701_init() {
  spi_bus_config_t buscfg = {
    .mosi_io_num = LCD_MOSI_PIN,
    .miso_io_num = -1,
    .sclk_io_num = LCD_CLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 64,
  };

  spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  spi_device_interface_config_t devcfg = {
    .command_bits = 1,
    .address_bits = 8,
    .mode = SPI_MODE0,
    .clock_speed_hz = 40000000,
    .spics_io_num = -1,                      
    .queue_size = 1,
  };
  spi_bus_add_device(SPI2_HOST, &devcfg, &SPI_handle);            

  // gpio_reset_pin(LCD_CS_EX_PIN);
  // gpio_set_direction(LCD_CS_EX_PIN, GPIO_MODE_OUTPUT);

  st7701_cs_en();
  st7701_write_command(0xFF);
  st7701_write_data(0x77);
  st7701_write_data(0x01);
  st7701_write_data(0x00);
  st7701_write_data(0x00);
  st7701_write_data(0x10);

  st7701_write_command(0xC0);
  st7701_write_data(0x3B); //Scan line	
  st7701_write_data(0x00);

  st7701_write_command(0xC1);
  st7701_write_data(0x0B);	//VBP
  st7701_write_data(0x02);

  st7701_write_command(0xC2);
  st7701_write_data(0x07);
  st7701_write_data(0x02);

  st7701_write_command(0xCC);
  st7701_write_data(0x10);

  st7701_write_command(0xCD);  //RGB format
  st7701_write_data(0x08);

  st7701_write_command(0xB0);  // IPS   
  st7701_write_data(0x00);     // 255 
  st7701_write_data(0x11);     // 251    
  st7701_write_data(0x16);     // 247  down
  st7701_write_data(0x0e);     // 239    
  st7701_write_data(0x11);     // 231    
  st7701_write_data(0x06);     // 203    
  st7701_write_data(0x05);     // 175 
  st7701_write_data(0x09);     // 147    
  st7701_write_data(0x08);     // 108    
  st7701_write_data(0x21);     // 80  
  st7701_write_data(0x06);     // 52   

  st7701_write_command(0xB1);  //  IPS     
  st7701_write_data(0x00);     //  255 
  st7701_write_data(0x11);     //  251
  st7701_write_data(0x16);     //  247   down
  st7701_write_data(0x0e);     //  239
  st7701_write_data(0x11);     //  231
  st7701_write_data(0x07);     //  203    
  st7701_write_data(0x05);     //  175
  st7701_write_data(0x09);     //  147  
  st7701_write_data(0x09);     //  108  
  st7701_write_data(0x21);     //  80 
  st7701_write_data(0x05);     //  52   
  st7701_write_data(0x13);     //  24 
  st7701_write_data(0x11);     //  16 
  st7701_write_data(0x2a);     //  8  down 
  st7701_write_data(0x31);     //  4  
  st7701_write_data(0x18);     //  0  

  st7701_write_command(0xFF);
  st7701_write_data(0x77);
  st7701_write_data(0x01);
  st7701_write_data(0x00);
  st7701_write_data(0x00);
  st7701_write_data(0x11);

  st7701_write_command(0xB0);  // VOP  3.5375+ *x 0.0125
  st7701_write_data(0x6d);     // 5D
  
  st7701_write_command(0xB1);   // VCOM amplitude setting  
  st7701_write_data(0x37);
  
  st7701_write_command(0xB2);   // VGH Voltage setting  
  st7701_write_data(0x81);      // 12V

  st7701_write_command(0xB3);
  st7701_write_data(0x80);

  st7701_write_command(0xB5);   // VGL Voltage setting  
  st7701_write_data(0x43);      //-8.3V

  st7701_write_command(0xB7);
  st7701_write_data(0x85);

  st7701_write_command(0xB8);
  st7701_write_data(0x20);

  st7701_write_command(0xC1);
  st7701_write_data(0x78);

  st7701_write_command(0xC2);
  st7701_write_data(0x78);

  st7701_write_command(0xD0);
  st7701_write_data(0x88);

  st7701_write_command(0xE0);
  st7701_write_data(0x00);
  st7701_write_data(0x00);
  st7701_write_data(0x02);

  st7701_write_command(0xE1);
  st7701_write_data(0x03);  
  st7701_write_data(0xA0);
  st7701_write_data(0x00);  
  st7701_write_data(0x00);
  st7701_write_data(0x04);  
  st7701_write_data(0xA0);
  st7701_write_data(0x00);  
  st7701_write_data(0x00);  
  st7701_write_data(0x00);
  st7701_write_data(0x20);
  st7701_write_data(0x20);

  st7701_write_command(0xE2);
  st7701_write_data(0x00);  
  st7701_write_data(0x00);    
  st7701_write_data(0x00);  
  st7701_write_data(0x00);  
  st7701_write_data(0x00);  
  st7701_write_data(0x00);  
  st7701_write_data(0x00);  
  st7701_write_data(0x00);  
  st7701_write_data(0x00);  
  st7701_write_data(0x00);  
  st7701_write_data(0x00);  
  st7701_write_data(0x00);  
  st7701_write_data(0x00);

  st7701_write_command(0xE3);
  st7701_write_data(0x00);
  st7701_write_data(0x00);
  st7701_write_data(0x11);
  st7701_write_data(0x00);

  st7701_write_command(0xE4);
  st7701_write_data(0x22);
  st7701_write_data(0x00);

  st7701_write_command(0xE5);    
  st7701_write_data(0x05);  
  st7701_write_data(0xEC);  
  st7701_write_data(0xA0);
  st7701_write_data(0xA0);
  st7701_write_data(0x07);  
  st7701_write_data(0xEE);  
  st7701_write_data(0xA0);
  st7701_write_data(0xA0);
  st7701_write_data(0x00);
  st7701_write_data(0x00);  
  st7701_write_data(0x00);
  st7701_write_data(0x00);
  st7701_write_data(0x00);
  st7701_write_data(0x00);  
  st7701_write_data(0x00);
  st7701_write_data(0x00);

  st7701_write_command(0xE6);
  st7701_write_data(0x00);
  st7701_write_data(0x00);
  st7701_write_data(0x11);
  st7701_write_data(0x00);

  st7701_write_command(0xE7);
  st7701_write_data(0x22);
  st7701_write_data(0x00);

  st7701_write_command(0xE8);    
  st7701_write_data(0x06);  
  st7701_write_data(0xED);  
  st7701_write_data(0xA0);
  st7701_write_data(0xA0);
  st7701_write_data(0x08);  
  st7701_write_data(0xEF);  
  st7701_write_data(0xA0); 
  st7701_write_data(0xA0);
  st7701_write_data(0x00);  
  st7701_write_data(0x00);  
  st7701_write_data(0x00);
  st7701_write_data(0x00);
  st7701_write_data(0x00);  
  st7701_write_data(0x00);  
  st7701_write_data(0x00);
  st7701_write_data(0x00);

  st7701_write_command(0xEB);
  st7701_write_data(0x00);   
  st7701_write_data(0x00);
  st7701_write_data(0x40);
  st7701_write_data(0x40);
  st7701_write_data(0x00);
  st7701_write_data(0x00);
  st7701_write_data(0x00);  

  st7701_write_command(0xED);  
  st7701_write_data(0xFF); 
  st7701_write_data(0xFF);  
  st7701_write_data(0xFF);   
  st7701_write_data(0xBA);     
  st7701_write_data(0x0A);   
  st7701_write_data(0xBF);   
  st7701_write_data(0x45);   
  st7701_write_data(0xFF); 
  st7701_write_data(0xFF);  
  st7701_write_data(0x54);   
  st7701_write_data(0xFB);   
  st7701_write_data(0xA0);   
  st7701_write_data(0xAB);   
  st7701_write_data(0xFF); 
  st7701_write_data(0xFF); 
  st7701_write_data(0xFF); 

  st7701_write_command(0xEF);
  st7701_write_data(0x10); 
  st7701_write_data(0x0D); 
  st7701_write_data(0x04); 
  st7701_write_data(0x08); 
  st7701_write_data(0x3F); 
  st7701_write_data(0x1F);

  st7701_write_command(0xFF);
  st7701_write_data(0x77);
  st7701_write_data(0x01);
  st7701_write_data(0x00);
  st7701_write_data(0x00);
  st7701_write_data(0x13);

  st7701_write_command(0xEF);
  st7701_write_data(0x08);

  st7701_write_command(0xFF);
  st7701_write_data(0x77);
  st7701_write_data(0x01);
  st7701_write_data(0x00);
  st7701_write_data(0x00);
  st7701_write_data(0x00);


  st7701_write_command(0x36);
  st7701_write_data(0x00);

  st7701_write_command(0x3A);
  st7701_write_data(0x66);

  st7701_write_command(0x11);
  
  vTaskDelay(pdMS_TO_TICKS(480));

  st7701_write_command(0x20); //
  vTaskDelay(pdMS_TO_TICKS(120));
  st7701_write_command(0x29); 
  st7701_cs_dis();
  vTaskDelay(pdMS_TO_TICKS(480));

  st7701_write_command(0x20); //
  vTaskDelay(pdMS_TO_TICKS(120));
  st7701_write_command(0x29); 
  st7701_cs_dis();

  //  RGB
  esp_lcd_rgb_panel_config_t rgb_config = {
    .clk_src = LCD_CLK_SRC_XTAL,
    .timings =  {
      .pclk_hz = ESP_PANEL_LCD_RGB_TIMING_FREQ_HZ,
      .h_res = ESP_PANEL_LCD_HEIGHT,
      .v_res = ESP_PANEL_LCD_WIDTH,
      .hsync_pulse_width = ESP_PANEL_LCD_RGB_TIMING_HPW,
      .hsync_back_porch = ESP_PANEL_LCD_RGB_TIMING_HBP,
      .hsync_front_porch = ESP_PANEL_LCD_RGB_TIMING_HFP,
      .vsync_pulse_width = ESP_PANEL_LCD_RGB_TIMING_VPW,
      .vsync_back_porch = ESP_PANEL_LCD_RGB_TIMING_VBP,
      .vsync_front_porch = ESP_PANEL_LCD_RGB_TIMING_VFP,
      .flags = {
        .hsync_idle_low = 0,
        .vsync_idle_low = 0,
        .de_idle_high = 0,
        .pclk_active_neg = false,
        .pclk_idle_high = 0,
      },
    },
    .data_width = ESP_PANEL_LCD_RGB_DATA_WIDTH,
    .bits_per_pixel = ESP_PANEL_LCD_RGB_PIXEL_BITS,
    .num_fbs = ESP_PANEL_LCD_RGB_FRAME_BUF_NUM,
    .bounce_buffer_size_px = 10 * ESP_PANEL_LCD_HEIGHT,
    .psram_trans_align = 64,
    .hsync_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_HSYNC,
    .vsync_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_VSYNC,
    .de_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_DE,
    .pclk_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_PCLK,
    .disp_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_DISP,
    .data_gpio_nums = {
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA0,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA1,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA2,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA3,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA4,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA5,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA6,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA7,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA8,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA9,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA10,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA11,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA12,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA13,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA14,
      ESP_PANEL_LCD_PIN_NUM_RGB_DATA15,
    },
    .flags = {
      .disp_active_low = 0,
      .refresh_on_demand = 0,
      .fb_in_psram = true,
      .double_fb = true,
      .no_fb = 0,
      .bb_invalidate_cache = 0,
    },
  };
  esp_lcd_new_rgb_panel(&rgb_config, &panel_handle); 
  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);
}

void lcd_init() {
  st7701_reset();
  st7701_init();
  backlight_init();
}

void lcd_add_window(uint16_t Xstart, uint16_t Xend, uint16_t Ystart, uint16_t Yend, uint8_t *color) {
  Xend = Xend + 1;
  Yend = Yend + 1;
  if (Xend >= ESP_PANEL_LCD_WIDTH)
    Xend = ESP_PANEL_LCD_WIDTH;
  if (Yend >= ESP_PANEL_LCD_HEIGHT)
    Yend = ESP_PANEL_LCD_HEIGHT;
   
  esp_lcd_panel_draw_bitmap(panel_handle, Xstart, Ystart, Xend, Yend, color);
}

void backlight_init() {
  ledcAttach(LCD_BACKLIGHT_PIN, frequency, resolution);  
}

void set_backlight(uint8_t light) {
  if (light > backlight_max) {
    light = backlight_max;
  }

  uint32_t backlight = (light * 255) / backlight_max;
  ledcWrite(LCD_BACKLIGHT_PIN, backlight);
}


