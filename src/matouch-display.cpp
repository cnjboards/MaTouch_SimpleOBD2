#define SCREEN_REFRESH 100 // how fast to update the main screen

#include "stationId.h"
#include <Arduino.h>
#include "matouch-pins.h"
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <WiFi.h>
#include "touch.h"
#include "Globals.h"

// Specific to GFX hardware setup
#define TOUCH_RST -1 // 38
#define TOUCH_IRQ -1 // 0
#define DISPLAY_DC GFX_NOT_DEFINED 
#define DISPLAY_CS 1 
#define DISPLAY_SCK 46 
#define DISPLAY_MOSI 0 
#define DISPLAY_MISO GFX_NOT_DEFINED

#define DISPLAY_DE 2 
#define DISPLAY_VSYNC 42 
#define DISPLAY_HSYNC 3 
#define DISPLAY_PCLK 45 
#define DISPLAY_R0 11 
#define DISPLAY_R1 15 
#define DISPLAY_R2 12 
#define DISPLAY_R3 16 
#define DISPLAY_R4 21 
#define DISPLAY_G0 39 
#define DISPLAY_G1 7 
#define DISPLAY_G2 47 
#define DISPLAY_G3 8 
#define DISPLAY_G4 48 
#define DISPLAY_G5 9 
#define DISPLAY_B0 4 
#define DISPLAY_B1 41 
#define DISPLAY_B2 5 
#define DISPLAY_B3 40 
#define DISPLAY_B4 6 
#define DISPLAY_HSYNC_POL 1 
#define DISPLAY_VSYNC_POL 1 
#define DISPLAY_HSYNC_FRONT_PORCH 10 
#define DISPLAY_VSYNC_FRONT_PORCH 10 
#define DISPLAY_HSYNC_BACK_PORCH 50 
#define DISPLAY_VSYNC_BACK_PORCH 20 
#define DISPLAY_HSYNC_PULSE_WIDTH 8 
#define DISPLAY_VSYNC_PULSE_WIDTH 8

#if defined(STATION_A)
  // for 2.1" MaTouch display
  Arduino_DataBus *bus = new Arduino_SWSPI( DISPLAY_DC /* DC */, DISPLAY_CS /* CS */, DISPLAY_SCK /* SCK */, DISPLAY_MOSI /* MOSI */, DISPLAY_MISO /* MISO */);

  Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel( DISPLAY_DE, DISPLAY_VSYNC , DISPLAY_HSYNC, DISPLAY_PCLK, DISPLAY_R0, DISPLAY_R1, DISPLAY_R2, 
        DISPLAY_R3, DISPLAY_R4, DISPLAY_G0, DISPLAY_G1, DISPLAY_G2, DISPLAY_G3, DISPLAY_G4, DISPLAY_G5, DISPLAY_B0, DISPLAY_B1, DISPLAY_B2, DISPLAY_B3, 
        DISPLAY_B4, DISPLAY_HSYNC_POL, DISPLAY_HSYNC_FRONT_PORCH, DISPLAY_HSYNC_PULSE_WIDTH, DISPLAY_VSYNC_BACK_PORCH, DISPLAY_VSYNC_POL, DISPLAY_VSYNC_FRONT_PORCH, 
        DISPLAY_VSYNC_PULSE_WIDTH, DISPLAY_VSYNC_BACK_PORCH);

  Arduino_RGB_Display *gfx = new Arduino_RGB_Display( 480 /* width */, 480 /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */, bus, GFX_NOT_DEFINED /* RST */, 
        st7701_type5_init_operations, sizeof(st7701_type5_init_operations));

  /*Change to your screen resolution*/
  static const uint16_t screenWidth = 480;
  static const uint16_t screenHeight = 480;
#else
  // MaTouch 1.28" display
  Arduino_ESP32SPI *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, TFT_MISO, HSPI, true); // Constructor
  Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RES, 0 /* rotation */, true /* IPS */);

  /*Change to your screen resolution*/
  static const uint16_t screenWidth = 240;
  static const uint16_t screenHeight = 240;
#endif

#define COLOR_NUM 5
int ColorArray[COLOR_NUM] = {WHITE, BLUE, GREEN, RED, YELLOW};

/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

// LVGL stuff

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
    gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

    lv_disp_flush_ready(disp);
} // end my_disp_flush

/* Read the touchscreen */
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    int touchX = 0, touchY = 0;

    if (read_touch(&touchX, &touchY) == 1)
    {
        data->state = LV_INDEV_STATE_PR;

        data->point.x = (uint16_t)touchX;
        data->point.y = (uint16_t)touchY;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
} // end my_touchpad_read

// read the dial/pb bezle
void my_encoder_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    if (counter > 0)
    {
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = LV_KEY_LEFT;
        counter--;
        move_flag = 1;
    } // end if

    else if (counter < 0)
    {
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = LV_KEY_RIGHT;
        counter++;
        move_flag = 1;
    } // end else if
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    } // end else
} // end my_encoder_read

// end lvgl stuff

// objects for display
static lv_style_t border_style;
static lv_style_t indicator_style;
static lv_style_t smallIndicator_style;
static lv_timer_t *updateMainScreenTimer;
static lv_obj_t *Screen1;
static lv_obj_t *engineRpmGauge;
static lv_meter_indicator_t *engineRpmIndic;
static lv_obj_t *engineRPMIndicator;
static lv_obj_t *engineRPMText;
static lv_obj_t *sogIndicator;
static lv_obj_t *sogText;
static lv_obj_t *screenText;
static lv_obj_t *screenText1;
static lv_obj_t *screenText2;
static lv_obj_t *screenText3;
static lv_obj_t *screenText4;

// forward declarations
static void setStyle(void);
static void buildScreen(void);
static void updateMainScreen(lv_timer_t *);
static void buildRPMGuage(void);

// init lvgl graphics
void doLvglInit(){

    // print out version
    String LVGL_Arduino = "LVGL: ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
    Serial.println( LVGL_Arduino );
    
    // Init display hw
    gfx->begin();

    // init graphics lib
    lv_init();
    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /* Initialize the display */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    // start of UI code here
    setStyle();

    // create simple screen
    buildScreen();

    // activate the screen on startup
    lv_scr_load(Screen1);

    // start timer for the screen refresh 
    updateMainScreenTimer = lv_timer_create(updateMainScreen, SCREEN_REFRESH, NULL);

} // end do_lvgl_init

// setup the lvgl styles for this project
static void setStyle() {  
  // overall style 
  lv_style_init(&border_style);
  lv_style_set_border_width(&border_style, 2);
  lv_style_set_border_color(&border_style, lv_color_black());
  #if defined(STATION_A)
    lv_style_set_text_font(&border_style, &lv_font_montserrat_24);
  #else
    lv_style_set_text_font(&border_style, &lv_font_montserrat_14);

    // style for indicator readouts
    lv_style_init(&indicator_style);
    lv_style_set_border_width(&indicator_style, 2);
    lv_style_set_radius(&indicator_style,4);
    lv_style_set_border_color(&indicator_style, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_text_font(&indicator_style, &lv_font_montserrat_20);

    // style for indicator readouts
    lv_style_init(&smallIndicator_style);
    lv_style_set_border_width(&smallIndicator_style, 1);
    lv_style_set_radius(&smallIndicator_style,2);
    lv_style_set_border_color(&smallIndicator_style, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_text_font(&indicator_style, &lv_font_montserrat_10);
  #endif


} // end setstyle

// decide later if we want some edge boundary
#define MS_HIEGHT (screenHeight - 0)
#define MS_WIDTH (screenWidth - 0) 
// x direction +ve is right, -ve is left
// y direction +ve is down, -ve is up
#if defined(STATION_A)
    #define XOFF -150
    #define YBASE -70
    #define YINC 30
#else
    #define XOFF -85
    #define YBASE -35
    #define YINC 20
#endif

// screen builder
static void buildScreen(void) {
  // main screen frame
  Screen1 = lv_obj_create(NULL);
  lv_obj_add_style(Screen1, &border_style, 0);
  lv_obj_set_size(Screen1, MS_WIDTH, MS_HIEGHT);
  lv_obj_align(Screen1, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_scrollbar_mode(Screen1, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(Screen1, LV_OBJ_FLAG_SCROLLABLE);

  // build the rpm screen
  buildRPMGuage();

  #if 0
  // display AP information
  screenText = lv_label_create(Screen1);

  // x direction +ve is right, -ve is left
  // y direction +ve is down, -ve is up
  screenText1 = lv_label_create(Screen1);
  lv_obj_align_to(screenText1, Screen1, LV_ALIGN_CENTER, XOFF, YBASE);
  lv_label_set_text(screenText1, "AP Station ID:");

  lv_obj_align_to(screenText, Screen1, LV_ALIGN_CENTER, XOFF, (YBASE + YINC));
  String buf = WiFi.softAPSSID();
  lv_label_set_text(screenText, buf.c_str());

  screenText2 = lv_label_create(Screen1);
  lv_obj_align_to(screenText2, Screen1, LV_ALIGN_CENTER, XOFF, (YBASE + (2*YINC)));
  lv_label_set_text_fmt(screenText2, "Can Bus Message TX: %d", txCount);

  screenText3 = lv_label_create(Screen1);
  lv_obj_align_to(screenText3, Screen1, LV_ALIGN_CENTER, XOFF, (YBASE + (3*YINC)));
  lv_label_set_text_fmt(screenText3, "Can Bus Message RX: %d", rxCount);

  screenText4 = lv_label_create(Screen1);
  lv_obj_align_to(screenText4, Screen1, LV_ALIGN_CENTER, XOFF, (YBASE + (4*YINC)));
  lv_label_set_text_fmt(screenText4, "Dial Count: %d", counter);
  #endif

} // end buildScreen

// locate engine rpm guage on the main screen relative to centre of Screen1 object
#define RPM_GUAGE_HIEGHT 235
#define RPM_GUAGE_WIDTH 235
#define ENGINE_RPM_GUAGE_XOFFSET -52
#define ENGINE_RPM_GUAGE_YOFFSET -45

// build the rpm guage on the main screen
static void buildRPMGuage() {
  // create and rpm guage on main display
  engineRpmGauge = lv_meter_create(Screen1);
  // this removes the outline, allows for more usable space
  lv_obj_remove_style(engineRpmGauge, NULL, LV_PART_MAIN);
  
  // Locate and set size of rpm guage
  lv_obj_align_to(engineRpmGauge, Screen1, LV_ALIGN_CENTER, ENGINE_RPM_GUAGE_XOFFSET, ENGINE_RPM_GUAGE_YOFFSET);
  lv_obj_set_size(engineRpmGauge, RPM_GUAGE_WIDTH, RPM_GUAGE_HIEGHT);
  
  /*Add a scale first*/
  lv_meter_scale_t * scale = lv_meter_add_scale(engineRpmGauge);
  lv_meter_set_scale_range(engineRpmGauge, scale, 0, 60, 230, 155);
  lv_meter_set_scale_ticks(engineRpmGauge, scale, 61, 4, 20, lv_palette_main(LV_PALETTE_GREY));
  lv_meter_set_scale_major_ticks(engineRpmGauge, scale, 10, 6, 30, lv_color_black(), 15);
  
  /* Green range - arc */
  engineRpmIndic = lv_meter_add_arc(engineRpmGauge, scale, 3, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_meter_set_indicator_start_value(engineRpmGauge, engineRpmIndic, 0);
  lv_meter_set_indicator_end_value(engineRpmGauge, engineRpmIndic, 30);

  /* Green range - ticks */
  engineRpmIndic = lv_meter_add_scale_lines(engineRpmGauge, scale, lv_palette_main(LV_PALETTE_GREEN), lv_palette_main(LV_PALETTE_GREEN),false, 0);
  lv_meter_set_indicator_start_value(engineRpmGauge, engineRpmIndic, 0);
  lv_meter_set_indicator_end_value(engineRpmGauge, engineRpmIndic, 30);

  /* Red range - arc */
  engineRpmIndic = lv_meter_add_arc(engineRpmGauge, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
  lv_meter_set_indicator_start_value(engineRpmGauge, engineRpmIndic, 30);
  lv_meter_set_indicator_end_value(engineRpmGauge, engineRpmIndic, 60);

  /* Red range - arc */
  engineRpmIndic = lv_meter_add_scale_lines(engineRpmGauge, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false,0);
  lv_meter_set_indicator_start_value(engineRpmGauge, engineRpmIndic, 35);
  lv_meter_set_indicator_end_value(engineRpmGauge, engineRpmIndic, 60);

  /* Add the indicator needle and set initial value*/
  engineRpmIndic = lv_meter_add_needle_line(engineRpmGauge, scale, 6, lv_palette_main(LV_PALETTE_BLUE_GREY), -60);
  lv_meter_set_indicator_value(engineRpmGauge, engineRpmIndic, 0);

  /* some static text on the display */
  engineRPMText = lv_label_create(engineRpmGauge);
  lv_obj_align_to(engineRPMText, engineRpmGauge, LV_ALIGN_CENTER, 20, -45);
  lv_obj_set_style_text_font(engineRPMText, &lv_font_montserrat_20, 0);
  lv_label_set_text(engineRPMText,"RPM \nx100");

  /* display the rpm in numerical format as well */
  engineRPMIndicator = lv_label_create(engineRpmGauge);
  lv_obj_add_style(engineRPMIndicator, &indicator_style, 0);
  lv_obj_align_to(engineRPMIndicator, engineRpmGauge, LV_ALIGN_CENTER, -38, 18);
  lv_obj_set_style_text_font(engineRPMIndicator, &lv_font_montserrat_34, 0);
  lv_label_set_text_fmt(engineRPMIndicator, " %04d ", 0);

  #if 0
  /* some static text on the display */
  sogText = lv_label_create(engineRpmGauge);
  lv_obj_align_to(sogText, engineRpmGauge, LV_ALIGN_CENTER, -50, 77);
  lv_obj_set_style_text_font(sogText, &lv_font_montserrat_20, 0);
  lv_label_set_text(sogText,"SOG");

  /* display the sog in numerical format as well */
  sogIndicator = lv_label_create(engineRpmGauge);
  lv_obj_add_style(sogIndicator, &indicator_style, 0);
  lv_obj_align_to(sogIndicator, engineRpmGauge, LV_ALIGN_CENTER, -10, 70);
  lv_obj_set_style_text_font(sogIndicator, &lv_font_montserrat_20, 0);
  lv_label_set_text_fmt(sogIndicator, "%03.1f knts", (float)(locSOG * 1.944));
  #endif

} // end buildrpmguage

// helper to update the main display including header bar
static void updateMainScreen(lv_timer_t *timer)
{
    // update the meter
    lv_meter_set_indicator_value(engineRpmGauge, engineRpmIndic, (uint32_t)(locEngRPM/100));
    // display engine rpm
    lv_label_set_text_fmt(engineRPMIndicator, " %04d ", (uint32_t)(locEngRPM));
  #if 0
  // update counts on the screen
  lv_label_set_text_fmt(screenText2, "Can Bus Message TX: %d", txCount);
  lv_label_set_text_fmt(screenText3, "Can Bus Message RX: %d", rxCount);
  lv_label_set_text_fmt(screenText4, "Dial Count: %d", counter);
  #endif
} // end updateMainScreen
