#include "stationId.h"
#include <Arduino.h>
#include "Globals.h"
#include <ESP32-TWAI-CAN.hpp>
#include <Arduino.h>
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include "touch.h"
#include "matouch-display.h"
#include <WebServer.h>
#include "obd2.h"

#if defined(STATION_A)
  #warning "Compiling for Station_A"
#else
  #warning "Compiling for Station_B"
#endif

#include "matouch-pins.h"

/* ESP32 Arduino CAN RTOS Example
 *     This example will send and receive messages on the CAN bus using RTOS tasks
 *     
 *     When controlling a device over CAN bus that device may expect a CAN message to be received at a specific frequency.
 *     Using RTOS tasks simplifies sending dataframes and makes the timing more consistent.
 *
 *     An external transceiver is required and should be connected 
 *     to the CAN_tx and CAN_rx gpio pins specified by CANInit. Be sure
 *     to use a 3.3V compatable transceiver such as the SN65HVD23x
 *
 */

// Change the file "stationId.h" to select which station to build
// Station A is MaTouch 2.1" display
// Wiki:https://wiki.makerfabs.com/MaTouch_ESP32_S3_2.1_Rotary_TFT_with_Touch.html

// Station B is MaTouch 1.28" display (GC9A01)
// Wiki:https://wiki.makerfabs.com/MaTouch_ESP32_S3_Rotary_IPS_Display_1.28_GC9A01.html

// Both using CNJBoards Can bus adapter card:

// define how long to wait 
#define STARTUPDELAY 2500
#define MAXSCREENCOUNT 4

// use this for unique Id
u_int32_t chipId;

// Globals
unsigned long pm=0;
int counter = 0; // use to track current screen
int State;
int old_State;

int move_flag = 0;
int button_flag = 0;
int flesh_flag = 1;

// count frames
uint32_t txCount = 0;
uint32_t rxCount = 0;

// variables to handle the dial push button debounce etc
int buttonState;            // the current reading from the input pin
int buttonStateLong;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin
int shortButtonStateLatched = LOW;  // latch if button is pressed for short interval
int longButtonStateLatched = LOW;  // latch if pressed for long interval
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled 
unsigned long shortDebounceDelay = 80;    // the short debounce time
unsigned long longDebounceDelay = 1000;    // the long debounce time
unsigned long latchClearDelay = 500;    // the time to allow latched states to be consumed befor autonomous reset

// GLOBALS FOR PID VALUES
float locEngRPM = 0;
float locVehSpd = 0;          
float locEngCoolTemp = 0;          
float locFuelPres = 0;          
float locFuelTankLvl = 0;          

// forward declarations
void pin_init();
void encoder_irq();
void checkButton(void);
void setupCanBus(int8_t , int8_t );

// externs from otaWeb
extern void otaSetup(void);
extern WebServer server;
extern bool startUpDelayDone;
extern bool fileUploadStarted;

// can bus speed
#define CANSPEED 500 // set bus speed, use 250, 500, 800, 1000 mbps

/* RTOS priorities, higher number is more important */
#define CAN_TX_PRIORITY     3
#define CAN_RX_PRIORITY     1

#define CAN_TX_RATE_ms      500 // how fast to poll pid in msec

/* can frame structures for Tx and Rx */
CanFrame txFrame;
CanFrame rxFrame;

/* CAN RTOS callback functions */
void getOBD2(void *pvParameters);

/* CAN RTOS task handles */
static TaskHandle_t getOBD2task = NULL;

// setup serial, tasks, can bus etc
void setup() {
  uint8_t chipid [ 6 ];

  // derive a unique chip id from the burned in MAC address
  esp_efuse_mac_get_default ( chipid );
  for ( int i = 0 ; i < 6 ; i++ )
  chipId += ( chipid [ i ] << ( 7 * i ));

  Serial.begin(115200);
  delay(STARTUPDELAY);
  
  #if defined(STATION_A)
    Serial.println("ESP32-Arduino-CAN RTOS Example - MaTouch 2.1 Display");
    Serial.printf( "Station A Chip ID: %x\n" , chipId );
  #else
    Serial.println("ESP32-Arduino-CAN RTOS Example - MaTouch 1.28 Display\n");
    Serial.printf ( "Station B Chip ID: %x\n" , chipId );
  #endif
  
  // setup digital IO
  pin_init();

  // setup ota stuff
  otaSetup();

  #if defined(STATION_A)
    // I2C setup, only for 2.1"
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Serial.println( "Wire Setup done");
  #else
    // I2C setup
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    Serial.printf ( "Wire Setup done");
  #endif

  // setup the diaply
  doLvglInit();

  // helper function to init the can bus
  setupCanBus(CAN_TX, CAN_RX);

  /* setup can send RTOS task */
  xTaskCreatePinnedToCore(getOBD2,         /* callback function */
                          "CAN TX",        /* name of task */
                          4096,            /* stack size (bytes in ESP32, words in FreeRTOS */
                          NULL,            /* parameter to pass to function */
                          CAN_TX_PRIORITY, /* task priority (0 to configMAX_PRIORITES - 1 */
                          &getOBD2task,    /* task handle */
                          1);              /* CPU core, Arduino runs on 1 */

  // grab millis for startup delay
  pm = millis();
} // end setup

// helper function to init the Can bus
void setupCanBus(int8_t can_tx, int8_t can_rx){

  // start Can Bus stuff
  // Set pins for Can bus xciever, defines above
  ESP32Can.setPins(can_tx, can_rx);

  // setup some tx and rx queues
  ESP32Can.setRxQueueSize(64);
  ESP32Can.setTxQueueSize(64);

  // .setSpeed() and .begin() functions require to use TwaiSpeed enum,
  // but you can easily convert it from numerical value using .convertSpeed()
  // 500kBps is typical automotive can bus speed
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));

  // It is also safe to use .begin() without .end() as it calls it internally
  if(OBD2.begin()) {
      Serial.println("CAN bus started!");
  } else {
      Serial.println("CAN bus failed!");
  } // end if

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  } // end if
} // end setUPCanbus

// main processing loop, tx and rx are handled by tasks
// this handles IO, Display and OTA
void loop() {

  // flag startup delay
  if (millis()-pm > STARTUPDELAY){
    startUpDelayDone = true;
  } // end if

  // check for Input (PB)
  checkButton();

  // use button to reset counts
  if (shortButtonStateLatched == HIGH) {
    // reset counters
    txCount = 0;
    rxCount = 0;
    // reset latched
    shortButtonStateLatched = LOW;
  } // end if

  // web ota stuff, check for ota request
  server.handleClient();

  // for display processing
  lv_timer_handler(); 

  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1000));
  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  } // end if
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %lu\n", twaistatus.bus_error_count);
  } // end if
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("RX buffered: %lu\t", twaistatus.msgs_to_rx);
    Serial.printf("RX missed: %lu\t", twaistatus.rx_missed_count);
    Serial.printf("RX overrun %lu\n", twaistatus.rx_overrun_count);
  } // end if
} // end loop

/* function to poll obd2 pids */
void getOBD2(void *pvParameters) {
  
  // pid to poll
  uint8_t locPID;

  /* this task will run forever at frequency set above */
	for (;;) {
    float dbg = 0;
    switch(counter) {

        case 0:
          locPID = ENGINE_RPM;
          locEngRPM = OBD2.pidRead(locPID);
          dbg = locEngRPM;
          break;

        case 1:
          locPID = VEHICLE_SPEED;
          locVehSpd = OBD2.pidRead(locPID);          
          dbg = locVehSpd;
          break;

        case 2:
          locPID = ENGINE_COOLANT_TEMPERATURE;
          locEngCoolTemp = OBD2.pidRead(locPID);          
          dbg = locEngCoolTemp;
          break;

        case 3:
          locPID = FUEL_PRESSURE;
          locFuelPres = OBD2.pidRead(locPID);          
          dbg = locFuelPres;
          break;

        case 4:
          locPID = FUEL_TANK_LEVEL_INPUT;
          locFuelTankLvl = OBD2.pidRead(locPID);          
          dbg = locFuelTankLvl;
          break;
    } // end switch

    // debug code, dump the pid name and value
    Serial.print(OBD2.pidName(locPID));
    Serial.print(" = ");
    Serial.print(dbg);
    Serial.print(OBD2.pidUnits(locPID));
    Serial.println();

    // this is our pid polling frequency
    vTaskDelay(CAN_TX_RATE_ms/portTICK_PERIOD_MS);
	} // end for

  // if we get here, reboot
  esp_restart();
} // end getOBD2

// setup all the IO
void pin_init()
{
  #if defined(STATION_A)
    // dial PB
    pinMode(BUTTON_PIN,INPUT_PULLUP);

    // backlight display, leave on for now
    pinMode(TFT_BL,OUTPUT_OPEN_DRAIN);
    digitalWrite(TFT_BL, HIGH);

    // rotary encoder IO
    pinMode(ENCODER_CLK, INPUT_PULLUP);
    pinMode(ENCODER_DT, INPUT_PULLUP);
    old_State = digitalRead(ENCODER_CLK);
    attachInterrupt(ENCODER_CLK, encoder_irq, CHANGE);
    
    // setup tft backlight output
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    // for debounce init
    lastButtonState = digitalRead(BUTTON_PIN);
    buttonState = lastButtonState;
    buttonStateLong = lastButtonState;
  #else

    const int ledChannel = 0;
    const int ledPin = 5;
    const int frequency = 5000;
    const int resolution = 8;

    // try using pwm for backlight control
    pinMode(TFT_BLK, OUTPUT);
    //digitalWrite(TFT_BLK, HIGH);
    ledcSetup(ledChannel, frequency, resolution);
    ledcAttachPin(TFT_BLK, ledChannel);
    ledcWrite(ledChannel, 0xff);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(ENCODER_CLK, INPUT_PULLUP);
    pinMode(ENCODER_DT, INPUT_PULLUP);
    old_State = digitalRead(ENCODER_CLK);

    attachInterrupt(ENCODER_CLK, encoder_irq, CHANGE);

    // for debounce init
    lastButtonState = digitalRead(BUTTON_PIN);
    buttonState = lastButtonState;
    buttonStateLong = lastButtonState;
  #endif
} // end pin_init

// this debounces the dial PB and will latch both a short and long press
void checkButton(void){
  // read the state of the switch into a local variable:
  int reading = digitalRead(BUTTON_PIN);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  } // end if

  if ((millis() - lastDebounceTime) > shortDebounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // if we timeout on debounce then latch short button pressed
      if (buttonState == HIGH) {
        shortButtonStateLatched = HIGH;
        Serial.println("Dial button pressed, short debounce");
      } // end if
    } // end if
  } // end if

  if ((millis() - lastDebounceTime) > longDebounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonStateLong) {
      buttonStateLong = reading;

      // if we timeout on debounce then latch short button pressed
      if (buttonState == HIGH) {
        longButtonStateLatched = HIGH;
        Serial.println("Dial button pressed, long debounce");
      } // end if
    } // end if
  } // end if

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;

} // end checkButton

// irq handler for rotary encoder
void encoder_irq()
{
  State = digitalRead(ENCODER_CLK);
  if (State != old_State)
  {
    if (digitalRead(ENCODER_DT) == State)
    {
      // counter clockwise is -ve 
      counter--;
      if (counter < 0)
        counter =0;
    }
    else
    {
      // clockwise is +ve
      counter++;
      if (counter > MAXSCREENCOUNT)
        counter = MAXSCREENCOUNT;
    } // end if
    
  } // end if
  old_State = State; // the first position was changed
  move_flag = 1;
} // end encoder_irq