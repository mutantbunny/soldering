#include "SPI.h"
#include "TFT_22_ILI9225.h"
#include <PID_v1.h>
#include <stdlib.h>
#include <string.h>  // use C-style strings (\0-terminated char arrays) for lower memory usage
#include <stddef.h>  // for offsetof() macro
#include <EEPROM.h>

//  Allow turning save to EEPROM feature on or off
#define SAVE_TO_EEPROM

//  Allow turning read from EEPROM feature on or off
#define LOAD_FROM_EEPROM

//  Define debug serial printing (from answer by Sergio in https://stackoverflow.com/questions/1941307/debug-print-macro-in-c)
//  #define DEBUG
#ifdef DEBUG
  #define DEBUG_MSG Serial.println
#else
  #define DEBUG_MSG(...)
#endif

/* Use inline functions to tidy up the loop() section
 * These functions are used at most twice in the code, so it makes sense to make them inline and avoid the function call overhead
 * Force inline syntax from https://www.reddit.com/r/arduino/comments/48o0u5/what_is_inline_keyword_supposed_to_do_exactly/ */

inline void computePID(void)                  __attribute__((always_inline));
inline void debounceSwitches(void)            __attribute__((always_inline));
inline void drawCurrentTemp(void)             __attribute__((always_inline));
inline void drawEntireScreen(void)            __attribute__((always_inline));
inline void drawGoalTemp(void)                __attribute__((always_inline));
inline void ensure_valid(void)                __attribute__((always_inline));
inline void handleMenu(void)                  __attribute__((always_inline));
inline void handleSerial(void)                __attribute__((always_inline));
inline void handleStandby(void)               __attribute__((always_inline));
inline void pin9PWMWrite(uint8_t)             __attribute__((always_inline));

// =============================== EEPROM variables ==========================================

struct __attribute__((__packed__)) EEPROM_store {
    double  kp;           //  Proportional parameter for the PID control
    double  ki;           //  Integral parameter for the PID control
    double  kd;           //  Derivative parameter for the PID control
    double  tempOffset;   //  ADC temperature conversion offset
    double  tempGain;     //  ADC temperature conversion gain
    double  curGoalTemp;  //  Goal temperature in normal operating mode
    double  standbyTemp;  //  Goal temperature in standby operating mode
    uint8_t brightness;   //  LCD brightness
    bool    serialMon;    //  Whether serial monitoring should be enabled
  } store;  // 72 bytes packed

  //  Defaults in case values stored in EEPROM are invalid (checked by ensure_valid() function)
  constexpr double  DEFAULT_KP           =  10.0;
  constexpr double  DEFAULT_KI           =   0.0;
  constexpr double  DEFAULT_KD           =   0.0;
  constexpr double  DEFAULT_TEMP_OFFSET  =  25.0;  //  Initial offset and gain parameters from
  constexpr double  DEFAULT_TEMP_GAIN    =  0.53;  //  https://github.com/ConnyCola/SolderingStation/blob/master/arduino/SolderStation.ino
  constexpr double  DEFAULT_GOAL_TEMP    = 270.0;
  constexpr double  DEFAULT_STANDBY_TEMP = 100.0;

  constexpr uint8_t BASE_ADDRESS         =     0;  //  Where to store struct in EEPROM 
  constexpr uint8_t CYCLES_BEFORE_SAVE   =    50;  //  Number of main loop iterations before saving goal temperature to EEPROM

  #ifdef SAVE_TO_EEPROM
    bool    mustSaveGoalTemp    = false;           //  indicates if goal temperature must be saved to EEPROM
    uint8_t saveGoalTempCounter =     0;           //  Counter of main loop iterations for saving goal temperature to EEPROM
  #endif
    
  //  Macro to get address of a struct member
  #define get_address(st, member) BASE_ADDRESS + offsetof(st, member)

// ====================================== TFT setup ========================================

// Constants for TFT library
constexpr uint8_t  TFT_LED =  6;  //  Backlight pin
constexpr uint8_t  TFT_RST =  7;  //  Reset pin
constexpr uint8_t  TFT_CS  = 10;  //  SPI Chip Select pin
constexpr uint8_t  TFT_RS  =  8;  //  Register Select pin
constexpr uint8_t  TFT_CLK = 13;  //  SPI clock pin
constexpr uint8_t  TFT_SDI = 11;  //  SPI MOSI pin

constexpr uint16_t ACTIVE_MENU_TEXT_COLOR   = COLOR_BLACK;
constexpr uint16_t ACTIVE_MENU_BG_COLOR     = COLOR_WHITE;
constexpr uint16_t INACTIVE_MENU_TEXT_COLOR = COLOR_WHITE;

constexpr uint16_t GOAL_TEXT_Y   =  10;
constexpr uint16_t GOAL_TEMP_Y   =  32;
constexpr uint16_t ACTUAL_TEXT_Y = 105;
constexpr uint16_t ACTUAL_TEMP_Y = 127;
constexpr uint16_t MENU_Y        = 200;
constexpr uint16_t MAX_X         = 176;
constexpr uint16_t MAX_Y         = 220;

constexpr double   LOW_TEMP_LIM  = 5.;
constexpr double   HIGH_TEMP_LIM = 5.;

// Custom font for temperature display
extern const uint8_t Droid_Sans_Mono40x57[];

// TFT object
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED, 0);

// ============================== Rotary encoder setup ========================================
// Rotary encoder and standby pins
constexpr uint8_t  ENC_DT         =  2;  //  DT rotary encoder quadrature pin
constexpr uint8_t  ENC_CLK        =  3;  //  CLK rotary encoder quadrature pin
constexpr uint8_t  ENC_SW         =  4;  //  SW rotary encoder push-button switch pin 
constexpr uint8_t  STDBY          =  5;  //  Standby switch (connected to soldering tip stand)
constexpr uint32_t DEBOUNCE_DELAY = 30;  //  Debounce time

// volatile variables used by encoder ISRs PinA() and PinB() by SimonM83
volatile byte  aFlag       =      0;  //  expect rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte  bFlag       =      0;  //  expect a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte  reading     =      0;  //  temporary variable for values read from interrupt pins
volatile bool  encoderUp   =  false;  //  signal key-up input from user is pending evaluation
volatile bool  encoderDown =  false;  //  signal key-down input from user is pending evaluation

//Rotary encoder and menu control variables
bool     goalTempChanged    = false;  //  signal goal temperature has changed and display must be updated
bool     debouncedStandby   =  HIGH;  //  debounced standy switch pin value
bool     lastStandby        =  HIGH;  //  last standby switch pin value evaluated by debounceSwitches() function
bool     debouncedEncSwitch =  HIGH;  //  debounced encoder push-buton switch pin value
bool     lastEncSwitch      =  HIGH;  //  last encoder push-buton switch pin value evaluated by debounceSwitches() function
uint32_t lastStdbyDebcTime  =     0;  //  debounce starting time for standy switch pin
uint32_t lastEncSwDebcTime  =     0;  //  debounce starting time for encoder push-buton switch pin
bool     inStandbyMode      = false;  //  is station currently in standby mode?
bool     buttonPress        = false;  //  signal push-button switch input from user is pending evaluation

// ============================ Menu setup and strings ========================================

uint8_t  menuLevel        =     0;  //  current level of the menu
uint8_t  menuOption       =     0;  //  current option on current level of the menu
bool     serialMon        =  true;  //  if true, current and goal temperatures are sent to serial port for plotting
bool     cmdComplete      = false;  //  indicates a serial command is ready to be read
char     serialCmd[10]    =    "";  //  serial command string

// Menu option strings
constexpr char strMenu0[]    = "Menu";       
constexpr char strMenu1_0[]  = "Stdby Temp";
constexpr char strMenu1_1[]  = "Brightness";
constexpr char strMenu1_2[]  = "Monitor";
constexpr char strMenu1_3[]  = "Back";

constexpr char strMenuOn[]   = "On";
constexpr char strMenuOff[]  = "Off";

// Temperature indication strings
constexpr char strGoalTemp[]         = "Goal";
constexpr char strActualTemp[]       = "Actual           ";
constexpr char strActualTempStdby[]  = "Actual (STDBY)";

// Array of menu option strings
const char *const menuLevel1[] = {strMenu1_0, strMenu1_1, strMenu1_2, strMenu1_3};

char tmpStr[18];  //  temporary variable for string operations

// ============================== Arduino PID setup ===========================================

// Arduino PID library parameters
double   currentTemp        =   0.;        //  Current temperature (input)
double   heaterOutput       =   0.;        //  Output to the heater control output PWM pin

// Global variables for PID control
double   activeGoalTemp;                   //  Goal temperature in active (not standby) mode
uint8_t  PIDMode        =           1;     //  PID mode (full on, full off or automatic)
int      adcValue       =           0;
bool     overheat       =       false;

// PID control constants
constexpr uint8_t  MIN_PWM       =      0;  //  Minimum heater output value
constexpr uint8_t  MAX_PWM       =    255;  //  Maximum heater output value
constexpr double   FULL_ON_FACT  =    0.2;  //  if the temperature is less than FULL_ON_FACT times current temperature, set output to MAX_PWM
constexpr double   FULL_OFF_FACT =    1.2;  //  if the temperature is more than FULL_OFF_FACT times current temperature, set output to MIN_PWM
constexpr double   MAX_TEMP_C    =  350.0;  //  Maximum allowed setpoint temperature
constexpr double   MIN_TEMP_C    =   50.0;  //  Minimum allowed setpoint temperature
constexpr double   TEMP_OVERHEAT =  400.0;

// Pins for reading temperature and controlling heater
constexpr uint8_t HEATER         =   9;
constexpr uint8_t TEMP_SENSE     =  A1;

// Arduino PID object
PID solderPID(&currentTemp, &heaterOutput, &store.curGoalTemp, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);

unsigned long loopDuration = 0;

// =============================== Initialization ============================================
void setup() {
  
  pinMode(HEATER, OUTPUT);
  digitalWrite(HEATER,0);
  pinMode(TEMP_SENSE, INPUT);  
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(STDBY, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), PinA, RISING);   //  set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), PinB, RISING);  //  set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)

  #ifdef LOAD_FROM_EEPROM
    EEPROM.get(BASE_ADDRESS, store);
    ensure_valid();
  #else
    store.kp          = DEFAULT_KP;
    store.ki          = DEFAULT_KI;
    store.kd          = DEFAULT_KD;
    store.tempOffset  = DEFAULT_TEMP_OFFSET;
    store.tempGain    = DEFAULT_TEMP_GAIN;  
    store.curGoalTemp = DEFAULT_GOAL_TEMP;
    store.standbyTemp = DEFAULT_STANDBY_TEMP;
    store.brightness  = 255;
    store.serialMon   = true;
  #endif

  tft.begin();
  tft.setBacklightBrightness(store.brightness);
  tft.setOrientation(0);
  tft.setBackgroundColor(COLOR_BLACK);
  drawEntireScreen();
  tft.setBacklight(true);

  /* Timer/Counter1 Control Registers
   * Bit          |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
   * TCCR1A       |  Port A   |  Port B   | Read-Only | Mode low  |
   * TCCR1B       | Inp Capt  | RO  | Mode high |    Prescaler    |
   *
   * Configuration from RTM TimerCalc (https://runtimemicro.com/Projects/WYSIWYG-Arduino-PWM-Code-Generator)
   * and ATMega328P datasheet (https://www.microchip.com/wwwproducts/en/ATmega328P)
   * WARNING: Code below is specific to heater output on pin 9!
   */
   
  TCCR1B = B00011000;                    //  Temporarily stop timer 1 and set high bits of mode 14 (Fast PWM, ICR top)
  TCCR1A = B10000010;                    //  Set port A (pin 9) to non-inverting PWM mode, port B to off and low bits of mode 14
  ICR1 = 800-1;                          //  Set top of PWM waveform
  OCR1A  = (int) (ICR1 * 0,25);          //  Set initial heater output to a small value
  TCNT1  = 0x0;                          //  Reset PWM counter to avoid invalid state
  pinMode(9, OUTPUT);                    //  Confirm pin 9 is set to output mode
  TCCR1B |= 1;                           //  Set prescaler to 1 and re-start timer 1 

  Serial.begin(115200);
  DEBUG_MSG("Startup");
  
  //  PID controller setup
  PIDMode = 1;
  solderPID.SetTunings(store.kp, store.ki, store.kd);
  solderPID.SetMode(AUTOMATIC);
  solderPID.SetSampleTime(100);
  solderPID.SetOutputLimits(MIN_PWM, MAX_PWM);
  activeGoalTemp = store.curGoalTemp;
  goalTempChanged = true;
}


// =============================== Main loop ============================================
void loop() {  
  debounceSwitches();    
  handleMenu();    
  handleStandby();
  computePID();    
  handleSerial();    
  
  if (goalTempChanged) {
    drawGoalTemp();
    if (!inStandbyMode) {
      store.curGoalTemp = activeGoalTemp;
    }
    if (store.serialMon) {
      Serial.print('G');
      Serial.println(store.curGoalTemp);
    }
    goalTempChanged = false;
  }    
  
  drawCurrentTemp();

  #ifdef SAVE_TO_EEPROM
    if (mustSaveGoalTemp) {
      if (saveGoalTempCounter != CYCLES_BEFORE_SAVE) {
        saveGoalTempCounter++;
      }
      else {
        EEPROM.put(get_address(EEPROM_store, curGoalTemp), activeGoalTemp);
        mustSaveGoalTemp = false;
      }
    }
  #endif
}

// =========================== Utility functions ====================================

/*  Draws the menu bar on the bottom of the screen.
 *  
 *  Parameters:
 *  s: C-style string with the text to be drawn at the menu bar.
 *  
 *  isActive: boolean specifying whether the menu should be drawn with colors 
 *            indicating the menu bar is active (if it is selected by the user) or 
 *            inactive (not selected).
 */
void drawMenu(const char* s, bool isActive) {
  tft.setFont((uint8_t*) Terminal12x16, false);

  if (isActive) {
    tft.setBackgroundColor(ACTIVE_MENU_BG_COLOR);
    tft.fillRectangle(0, MENU_Y, MAX_X, MAX_Y, ACTIVE_MENU_BG_COLOR);
    tft.drawText(10, MENU_Y, F("<"), ACTIVE_MENU_TEXT_COLOR);
    tft.drawText((MAX_X - tft.getTextWidth(s)) >> 1, MENU_Y, s, ACTIVE_MENU_TEXT_COLOR);
    tft.drawText(MAX_X - 22, MENU_Y, F(">"), ACTIVE_MENU_TEXT_COLOR);
    tft.setBackgroundColor(COLOR_BLACK);
  }
  
  else {
    tft.fillRectangle(0, MENU_Y, MAX_X, MAX_Y, COLOR_BLACK);
    tft.drawText((MAX_X - tft.getTextWidth(s)) >> 1, MENU_Y, s, INACTIVE_MENU_TEXT_COLOR);
  }  
  
  tft.setFont((uint8_t*) Droid_Sans_Mono40x57, true);
}

/*  Draws the numbers indicating the current goal temperature value.
 */
void drawGoalTemp() {
  uint16_t currentX = 10;
  
  if (activeGoalTemp >= 100) {
    currentX = 10;
  }
  
  else if (activeGoalTemp >= 10) {
    currentX = 51;
    tft.fillRectangle(10, GOAL_TEMP_Y, 50, 102, COLOR_BLACK);
  }
  
  else {
    currentX = 92;
    tft.fillRectangle(10, GOAL_TEMP_Y, 91, 102, COLOR_BLACK);
  }
  
  tft.drawText(currentX, GOAL_TEMP_Y, itoa((uint16_t) activeGoalTemp, tmpStr, 10), COLOR_TURQUOISE);
  tft.drawText(133, GOAL_TEMP_Y, ":", COLOR_TURQUOISE);
}

/*  Draws the numbers indicating the current tip temperature value.
 */
void drawCurrentTemp() {
  uint16_t currentX = 10;
  uint16_t currentColor;

  if (currentTemp < store.curGoalTemp - LOW_TEMP_LIM) currentColor = COLOR_GOLD;
  else if (currentTemp > store.curGoalTemp + HIGH_TEMP_LIM) currentColor = COLOR_TOMATO;
  else currentColor = COLOR_LIGHTGREEN;

  if (currentTemp >= 100.) {
    currentX = 10;
  }
  else if (currentTemp >= 10.) {
    currentX = 51;
    tft.fillRectangle(10, ACTUAL_TEMP_Y, 50, 186, COLOR_BLACK);
  }
  else {
    currentX = 92;
    tft.fillRectangle(10, ACTUAL_TEMP_Y, 91, 186, COLOR_BLACK);
  }
  tft.drawText(currentX, ACTUAL_TEMP_Y, itoa((uint16_t) currentTemp, tmpStr, 10), currentColor);
  tft.drawText(133, ACTUAL_TEMP_Y, ":", currentColor);
}

/*  Draws the entire screen (temperature description text, temperature values and menu).
 */
void drawEntireScreen() {
  tft.setFont((uint8_t*) Terminal12x16, false);
  tft.drawText(10, GOAL_TEXT_Y, strGoalTemp, COLOR_WHITE);
  tft.drawText(10, ACTUAL_TEXT_Y, strActualTemp, COLOR_WHITE);
  drawMenu(strMenu0, false);
  drawGoalTemp();
  drawCurrentTemp();
}

/*  Reads first rotary encoder input using interrupts.
 *  Slightly modified from encoder interrupt service routines by SimonM83
 *  (described on https://www.instructables.com/id/Improved-Arduino-Rotary-Encoder-Reading/).
 */
void PinA() {
  cli();                                //  stop interrupts happening before we read pin values
  reading = PIND & 0xC;                 //  read all eight pin values then strip away all but pinA and pinB's values
  //  check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
  if (reading == B00001100 && aFlag) {
    encoderUp = true;                   //  decrement the encoder's position count
    bFlag = 0;                          //  reset flags for the next turn
    aFlag = 0;                          //  reset flags for the next turn
  }
  //  signal that we're expecting pinB to signal the transition to detent from free rotation
  sei();                                //  restart interrupts
  else if (reading == B00000100) bFlag = 1;
}
/*  Reads second rotary encoder input using interrupts.
 *  Slightly modified from encoder interrupt service routines by SimonM83
 *  (described on https://www.instructables.com/id/Improved-Arduino-Rotary-Encoder-Reading/).
 */
void PinB() {
  cli();                                //  stop interrupts happening before we read pin values
  reading = PIND & 0xC;                 //  read all eight pin values then strip away all but pinA and pinB's values
  //  check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
  if (reading == B00001100 && bFlag) {
    encoderDown = true;                 //  increment the encoder's position count
    bFlag = 0;                          //  reset flags for the next turn
    aFlag = 0;                          //  reset flags for the next turn
  }
  //  signal that we're expecting pinA to signal the transition to detent from free rotation
  else if (reading == B00001000) aFlag = 1;
  sei();                                //  restart interrupts
}

/*  Reads and debounces the input from the switch embedded in the rotary encoder and the standby "switch" 
 *  without using interrupts (Arduino Nano has two interrupts, both are used by rotary encoder inputs).
 *  Based on code from https://www.arduino.cc/en/Tutorial/Debounce.
 */
void debounceSwitches() {
  //  debounce standby input
  int reading = digitalRead(STDBY);
  if (reading != lastStandby) {
    lastStdbyDebcTime = millis();
  }
  if ((millis() - lastStdbyDebcTime) > DEBOUNCE_DELAY) {
    if (reading != debouncedStandby) {
      debouncedStandby = reading;
      DEBUG_MSG(F("Standby: "));
      DEBUG_MSG(debouncedStandby);
    }
  }
  lastStandby = reading;
  //  debounce encoder switch
  reading = digitalRead(ENC_SW);
  if (reading != lastEncSwitch) {
    lastEncSwDebcTime = millis();
  }
  if ((millis() - lastEncSwDebcTime) > DEBOUNCE_DELAY) {
    if (reading != debouncedEncSwitch) {
      debouncedEncSwitch = reading;
      if (debouncedEncSwitch == HIGH) {
        buttonPress = true;
      }
    }
  }
  lastEncSwitch = reading;
}

/*  Reads temperature from thermocouple, calculates output and activates the heater for a set period.
 */
void computePID() {

  //  Wait before read technique from https://github.com/ConnyCola/SolderingStation/
  pin9PWMWrite(0);
  delay(50);   
  adcValue = analogRead(TEMP_SENSE);
  currentTemp = adcValue * store.tempGain + store.tempOffset;
  
  if (currentTemp < TEMP_OVERHEAT) {
    overheat = false;
    pin9PWMWrite(heaterOutput);
  }
  else {
    overheat = true;
    DEBUG_MSG(F("Overheat"));
    pin9PWMWrite(0);
  }
  
  if (currentTemp > store.curGoalTemp * FULL_OFF_FACT) {
    if (PIDMode != 0) {
      DEBUG_MSG(F("Full off"));
      solderPID.SetMode(MANUAL);
      PIDMode = 0;
    }
    heaterOutput = MIN_PWM;
  }
  else if (currentTemp < (store.curGoalTemp * FULL_ON_FACT)) {
    if (PIDMode != 2) {
      DEBUG_MSG(F("Full on"));
      solderPID.SetMode(MANUAL);
      PIDMode = 2;
    }
    heaterOutput = MAX_PWM;
  }
  else {
    if (PIDMode != 1) {
      DEBUG_MSG(F("PID on"));
      solderPID.SetMode(AUTOMATIC);
      PIDMode = 1;
    }
  }
  solderPID.Compute();
  pin9PWMWrite(heaterOutput);
   analogWrite(HEATER, heaterOutput);
}

/*  Handles the user interface, either directly setting the goal temperature or 
 *  interacting through the main menu and submenus.
 *  This includes drawing the appropriate menu text, adjusting settings and
 *  saving them to the EEPROM, if the SAVE_TO_EEPROM option is defined.
 *  In the case of changing the goal temperature, a counter is started that saves
 *  the changed setting to the EEPROM after a number of main loop cycles, to avoid
 *  repetitive writes to the EEPROM (255 cycles should take roughly one minute).
 */
void handleMenu() {
  if (buttonPress) {
    switch (menuLevel) {
      case 0:
        //  Outside menu -> go to main menu
        ++menuLevel;
        menuOption = 0;
        drawMenu(strMenu1_0, true);
        break;
      case 1:
        //  Inside main menu -> go to one of the options
        switch (menuOption) {
          case 0:
            //  Standby temperature -> show standby temperature value
            drawMenu(itoa((uint16_t) store.standbyTemp, tmpStr, 10), true);
            ++menuLevel;
            break;
          case 1:
            //  Brightness -> show brightness value
            drawMenu(itoa(store.brightness, tmpStr, 10), true);
            ++menuLevel;
            break;
          case 2:
            //  Serial monitor -> go to On/Off options
            drawMenu((store.serialMon) ? strMenuOn : strMenuOff, true);
            ++menuLevel;
            break;
          default:
            //  Back -> go out of menu
            drawMenu(strMenu0, false);
            menuLevel = 0;
        }
        break;
      
      //  Inside one of the options
      case 2:
      //  Properties are already set, save option to EEPROM and just go out of menu
      #ifdef SAVE_TO_EEPROM
        switch (menuOption) {
          case 0:
            //  Save standby temperature
            EEPROM.put(get_address(EEPROM_store, standbyTemp), store.standbyTemp);
            break;
          case 1:
            //  Save brightness value
            EEPROM.put(get_address(EEPROM_store, brightness), store.brightness);
            break;
          case 2:
            //  Save serial monitor status
            EEPROM.put(get_address(EEPROM_store, serialMon), store.serialMon);
            break;
        }
      #endif
        drawMenu(strMenu0, false);
        menuLevel = 0;
    }
    buttonPress = false;
  }

  else if (encoderDown) {
    switch (menuLevel) {
      case 0:
        //  outside the menu, encoder rotation should control goal temperature
        (activeGoalTemp > MIN_TEMP_C + 5) ? (activeGoalTemp -= 5) : (activeGoalTemp = MIN_TEMP_C); 
        goalTempChanged = true;
        
        #ifdef SAVE_TO_EEPROM
          mustSaveGoalTemp = true;
          saveGoalTempCounter = 0;
        #endif
        break;
      case 1:
        (menuOption > 0) ? (--menuOption) : (menuOption = 3);
        drawMenu(menuLevel1[menuOption], true);
        break;
      case 2:
        switch (menuOption) {
          case 0:
            //  Standby temperature values -> update temperature
            (store.standbyTemp > MIN_TEMP_C + 5) ? (store.standbyTemp -= 5) : (store.standbyTemp = MIN_TEMP_C);
            if (inStandbyMode) {
              store.curGoalTemp = store.standbyTemp;
            }
            drawMenu(itoa((uint16_t) store.standbyTemp, tmpStr, 10), true);
            break;
          case 1:
            //  Brightness values -> update brightness
            (store.brightness > 5) ? (store.brightness -= 5) : (store.brightness = 0);
            tft.setBacklightBrightness(store.brightness);
            drawMenu(itoa(store.brightness, tmpStr, 10), true);
            break;
          case 2:
            store.serialMon = !store.serialMon;
            drawMenu((store.serialMon) ? strMenuOn : strMenuOff, true);
            break;
          default:
            //  Invalid option -> go out of menu
            drawMenu(strMenu0, false);
            menuLevel = 0;
        }
    }
    encoderDown = false;
  }

  else if (encoderUp) {
    switch (menuLevel) {
      case 0:
        //  outside the menu, encoder rotation should control goal temperature
        (activeGoalTemp < MAX_TEMP_C - 5) ? (activeGoalTemp += 5) : (activeGoalTemp = MAX_TEMP_C);
        goalTempChanged = true;
        if (store.serialMon) {
          Serial.print('G');
          Serial.println(store.curGoalTemp);
        }
        break;
      case 1:
        (menuOption < 3) ? (++menuOption) : (menuOption = 0);
        drawMenu(menuLevel1[menuOption], true);
        break;
      case 2:
        switch (menuOption) {
          case 0:
            //  Standby temperature slider -> update temperature
            (store.standbyTemp < MAX_TEMP_C - 5) ? (store.standbyTemp += 5) : (store.standbyTemp = MAX_TEMP_C);
            if (inStandbyMode) {
              store.curGoalTemp = store.standbyTemp;
            }
            drawMenu(itoa((uint16_t) store.standbyTemp, tmpStr, 10), true);
            break;
          case 1:
            //  Brightness slider -> update brightness
            (store.brightness < 250) ? (store.brightness += 5) : (store.brightness = 255);
            tft.setBacklightBrightness(store.brightness);
            drawMenu(itoa(store.brightness, tmpStr, 10), true);
            break;
          case 2:
            store.serialMon = !store.serialMon;
            drawMenu((store.serialMon) ? strMenuOn : strMenuOff, true);
            break;
          default:
            //  Invalid option -> go out of menu
            drawMenu(strMenu0, false);
            menuLevel = 0;
        }
    }
    encoderUp = false;
  }
}

/*  Handles serial events received from serial monitor app written in Matlab, creating
 *  a command string to be processed by handleSerial().
 *  Adapted from https://www.arduino.cc/en/Tutorial/SerialEvent.
 */
void serialEvent() {
  while (store.serialMon && Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    size_t len = strlen(serialCmd);
    if (len < 8) {
      if (inChar == '\n') {
        cmdComplete = true;
      }
      else {
        serialCmd[len] = inChar;
        serialCmd[len+1] = '\0';
      }
    }
    else {
      serialCmd[0] = '\0';
      Serial.println("EINVC");
    }
  }
}

/*  Handles serial communications to and from serial monitor app written in Matlab. 
 */
void handleSerial() {
  if (store.serialMon) {
//  if (true) {
    Serial.print('T');
    Serial.println(currentTemp);
    Serial.print('S');
    Serial.println(adcValue);    
    Serial.print('O');
    Serial.println(heaterOutput);
    
    double tmpVal;
    if (cmdComplete) {
      switch(serialCmd[0]) {
        // set PID P parameter
        case 'P':
          tmpVal = atof(&serialCmd[1]);
          if (tmpVal >= 0. && tmpVal <= 30.0) {
            store.kp = tmpVal;
            solderPID.SetTunings(store.kp, store.ki, store.kd);
            Serial.print('P');
            Serial.println(store.kp);
          }
          else {
              Serial.println("EINVP");
            }
          break;
        // set PID I parameter
        case 'I':
          tmpVal = atof(&serialCmd[1]);
          if (tmpVal >= 0. && tmpVal <= 30.0) {
            store.ki = tmpVal;
            solderPID.SetTunings(store.kp, store.ki, store.kd);
            Serial.print('I');
            Serial.println(store.ki);
          }
          else {
              Serial.println("EINVI");
            }
          break;
        // set PID D parameter
        case 'D':
          tmpVal = atof(&serialCmd[1]);
          if (tmpVal >= 0. && tmpVal <= 30.0) {
            store.kd = tmpVal;
            solderPID.SetTunings(store.kp, store.ki, store.kd);
            Serial.print("D");
            Serial.println(store.kd);
          }
          else {
              Serial.println("EINVD");
            }
          break;
        // set goal temperature
        case 'G':
          tmpVal = atof(&serialCmd[1]);
          if (tmpVal >= MIN_TEMP_C && tmpVal <= MAX_TEMP_C) {
            activeGoalTemp = tmpVal;
            goalTempChanged = true;
            Serial.print('G');
            Serial.println(store.curGoalTemp);
          }
          else {
              Serial.println("EINVG");
            }
          break;
          case 'U':
            Serial.print('P');
            Serial.println(store.kp);
            Serial.print('I');
            Serial.println(store.ki);
            Serial.print('D');
            Serial.println(store.kd);
            Serial.print('G');
            Serial.println(store.curGoalTemp);
            break;
            
          case 'M':
            //  This does not need to depend on SAVE_TO_EEPROM definition, because it is not automatic
            EEPROM.put(BASE_ADDRESS, store);
            break;
          default:
            Serial.println("EINVC");
      }
      serialCmd[0] = '\0';
      cmdComplete = false;
    }
  }  
}

/* Handles standby state, including drawing text and setting goal temperature.
 */
void handleStandby() {
  if (!debouncedStandby && !inStandbyMode) {
    store.curGoalTemp = store.standbyTemp;
    inStandbyMode = true;
    tft.setFont((uint8_t*) Terminal12x16, false);
    tft.drawText(10, ACTUAL_TEXT_Y, strActualTempStdby, COLOR_RED);
    tft.setFont((uint8_t*) Droid_Sans_Mono40x57, true);
  }
  
  else if (debouncedStandby && inStandbyMode) {
    store.curGoalTemp = activeGoalTemp;
    inStandbyMode = false;
    tft.setFont((uint8_t*) Terminal12x16, false);
    tft.drawText(10, ACTUAL_TEXT_Y, strActualTemp, COLOR_WHITE);
    tft.setFont((uint8_t*) Droid_Sans_Mono40x57, true);
  }
  if (store.serialMon) {
    Serial.print('G');
    Serial.println(store.curGoalTemp);
  }
}

/*  Custom PWM write to pin 9 function. This is necessary because the default PWM frequency used by
 *  Arduino is in the audible range, creating an annoying sound when the heater is active. The frequency
 *  of Timer/Counter 1 is set during initialization to 20 kHz.
 *  WARNING: this function is specific for pin 9! Be careful if other pin is used for output, as the other
 *  timers are used by functions in the Arduino library. Please consult the ATMega328P datasheet
 *  (https://www.microchip.com/wwwproducts/en/ATmega328P) for more information.
 */

void pin9PWMWrite(uint8_t val) {
  if (val == 0) {
    digitalWrite(9, 0);
  }
  else {
    TCCR1A |= B10000000;                    //  make sure PWM is active
    static double rangeConv = ICR1 / (MAX_PWM - MIN_PWM + 1);
    OCR1A = (int) ((val + 1) * rangeConv);  //  write duty cycle
  }
}

/*  Ensures values in EEPROM are within (hopefully) reasonable ranges, load defaults if they are not.
 *  Some functionality was disabled due to an incident where instability led to damage to the output 
 *  transistors and a soldering tip! Validity checks should be improved and safe default values should 
 *  be written to the EEPROM before attempting to re-enable these settings.
 */
void ensure_valid() {
  //  Don't load PID parameters from EEPROM (risk of instability!)
  store.kp = DEFAULT_KP;
  store.ki = DEFAULT_KI;
  store.kd = DEFAULT_KD;
  //  Don't load thermocouple parameters from EEPROM (risk of instability!)
  store.tempOffset = DEFAULT_TEMP_OFFSET;
  store.tempGain = DEFAULT_TEMP_GAIN;
  if ((store.curGoalTemp < MIN_TEMP_C) || (store.curGoalTemp > MAX_TEMP_C)) store.curGoalTemp = DEFAULT_GOAL_TEMP;
  if ((store.standbyTemp < MIN_TEMP_C) || (store.standbyTemp > MAX_TEMP_C)) store.standbyTemp = DEFAULT_STANDBY_TEMP;
}
