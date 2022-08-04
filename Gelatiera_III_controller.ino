// Gelatiera III Controller
//
// For Temperature Gradient stage with two Peltier Elements and a linear actuator
// to be controlled by an Arduino Mega
// Original Code by Dominic Gerber and Lawrence Wilen
//
// More Information at https://github.com/dogerber/temperature_gradient_microscopy_stage
// ------------------------------------------------------------------------------------


//////////////////////////////// Libraries ///////////////////////////////////////////
#include <Adafruit_GFX.h>           // Core graphics library
#include <Adafruit_SSD1351.h>       // Hardware-specific library (https://www.adafruit.com/product/2088)
#include <Adafruit_SSD1306.h>       // OLED 1.3 " display (https://www.adafruit.com/product/938)
#include <SdFat.h>                  // SD card & FAT filesystem library
#include <Adafruit_SPIFlash.h>      // SPI / QSPI flash library
#include <Adafruit_ImageReader.h>   // Image-reading functions
#include <GEM_adafruit_gfx.h>       // GEM (https://github.com/Spirik/GEM)
#include <KeyDetector.h>            // GEM 
#include <Adafruit_ADS1X15.h>       // used to measure temperature with ADS1115
#include "DRV8834.h"                // DRV8834 driver for linear actuator


//////////////////////////////// PINS ////////////////////////////////////////////////
// TFT display (large), OLED (small) display and SD card
#define SD_CS    40 // SD card select pin
#define TFT_CS   41 // large  SSD1351 OLED
#define OLED_CS  43 // small SSD1306 OLED
#define TFT_DC   38 // TFT display/command pin, could be combined
#define OLED_DC   42 // TFT display/command pin

// Pololu DRV8834 motor driver for the linear actuator
#define MOTOR_STEPS 200 // Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define RPM 120
#define DIR 22
#define STEP 23
#define M0 24
#define M1 25
#define SLEEP 26

// Pololu VNH5019 peltier dual motor driver Pins
#define M1A 4
#define M1B 5
#define M1ENABLE 6
#define M1PWM 7
#define M2A 8
#define M2B 9
#define M2ENABLE 10
#define M2PWM 11

// shared pins (SPI BUS)
#define TFT_RST  -1 // set to -1 and connect to Arduino RESET pin
#define OLED2_RST -1

// Various
#define BUZZER_PIN 12 // for the buzzer

// Navigating Buttons for GEM
const byte downPin = 35;
const byte leftPin = 32;
const byte rightPin = 37;
const byte upPin = 34;
const byte cancelPin = 31;
const byte okPin = 30;

/* Summary of PINS used
   4 VNH5019 - M1A
   5 VNH5019 - M1B
   6 VNH5019 - M1ENABLE
   7 VNH5019 - M1PWM
   8 VNH5019 - M2A
   9 VNH5019 - M2B
   10 VNH5019 - M2ENABLE
   11 VNH5019 - M2PWM
   12 BUZZER-PIN

   22 DRV8834 - DIR
   23 DRV8834 - STEP
   24 DRV8834 - M0
   25 DRV8834 - M1
   26 DRV8834 - Sleep

   30 Button - OK
   31 Button - cancel
   32 Joystick - Left
   35 Joystick - Push ? (not used)
   34 Joystick - Up
   35 Joystick - Down
   37 Joystick - Right

   38 TFT_DC
   40 SD_CS "CCS" on display
   41 TFT_CS
   42 OLED_DC
   43 OLED_CS

   50 MISO
   51 MOSI
   52 SCK, Clock

   SCL, SDA (I2C): for ADS1115 (temperature measurement)
*/



//////////////////////////////// Global Variables //////////////////////////////////////
#define CODE_VERSION "1.0.0"

// Settings
#define T_MAX 45 // in Celsius, maximum allowed temperature, should be at least 5 degrees out of operation range
#define T_MIN -40 // peltier will shut down when this is reached to avoid damage
#define T_RANGE_MAX 40 // this will be the maximum possible set temperature
#define T_RANGE_MIN -35
#define MAX_COOLING_PLATE_TEMPERATURE 40 // will shut all cooling down when reached until user enables it again
const boolean do_Serial_communication = false; // good for debugging, but makes code slower
boolean do_beep = true;

// Warning / Alarms
unsigned long t_last_alarm_beep;
#define ALARM_BEEP_INTERVAL_MS 10000
String warning_string = "";
unsigned long t_last_warning_message = 0;
#define WARNING_MESSAGE_DISPLAY_TIME 30000 // [ms]

// Temperature control
float SetTemp1 = 25.0;
float PlanTemp1 =  25.0; // PlanTemp is the target, SetTemp the current setting (when changing slowly)
float SetTemp2 = 25.0;
float PlanTemp2 = 25.0;

boolean commitChangesT = false; //
unsigned long t_commitChangesT_last_step = 0;
float TempChangeRate = 00.00; // [K/min] how fast temperature is changed, 0 for instant change

// Temperature measurement (ADS1115)
float Temp0; // Actual Temperature measured
float Temp1;
float Temp2;
float Temp3;
float T_Peltier_1;
float T_Peltier_2;
float T_cooling_plate;
float T_sensor;
boolean displayT3 = false; // show T_sensor on OLED display

const float R_Ref0 = 30.13; // Known resistances of the voltage divider
const float R_Ref1 = 30.11; // measured with old Multimeter before assembly
const float R_Ref2 = 30.13;
const float R_Ref3 = 30.11;
const float Volt_in = 2.048; // [V] depending on wiring of LM4040
const float T_num_average = 10.0; // number of readings of the thermistor resistance. More = slower but less noisy
float R0; // Resistance of the thermistor to be measured
float R1;
float R2;
float R3;


// Motor control
long mot_position; // current position [steps]
long mot_SetPosition; // current target position [steps]
long mot_PlanPosition; // ultimate position to be reached [steps]
float mot_PlanPosition_um; // ultimate position to be reached [um], in GEM
long mot_maxPos = 25000; // [steps] max distance in steps from the middle (=0) in both directions
// linear actuator travels 1 mm per rotation, has 200 normal steps, here with 32 microsteps, so 1mm = 32*200 microsteps
float mot_speed = 1000.00; // [micrometers/minutes]
long mot_stepsToDo; // [steps] for current movement step
float mot_stepsize = 0.315305;// in [mum/step]
boolean commitMovement = false; // starts movement of actuator
long last_mot_position_saved; // keep track of value written to SD card to only write when it changed
boolean mot_position_loaded = false; // initialize as NaN so it's read from SD card
boolean mot_holding = true; // stepper.enable() -> causes motor to be held and unmovable
boolean mot_first_movement = true;
char filename_mot[GEM_STR_LEN] = "motpoy.txt";
unsigned long mot_t_startMovement;
long mot_Position_startMovement;
const float speedConversionFactor = 1 / (mot_stepsize * 60 * 1000); // mot_speed is given in mum/min, we need it in [steps/millis] 
float mot_QuickMoveStep_um = 200; // [um]

// Variables for PID of peltier control (VNH5019)
boolean enablePeltiers = true;
float errorval1; // this stores the difference between the setpoint and the actual temperature
float errorval2;
float previous_errorval1 = 0;
float previous_errorval2 = 0;
float peltier1; // this stores the actual power level you are sending to the peltier device; can be negative or positive
float peltier2;
float peltier1_percentage; // for displaying power usage
float peltier2_percentage;
float Pgain = 305; // the proportional gain
float Igain = 1.9; // the integral gain
float Dgain = 4000; // differential gain
const float antiwindval = 2; // a value that keeps the integrator from integrating up if you are far from your setpoint
float offset1 = 0; // stores the value in the integrator
float offset2 = 0;
float offset_dif1 = 0;
float offset_dif2 = 0;
unsigned long peltier_t_last_tp;

// Screens
#define SCREEN_WIDTH  128 // Screen dimensions in pixel
#define SCREEN_HEIGHT 128
#define SCREEN2_WIDTH  128
#define SCREEN2_HEIGHT 64

// Menu and data logging
boolean do_plot_temperatures = false; // enables Serial print of T values to be plotted in Arduino IDE
unsigned long t_MenuLastRefresh = 0;
int runtime = 0;  // this is wrong, should be unsigned long, but that conflicts with GEM-menu item
int time_for_one_loop_ms = 0;
boolean enableDataLogging = false;
char filename_log[GEM_STR_LEN] = "log01.txt";  // not to long please, max 8 char i think



/////////////////////////////////Setup GEM Menu and SD-card//////////////////////////////////////
// SD card
SdFat                SD;         // SD card filesystem
SdFile myFile;
Adafruit_ImageReader reader(SD); // Image-reader object, pass in SD filesys

// Screens
Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, TFT_CS, TFT_DC, TFT_RST);
Adafruit_SSD1306 oled2 = Adafruit_SSD1306(SCREEN2_WIDTH, SCREEN2_HEIGHT, &SPI, OLED_DC, OLED2_RST, OLED_CS);

// Temperature measurement
Adafruit_ADS1115 ads;  // load library to run ADS1115, used for temperature measurement

// Motor driver DRV8834 for linear actuator
DRV8834 stepper(MOTOR_STEPS, DIR, STEP, SLEEP, M0, M1);


// GEM
// Array of Key objects that will link GEM key identifiers with dedicated pins
Key keys[] = {{GEM_KEY_UP, upPin}, {GEM_KEY_RIGHT, rightPin}, {GEM_KEY_DOWN, downPin}, {GEM_KEY_LEFT, leftPin}, {GEM_KEY_CANCEL, cancelPin}, {GEM_KEY_OK, okPin}};
KeyDetector myKeyDetector(keys, sizeof(keys) / sizeof(Key)); // Create KeyDetector object

// Main page
GEMItem menuItemEnablePeltiers("T control:", enablePeltiers);
void checkPlanTemps(); // forward declaration?
GEMItem menuItemSetTemp1("Set T1:", PlanTemp1, checkPlanTemps);
GEMItem menuItemSetTemp2("Set T2:", PlanTemp2, checkPlanTemps);
GEMItem menuItemTemp3("T3:", T_sensor, false); // for gradient measurement in the experimental cell
GEMItem menuItemChangeRate("[K/min]:", TempChangeRate);
void doCommitChangesT(); // forward declaration?
GEMItem menuItemCommit("Change T:", commitChangesT, doCommitChangesT);

// Motor control
GEMItem menuItemTitleActuator("--Actuator ", "      --", false);
void doCheckPlanPosition();
GEMItem menuItemMotPlanPosition("Target:", mot_PlanPosition_um, doCheckPlanPosition);
GEMItem menuItemMotSpeed("[um/min]:", mot_speed);
void doCommitMovement(); // forward declaration
GEMItem menuItemCommitMovement("Move:", commitMovement, doCommitMovement);
void resetMotPosition(); // forward declaration
GEMItem menuItemResetMotPosition("Calibrate 0", resetMotPosition);
GEMItem menuItemMotHold("Motor hold:", mot_holding, commitMovement);  // last option locks the option when movement is in progress

GEMItem menuSDFilename("Filename", filename_log);
GEMItem menuEnableDataLogging("Log data", enableDataLogging);

GEMItem menuCodeVersion("Code v.", CODE_VERSION, true);
GEMItem menuRuntime("Time [s]:", runtime, true);

GEMPage menuPageMain("Gelatiera III");
GEMItem menuItemMainLink("Main Menu", menuPageMain);
GEMItem menuItemMainLink2("Main Menu", menuPageMain);
GEMItem menuItemMainLink3("Main Menu", menuPageMain);

// Quick change menu
GEMPage menuPageQuick("Quick change");
GEMItem menuItemMainQuick("Quick change", menuPageQuick);
void doQuick2525(); // forward declaration
GEMItem menuItemQuick2525("T 25 / 25:", doQuick2525);
void doQuick11(); // forward declaration
GEMItem menuItemQuick11("T 1 / 1:", doQuick11);
void doQuick0105(); // forward declaration
GEMItem menuItemQuick0105("T -0.1 / 0.5:", doQuick0105);
void doQuickMotSpeed(); // forward declaration
GEMItem menuItemQuickMaxMotSpeed("Max mot_speed:", doQuickMotSpeed);
void doQuickMoveMiddle(); // forward declaration
GEMItem menuItemQuickMoveTo0("Move to 0:", doQuickMoveMiddle);
void doQuickMovePos(); // forward declaration
GEMItem menuItemQuickMovePos("Move right:", doQuickMovePos);
void doQuickMoveNeg(); // forward declaration
GEMItem menuItemQuickMoveNeg("Move left:", doQuickMoveNeg);
GEMItem menuItemQuickMoveStep("Step size:", mot_QuickMoveStep_um);

// Settingspage
GEMPage menuPageSettings("Settings");
GEMItem menuItemMainSettings("Settings", menuPageSettings);
GEMItem menuItemDoT3("Show T3", displayT3);
GEMItem menuItemDoBeep("beep", do_beep);
GEMItem menuItemMotStepsize("[um/step]", mot_stepsize);
void showLastWarning();
GEMItem menuItemShowLastWarning("Last warning", showLastWarning);
GEMItem menuItemPlotTemperatures("Output T to console", do_plot_temperatures);
GEMItem menuItemTitlePID("--PID ", "      --", false);
GEMItem menuItemPgain("Pgain", Pgain);
GEMItem menuItemIgain("Igain", Igain);
GEMItem menuItemDgain("Dgain", Dgain);


// Infopage
GEMPage menuPageInfo("Stats");
GEMItem menuItemMainStats("Stats", menuPageInfo);
GEMItem menuItemInfoR0("R0", R0, true);
GEMItem menuItemInfoR1("R1", R1, true);
GEMItem menuItemInfoR2("R2", R2, true);
GEMItem menuItemInfoR3("R3", R3, true);
GEMItem menuItemInfoPeltier1("Peltier1", (peltier1), true);
GEMItem menuItemInfoPeltier2("Peltier1", (peltier2), true);
GEMItem menuItemOffset1("offset1", offset1, true);
GEMItem menuItemOffset2("offset2", offset2, true);
GEMItem menuItemInfoLoopTime("Loop time [ms]", time_for_one_loop_ms, true);

GEM_adafruit_gfx menu(tft, GEM_POINTER_ROW,
                      /*Menu Items per screen */ GEM_ITEMS_COUNT_AUTO, /*Menu Item Height */ 10, /*Menu Page screen top ofset */ 10, /*Menu Val Left offset */ 72);
// see https://github.com/Spirik/GEM/wiki/Menu-appearance to customize



////////////////////////////////////////// Setup() //////////////////////////////////////////////////
void setup(void) {
  beep(1000); //Power on
  Serial.begin(9600);
  if (do_Serial_communication) {
    Serial.println(F("Gelatiera III controller started."));
    Serial.print(F("Version: "));
    Serial.println(CODE_VERSION);
  }

  // ---------------------- Screen Initialization ------------------------------------------- //
  oled2.begin(); // small display
  oled2.display();
  oled2.setTextWrap(true);

  tft.begin(); // large display
  tft.fillScreen(0x00); // clears screen
  tft.setRotation(1);

  // ---------------------- SD card Initialization ------------------------------------------- //
  if (do_Serial_communication) {
    Serial.print(F("Initializing filesystem..."));
  }

  if (!SD.begin(SD_CS, SD_SCK_MHZ(10))) { // Breakouts require 10 MHz limit due to longer wires
    Serial.println(F("SD begin() failed. Try to disconnect power and restarting."));
    for (;;); // Fatal error, do not continue
  }

  // ---------------------- GEM Menu system Initialization -------------------------------------- //
  // Push-buttons pin modes
  pinMode(downPin, INPUT);
  pinMode(leftPin, INPUT);
  pinMode(rightPin, INPUT);
  pinMode(upPin, INPUT);
  pinMode(cancelPin, INPUT);
  pinMode(okPin, INPUT);


  // Menu init, setup and draw
  menu.setSplashDelay(0); // disabled GEM splashscreen
  menu.init();
  setupMenu();
  menu.drawMenu();


  // ---------------------- DRV8834 Linear actuator driver ---------------------------------------- //
  stepper.begin(RPM, 32); // begin(RPM,Microsteps)

  // ----------- load Motor-position from SD card (to not loose steps)
  SDMotPositionLoad();
  if (do_Serial_communication) {
    Serial.println(F("Motor-position loaded"));
  }


  // --------------------- ADS1115 setup (temperature measurement) ------------------------------- //
  ads.setGain(GAIN_TWO); // depending on the wiring of the LM4040, see ADS1115 examples for more informations
  ads.setDataRate(RATE_ADS1115_860SPS  ); // fastest DataRate, significantly reduces noise
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  // ----------------- VNH5019 peltier controller ------------------------------------------------ //
  peltierControlSetup();
  if (do_Serial_communication) {
    Serial.println(F("Setup() complete"));
  }
  beep(1200); // main loop started


} // setup()



//////////////////////////////////// loop() /////////////////////////////////////////////////////
void loop() {

  // ------ GEM Menu ready
  if (menu.readyForKey()) {
    myKeyDetector.detect();     // ...detect key press using KeyDetector library
    menu.registerKeyPress(myKeyDetector.trigger); // Pass pressed button to menu
  }


  // ----- Measure Temperature
  measureResistance();
  Temp0 = calcTemperature(R0);
  Temp1 = calcTemperature(R1);
  Temp2 = calcTemperature(R2);
  Temp3 = calcTemperature_thin(R3);

  // assign temperatures
  T_Peltier_1 = Temp0;
  T_Peltier_2 = Temp1;
  T_cooling_plate = Temp2;
  T_sensor = Temp3; // currently not used

  // check for cooling plate nan or too hot
  if ((isnan(T_cooling_plate)) || (T_cooling_plate > MAX_COOLING_PLATE_TEMPERATURE)) {
    enablePeltiers = false; // turn peltiers off, has to be manually turned on again
    warningMessage("Cooling plate too hot/nan!");

    //    if (ALARM_BEEP_INTERVAL_MS > millis() - t_last_alarm_beep) { // conitnuous beeping as alert
    ////      beep(1000); delay(100); beep(1000); delay(100); beep(1000); delay(100);
    //
    //      t_last_alarm_beep = millis();
    //    }
  }


  // ---- Update Set Temperature
  if (commitChangesT) {
    if (TempChangeRate == 0) { // change temperature as fast as possible
      SetTemp1 = PlanTemp1;
      SetTemp2 = PlanTemp2;
    }
    else { // change temperature at a fixed rate
      // SetTemp(t) = SetTemp(t-1) + Rate * [t - (t-1)] * sign
      SetTemp1 = SetTemp1 + TempChangeRate * (millis() - t_commitChangesT_last_step) / 60 / 1000 * signOfX(PlanTemp1 - SetTemp1);
      SetTemp2 = SetTemp2 + TempChangeRate * (millis() - t_commitChangesT_last_step) / 60 / 1000 * signOfX(PlanTemp2 - SetTemp2);
      t_commitChangesT_last_step = millis();
    }

    if (abs(SetTemp1 - PlanTemp1) <= 0.01) { // target1 reached
      SetTemp1 = PlanTemp1;
    }

    if (abs(SetTemp2 - PlanTemp2) <= 0.01) { // target2 reached
      SetTemp2 = PlanTemp2;
    }

    if ((abs(SetTemp1 - PlanTemp1) <= 0.01) && (abs(SetTemp2 - PlanTemp2) <= 0.01)) { // both target reached
      commitChangesT = false;
      menu.drawMenu(); // refreshes menu to remove the tick from the commit-box
      beep(800); // marks end of T-change
    }
  }

  // For Debug/PID tuning: plot temperature and set temperature
  if (do_plot_temperatures) { // connect through USB and use Serial plotter in Arduino IDE
    Serial.print("S1:"); Serial.print(SetTemp1);
    Serial.print(", T1:"); Serial.print(T_Peltier_1);
    Serial.print(", S2:"); Serial.print(SetTemp2);
    Serial.print(", T2:"); Serial.println(T_Peltier_2);
  }

  // ----- Determine Peltier control
  peltierControl();

  // ----- Determine Motor movement
  if (mot_holding || commitMovement) {
    stepper.enable();
    mot_holding = true;
  }
  else {
    stepper.disable();
  }

  moveMotor();

  SDMotPositionWrite(); // write current mot_position to SD card to not loose steps when powered down

  // --- Update OLED2 with status informations
  writeToOLED2(SetTemp1, SetTemp2, T_Peltier_1, T_Peltier_2);


  // --- Write to log file
  if (enableDataLogging) {
    // Serial.println(F("Data logging enabled."));
    // open the file for write at end like the Native SD library
    if (!SD.exists(filename_log)) {
      // if it doesnt exist, try to make it
      if (!myFile.open(filename_log, O_RDWR | O_CREAT | O_AT_END)) {
        SD.errorHalt("opening filename_log for write failed");
      }

      // write Header to explain the file
      myFile.println(F("File generated by Gelatiera III data logging"));
      myFile.println(F("time [ms], Set1 [°C], T1[°C], Set2[°C], T2[°C], T3[°C], T4[°C], mot_position [mum]"));
      //Serial.println(F("Made new file and wrote header"));
    }
    else {
      myFile.open(filename_log, O_RDWR | O_AT_END); // open with read/write, set pointer to end of file
    }

    // if the file opened okay, write to it:
    myFile.print(millis());
    myFile.print(F(", "));
    myFile.print(SetTemp1);
    myFile.print(F(", "));
    myFile.print(T_Peltier_1);
    myFile.print(F(", "));
    myFile.print(SetTemp2);
    myFile.print(F(", "));
    myFile.print(T_Peltier_2);
    myFile.print(F(", "));
    myFile.print(T_cooling_plate);
    myFile.print(F(", "));
    myFile.print(T_sensor);
    myFile.print(F(", "));
    myFile.print(mot_position * mot_stepsize);
    myFile.println(F(""));

    // close the file:
    myFile.close();
  } // if enableDataLogging

  time_for_one_loop_ms =  millis() - runtime * 1000;
  runtime = millis() / 1000;

} // end main loop


//////////////////// Sub Functions /////////////////////////////////////////////////////////
void setupMenu() {
  // here the GEM Menu is assembled, the menuItems are declared as global variables further up

  // Subpage SETTINGS
  menuPageSettings.addMenuItem(menuItemMainLink);
  menuPageSettings.addMenuItem(menuItemDoBeep);
  menuPageSettings.addMenuItem(menuItemMotStepsize);
  menuPageSettings.addMenuItem(menuItemDoT3);
  menuPageSettings.addMenuItem(menuItemShowLastWarning);
  menuPageSettings.addMenuItem(menuItemPlotTemperatures);
  menuPageSettings.addMenuItem(menuSDFilename);
  menuPageSettings.addMenuItem(menuEnableDataLogging);
  menuPageSettings.addMenuItem(menuItemTitleActuator);
  menuPageSettings.addMenuItem(menuItemResetMotPosition);
  menuPageSettings.addMenuItem(menuItemMotHold);

  // - PID debug
  menuPageSettings.addMenuItem(menuItemTitlePID);
  menuPageSettings.addMenuItem(menuItemPgain);
  menuPageSettings.addMenuItem(menuItemIgain);
  menuPageSettings.addMenuItem(menuItemDgain);

  // Subpage INFO
  menuPageInfo.addMenuItem(menuItemMainLink2);
  menuPageInfo.addMenuItem(menuCodeVersion);
  menuPageInfo.addMenuItem(menuItemInfoR0);
  menuPageInfo.addMenuItem(menuItemInfoR1);
  menuPageInfo.addMenuItem(menuItemInfoR2);
  menuPageInfo.addMenuItem(menuItemInfoR3);
  menuPageInfo.addMenuItem(menuItemTemp3);
  menuItemTemp3.setPrecision(2);
  menuPageInfo.addMenuItem(menuItemInfoPeltier1);
  menuPageInfo.addMenuItem(menuItemInfoPeltier2);
  menuPageInfo.addMenuItem(menuItemInfoLoopTime);
  menuPageInfo.addMenuItem(menuRuntime);
  menuPageInfo.addMenuItem(menuItemOffset1);
  menuPageInfo.addMenuItem(menuItemOffset2);

  // Subpage QUICK
  menuPageQuick.addMenuItem(menuItemMainLink3);
  menuPageQuick.addMenuItem(menuItemQuick2525);
  menuPageQuick.addMenuItem(menuItemQuick11);
  menuPageQuick.addMenuItem(menuItemQuick0105);
  menuPageQuick.addMenuItem(menuItemQuickMaxMotSpeed);
  menuPageQuick.addMenuItem(menuItemQuickMoveTo0);
  menuPageQuick.addMenuItem(menuItemQuickMoveStep);
  menuPageQuick.addMenuItem(menuItemQuickMovePos);
  menuPageQuick.addMenuItem(menuItemQuickMoveNeg);
  
  // Add menu items to MAIN menu page
  menuPageMain.addMenuItem(menuItemEnablePeltiers);
  menuPageMain.addMenuItem(menuItemSetTemp1);
  menuItemSetTemp1.setPrecision(2);
  menuPageMain.addMenuItem(menuItemSetTemp2);
  menuItemSetTemp2.setPrecision(2);
  menuPageMain.addMenuItem(menuItemChangeRate);
  menuItemChangeRate.setPrecision(3);
  menuPageMain.addMenuItem(menuItemCommit);

  // motor
  menuPageMain.addMenuItem(menuItemMotPlanPosition);
  menuItemMotPlanPosition.setPrecision(2);
  menuPageMain.addMenuItem(menuItemMotSpeed);
  menuItemMotSpeed.setPrecision(2);
  menuPageMain.addMenuItem(menuItemCommitMovement);
  menuPageMain.addMenuItem(menuItemMainQuick);
  menuPageMain.addMenuItem(menuItemMainSettings);
  menuPageMain.addMenuItem(menuItemMainStats);

  // Add menu page to menu and set it as current
  menu.setMenuPageCurrent(menuPageMain);
}

void writeToOLED2(float S1, float S2, float T1, float T2) {
  // the small right screen that shows the current state
  //todo speed: check if something changed or not! to not refresh all the time
  // 8 lines times 21 signs total possible
  // line spacing: 8px
  // horizontal spacing: 6px

  oled2.clearDisplay();
  oled2.setTextSize(1);
  oled2.setTextColor(SSD1306_WHITE);

  if (true) {
    // mental note:    SetTemp1 = SetTemp1 + TempChangeRate * (millis() - t_commitChangesT_last_step) / 60 / 1000 * signOfX(PlanTemp1 - SetTemp1);

    // temporary T3
    if (displayT3) {
      oled2.setCursor(0, 0);
      oled2.print("T3: ");
      oled2.println(T_sensor, 2);
    }

    // Set and actual Temperatures
    oled2.setCursor(12, 8);
    oled2.print("Set");
    //if (SetTemp1 <0){ // this prevents jumping of digits when changing from positive to negative
    //  oled2.setCursor(30, 8);
    //}
    //else{
    //oled2.setCursor(36,8);
    //}
    oled2.setCursor(36, 8);
    oled2.print(SetTemp1);

    // Up/down arrows
    if (PlanTemp1 > SetTemp1 && commitChangesT) {
      oled2.drawChar(0, 8, 0x18, SSD1306_WHITE, SSD1306_WHITE, 1 );
    }
    else if (PlanTemp1 < SetTemp1 && commitChangesT) {
      oled2.drawChar(0, 8, 0x19, SSD1306_WHITE, SSD1306_WHITE, 1 );
    }
    oled2.setCursor(78, 8);
    oled2.print(SetTemp2);
    if (PlanTemp2 > SetTemp2 && commitChangesT) {
      oled2.drawChar(120, 8, 0x18, SSD1306_WHITE, SSD1306_WHITE, 1 );
    }
    else if (PlanTemp2 < SetTemp2 && commitChangesT) {
      oled2.drawChar(120, 8, 0x19, SSD1306_WHITE, SSD1306_WHITE, 1 );
    }

    // Act temperature
    oled2.setCursor(12, 16);
    oled2.print("Act");
    oled2.setCursor(36, 16);
    oled2.print(T1);

    oled2.setCursor(78, 16);
    oled2.print(T2);

    // cooling circuit temperature
    oled2.setCursor(12, 24);
    oled2.print("Cooling T: ");
    oled2.setCursor(90, 24);
    oled2.print(T_cooling_plate);

    // Display Motor position
    oled2.setCursor(12, 32);
    oled2.print("Pos: "); oled2.print(mot_position * mot_stepsize);
    oled2.print(" um");
    // left/right arrow for movement
    if (mot_PlanPosition - mot_position < 0) {
      oled2.print("<- "); //oled2.print((mot_PlanPosition - mot_position)*mot_stepsize, 0);
    }
    else if (mot_PlanPosition - mot_position > 0) {
      oled2.print("-> "); // oled2.print((mot_PlanPosition - mot_position)*mot_stepsize, 0);
    }


    // Peltier power
    if (true) {
      peltier1_percentage =  constrain(peltier1, -255, 255) * 100 / 255;
      peltier2_percentage =  constrain(peltier2, -255, 255) * 100 / 255;
      oled2.setCursor(12, 40);

      if (peltier1_percentage < 0) {
        oled2.print("Pow:");
      }
      else {
        oled2.print("Pow: ");
      }

      oled2.print(peltier1_percentage, 0);
      if (peltier2_percentage < 0) {
        oled2.setCursor(72, 40);
      }
      else {
        oled2.setCursor(78, 40);
      }
      oled2.print(peltier2_percentage, 0);
    }

    // Warnings
    if (((millis() - t_last_warning_message < WARNING_MESSAGE_DISPLAY_TIME)) && (t_last_warning_message > 0)) {
      oled2.setCursor(0, 48);
      oled2.print(warning_string);
      oled2.invertDisplay(true);
    }
    else {
      oled2.invertDisplay(false);
    }
  }
  oled2.display();
}

int signOfX(float n) {
  if (n > 0) {
    return 1;
  }
  else if (n < 0) {
    return -1;
  }
  else {
    return 0;
  }
}

void warningMessage(char *str_in) {
  if (warning_string != str_in) {
    // prevents continuous beeping with the same message
    warning_string = str_in;
    // beep
    beep(800); delay(100);
    beep(800); delay(100);
    beep(800); delay(100);
  }
  t_last_warning_message = millis();
}

void showLastWarning() {
  t_last_warning_message = millis();
}

void doCheckPlanPosition() {
  // make sure the input PlanPosition corresponds to a possible step position
  mot_PlanPosition = round(mot_PlanPosition_um / mot_stepsize); // convert to steps and round
  mot_PlanPosition = constrain(mot_PlanPosition, -mot_maxPos, mot_maxPos); // keep in allowed range
  mot_PlanPosition_um = mot_PlanPosition * mot_stepsize; // update mot_PlanPosition_um
}

void checkPlanTemps() {
  // make sure the PlanTemp are within the allowed range
  PlanTemp1 = constrain(PlanTemp1, T_RANGE_MIN, T_RANGE_MAX);
  PlanTemp2 = constrain(PlanTemp2, T_RANGE_MIN, T_RANGE_MAX);
  // could add beep here to signify start of T-change?
}

void doCommitChangesT() {
  // called when commit button is pressed
  t_commitChangesT_last_step = millis();
  if (TempChangeRate < 0) {
    TempChangeRate = 0; // maximum rate
  }
}

void SDMotPositionLoad() {
  // read mot_position from SD card
  //if (!mot_position_loaded) {
  if (do_Serial_communication) {
    Serial.println(F("reading mot position from SD card"));
  }
  // Serial.print(F("mot_position before loading: ")); Serial.println(mot_position);

  if (!SD.exists(filename_mot)) {
    Serial.print(F("mot_position file not found on SD-card: ")); Serial.println(filename_mot);
    warning_string = String(F("mot_position file not found on SD"));
    // add a termination here?
  }
  else {
    myFile.open(filename_mot, O_RDWR ); // open with read/write, set pointer to end of file

    char buffer_read[64];
    uint8_t idx_read = 0;
    
    while (myFile.available()) {
      buffer_read[idx_read] = myFile.read();
      idx_read ++;
    }
    buffer_read[idx_read] = '\0'; // need this to terminate the string
    mot_position = atoi(buffer_read); // convert to number
    mot_SetPosition = mot_position; // initialize with same values
    mot_PlanPosition = mot_position;
    mot_PlanPosition_um = mot_PlanPosition * mot_stepsize;

    if (do_Serial_communication) {
      Serial.print(F("mot_position loaded from SD-card: ")); Serial.println(mot_position);
    }
    last_mot_position_saved = mot_position;
    mot_position_loaded = true;

  }
  myFile.close();
  menu.drawMenu(); // refreshes menu to update mot_PlanPos
}


void SDMotPositionWrite() {
  // update noted down mot_position on SD-card
  if (mot_position != last_mot_position_saved) {
    myFile.open(filename_mot, O_RDWR ); // opening like this removes previous content of file
    myFile.println(mot_position);
    myFile.close();
    last_mot_position_saved = mot_position;
  }
}

void measureResistance() {
  // updates the global variables R0, R1, R2, R3 with the measrued voltages of the ADS1115
  // which will be used to calculate the temperature
  float Volt_0 = 0;
  float Volt_1 = 0;
  float Volt_2 = 0;
  float Volt_3 = 0;

  // average value measurement
  for (int i = 0; i < T_num_average; i++) {
    Volt_0 = Volt_0 + ads.readADC_SingleEnded(0);
    Volt_1 = Volt_1 + ads.readADC_SingleEnded(1);
    Volt_2 = Volt_2 + ads.readADC_SingleEnded(2);
    Volt_3 = Volt_3 + ads.readADC_SingleEnded(3);
  }

  // convert ADC to Volt measurement
  Volt_0 = ads.computeVolts(Volt_0 / T_num_average);
  Volt_1 = ads.computeVolts(Volt_1 / T_num_average);
  Volt_2 = ads.computeVolts(Volt_2 / T_num_average);
  Volt_3 = ads.computeVolts(Volt_3 / T_num_average);

  // calculate resistances from voltage divider
  R0 = R_Ref0 / ( ( Volt_in / Volt_0) - 1.0 );
  R1 = R_Ref1 / ( ( Volt_in / Volt_1) - 1.0 );
  R2 = R_Ref2 / ( ( Volt_in / Volt_2) - 1.0 );
  R3 = R_Ref3 / ( ( Volt_in / Volt_3) - 1.0 );
}


float calcTemperature(float res_thermistor ) {
  int res_thermistor_nominal = 10; // 10 kOhm
  // for these: https://www.digikey.ch/de/products/detail/MC65F103AN/235-1451-ND/6165822?itemSeq=339770248
  // Amphenol MC65F103AN
  //#define SALPHA 3.3538646e-03  // -50 to 0 °C
  //#define SBETA 2.5654090e-04
  //#define SGAMMA 1.9243889e-06
  //#define SDELTA 1.0969244e-07

#define SALPHA 3.3540154e-03 // 0 to +50 °C
#define SBETA 2.5627725e-04
#define SGAMMA 2.0829210e-06
#define SDELTA 7.3003206e-08

  float ratio = res_thermistor / res_thermistor_nominal;

  float temperature = (1 / (SALPHA + SBETA * log(ratio) + SGAMMA * pow(log(ratio), 2) + SDELTA * pow(log(ratio), 3))) - 273.15;

  if (temperature < -69) { // a reading of -99 or lower means thermistor not connected
    temperature = sqrt(-1); // makes it NaN
  }

  return temperature;
}

float calcTemperature_thin(float res_thermistor ) {
  // https://eu.mouser.com/ProductDetail/Amphenol-Advanced-Sensors/AN6N4-GC11KA143L-37C?qs=yp8j7TdJNfKDOUJg7L29DQ%3D%3D
  // calculated coefficients with this: https://rusefi.com/Steinhart-Hart.html
  //#define SALPHA 0.0007189324884286944
  //#define SBETA 0.000250955278117255
  //#define SDELTA 1.258485006192324e-7
  res_thermistor = res_thermistor * 1000; // formula needs Ohm, not kOhm
  //float temperature = 1 / (0.0006297634226689573 + 0.00026311843245081096 * log(res_thermistor) + 9.773976481292966e-8 * pow(log(res_thermistor), 3)) - 273.15;
  float temperature = 1 / (0.0007721425061974199 + 0.0002444807033247905 * log(res_thermistor) + 1.5134377709759929e-7 * pow(log(res_thermistor), 3)) - 273.15; // second thin thermistor
  if (temperature < -69) { // a reading of -99 or lower means thermistor not connected
    temperature = sqrt(-1); // makes it NaN
  }
  return temperature;
}

void resetMotPosition() {
  // for calibrating the motor position
  mot_position = 0;
  mot_PlanPosition = 0;
  mot_PlanPosition_um = 0;
}

void doCommitMovement() {
  if (mot_first_movement) { // remind user to reset the motor!
    warningMessage("Dont forget to recalibrate the motor!");
    mot_first_movement = false;
    commitMovement = false;
  } else {
    // called when commit for motor movement is pressed
    mot_t_startMovement = millis();
    mot_Position_startMovement = mot_position;
    stepper.enable();
  }
}

void moveMotor() {
  // moves linear acutator with dirver drv8834 accodring to user input
  if (commitMovement) {
    // calculate where we should be at this time point = mot_SetPosition
    if (mot_speed == 0) { // maximum speed
      mot_SetPosition = mot_PlanPosition;
    }
    else {
      mot_SetPosition = mot_Position_startMovement + mot_speed * speedConversionFactor * (millis() - mot_t_startMovement); // speed * conversion * dt = steps to do
    }


 // move motor if necessary and allowed
    if (abs(mot_SetPosition - mot_position) >= 1) { // 0 means no changes, can skip all these checks then
      // check if setPos is still allowed
      if (abs(mot_SetPosition) > mot_maxPos) {
        mot_SetPosition = mot_position;
        mot_PlanPosition = mot_position;
        warningMessage("Move limit reached.");
        if (do_Serial_communication) {
          Serial.println(F("Motor wanted to move outside allowed borders. Stopping..."));
        }
      }

      // prevent overshoot over planPos
      if (signOfX(mot_PlanPosition - mot_position) != signOfX(mot_PlanPosition - mot_SetPosition)) {
        mot_SetPosition = mot_PlanPosition;
        if (do_Serial_communication) {
          Serial.println(F("Motor wanted to move past the planPos. stopping."));
        }
      }


      // check if a step has to be taken
      if (abs(mot_SetPosition - mot_position) >= 1) {
        mot_stepsToDo = floor(abs(mot_SetPosition - mot_position)) * signOfX(mot_PlanPosition - mot_position);
        stepper.move(mot_stepsToDo);
        mot_position += mot_stepsToDo; // update current position


        if (abs(mot_stepsToDo) > 1) {
          //warning, not optimal small step size, because interval between ideal steps is shorter than the loop time
          if (do_Serial_communication) {
            Serial.print(F("Not optimal stepsize, had to do: "));
            Serial.println(mot_stepsToDo); // acutal movement step done = mot_stepsToDo*mot_stepsize
          }
        }

      }

      // abort if target is reached (can be for different reasons)
      if (abs(mot_PlanPosition - mot_position) < 1) {
        mot_PlanPosition = mot_position;
        mot_SetPosition = mot_position;
        // stepper.disable(); // this makes the stepper free to be moved, which is undesired. If the motor holding makes noise this could be used
        commitMovement = false;
        beep(600); // sound that movement is done
        menu.drawMenu(); // refresh menu to untick the move box
      }
    }

  }
}


void peltierControl() {

  unsigned dT_seconds = millis() - peltier_t_last_tp;

  if (enablePeltiers) {
    // -------------- calculate appropriate voltage for PID temperature control
    errorval1 = T_Peltier_1 - SetTemp1;
    errorval2 = T_Peltier_2 - SetTemp2;

    // warning for no singal
    //    if ((isnan(T_Peltier_1))||(isnan(T_Peltier_2))){
    //      warningMessage("T signal missing");
    //    }

    // Peltier 1
    if (isnan(T_Peltier_1) || T_Peltier_1 < T_MIN) {
      // if temperature measuerement fails, switch off peltier. Sometimes if there's no signal
      // it faultly measures -80 degrees or so, which would cause the peltier to heat up a lot,
      // hence the second catch
      offset1 = 0;
      peltier1 = 0;
    }
    else if (T_Peltier_1 > T_MAX) {
      // maximum tempearture reached, shut it off
      offset1 = 0; // reset offset, it might has been adding up, causing the problem
      peltier1 = 200; // positive number for cooling, see below
    }
    else if (T_Peltier_1 < T_MIN) {
      // minimum tempearture reached, shut it off
      offset1 = 0; // reset offset, it might has been adding up, causing the problem
      peltier1 = -200;
    }
    else {
      // tempearture in range, do the normal adjustment
      if (abs(errorval1) < antiwindval) {
        offset1 = offset1 + (Igain * errorval1); 
        offset_dif1 = (errorval1 - previous_errorval1) * Dgain; 
      }
      else {
        offset1 = 0;
        offset_dif1 = 0;
      }
      peltier1 = Pgain * errorval1 + offset1 + offset_dif1;  
    }

    // Peltier 2
    if (isnan(T_Peltier_2) || T_Peltier_2 < -50) {
      // if temperature measuerement fails, switch off peltier
      offset2 = 0;
      peltier2 = 0;
    }
    else if (T_Peltier_2 > T_MAX) {
      // maximum tempearture reached, shut it off
      offset2 = 0; // reset offset, it might has been adding up, causing the problem
      peltier2 = 200; // positive number for cooling, see below
    }
    else if (T_Peltier_2 < T_MIN) {
      // minimum tempearture reached, shut it off
      offset2 = 0; // reset offset, it might has been adding up, causing the problem
      peltier2 = -200;
    }
    else {
      // tempearture in range, do the normal adjustment
      if (abs(errorval2) < antiwindval) {
        offset2 = offset2 + (Igain * errorval2) ;//test * dT_seconds;
        offset_dif2 = (errorval2 - previous_errorval2) * Dgain; // should we add dt again?
      }
      else {
        offset2 = 0;
        offset_dif2 = 0;
      }
      peltier2 = Pgain * errorval2 + offset2 + offset_dif2;
    }
  }
  else { // enablePeltier = false, peltier control off, could add enable->off here?
    peltier1 = 0; offset1 = 0;
    peltier2 = 0; offset2 = 0;
  } // if (enablePeltiers) end


  // ------------------    write voltage to peltiers
  if (peltier1 <= 0) {
    digitalWrite(M1A, LOW);//polarity for heating
    digitalWrite(M1B, HIGH);
    analogWrite(M1PWM, constrain(int(abs(peltier1)), 0, 255));
  }
  if (peltier1 > 0) {
    digitalWrite(M1A, HIGH);//polarity for cooling
    digitalWrite(M1B, LOW);
    analogWrite(M1PWM, constrain(int(abs(peltier1)), 0, 255));
  }

  if (peltier2 <= 0) {
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, HIGH);
    analogWrite(M2PWM, constrain(int(abs(peltier2)), 0, 255));
  }
  if (peltier2 > 0) {
    digitalWrite(M2A, HIGH);
    digitalWrite(M2B, LOW);
    analogWrite(M2PWM, constrain(int(abs(peltier2)), 0, 255));
  }

  peltier_t_last_tp = millis();
  previous_errorval1 = errorval1;
  previous_errorval2 = errorval2;
}

void peltierControlSetup() {
  // ---- Initialize Pins for Peltier control
  pinMode(M1A, OUTPUT);// m1 in A; these control the polarity of the drive voltage to the peltier device
  pinMode(M1B, OUTPUT);// m1 in B

  pinMode(M2A, OUTPUT);// m2 in A ; these control the polarity of the drive voltage to the peltier device
  pinMode(M2B, OUTPUT);// m2 in B

  pinMode(M1ENABLE, OUTPUT); //m1 enable; these are set to be on always for now
  pinMode(M2ENABLE, OUTPUT); //m2 enable

  pinMode(M1PWM, OUTPUT); //m1 pwm; these control the drive level
  pinMode(M2PWM, OUTPUT); //m2 pwm

  // Start with everything turned off (but enabled)
  digitalWrite(M1ENABLE, HIGH); //m1; enable on
  digitalWrite(M1PWM, LOW);// start out with drive at zero
  digitalWrite(M1A, LOW);// everything off
  digitalWrite(M1B, LOW);

  digitalWrite(M2ENABLE, HIGH); //m1; enable on
  digitalWrite(M2PWM, LOW);// start out with drive at zero
  digitalWrite(M2A, LOW);// everything off
  digitalWrite(M2B, LOW);

  peltier_t_last_tp = millis();
}

void beep(int frequency) {
  if (do_beep) {
    tone(BUZZER_PIN, frequency);
    delay(20);
    noTone(BUZZER_PIN);
  }
}


// --- Quick change functions
void doQuick2525() {
  TempChangeRate = 0; //
  PlanTemp2 = 25;
  PlanTemp1 = 25;
  commitChangesT = true;
}

void doQuick11() {
  TempChangeRate = 0; //
  PlanTemp2 = 1;
  PlanTemp1 = 1;
  commitChangesT = true;
}

void doQuick0105() {
  TempChangeRate = 0; //
  PlanTemp2 = 0.5;
  PlanTemp1 = -0.1;
  commitChangesT = true;
}

void doQuickMotSpeed() {
  mot_speed = 0;
}

void doQuickMoveMiddle() {
  mot_PlanPosition = 0;
  doCheckPlanPosition();
  commitMovement = true;
  doCommitMovement();
}

void doQuickMovePos() {
  mot_PlanPosition_um = mot_PlanPosition_um + mot_QuickMoveStep_um;
  doCheckPlanPosition();
  commitMovement = true;
  doCommitMovement();
}

void doQuickMoveNeg() {
   mot_PlanPosition_um = mot_PlanPosition_um - mot_QuickMoveStep_um;
  doCheckPlanPosition();
  commitMovement = true;
  doCommitMovement();
}


void cycleTemperature() {
  //todo this is a sketch

  // variable list (global)
  boolean commitCycleT;
  int cycleTNrCurrent;
  int cyclteTNrTot;
  int cycleTPhase;

  float cycleT1a;
  float cycleT1b;
  float cycleT2a;
  float cycleT2b;
  float cycleTRateA;
  float cycleTRateB;

  unsigned long cycleTPhaseStart;
  unsigned long cycleTPhasetimeA;
  unsigned long cycleTPhasetimeB;

  if (commitCycleT) {

    if ((cycleTNrCurrent <= cyclteTNrTot) || (cyclteTNrTot == 0)) {

      switch (cycleTPhase) {
        case 1:
          TempChangeRate = cycleTRateA; //
          PlanTemp2 = cycleT2a;
          PlanTemp1 = cycleT1a;
          commitChangesT = true;

          if ((PlanTemp1 == SetTemp1) && (PlanTemp1 == SetTemp1)) { // targets reached
            cycleTPhase = 2;
            cycleTPhaseStart = millis();
          }
          break;
        case 2:
          if ((millis() - cycleTPhaseStart) > cycleTPhasetimeA) {
            cycleTPhase = 3;
          }
          break;
        case 3:
          TempChangeRate = cycleTRateB; //
          PlanTemp2 = cycleT2b;
          PlanTemp1 = cycleT1b;
          commitChangesT = true;
          if ((PlanTemp1 == SetTemp1) && (PlanTemp1 == SetTemp1)) { // targets reached
            cycleTPhase = 4;
            cycleTPhaseStart = millis();
          }
          break;
        case 4:
          if ((millis() - cycleTPhaseStart) > cycleTPhasetimeB) {
            cycleTPhase = 1;
            cycleTNrCurrent ++;
          }
          break;
        default:
          // failure
          commitCycleT = false;
          break;
      }
    }
    else {
      // cycleT finished
      commitCycleT = false;
    }
  }

} // end cycleTemperature()
