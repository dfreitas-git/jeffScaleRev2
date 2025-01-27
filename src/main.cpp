/*******************************************************************************************************
This is the code to implement Jeff's 20Kg load-cell based scale.  Conditionally, we also use the 
code to build the KITTY_SCALE and the FIVE_KG_SCALE.
  
Uses an HX711 to interface to a full Whetstone Bridge load-cell.  Weight 
displayed on a 128x64 SH1106 OLED display.

Scale will auto zero upon power up. Can be re-zeroed using the menu command "ReZero".
Weight will be displayed in lbs and Kg on the OLED.  

The 9v battery is monitored via a resistor divider and analoginput.  If it drops too low, 
a low-battery warning is flashed in the display.

The scale's calibration constant is stored in EEPROM and can be reset using the "Calibrate" command.
To calibrate:
   1) Get a known reference object (around a pound in weight).
   2) Go to the calibrate menu and click on "Enter Ref"  and dial in the reference weight in x.yy pounds.
   3) Click on the "Run Cal" command.
   4) A new cal value will be calculated.  Click on "Save Cal" to store the value in EEPROM.

You can also manually adjust the calVal by clicking on "Edit Cal".  Dial in the number you want then
click on "Save Cal" to store it in EEPROM.

Scale readings can be stored in eight memory locations (M0-M7).  
- To store a value, go to the Memory menu,
  move the cursor to the location you want to store at then click the rotary switch.  Confirm you want to
  store by double-clicking.  If you single-click you will abort the store. 
- To clear an individual memory location, long-press the switch.  
- To clear all the memory locations at once, click on "Clear Mem".


We are using the SSD1306Ascii library as it's a lighter weight driver for the OLED.  The full frame-buffer
version of the library uses up too much memory in the Nano, not leaving any for additional variables...

Jeff's and the KITTY_SCALE use the wire library to implement I2C interface.  The FIVE_KG_SCALE uses an SPI display.

Using a rotary encoder with click-switch to implement a simple menu system to allow
things like storing/recalling measurments, re-zeroing the scale, re-calibrating, etc.
Currently a max of 8 rows per menu is allowed.  Clicking pushes into a menu/item.  
Double-clicking returns to the parent menu.  Rotating the knob scrolls through the 
menu items.  Library at: https://github.com/0xPIT/encoder

Uses HX711 ADC library to drive the load-cell ADC/amplifier
https://github.com/olkal/HX711_ADC

*******************************************************************************************************/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include <HX711_ADC.h>
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <EEPROM.h>

//#define KITTY_SCALE   // Settings for the kitty scale version.  Comment both out for building Jeff's version
#define FIVE_KG_SCALE   // Uncomment one or the other to build that version.  Don't uncomment both!

#ifdef FIVE_KG_SCALE
#include "SSD1306AsciiSpi.h"
SSD1306AsciiSpi oled; // Create an instance of the SPI OLED object
#else
#include "SSD1306AsciiAvrI2c.h"
SSD1306AsciiAvrI2c oled;  // Create an instance of the OLED object
#define I2C_ADDRESS 0x3c  // OLED address
#endif

// Size variables
const int NUM_MEMORY_ENTRIES = 8;  // Set up eight memory locations to store measurments

// Rotary encode pins
#ifdef KITTY_SCALE   // Wired my pins flipped, oops...
const int ENC_A = 7;
const int ENC_B = 6;
const int ENC_SW = 8;
#else
const int ENC_A = 6;
const int ENC_B = 7;
const int ENC_SW = 8;
#endif

// Battery low variables
const int BAT_PIN = A7;
int low_battery_limit = 7000;  // Display low bat message if we drop below 7v (7000mv)
unsigned long displayUpdateTimer = millis();
boolean display_low_battery = false;
int battery_voltage;

// HX711 ADC/Amplifier pins and setup
unsigned long adc_read_time = 0;
const int readInterval = 100;  // Increase value (in ms) to slow down number of readings
const int HX711_dout = 4;  //Arduino d4 pin for the data
const int HX711_sck = 5;   //Arduino d5 pin for the clock
HX711_ADC loadCell(HX711_dout, HX711_sck);

// EEPROM addresses for the calibration value and weight storage
const unsigned int calVal_eepromAdress = 0;
int mem_eepromAddress[NUM_MEMORY_ENTRIES];

// Calibration constant for the load cell - Run the HX711 calibration sketch 
// from the examples directory in Arduino IDE to get this number.  Reference weight is x.y lbs
//float calVal = 47672.54;  
float calVal;  

// Values for weight measurment
float pounds = 0.0;
float kilograms = 0.0;
float lastPounds = -1.0;       // Used for keeping previous measurment to see if measurment is stabilizing
float lastKilograms = 0.0;     // Used for keeping previous measurment to see if measurment is stabilizing
float storeArr[NUM_MEMORY_ENTRIES];   // memory storage for weight results
float calRefWeight = 1.0;      // Weight (in pounds) used for calibration.  Initialize to one pound.

// OLED Display variables
int DISPLAY_REFRESH_TIME =200; // Time (in ms) between results display update
uint8_t rowsPerChar;           // Number of rows per character (double when using 2X fonts)
uint8_t col;                   // Column that the weight fields start at
char padding[] = " ";          // Leading blanks to center the display
bool dispUpdateNeeded = true;  // This is set true only when a display refresh is needed.  That way
                               // we can eliminate a flashing screen as you need to clear a line before writing it.
  
// Rotary Encoder setup
ClickEncoder *encoder;         // Create an instance of the rotary encoder object
int last = 0;
int value = 0;
boolean buttonBeingHeld = false;  // Used to test if rotary button is being held down

// Used by the encoder library to read encoder
void timerIsr() {
  encoder->service();
}

// Menu/display state variables. 
int cursorPosition = 0;        // Which menu row we are on

// Create a stack to store the pointers to the current level menu structure array
// We push the current menu level structure onto the stack each time we push into a sub-menu.
// When returning from the sub-menu, we pop the stack to get the parent menu structure to display
// A StackPointer (sp) holds the index of the "top-of-stack".  When we "push" an entry onto the stack,
// we don't actually shift the items, we just increment the stack pointer and store the new pointer..  
// For example, when pushing into L1 menus,  we store the L1 menu array pointer in levelStack[1] and change 
// the sp to 1 so levelStack[1] is now considered the top-of-stack.  levelStack[0] is one level "down".  When 
// we pop the stack, we just decrement the sp to get back to the parent.
  
struct menuItem *levelStack[5];   // Stack -  Array of pointers to structure-arrays
int sp = 0;                       // Stack pointer (Index of the stack entry that is currenty the top-of-stack)

// Function prototype declarations
void doNothing();
void displayMenu();
void displayMessage(const char * str, int delayVal);
void displayWeights();
void clearAllMem();
void memClear();
void memStore();
void memRecall();
void rezero();
void enterKnownWeight();
void calibrate();
void editCal();
void saveCal();
void waitForClick();
int waitForClickOrDoubleClick();

// ************************************************************************************************
// Structure initialization
// One structure template used for all menu items.  
// For a given menu, we create an array of structure pointers to each of the menu's items.
// To create a new menu, define another struct array, initialize with the menu's entry structures
// and go create any necessary callback functions.
// ************************************************************************************************
struct menuItem {
   const char *menuTitle;        // Name of the menu
   int numMenuItems;             // Total number of menu items for a given level
   int menuLevel;                // L0 is the top, each sub-menu pushed the level (L1, L2, etc.)
   const char *menuItem;         // Item title
   void (*clickFuncPtr)();       // Pointer to rotary-switch "click" callback function
   void (*heldFuncPtr)();        // Pointer to rotary-switch "held" callback function
   struct menuItem *childMenu;   // Pointer to the child menu structure-array
};

// This is just a dummy placeholder for leaf-level menus that have no child as we 
// need a valid structure pointer to store in the leaf's child-structure entry.
struct menuItem noMenuPlaceholder[] = {
   "noMenuPlaceholder",1,3,"No Menu",doNothing,doNothing,noMenuPlaceholder
};

// Menu for displaying/storing/clearing each of the store-result locations.
// Currently we allow up to eight results to be stored (named M0-M7).
struct menuItem L2_mem_menu[] = {
   "L2_mem_menu",NUM_MEMORY_ENTRIES,2,"M0 ",memStore,memClear,noMenuPlaceholder,
   "L2_mem_menu",NUM_MEMORY_ENTRIES,2,"M1 ",memStore,memClear,noMenuPlaceholder,
   "L2_mem_menu",NUM_MEMORY_ENTRIES,2,"M2 ",memStore,memClear,noMenuPlaceholder,
   "L2_mem_menu",NUM_MEMORY_ENTRIES,2,"M3 ",memStore,memClear,noMenuPlaceholder,
   "L2_mem_menu",NUM_MEMORY_ENTRIES,2,"M4 ",memStore,memClear,noMenuPlaceholder,
   "L2_mem_menu",NUM_MEMORY_ENTRIES,2,"M5 ",memStore,memClear,noMenuPlaceholder,
   "L2_mem_menu",NUM_MEMORY_ENTRIES,2,"M6 ",memStore,memClear,noMenuPlaceholder,
   "L2_mem_menu",NUM_MEMORY_ENTRIES,2,"M7 ",memStore,memClear,noMenuPlaceholder
};

// Calibration menu.  Allow the user to re-calibrate the scale.  They will need to 
// supply a known weight.  The calibration is run and a new calibration constant is
// generated.  The user can manually edit the cal value as well.
// Finally, the cal constant can be stored in the EEPROM if desired.
struct menuItem L2_calibrate_menu[] = {
   "L2_calibrate_menu",4,2,"Enter Ref",enterKnownWeight,doNothing,noMenuPlaceholder,
   "L2_calibrate_menu",4,2,"Run Cal",calibrate,doNothing,noMenuPlaceholder,
   "L2_calibrate_menu",4,2,"Edit Cal",editCal,doNothing,noMenuPlaceholder,
   "L2_calibrate_menu",4,2,"Save Cal",saveCal,doNothing,noMenuPlaceholder
};

// L1 main menu.  The first level menu.  Displays additional sub-menu options.
// Click the rotary-encoder to enter a sub-menu.  Double-click to return to the
// Scale's weight screen.
struct menuItem L1_menu[] = {
   "L1_menu",4,1,"Memory",doNothing,doNothing,L2_mem_menu,
   "L1_menu",4,1,"Clear Mem",clearAllMem,doNothing,noMenuPlaceholder,
   "L1_menu",4,1,"Re-Zero",rezero,doNothing,noMenuPlaceholder,
   "L1_menu",4,1,"Calibrate",doNothing,doNothing,L2_calibrate_menu
};

// Needed to define a menu structure for the L0 level which is actually not a menu at all.
// It's the display that shows the weight, but we needed a valid structure pointer for the
// level stack so this is juat a do-nothing structure array.
struct menuItem L0_menu[] = {
   "L0_menu",1,0,"",doNothing,doNothing,L1_menu
};


// ************************************************************************************
// ************************************************************************************
// * Setup code
// ************************************************************************************
// ************************************************************************************
void setup()   {
   // Uncomment when using the serial monitor
   Serial.begin(115200);
   delay(1000);  // Wait a second to avoid double reset

   // Initialize the EEPROM address array for weight storage.  Addresses are 
   // four bytes apart as we are storing floats
   for(int i=0;i<8;i++) {
      mem_eepromAddress[i]=sizeof(float)+(i*sizeof(float));
   }

   // Load the weight storage array from the EEPROM
   for(int i=0;i<NUM_MEMORY_ENTRIES;i++) { 
      EEPROM.get(mem_eepromAddress[i], storeArr[i]);
   }
   
   // Set up battery monitor pin
   pinMode(BAT_PIN, INPUT);

   // Initalize the OLED display
   #ifdef FIVE_KG_SCALE
   oled.begin(&SH1106_128x64, 2, 9, 3);  // CS_PIN, DC_PIN, RST_PIN
   #else
   oled.begin(&SH1106_128x64, I2C_ADDRESS);
   #endif

   oled.setFont(System5x7);

   // Display a "splash screen" during boot-up
   oled.set1X();
   oled.clear();
   oled.println();
   oled.set2X();
   #ifdef KITTY_SCALE
   oled.println(F("   Range"));
   oled.set1X();
   oled.println();
   oled.set2X();
   oled.println(F(" 0-44 lbs"));
   #elif defined FIVE_KG_SCALE
   oled.println(F("   Range"));
   oled.set1X();
   oled.println();
   oled.set2X();
   oled.println(F(" 0-11 lbs"));
   #else
   oled.println(F("Property Of"));
   oled.set1X();
   oled.println();
   oled.set2X();
   oled.println(F(" J. Penney"));
   #endif
   delay(1000);
  
   // Initialize the HX711/ADC
   loadCell.begin();

   // Initialize the rotary encoder.  Enable the Arduino builtin pullup resistors.
   pinMode(ENC_A, INPUT_PULLUP);
   pinMode(ENC_B, INPUT_PULLUP);
   pinMode(ENC_SW, INPUT_PULLUP);
   encoder = new ClickEncoder(ENC_A, ENC_B, ENC_SW, 4);  // Set up with 4 steps per notch for our encoder
   encoder->setAccelerationEnabled(false);  // Don't want acceleration

   // Set up timer to use with rotary encoder.  The timer is used by the library 
   // to determine double-click, long hold, short hold, etc.
   Timer1.initialize(1000);
   Timer1.attachInterrupt(timerIsr);
  
   // Load the calibration constant from EEPROM
   // EEPROM.put(calVal_eepromAdress, 1.0);  // Uncomment for first time power-on to set to an initialization value
   EEPROM.get(calVal_eepromAdress, calVal);

   loadCell.start(3000, true);    // Start the ADC, wait a few seconds then get zero out the reading.
   loadCell.setCalFactor(calVal); // Set calibration value (float)

   // Get OLED character offsets so we know where to clear fields
   rowsPerChar = oled.fontRows();
   col = oled.fieldWidth(strlen(padding)); 

   // Initialize level-0 of the display stack.  Level-0 is the weight display. Level-1 starts the menu display.  
   // All lower levels are more layers of sub-menu.
   levelStack[0] = L0_menu;        // Initialize to display the weights
   cursorPosition = 0;             // Start with menu item cursor at first row

}


// ************************************************************************************
// ************************************************************************************
// * Loop code
// ************************************************************************************
// ************************************************************************************
void loop() {

   // If we are not displaying the weights, go update the current menu list.
   // Only update if something changed or this is the initial display of the menu.
   if(sp != 0 && dispUpdateNeeded) {
      displayMenu();
   }

   // ***************************************************************************
   // Check the rotary encoder.  This is our control knob to scroll/select menu items.
   // Display in groups of four rows as that's all we have in the OLED 2X font
   // ***************************************************************************
   value += encoder->getValue();
   int arrLen;
   if (value != last) {
      arrLen = levelStack[sp][0].numMenuItems;
      if(value > last) { 
         cursorPosition--;  // cursor moving up
         // Wrap the cursor if at the top
         if(cursorPosition < 0) {
            cursorPosition = arrLen-1;
         }
      }else{
         cursorPosition++;   // cursor moving down
         // Wrap the cursor if at the bottom
         if(cursorPosition > arrLen-1) {
            cursorPosition = 0;
         }
      }
      last = value;

      dispUpdateNeeded = true;
   }

   // ***************************************************************************
   // Check if user is entering/exiting a menu
   // See if the encoder button was clicked, double clicked, held, or released
   // ***************************************************************************
   ClickEncoder::Button b = encoder->getButton();

   if (b != ClickEncoder::Open) {
      int cursorPositionBeforeClick;
      switch (b) {

         case ClickEncoder::Released:
            buttonBeingHeld = false;
            break;

         case ClickEncoder::Clicked:
            sp++;
            cursorPositionBeforeClick = cursorPosition;
            levelStack[sp]=levelStack[sp-1][cursorPositionBeforeClick].childMenu; // Store child structure-array pointer in stack
            levelStack[sp-1][cursorPositionBeforeClick].clickFuncPtr();           // Execute parent callback for clicked item
            dispUpdateNeeded = true;
            buttonBeingHeld = false;
            break;
            
         case ClickEncoder::Held:
            if(buttonBeingHeld) {
               break;
            }else{
               sp++;
               cursorPositionBeforeClick = cursorPosition;
               levelStack[sp]=levelStack[sp-1][cursorPositionBeforeClick].childMenu; // Store child structure-array pointer in stack
               levelStack[sp-1][cursorPositionBeforeClick].heldFuncPtr();            // Execute parent callback for held item
               dispUpdateNeeded = true;
               buttonBeingHeld = true;
               break;
            }

         case ClickEncoder::DoubleClicked:
            if(levelStack[sp]->menuLevel != 0) {
               sp--;
               cursorPosition=0;
               dispUpdateNeeded = true;
            }
            break;
          default:
            break;
      }
   }    
   
   // *****************************************************
   // Go measure the object sitting on the scale
   // *****************************************************
   static boolean newDataReady = 0;

   if(loadCell.update()) {
      newDataReady = true;
   }
   if(newDataReady) {
      if(millis() > adc_read_time + readInterval) {

         // Read the HX711 to the latest measurment
         // Store the previous reading for when we want to see if the measurment is stable
         lastKilograms = kilograms;
         pounds = loadCell.getData();
         kilograms = pounds * .454;
         newDataReady = 0;
         adc_read_time = millis();
      }
   }

   // ****************************************************************
   // Check if the top level weight display update is needed
   // Only updating periodically so we don't flash the screen so much 
   // ****************************************************************
   if(sp == 0 && (millis() > (displayUpdateTimer + DISPLAY_REFRESH_TIME))) {

      // Only update the screen if the weight is changing.  When weight is stable, screen
      // stops flashing.  The "flashing" is actually the screen being cleared then re-written.
      if(abs(pounds - lastPounds) > .001 || dispUpdateNeeded){
         displayWeights();
         dispUpdateNeeded = false;
      }
      lastPounds = pounds; 
       
      // The battery is connected to an analog input pin through a 10k/10k resistor divider.
      // So, voltage at the analog pin is 1/2 the supply voltage.  We read the divider, 
      // map that to 0-5v then multiple by two to give us the actual battery voltage.

      battery_voltage = map(analogRead(BAT_PIN), 0, 1023, 0, 5000) * 2;
      if(battery_voltage < low_battery_limit) {
           
         // Will blink the warning message if the battery is low
         display_low_battery = !display_low_battery;
      } else {
         display_low_battery = false;
      }
      oled.println();
      oled.set1X();
      oled.println();

      if(display_low_battery) {        
         //oled.println(F("      Low Battery      "));
         oled.print(F("Low Battery => "));
         char str[5];
         float bv = battery_voltage/1000.0;
         sprintf(str, "%d.%02d V", (int)bv, (int)(bv*100)%100);
         oled.println(str);
      } else {
         oled.clearToEOL();
      }
      oled.set2X();
      displayUpdateTimer = millis();
  }
}

//********************************************************************
//********************************************************************
//**   Functions                                                    **
//********************************************************************
//********************************************************************

// Do nothing - no call back for the given menu/item, just going to another display level
void doNothing(){
}

//************************************************************************************
// Update the display to show the current weight measurments
// This is the L0 display level
//************************************************************************************
void displayWeights() {
         oled.clear();
         oled.set2X();
         oled.print(padding);
         oled.print("0.00");
         oled.println(F("  lbs"));
         oled.println();
         oled.print(padding);
         oled.print("0.00");
         oled.println(F("  kg"));
         oled.clearField(col,rowsPerChar*0,5);
         
         // keep the digits lined up with or without minus sign in value)
         if(pounds > 0) {
            oled.print(" ");
         }
         oled.print(pounds);
         oled.clearField(col,rowsPerChar*2,5);
         
         // keep the digits lined up with or without minus sign in value)
         if(kilograms > 0) {
            oled.print(" ");
         }
         oled.print(kilograms);    
}
//************************************************************************************
// Update the display to show the menu for a given stack level
// Display in groups of four rows as that is all the OLED can display with 2X font size
//************************************************************************************
void displayMenu(){
   int startIndex,stopIndex;
   int rows=levelStack[sp][0].numMenuItems;
   if(cursorPosition > rows -1) {
      cursorPosition = 0;
   }
   oled.clear();
   oled.set2X();

   if(cursorPosition > 3) {
      startIndex=4;
      stopIndex=rows;
   }else{
      startIndex=0;
      if(rows < 4) {
         stopIndex=rows;
      }else{
         stopIndex=4;
      }
   }
   for(int i=startIndex; i < stopIndex ; i++){
      if(cursorPosition == i) {
         oled.print(">");
      }else{
         oled.print(" ");
      }
      oled.print(levelStack[sp][i].menuItem);

      // Special case for memory menu.  We want to display the vaule for each memory location.
      if(strcmp(levelStack[sp][i].menuTitle,"L2_mem_menu") == 0) {
         oled.print(storeArr[i]);
         oled.set1X();
         oled.print(" lbs");
         oled.set2X();
         oled.println();
      }else{
         oled.println();
      }
   }
   dispUpdateNeeded = false;
}

//************************************************************************************
// Store the current measurment in the given memory location
// User has to double-click to confirm storing.  This is so they don't accidentally
// overwrite a result.  
// A single-click here will abort the store.
//************************************************************************************
void memStore() {
   int clickType; //1 for single click, 2 for double click
   displayMessage("DoubleClik\nto Store",0);
   oled.println("SingleClik\nto Abort");
   clickType=waitForClickOrDoubleClick();
   if(clickType == 2) {
      storeArr[cursorPosition]=pounds;
      EEPROM.put(mem_eepromAddress[cursorPosition], storeArr[cursorPosition]);
      EEPROM.get(mem_eepromAddress[cursorPosition], storeArr[cursorPosition]);
      displayMessage("Stored\nWeight",1000);
   }else{
      displayMessage("Store\nAborted",1000);
   }
   dispUpdateNeeded = true;
   sp--;
}

//************************************************************************************
// Clear the current measurment in the given memory location
// The user long-pushed the rotary button so just clear this one location.
//************************************************************************************
void memClear() {
   storeArr[cursorPosition]=0.00;
   EEPROM.put(mem_eepromAddress[cursorPosition], storeArr[cursorPosition]);
   dispUpdateNeeded = true;
   sp--;
}

//************************************************************************************
// Clear all the memory locations
// Easy way to clear all eight locations when starting another round of measurments.
//************************************************************************************
void clearAllMem() {
   displayMessage("Clearing\nMemory...",1000);
   for(int i=0;i<NUM_MEMORY_ENTRIES;i++) {
      storeArr[i]=0.00;
      EEPROM.put(mem_eepromAddress[i], storeArr[i]);
   }
   sp--; // Jump back to the L1 display
}

//************************************************************************************
// Re-Zero the scale.  Used when adding a weight after power on that we want to 
// null out (like a tray to put items in).
//************************************************************************************
void rezero() {
   loadCell.tareNoDelay();  // Reset the scale to zero
   displayMessage("Zeroing\nScale...",1000);
   sp-=2; // Jump back to the top weight display
   cursorPosition=0;
   dispUpdateNeeded = true;
}

//************************************************************************************
// Need to enter a know weight, in pounds,  when doing the calibration
// Rotary pot to increase/decrease value.  Terminate with a single-click
//************************************************************************************
void enterKnownWeight() {
   boolean returnFlag = false;
   float lastWeight;
   displayMessage("Rotate and\nClick To\nSet Ref",0);
   while(!returnFlag) {
      value += encoder->getValue();
      if (value != last) {
         if(value > last) { 
            calRefWeight+=.01;   // Increase reference weight
         }else{
            calRefWeight-=.01;   // Reduce the reference weight
         }
         last = value;
      }

      // Update the display with new value if it has changed
      if(abs(calRefWeight-lastWeight) >= .001) {
         oled.clearField(col,rowsPerChar*3,10);
         oled.print(calRefWeight);
         oled.print(" lbs");
         lastWeight=calRefWeight;
      }

      // Go see if they clicked to confirm
      ClickEncoder::Button button = encoder->getButton();
      if (button != ClickEncoder::Open) {
         switch (button) {
            case ClickEncoder::Clicked:
               sp--;
               dispUpdateNeeded = true;
               returnFlag=true;
               break;
            default:
               break;
               
         }
      }
   }
}

//************************************************************************************
// Run the calibration code.  Creates a new scale calibration constant
//************************************************************************************
void calibrate() {
   displayMessage("Remove Any\nWeight on\nScale then\nclick",0);
   waitForClick();

   displayMessage("Resetting\ncalVal\nFactor...",0);

   loadCell.begin();
   loadCell.start(2000, true);    // Start the ADC, wait a few seconds then get zero out the reading.
   loadCell.setCalFactor(1.0);    // Calibration value (float).  Library uses 1.0 as an initial starting point.
   while (!loadCell.update());    // Make sure we aren't in the middle of a read

   displayMessage("Place Ref\nWeight On\nScale Then\nclick",0);
   waitForClick();

   displayMessage("Calibrating",0);
   loadCell.update();
   loadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
   calVal = loadCell.getNewCalibration(calRefWeight); //get the new calibration value
   oled.println("\nNew calVal");
   oled.println(calVal);
   delay(2000);
   sp--;
}

//************************************************************************************
// Edit the calibration constant
// Allow user to manually tweak the calibration constant if they find the scale
// slightly drifted and they want to trim it without doing a full re-calibration.
// The value is not stored permanently unless the user clicks on the "Save Cal" menu.
//************************************************************************************
void editCal() {
   boolean returnFlag = false;
   float lastCalVal;
   displayMessage("Rotate and\nClick To\nEdit calVal",0);

   // Round off existing calVal to look nicer in display...
   calVal=round(calVal) * 1.0;
   while(!returnFlag) {
      value += encoder->getValue();
      if (value != last) {
         if(value > last) { 
            calVal+=1.0;   // Increase the calibration value
         }else{
            calVal-=1.0;   // Reduce the calibration value
         }
         last = value;
      }

      // Update the display with new value if it has changed
      if(abs(calVal-lastCalVal) >= .1) {
         oled.clearField(col,rowsPerChar*3,10);
         oled.print(calVal);
         lastCalVal=calVal;
      }

      // Go see if they clicked to confirm
      ClickEncoder::Button button = encoder->getButton();
      if (button != ClickEncoder::Open) {
         switch (button) {
            case ClickEncoder::Clicked:
               sp--;
               dispUpdateNeeded = true;
               returnFlag=true;
               break;
            default:
               break;
         }
      }
   }
}

//************************************************************************************
// Save the calibration constant to EEPROM
//************************************************************************************
void saveCal() {
   EEPROM.put(calVal_eepromAdress, calVal);
   displayMessage("Saving",0);
   oled.println(calVal);
   oled.println("to EEPROM");
   delay(2000);
   sp--;
}

//************************************************************************************
// Clear the OLED and display the message for delayVal length of time
//************************************************************************************
void displayMessage(const char * str, int delayVal) {
   oled.clear();
   oled.set2X();
   oled.println(str);
   delay(delayVal);
}

//************************************************************************************
// Wait for a click to proceed
// Used when waiting for the user to respond to a menu request
//************************************************************************************
void waitForClick() {
   boolean returnFlag = false;
   ClickEncoder::Button btn;
   while(!returnFlag) {
      btn = encoder->getButton();
      delay(500); // Encoder lib seems to need some delay between reading the button testing result
      if (btn != ClickEncoder::Open) {
         if(btn == ClickEncoder::Clicked) {
            returnFlag=true;
         }
      }
   }
}

//************************************************************************************
// Wait for click or double-click to proceed
// Return a 1 for single click, 2 for double click
//************************************************************************************
int waitForClickOrDoubleClick() {
   boolean returnFlag = false;
   int returnResult;
   ClickEncoder::Button btn;
   while(!returnFlag) {
      btn = encoder->getButton();
      delay(500); // Encoder lib seems to need some delay between reading the button testing result
      if (btn != ClickEncoder::Open) {
         if(btn == ClickEncoder::Clicked) {
            returnResult=1;
            returnFlag=true;
         }
         if(btn == ClickEncoder::DoubleClicked) {
            returnResult=2;
            returnFlag=true;
         }
      }
   }
   return(returnResult);
}