/*
  ----------------------------------------------------------
  =    //        //            ///               ///       =
  =    ////      //         ///  ///         ///    ///    =
  =    // //     //         ///              ///           =
  =    //  //    //          ///              ///          =
  =    //   //   //            ///              ///        =
  =    //    //  //              ///              ///      =
  =    //     // //               ///              ///     =
  =    //      ////        ///   ///         ///   ///     =
  =    //       ///           ///               ///        =
  ----------------------------------------------------------
*/

//AUTHOR: Ivan Gourlay 22 June 2016
//VERSION: 2.6 (9 May 2017)
/*
   REVISION HISTORY:
   27 Jun 2016
   08 Jul 2016
   11 Jul 2016
   15 Jul 2016
   18 Jul 2016
   29 Jul 2016

   02 Aug 2016
   03 Aug 2016
   16 Aug 2016
   18 Aug 2016
   25 Aug 2016
   26 Aug 2016
   27 Aug 2016
   29 Aug 2016
   30 Aug 2016

   04 Sept 2016
   13 Sept 2016
   14 Sept 2016
   19 Sept 2016
   26 Sept 2016
   27 Sept 2016
   29 Sept 2016

   06 Oct 2016
   12 Oct 2016
   17 Oct 2016
   25 Oct 2016

   04 Nov 2016
   07 Nov 2016
   30 Nov 2016

   02 Dec 2016
   12 Dec 2016
   13 Dec 2016
   14 Dec 2016
   15 Dec 2016
   20 Dec 2016
   23 Dec 2016
   24 Dec 2016
   30 Dec 2016

   03 Jan 2017
   10 Jan 2017
   19 Jan 2017
   22 Jan 2017
   26 Jan 2017

   18 Feb 2017
   19 Feb 2017
   20 Feb 2017
   21 Feb 2017
   23 Feb 2017
   24 Feb 2017
   25 Feb 2017
   27 Feb 2017
   28 Feb 2017

   01 Mar 2017
   02 Mar 2017
   16 Mar 2017

   04 Apr 2017
   10 Apr 2017
   12 Apr 2017
   14 Apr 2017
   15 Apr 2017
   24 Apr 2017
*/

#include <EEPROM.h>
#include "Mouse.h"
#include <math.h>

//***PIN ASSIGNMENTS***//

#define MODE_SELECT 12                            // LipSync Mode Select - USB mode (comm_mode = 0; jumper on) or Bluetooth mode (comm_mode = 1; jumper off) - digital input pin 12 (internally pulled-up)
#define PUSH_BUTTON_UP 8                          // Cursor Control Button 1: UP - digital input pin 8 (internally pulled-up)
#define PUSH_BUTTON_DOWN 7                        // Cursor Control Button 2: DOWN - digital input pin 7 (internally pulled-up)
#define LED_1 4                                   // LipSync LED Color1 : GREEN - digital output pin 5
#define LED_2 5                                   // LipSync LED Color2 : RED - digital outputpin 4

#define TRANS_CONTROL A3                          // Bluetooth Transistor Control Pin - digital output pin A3
#define PIO4 A4                                   // Bluetooth PIO4 Command Pin - digital output pin A4

#define PRESSURE_CURSOR A5                        // Sip & Puff Pressure Transducer Pin - analog input pin A5
#define X_DIR_HIGH A0                             // X Direction High (Cartesian positive x : right) - analog input pin A0
#define X_DIR_LOW A1                              // X Direction Low (Cartesian negative x : left) - digital output pin A1
#define Y_DIR_HIGH A2                             // Y Direction High (Cartesian positive y : up) - analog input pin A2
#define Y_DIR_LOW A10                             // Y Direction Low (Cartesian negative y : down) - analog input pin A10

//***VARIABLE DECLARATION***//
byte number; 


unsigned long start, finished, elapsed;
long lastMillis = 0; //for timing 
long loops = 0; // for timing 

int Smoothdatax ; 
float LPF_Beta = 1; 
int Smoothdatay ;

int xh, yh, xl, yl;                               // xh: x-high, yh: y-high, xl: x-low, yl: y-low
int x_right, x_left, y_up, y_down;                // individual neutral starting positions for each FSR

int xh_max, xl_max, yh_max, yl_max;               // may just declare these variables but not initialize them because
// these values will be pulled from the EEPROM

float constant_radius = 30.0;                     // constant radius is initialized to 30.0 but may be changed in joystick initialization
float xh_yh_radius, xh_yl_radius, xl_yl_radius, xl_yh_radius;
float xh_yh, xh_yl, xl_yl, xl_yh;
int box_delta;                                    // the delta value for the boundary range in all 4 directions about the x,y center
int cursor_delta;                                 // amount cursor moves in some single or combined direction
int speed_counter = 4;                            // cursor speed counter
int cursor_click_status = 0;                      // value indicator for click status, ie. tap, back and drag
int comm_mode = 0;                                // 0 == USB Communications or 1 == Bluetooth Communications
int config_done;                                  // Binary check of completed Bluetooth configuration
unsigned int puff_count, sip_count;               // int puff and long sip incremental counter :: changed from unsigned long to unsigned int

int poll_counter = 0;                             // cursor poll counter
int init_counter_A = 0;                           // serial port initialization counter
int init_counter_B = 0;                           // serial port initialization counter

int default_cursor_speed = 30;
int delta_cursor_speed = 5;

int cursor_delay;
float cursor_factor;
int cursor_max_speed;


float yh_comp = 1.0;
float yl_comp = 1.0;
float xh_comp = 1.0;
float xl_comp = 1.0;

float yh_check, yl_check, xh_check, xl_check;
int xhm_check, xlm_check, yhm_check, ylm_check;
float sip_threshold, puff_threshold, cursor_click, cursor_back;

typedef struct {
  int _delay;
  float _factor;
  int _max_speed;
} _cursor;

_cursor setting1 = {5, -1.1, default_cursor_speed - (4 * delta_cursor_speed)}; // 5,-1.0,10
_cursor setting2 = {5, -1.1, default_cursor_speed - (3 * delta_cursor_speed)}; // 5,-1.2,10
_cursor setting3 = {5, -1.1, default_cursor_speed - (2 * delta_cursor_speed)};
_cursor setting4 = {5, -1.1, default_cursor_speed - (delta_cursor_speed)};
_cursor setting5 = {5, -1.1, default_cursor_speed};
_cursor setting6 = {5, -1.1, default_cursor_speed + (delta_cursor_speed)};
_cursor setting7 = {5, -1.1, default_cursor_speed + (2 * delta_cursor_speed)};
_cursor setting8 = {5, -1.1, default_cursor_speed + (3 * delta_cursor_speed)};
_cursor setting9 = {5, -1.1, default_cursor_speed + (4 * delta_cursor_speed)};

_cursor cursor_params[9] = {setting1, setting2, setting3, setting4, setting5, setting6, setting7, setting8, setting9};

/*variables added for zone speed increase*/
int speeds[17] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
unsigned int *defaultspeed = &speeds[3]; //pointer to 3 

int single = 0;
int puff1, puff2;

//-----------------------------------------------------------------------------------------------------------------------------------

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {
  Serial.println("hey!");
int c; 
  Serial.begin(115200);                           // setting baud rate for serial coms for diagnostic data return from Bluetooth and microcontroller ***MAY REMOVE LATER***
  Serial1.begin(115200);                          // setting baud rate for Bluetooth module

  pinMode(LED_1, OUTPUT);                         // visual feedback #1
  pinMode(LED_2, OUTPUT);                         // visual feedback #2
  pinMode(TRANS_CONTROL, OUTPUT);                 // transistor pin output
  pinMode(PIO4, OUTPUT);                          // command mode pin output

  pinMode(PRESSURE_CURSOR, INPUT);                // pressure sensor pin input
  pinMode(X_DIR_HIGH, INPUT);                     // redefine the pins when all has been finalized
  pinMode(X_DIR_LOW, INPUT);                      // ditto above
  pinMode(Y_DIR_HIGH, INPUT);                     // ditto above
  pinMode(Y_DIR_LOW, INPUT);                      // ditto above

  pinMode(MODE_SELECT, INPUT_PULLUP);             // LOW: USB (default with jumper in) HIGH: Bluetooth (jumper removed)
  pinMode(PUSH_BUTTON_UP, INPUT_PULLUP);          // increase cursor speed button
  pinMode(PUSH_BUTTON_DOWN, INPUT_PULLUP);        // decrease cursor speed button

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  delay(2000);                                    // DO NOT REMOVE DELAY!!!

  //while(!Serial);

  while(!Serial1);
  
  Joystick_Initialization();                      // home joystick and generate movement threshold boundaries
  delay(10);
  Pressure_Sensor_Initialization();
  delay(10);
  Set_Default();                                  // should only occur once per initialization of a new microcontroller
  delay(10);
  //Serial_Initialization();
  delay(10);
  Communication_Mode_Status();                    // identify the selected communication mode
  delay(10);
  Mouse_Configure();                              // conditionally activate the HID mouse functions
  delay(10);
  BT_Configure();                                 // conditionally configure the Bluetooth module [WHAT IF A NEW BT MODULE IS INSTALLED?]
  delay(10);
  cursor_speed_value();                           // reads saved cursor speed parameter from EEPROM
  delay(10);

  int exec_time = millis();
  Serial.print("Configuration time: ");
  Serial.println(exec_time);

  blink(4, 250, 3);                               // end initialization visual feedback

  Force_Cursor_Display();
  Display_Feature_List();  

  cursor_delay = cursor_params[speed_counter]._delay;
  cursor_factor = cursor_params[speed_counter]._factor;
  cursor_max_speed = cursor_params[speed_counter]._max_speed;

  // functions below are for diagnostic feedback only

  Serial.print("config_done: ");
  Serial.println(EEPROM.get(0, puff1));
  delay(5);
  Serial.print("speed_counter: ");
  Serial.println(EEPROM.get(2, puff2));
  delay(5);
  Serial.print("cursor_delay: ");
  Serial.println(cursor_params[puff2]._delay);
  delay(5);
  Serial.print("cursor_factor: ");
  Serial.println(cursor_params[puff2]._factor);
  delay(5);
  Serial.print("cursor_max_speed: ");
  Serial.println(cursor_params[puff2]._max_speed);
  delay(5);
  Serial.print("yh_comp factor: ");
  Serial.println(EEPROM.get(6, yh_check));
  delay(5);
  Serial.print("yl_comp factor: ");
  Serial.println(EEPROM.get(10, yl_check));
  delay(5);
  Serial.print("xh_comp factor: ");
  Serial.println(EEPROM.get(14, xh_check));
  delay(5);
  Serial.print("xl_comp factor: ");
  Serial.println(EEPROM.get(18, xl_check));
  delay(5);
  Serial.print("xh_max: ");
  Serial.println(EEPROM.get(22, xhm_check));
  delay(5);
  Serial.print("xl_max: ");
  Serial.println(EEPROM.get(24, xlm_check));
  delay(5);
  Serial.print("yh_max: ");
  Serial.println(EEPROM.get(26, yhm_check));
  delay(5);
  Serial.print("yl_max: ");
  Serial.println(EEPROM.get(28, ylm_check));
  delay(5);
  
}

//-----------------------------------------------------------------------------------------------------------------------------------

//***START OF INFINITE LOOP***//

void loop() {
  /// code for counting how many cycles per second (roughly)
  start = millis();
  // code for counting how many cycles per second (roughly)
  long currentMillis = 1;//millis();not in USE
  loops++;//not in use 
  delay(4); //as we increase this delay mouse movement will decrease 
      /*LOG OF USEABILITY OF MOUSE WITH DIFFERENT DELAYs (all of these consider the integrated math) 
        * delay(0) delta 5 (x) a little unstable 6-7 ms
        * delay 1 delta 4 is okay 8-9ms 
        * delay 2 delta 4 is okay but a little too slow 9-10ms
        * delay 1 delta 3 would be a good correction speed same for delta 2
        */
  
  /*
    if (single == 0) {
      int ral3 = 5;
      EEPROM.put(4, ral3);        // testing Bluetooth config ***CAN BE REMOVED
      my_delay(100);

    single++;
    }
    //*/

  xh = analogRead(X_DIR_HIGH);                    // A0 :: NOT CORRECT MAPPINGS
  xl = analogRead(X_DIR_LOW);                     // A1
  yh = analogRead(Y_DIR_HIGH);                    // A2
  yl = analogRead(Y_DIR_LOW);                     // A10
///*
  xh_yh = sqrt(sq(((xh - x_right) > 0) ? (float)(xh - x_right) : 0.0) + sq(((yh - y_up) > 0) ? (float)(yh - y_up) : 0.0));     // sq() function raises input to power of 2, returning the same data type int->int ...
  xh_yl = sqrt(sq(((xh - x_right) > 0) ? (float)(xh - x_right) : 0.0) + sq(((yl - y_down) > 0) ? (float)(yl - y_down) : 0.0));   // the sqrt() function raises input to power 1/2, returning a float type
  xl_yh = sqrt(sq(((xl - x_left) > 0) ? (float)(xl - x_left) : 0.0) + sq(((yh - y_up) > 0) ? (float)(yh - y_up) : 0.0));      // These are the vector magnitudes of each quadrant 1-4. Since the FSRs all register
  xl_yl = sqrt(sq(((xl - x_left) > 0) ? (float)(xl - x_left) : 0.0) + sq(((yl - y_down) > 0) ? (float)(yl - y_down) : 0.0));    // a larger digital value with a positive application force, a large negative difference
/*/
/*
  xh_yh = ((xh - x_right) > 0 ? (float)(xh - x_right) : 0) + ((yh - y_up) > 0 ? (float)(yh - y_up) : 0);
  xh_yl = ((xh - x_right) > 0 ? (float)(xh - x_right) : 0) + ((yl - y_down) > 0 ? (float)(yl - y_down) : 0);
  xl_yh = ((xl - x_left) > 0 ? (float)(xl - x_left) : 0) + ((yh - y_up) > 0 ? (float)(yh - y_up) : 0);
  xl_yl = ((xl - x_left) > 0 ? (float)(xl - x_left) : 0) + ((yl - y_down) > 0 ? (float)(yl - y_down) : 0);
*/
  if ((xh_yh > xh_yh_radius) || (xh_yl > xh_yl_radius) || (xl_yl > xl_yl_radius) || (xl_yh > xl_yh_radius)) {

    poll_counter++;

   // my_delay(0);    // originally 15 ms vary this MAY 7th fastest speed 1/100th delay //////////////////////////////////////////////////////////////SW changed to 1 from 20 is alot faster. 

    if (poll_counter >= 3) {

      if (comm_mode == 0) {

        if ((xh_yh >= xh_yl) && (xh_yh >= xl_yh) && (xh_yh >= xl_yl)) {
          //Serial.println("quad1");
          Mouse.move(x_cursor_high(xh), y_cursor_high(yh), 0);
          delay(cursor_delay);
          Serial.println(cursor_delay);
          poll_counter = 0;
        } else if ((xh_yl > xh_yh) && (xh_yl > xl_yl) && (xh_yl > xl_yh)) {
          //Serial.println("quad4");
          Mouse.move(x_cursor_high(xh), y_cursor_low(yl), 0);
          delay(cursor_delay);
          poll_counter = 0;
        } else if ((xl_yl >= xh_yh) && (xl_yl >= xh_yl) && (xl_yl >= xl_yh)) {
          //Serial.println("quad3");
          Mouse.move(x_cursor_low(xl), y_cursor_low(yl), 0);
          delay(cursor_delay);
          poll_counter = 0;
        } else if ((xl_yh > xh_yh) && (xl_yh >= xh_yl) && (xl_yh >= xl_yl)) {
          //Serial.println("quad2");
          Mouse.move(x_cursor_low(xl), y_cursor_high(yh), 0);
          delay(cursor_delay);
          poll_counter = 0;
        }
      } else {

        if ((xh_yh >= xh_yl) && (xh_yh >= xl_yh) && (xh_yh >= xl_yl)) {
          //Serial.println("quad1");
          //code added in to test if making these 1 works or not
          mouseCommand(cursor_click_status, x_cursor_high(xh), y_cursor_high(yh) , 0, currentMillis ,loops );
         // mouseCommand(cursor_click_status, x_cursor_high(xh), y_cursor_high(yh), 0); //old function
         
          delay(cursor_delay);
          
                /*FINDS THE NUMBER OF MILLISECONDS IT TAKES TO COMPLETE THE INSTRUCTION
                 * FOR DEBUGGING LINES 368-372
                 */
              /*  finished = millis();
                Serial.println("Finished");
                elapsed=finished-start;
                Serial.print(elapsed);
                Serial.println(" milliseconds elapsed");*/
                
          poll_counter = 0;
        } else if ((xh_yl > xh_yh) && (xh_yl > xl_yl) && (xh_yl > xl_yh)) {
          //Serial.println("quad4");
          mouseCommand(cursor_click_status, x_cursor_high(xh), y_cursor_low(yl), 0,currentMillis  ,loops ); 
          //mouseCommand(cursor_click_status, x_cursor_high(xh), y_cursor_low(yl), 0); //old function added currentMillis and loops to potentially track the time 
          
          delay(cursor_delay);
       

               //gets milliseconds and subtracts the finished millisecond ammount by the starting amount to get the number of milliseconds it takes to complete an instruction.
            /*  finished = millis();
              Serial.println("Finished");
              elapsed=finished-start;
              Serial.print(elapsed);
              Serial.println(" milliseconds elapsed");*/
              //*****************************************************************************************************************************************************************

          
          poll_counter = 0;
        } else if ((xl_yl >= xh_yh) && (xl_yl >= xh_yl) && (xl_yl >= xl_yh)) {
          //Serial.println("quad3");
       
           mouseCommand(cursor_click_status, x_cursor_low(xl), y_cursor_low(yl), 0,currentMillis ,loops );
          //mouseCommand(cursor_click_status, x_cursor_low(xl), y_cursor_low(yl), 0);
        
         delay(cursor_delay);
       
        /*
            finished = millis();
            Serial.println("Finished");
            elapsed=finished-start;
            Serial.print(elapsed);
            Serial.println(" milliseconds elapsed"); */
         
          
          poll_counter = 0;
        } else if ((xl_yh > xh_yh) && (xl_yh >= xh_yl) && (xl_yh >= xl_yl)) {
          //Serial.println("quad2");
         
           mouseCommand(cursor_click_status,  x_cursor_low(xl), y_cursor_high(yh), 0,currentMillis ,loops );
          //mouseCommand(cursor_click_status, x_cursor_low(xl), y_cursor_high(yh), 0);
          
          delay(cursor_delay);
            
          /*
             finished = millis();
              Serial.println("Finished");
              elapsed=finished-start;
              Serial.print(elapsed);
              Serial.println(" milliseconds elapsed");*/

          poll_counter = 0;
        }
      }
    }
  }

  //cursor speed control push button functions below

  if (digitalRead(PUSH_BUTTON_UP) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
      Joystick_Calibration();
    } else {
     // increase_cursor_speed();      // increase cursor speed with push button up. works with exponential but not with zone and timer
     increasecursorspeed(); //new function to test increasing speed on zone movement method
 
    }
  }

  if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_UP) == LOW) {
      Joystick_Calibration();
    } else {
      //decrease_cursor_speed();      // decrease cursor speed with push button down
      decreasecursorspeed(); //new function to test zone movement method
   
    }
  }

  //pressure sensor sip and puff functions below

  cursor_click = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;

  if (cursor_click < puff_threshold) {
    while (cursor_click < puff_threshold) {
      cursor_click = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;
      puff_count++;         // NEED TO FIGURE OUT ROUGHLY HOW LONG ONE CYCLE OF THIS WHILE LOOP IS -> COUNT THRESHOLD
      delay(5);  
    }
    Serial.println(puff_count);             //***REMOVE

    if (comm_mode == 0) {
      if (puff_count < 150) {
        if (Mouse.isPressed(MOUSE_LEFT)) {
          Mouse.release(MOUSE_LEFT);
        } else {
          Mouse.click(MOUSE_LEFT);
        //  my_delay(5);  //********************************************************************************************************
        }
      } else if (puff_count > 150 && puff_count < 750) {
        if (Mouse.isPressed(MOUSE_LEFT)) {
          Mouse.release(MOUSE_LEFT);
        } else {
          Mouse.press(MOUSE_LEFT);
         // my_delay(5);
        }
      } else if (puff_count > 750) {
        blink(4, 350, 3);   // visual prompt for user to release joystick for automatic calibration of home position
        Manual_Joystick_Home_Calibration();
      }
    } else {
      if (puff_count < 150) {
        cursor_click_status = 1; //change this stuff to hex
         mouseCommand(cursor_click_status, 0, 0, 0,0,0); /////////////////////////////////ADDED MAY 10
//        mouseCommand(cursor_click_status, 0, 0, 0);
        mouseClear();
        cursor_click_status = 0;
       //delay(5); ///can possibly change **************************************************************************************************************
      } else if (puff_count > 150 && puff_count < 750) {
        if (cursor_click_status == 0) {
          cursor_click_status = 1;
        } else if (cursor_click_status == 1) {
          cursor_click_status = 0;
        }
      } else if (puff_count > 750 && puff_count < 1250 ) { //****************************************************************************************************************changing this to make a function to draw a circle/square
        blink(4, 350, 3);     // visual prompt for user to release joystick for automatic calibration of home position
        Manual_Joystick_Home_Calibration();
      }
      else if(puff_count >1250) {
      blink(3, 1000,2); // blinks 3 times with a delay of 1000 then blinks RED 
//       Draw_square();
      }
    }

    puff_count = 0;
  }

  if (cursor_click > sip_threshold) {
    while (cursor_click > sip_threshold) {
      cursor_click = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;
      sip_count++;         // NEED TO FIGURE OUT ROUGHLY HOW LONG ONE CYCLE OF THIS WHILE LOOP IS -> COUNT THRESHOLD
     delay(5); 
    }
    Serial.println(sip_count);             //***REMOVE

    if (comm_mode == 0) {
      if (sip_count < 150) {
        Mouse.click(MOUSE_RIGHT);
        delay(5);
      } else if (sip_count > 150 && sip_count < 750) {
        mouseScroll();
        delay(5);
      } else {
        sip_secondary();
        delay(5);
      }
    } else {
      if (sip_count < 150) {
        cursor_click_status = 2;
         mouseCommand(cursor_click_status, 0, 0, 0,0,0);////////////////////////////////////////////ADDED MAY 10
//        mouseCommand(cursor_click_status, 0, 0, 0);
        cursor_click_status = 0;
        mouseClear();
       delay(5);//*****************************************************************************************************************************************************can maybe 
      } else if (sip_count > 150 && sip_count < 750) {
        mouseScroll();
        delay(5);
      } else if (sip_count > 750) {
        sip_secondary();
        delay(5);
      }
    }
    sip_count = 0;
  }
}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------------------------------------------------------


void Display_Feature_List(void) {
/*
  Serial.println(" ");
  Serial.println(" --- ");
  Serial.println("This is the 09 May - FSR Bluetooth Euclidean WIP");
  Serial.println(" ");
  Serial.println("Enhanced functions:");
  Serial.println(" ");
  Serial.println("Tap and drag");
  Serial.println("Scrolling");
  Serial.println("Joystick calibration");
  Serial.println("Middle mouse button");
  Serial.println("Hands-free home positioning reset");
  Serial.println("Cursor security swipe function");
  Serial.println("Bluetooth capable connectivity");
  Serial.println(" --- ");
  Serial.println(" ");
*/
}

//***CURSOR: SIP SECONDARY FUNCTION SELECTION***//
void sip_secondary(void) {
  while (1) {

    xh = analogRead(X_DIR_HIGH);                    // A0 :: NOT CORRECT MAPPINGS
    xl = analogRead(X_DIR_LOW);                     // A1
    yh = analogRead(Y_DIR_HIGH);                    // A2
    yl = analogRead(Y_DIR_LOW);                     // A10

    digitalWrite(LED_2, HIGH);

    if (xh > (x_right + 50)) {
      mouse_middle_button();
      break;
    } else if (xl > (x_left + 50)) {
      mouse_middle_button();
      break;
    } else if (yh > (y_up + 50)) {
      cursor_swipe();
      break;
    } else if (yl > (y_down + 50)) {
      cursor_swipe();
      break;
    }
  }
  digitalWrite(LED_2, LOW);
}

void cursor_swipe(void) {
  Serial.println("gotcha");
  if (comm_mode == 0) {

    for (int i = 0; i < 3; i++) Mouse.move(0, 126, 0);
    Mouse.press(MOUSE_LEFT);
    delay(125);

    for (int j = 0; j < 3; j++) Mouse.move(0, -126, 0);
    Mouse.release(MOUSE_LEFT);
    delay(125);
  } else {

    cursor_click_status = 0;
    for (int i = 0; i < 3; i++) mouseCommand(cursor_click_status, 0, 126, 0,0,0); ////////////////////////////////////////////ADDED MAY 10
    //mouseCommand(cursor_click_status, 0, 126, 0);
    delay(125);

    cursor_click_status = 1;
    for (int j = 0; j < 3; j++) mouseCommand(cursor_click_status, 0, -126, 0,0,0); ////////////////////////////////////////ADDED MAY 10
    //////mouseCommand(cursor_click_status, 0, -126, 0);
    mouseClear();
    cursor_click_status = 0;
    delay(125);
  }
}

void mouse_middle_button(void) {
  Serial.println("too slow");
  if (comm_mode == 0) {
    Mouse.click(MOUSE_MIDDLE);
    delay(125);
  } else {
    cursor_click_status = 0x05;
     mouseCommand(cursor_click_status, 0, 0, 0,0,0); ////////////////////////////////////////////////////////////////////ADDED MAY 10 
//    mouseCommand(cursor_click_status, 0, 0, 0);
    delay(125);
    mouseClear();
    cursor_click_status = 0x00;
    delay(125);
  }
}

//***LED BLINK FUNCTIONS***//
//led = 1 then red light led = 2 is green light 
void blink(int num_Blinks, int delay_Blinks, int LED_number ) {
  if (num_Blinks < 0) num_Blinks *= -1;

  switch (LED_number) {
    case 1: {
        for (int i = 0; i < num_Blinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_1, LOW);
          delay(delay_Blinks);
        }
        break;
      }
    case 2: {
        for (int i = 0; i < num_Blinks; i++) {
          digitalWrite(LED_2, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_2, LOW);
          delay(delay_Blinks);
        }
        break;
      }
    case 3: {
        for (int i = 0; i < num_Blinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_1, LOW);
          delay(delay_Blinks);
          digitalWrite(LED_2, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_2, LOW);
          delay(delay_Blinks);
        }
        break;
      }
  }
}

//***HID MOUSE CURSOR SPEED FUNCTIONS***//

void cursor_speed_value(void) {
  int var;
  EEPROM.get(2, var);
  delay(5);
  *defaultspeed = var;
}
/*

void decrease_cursor_speed(void) {
  speed_counter--;

  if (speed_counter == -1) {
    blink(6, 50, 3);     // twelve very fast blinks
    speed_counter = 0;
  } else if (speed_counter == 0) {
    blink(1, 350, 1);

    cursor_delay = cursor_params[speed_counter]._delay;
    cursor_factor = cursor_params[speed_counter]._factor;
    cursor_max_speed = cursor_params[speed_counter]._max_speed;

    EEPROM.put(2, speed_counter);
    delay(25);
    Serial.println("-");
  } else {
    blink(speed_counter, 100, 1);

    cursor_delay = cursor_params[speed_counter]._delay;
    cursor_factor = cursor_params[speed_counter]._factor;
    cursor_max_speed = cursor_params[speed_counter]._max_speed;

    EEPROM.put(2, speed_counter);
    delay(25);
    Serial.println("-");
  }
}
*/
//***HID MOUSE CURSOR MOVEMENT FUNCTIONS***//
/* 
 *  NOTE ALL MATH seems to add 1-2 milliseconds to cycle completion time 
 */
int y_cursor_high(int j) {
  //int k =40

 
  if (j > y_up) {

  float y_up_factor =  1.25 * (yh_comp * (((float)(j - y_up)) / (yh_max - y_up)));
//-1*j;
    int k = (int)(round(-1.0 * pow(cursor_max_speed, y_up_factor)) - 1.0);
    
//int k =1 ;
    if (k <= (-1 * cursor_max_speed) ) {
      k = -1 * cursor_max_speed;
      return k;
    } else if ( (k < 0) && (k > (-1 * cursor_max_speed))) {
      return k;
    } else {
      k = 0;
      return k;
    }
  } else {
    return 0;
  }
  
}

int y_cursor_low(int j) {

  if (j > y_down) {
   Serial.println("y down, j, m ");
    Serial.println(y_down, DEC);
    Serial.println(j, DEC);
    Serial.println(yl_max, DEC);
    Serial.println(yl_comp, DEC);
    //Serial.println(y_down, DEC);
    float y_down_factor = 1.25 * (yl_comp * (((float)(j - y_down)) / (yl_max - y_down)));
// j; 
    int k=(int)(round(1.0 * pow(cursor_max_speed, y_down_factor)) - 1.0);
//int k = 1;
    if (k >= cursor_max_speed) {

      k = cursor_max_speed;
      return k;
    } else if ((k > 0) && (k < cursor_max_speed)) {
      return k;
    } else {
      k = 0;
      return k;
    }
  } else {
    return 0;
  }
}

int x_cursor_high(int j) {

  if (j > x_right) {
 //   Serial.println("
 //   Serial.println(
    float x_right_factor =1.25 * (xh_comp * (((float)(j - x_right)) / (xh_max - x_right)));

    int k =(int)(round(1.0 * pow(cursor_max_speed, x_right_factor)) - 1.0);
 //int k =1 ; 
    if (k >= cursor_max_speed) {

      k = cursor_max_speed;
      return k;
    } else if ((k > 0) && (k < cursor_max_speed)) {
      return k;
    } else {
      k = 0;
      return k;
    }
  } else {
    return 0;
  }
}

int x_cursor_low(int j) {

  if (j > x_left) {

   float x_left_factor = 1.25 * (xl_comp * (((float)(j - x_left)) / (xl_max - x_left))); //math was commented out to see how it would cost the performance of the mouse 
//-1*j;
    int k =  (int)(round(-1.0 * pow(cursor_max_speed, x_left_factor)) - 1.0);
 //int k =1 ;
    if ( k <= (-1 * cursor_max_speed) ) {
      k = -1 * cursor_max_speed;
      return k;
    } else if ( (k < 0) && (k > -1 * cursor_max_speed)) {
      return k;
    } else {
      k = 0;
      return k;
    }
  } else {
    return 0;
  }
}

//***BLUETOOTH HID MOUSE FUNCTIONS***//

void mouseCommand(int buttons, int x, int y, int scroll, long currentMillis,long loops) {
  
  /*
   * the following changes were made by shaemusw lines 855-887 may 8th 2018
   */

//**********************************************************************//for debugging
/*char buffer[10];              
sprintf (buffer, "x: %d, y: %d ",x, y ); // prints the x and y vectors
Serial.println(buffer);*/
//********************************************************************




if (x>0){
  x = smoothmovesswitchxpos(x);
}
else{
  x = smoothmovesswitchxneg(x);
}
if(y>0){
  y = smoothmovesswitchypos(y);
}
else{
  y=smoothmovesswitchyneg(y);
}


Smoothdatax = Smoothdatax - (LPF_Beta * (Smoothdatax - x));
Smoothdatay = Smoothdatay - (LPF_Beta * (Smoothdatay - y));
/*
 * 
 * END OF CHANGES 
 */

  byte BTcursor[7];

  BTcursor[0] = 0xFD;
  BTcursor[1] = 0x5;
  BTcursor[2] = 0x2;
  BTcursor[3] = lowByte(buttons); //check this out reads 
  BTcursor[4] = lowByte(Smoothdatax);       //check this out
  BTcursor[5] = lowByte(Smoothdatay);       //check this out
  BTcursor[6] = lowByte(scroll);  //check this out
  Serial1.write(BTcursor, 7);
 
  
 // Serial1.flush();

  //my_delay(0);    // reduced poll_counter delay by 30ms so increase my_delay(10) to delay (40)   **************************************************CHANGED used to be 10 MAY 9 2018 
}

void mouseClear(void) {

  byte BTcursor[7];

  BTcursor[0] = 0xFD;
  BTcursor[1] = 0x5;
  BTcursor[2] = 0x2;
  BTcursor[3] = 0x00;
  BTcursor[4] = 0x00;
  BTcursor[5] = 0x00;
  BTcursor[6] = 0x00;

  Serial1.write(BTcursor, 7);
 // Serial1.flush();


  
  //my_delay(10);                                                                                       *************************************************changed DELAY 
}

//***MOUSE SCROLLING FUNCTION***//

void mouseScroll(void) {
  while (1) {

    int scroll_up = analogRead(Y_DIR_HIGH);                      // A2
    int scroll_down = analogRead(Y_DIR_LOW);                     // A10

    float scroll_release = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;

    if (comm_mode == 0) {

      if (scroll_up > y_up + 30) {
        Mouse.move(0, 0, -1 * y_cursor_high(scroll_up));
        delay(cursor_delay * 35);   // started with this factor change as necessary
      } else if (scroll_down > y_down + 30) {
        Mouse.move(0, 0, -1 * y_cursor_low(scroll_down));
        delay(cursor_delay * 35);   // started with this factor change as necessary
      } else if ((scroll_release > sip_threshold) || (scroll_release < puff_threshold)) {
        break;
      }
    } else {

      if (scroll_up > y_up + 30) {
        mouseCommand(0, 0, 0, -1 * y_cursor_high(scroll_up), 0 ,0 ); ///////////////////////////////////////////////////ADDED MAY 10
        //mouseCommand(0, 0, 0, -1 * y_cursor_high(scroll_up));//old 
        delay(cursor_delay * 35);
      } else if (scroll_down > y_down + 30) {
        mouseCommand(0, 0, 0, -1 * y_cursor_low(scroll_down),0,0);////////////////////////////////////////////////////////ADDED MAY 10
//        mouseCommand(0, 0, 0, -1 * y_cursor_low(scroll_down));//old 
        delay(cursor_delay * 35);
      } else if ((scroll_release > sip_threshold) || (scroll_release < puff_threshold)) {
        break;
      }

    }
  }
  my_delay(250);
}

//***FORCE DISPLAY OF CURSOR***//

void Force_Cursor_Display(void) {
  if (comm_mode == 0) {
    Mouse.move(1, 0, 0);
    my_delay(25);
    Mouse.move(-1, 0, 0);
    my_delay(25);
  } else {
    /*
       Forcing the BT cursor requires some evaluation - come back.
    */

    /*
      mouseCommand(0, 1, 0, 0);
      my_delay(25);
      mouseCommand(0, -1, 0, 0);
      my_delay(25);
    */
  }
}

//***COMMUNICATION MODE STATUS***//

void Communication_Mode_Status(void) {
  if (digitalRead(MODE_SELECT) == LOW) {
    comm_mode = 0;                                // 0 == USB communication
    Serial.println("comm_mode = 0");
    BT_Low_Power_Mode();
    my_delay(10);
    //blink(6, 125, 1);
  } else if (digitalRead(MODE_SELECT) == HIGH) {
    comm_mode = 1;                                // 1 == Bluetooth communication
    my_delay(10);
    Serial.println("comm_mode = 1");
    //blink(6, 125, 2);
  }
}

//***JOYSTICK INITIALIZATION FUNCTION***//

void Joystick_Initialization(void) {
  xh = analogRead(X_DIR_HIGH);            // Initial neutral x-high value of joystick
  my_delay(10);

  xl = analogRead(X_DIR_LOW);             // Initial neutral x-low value of joystick
  my_delay(10);

  yh = analogRead(Y_DIR_HIGH);            // Initial neutral y-high value of joystick
  my_delay(10);

  yl = analogRead(Y_DIR_LOW);             // Initial neutral y-low value of joystick
  my_delay(10);

  x_right = xh;
  x_left = xl;
  y_up = yh;
  y_down = yl;

  EEPROM.get(6, yh_comp);
  my_delay(10);
  EEPROM.get(10, yl_comp);
  my_delay(10);
  EEPROM.get(14, xh_comp);
  my_delay(10);
  EEPROM.get(18, xl_comp);
  my_delay(10);
  EEPROM.get(22, xh_max);
  my_delay(10);
  EEPROM.get(24, xl_max);
  my_delay(10);
  EEPROM.get(26, yh_max);
  my_delay(10);
  EEPROM.get(28, yl_max);
  my_delay(10);

  constant_radius = 30.0;                       //40.0 works well for a constant radius

  xh_yh_radius = constant_radius;
  xh_yl_radius = constant_radius;
  xl_yl_radius = constant_radius;
  xl_yh_radius = constant_radius;

}

//***PRESSURE SENSOR INITIALIZATION FUNCTION***//

void Pressure_Sensor_Initialization(void) {
  float nominal_cursor_value = (((float)analogRead(PRESSURE_CURSOR)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]

  sip_threshold = nominal_cursor_value + 1;    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation

  puff_threshold = nominal_cursor_value - 0.5;   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
}

//***ARDUINO/GENUINO HID MOUSE INITIALIZATION FUNCTION***//

void Mouse_Configure(void) {
  if (comm_mode == 0) {                       // USB mode is comm_mode == 0, this is when the jumper on J13 is installed
    Mouse.begin();                            // Initialize the HID mouse functions from Mouse.h header file
    my_delay(25);                                // Allow extra time for initialization to take effect ***May be removed later
  }
}

//----------------------RN-42 BLUETOOTH MODULE INITIALIZATION SECTION----------------------//

//***BLUETOOTH CONFIGURATION STATUS FUNCTION***//

void BT_Config_Status(void) {
  int BT_EEPROM = 3;                               // Local integer variable initialized and defined for use with EEPROM GET function
  EEPROM.get(0, BT_EEPROM);                        // Assign value of EEPROM memory at index zero (0) to int variable BT_EEPROM
  delay(10);
  Serial.println(BT_EEPROM);                       // Only for diagnostics, may be removed later
  config_done = BT_EEPROM;                         // Assign value of local variable BT_EEPROM to global variable config_done ****************************************************************************************************************PART OF THE OLD CODE MAY 4th
   config_done = 0;        //CODE ADDED ON MAY 4th                  // Assign value of local variable BT_EEPROM to global variable config_done  ***********************************************************************************************************************SW
  delay(10);
}

//***BLUETOOTH CONFIGURATION FUNCTION***//

void BT_Configure(void) {
  Serial.print("we are here2"); 
  if (comm_mode == 1) {
    BT_Config_Status();                    // check if Bluetooth has previously been configured
    delay(10);
    if (config_done == 0) {                           // if Bluetooth has not been configured then execute configuration sequence
      BT_Command_Mode();                               // enter Bluetooth command mode
      BT_Config_Sequence();                           // send configuarion data to Bluetooth module
      BT_configAOK();                                 // returns diagnostic responses from Bluetooth //commnected back in may 4th ****************************************************************** sw
      Serial.println("we are here");
      delay(10);
    } else {
      Serial.println("Bluetooth configuration has previously been completed.");
      delay(10);
    }
  }
}

void BT_Command_Mode(void) {                 //***CHANGE THE TRANSISTOR CONTROLS ONCE THE PNP IS SWITCHED IN FOR THE NPN***
  digitalWrite(TRANS_CONTROL, HIGH);         // transistor base pin HIGH to ensure Bluetooth module is off
  digitalWrite(PIO4, HIGH);                 // command pin high
  my_delay(10);

  digitalWrite(TRANS_CONTROL, LOW);        // transistor base pin LOW to power on Bluetooth module
  my_delay(10);

  for (int i = 0; i < 3; i++) {             // cycle PIO4 pin high-low 3 times with 1 sec delay between each level transition
    digitalWrite(PIO4, HIGH);
    my_delay(150);
    digitalWrite(PIO4, LOW);
    my_delay(150);
  }

  digitalWrite(PIO4, LOW);                  // drive PIO4 pin low as per command mode instructions
  my_delay(10);
  Serial1.print("$$$");                     // enter Bluetooth command mode :: "$$$" CANNOT be Serial.println("$$$") ONLY Serial.print("$$$")
  my_delay(50);                              // time delay to visual inspect the red LED is flashing at 10Hz which indicates the Bluetooth module is in Command Mode
  Serial.println("Bluetooth Command Mode Activated");
}

void BT_Config_Sequence(void) {
  Serial1.println("ST,255");                 // turn off the 60 sec timer for command mode
  delay(15);
  Serial1.println("SA,2");                   // ***NEW ADDITION - Authentication Values 2: "any mode" work
  delay(15);
  Serial1.println("SX,0");                   // ***NEW ADDITION - Bonding 0: disabled
  delay(15);
  Serial1.println("SN,LipSyncBT_Autoload3"); // change name of BT module
  delay(15);
  Serial1.println("SM,6");                   // ***NEW ADDITION - Pairing "SM,6": auto-connect mode
  delay(15);
  Serial1.println("SH,0220");                // configure device as HID mouse
  delay(15);
  Serial1.println("S~,6");                   // activate HID profile
  delay(15);

  //Serial1.println("SW,<hex value>");       // sniff mode conserves power by polling the radio for comms ***POWER CONSERVATION
  //my_delay(100);
  //Serial1.println("SY,<hex value>");       // set transmit power settings ***POWER CONSERVATION

  Serial1.println("SQ,0");                   // configure for latency NOT throughput -> turn off: "SQ,0"
  delay(15);
  Serial1.println("S?,1");                   // 1:ENABLE role switch -> slave device attempts role switch -> indicates better performance for high speed data
  delay(15);
  Serial1.println("R,1");                    // reboot BT module
  delay(15);

  int val0 = 1;
  //int val1 = speed_counter;
  int val1 = *defaultspeed; ///////////////////////////////////////////////////////////////

  EEPROM.put(0, val0);                        // EEPROM address 0 gets configuration completed value (== 1)
  delay(15);
  EEPROM.put(2, val1);                        // EEPROM address 1 gets default cursor speed counter value (== 20) ***SHOULD ONLY OCCUR ONCE UNLESS THE LIPSYNC IS FACTORY RESET??
  delay(15);
  int val3;
  EEPROM.get(0, val3);  // diagnostics
  delay(15);            // diagnostics
  //Serial.println(val3); diagnostics
}

void BT_Low_Power_Mode(void) {
  BT_Command_Mode();                          // enter BT command mode
  Serial1.println('Z');                       // enter deep sleep mode (<2mA) when not connected
  my_delay(10);
  BT_configAOK();
  Serial.println("Bluetooth Deep Sleep Mode Activated");
  my_delay(10);
}

void BT_Connected_Status(void) {
  while (1) {
    Serial1.println("GK");
    my_delay(100);
    if (Serial1.available() > 0) {
      if ((char)Serial1.read() == '1') {
        Serial.println("BT is now connected!");
        my_delay(10);
        mouseCommand(0,1,0,0,0,0); //ADDED
        //mouseCommand(0, 1, 0, 0);   //*************************************************************************************************OLD
        my_delay(25);
        mouseCommand(0,-1,0,0,0,0);
//        mouseCommand(0, -1, 0, 0);  ****************************************************************************************************OLD 10
        my_delay(25);
        break;
      }
    }
  }
}

void BT_configAOK(void) {                    // diagnostic feedback from Bluetooth configuration
  while (Serial1.available() > 0) {
    Serial.print((char)Serial1.read());
  }
  /*
   * SW
   * MAY4th
   * commencted back in
   */
  Serial.println("");
  Serial.println("Configuration complete.");
}

//***JOYSTICK SPEED CALIBRATION***//

void Joystick_Calibration(void) {

  Serial.println("Prepare for joystick calibration!");
  Serial.println(" ");
  blink(4, 300, 3);

  Serial.println("Move mouthpiece to the furthest vertical up position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  yh_max = analogRead(Y_DIR_HIGH);
  blink(1, 1000, 2);
  Serial.println(yh_max);

  Serial.println("Move mouthpiece to the furthest horizontal right position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  xh_max = analogRead(X_DIR_HIGH);
  blink(1, 1000, 2);
  Serial.println(xh_max);

  Serial.println("Move mouthpiece to the furthest vertical down position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  yl_max = analogRead(Y_DIR_LOW);
  blink(1, 1000, 2);
  Serial.println(yl_max);

  Serial.println("Move mouthpiece to the furthest horizontal left position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  xl_max = analogRead(X_DIR_LOW);
  blink(1, 1000, 2);
  Serial.println(xl_max);

  int max1 = (xh_max > xl_max) ? xh_max : xl_max;
  int max2 = (yh_max > yl_max) ? yh_max : yl_max;
  float max_final = (max1 > max2) ? (float)max1 : (float)max2;

  //int delta_max_total = (yh_max - y_up) + (yl_max - y_down) + (xh_max - x_right) + (xl_max - x_left);

  Serial.print("max_final: ");
  Serial.println(max_final);

  //float avg_delta_max = ((float)(delta_max_total)) / 4;

  //Serial.print("avg_delta_max: ");
  //Serial.println(avg_delta_max);

  yh_comp = (max_final - y_up) / (yh_max - y_up);
  yl_comp = (max_final - y_down) / (yl_max - y_down);
  xh_comp = (max_final - x_right) / (xh_max - x_right);
  xl_comp = (max_final - x_left) / (xl_max - x_left);

  EEPROM.put(6, yh_comp);
  delay(10);
  EEPROM.put(10, yl_comp);
  delay(10);
  EEPROM.put(14, xh_comp);
  delay(10);
  EEPROM.put(18, xl_comp);
  delay(10);
  EEPROM.put(22, xh_max);
  delay(10);
  EEPROM.put(24, xl_max);
  delay(10);
  EEPROM.put(26, yh_max);
  delay(10);
  EEPROM.put(28, yl_max);
  delay(10);

  blink(5, 250, 3);

  Serial.println(" ");
  Serial.println("Joystick speed calibration procedure is complete.");
}

//***MANUAL JOYSTICK POSITION CALIBRATION***///
void Manual_Joystick_Home_Calibration(void) {

  xh = analogRead(X_DIR_HIGH);            // Initial neutral x-high value of joystick
  delay(10);
  Serial.println(xh);                     // Recommend keeping in for diagnostic purposes

  xl = analogRead(X_DIR_LOW);             // Initial neutral x-low value of joystick
  delay(10);
  Serial.println(xl);                     // Recommend keeping in for diagnostic purposes

  yh = analogRead(Y_DIR_HIGH);            // Initial neutral y-high value of joystick
  delay(10);
  Serial.println(yh);                     // Recommend keeping in for diagnostic purposes

  yl = analogRead(Y_DIR_LOW);             // Initial neutral y-low value of joystick
  delay(10);
  Serial.println(yl);                     // Recommend keeping in for diagnostic purposes

  x_right = xh;
  x_left = xl;
  y_up = yh;
  y_down = yl;

  int max1 = (xh_max > xl_max) ? xh_max : xl_max;
  int max2 = (yh_max > yl_max) ? yh_max : yl_max;
  float max_final = (max1 > max2) ? (float)max1 : (float)max2;

  //int delta_max_total = (yh_max - y_up) + (yl_max - y_down) + (xh_max - x_right) + (xl_max - x_left);

  Serial.print("max_final: ");
  Serial.println(max_final);

  //float avg_delta_max = ((float)(delta_max_total)) / 4;

  //Serial.print("avg_delta_max: ");
  //Serial.println(avg_delta_max);

  yh_comp = (max_final - y_up) / (yh_max - y_up);
  yl_comp = (max_final - y_down) / (yl_max - y_down);
  xh_comp = (max_final - x_right) / (xh_max - x_right);
  xl_comp = (max_final - x_left) / (xl_max - x_left);

  EEPROM.put(6, yh_comp);
  delay(10);
  EEPROM.put(10, yl_comp);
  delay(10);
  EEPROM.put(14, xh_comp);
  delay(10);
  EEPROM.put(18, xl_comp);
  delay(10);

  Serial.println("Home position calibration complete.");

}

//***SPECIAL INITIALIZATION OPERATIONS***//

void Serial_Initialization(void) {
  while (!Serial1) {
    while (!Serial) {
      if (init_counter_A < 100) {
        my_delay(5);
        init_counter_A++;
      } else {
        break;
      }
    }
    if (init_counter_B < 100) {
      my_delay(5);
      init_counter_B++;
    } else {
      break;
    }
  }
  my_delay(10);
  Serial.println(init_counter_A);
  Serial.println(init_counter_B);
  Serial.println("Serial and Serial1 are good!");
}

void Set_Default(void) {

  int default_config_setup;
  int default_cursor_setting;
  int set_default;
  float default_comp_factor = 1.0;

  EEPROM.get(4, set_default);
  my_delay(10);

  if (set_default != 1) {

    default_config_setup = 0;
    EEPROM.put(0, default_config_setup);
    my_delay(10);

    default_cursor_setting = 4;
    EEPROM.put(2, default_cursor_setting);
    my_delay(10);

    EEPROM.put(6, default_comp_factor);
    my_delay(10);

    EEPROM.put(10, default_comp_factor);
    my_delay(10);

    EEPROM.put(14, default_comp_factor);
    my_delay(10);

    EEPROM.put(18, default_comp_factor);
    my_delay(10);

    set_default = 1;
    EEPROM.put(4, set_default);
    my_delay(10);

  }
}

/*    Draw_square  (TEST METHOD FOR BLUETOOTH)
 *     DATE CREATED MAY 9th 2018
 *    by: shaemus 
 *    description: simple square drawing function to test the jitteriness of the mouse movement method of choice (working on circle version )
 */
 /*
void Draw_square(void)//will draw a square first to get the jist of how it should work then move onto making a circle drawing function
{

my_delay(100);
int x = 0; 
int done = 0; //flag that will be set once square has finished
while (x<20){
mouseCommand(0, 20, 0, 1); //mouse command takes in (clicksstatus, xdirection, ydirection, scroll)
my_delay(10);
x++;
}

while(x>=20&&x<40){
mouseCommand(0, 0, -20, 1);
my_delay(10);
x++;
}

while(x>=40 && x<60){
mouseCommand(0, -20, 0, 1);
my_delay(10);
x++;
}

while( x>=60&&x<80){
mouseCommand(0, 0, 20, 1);
my_delay(10);
x++;
done = 1; 
}

if (done == 1){
done = 0; 
x = 0;   //sets x to 0 for next time (reinforce that it will be 0) 
}
char buffer2[10];
sprintf(buffer2, " x: %d", x);
Serial.println( buffer2); 
Serial.println("done drawing square"); 


  
}*/

/*FOR DEBUGGING 
 * FUNCTION WAS MADE AS AN EASY WAY TO REPLACE ALL DELAYS IN ORDER TO SEE IF WE COULD OVER FLOW THE BLUE TOOTH BUFFER
 */
 
/*
 * FUNCTION to increase movement speed uses simple pointer logic default speed is set to 3 
 * at the start of the code and its other 2 zone speeds are set at 1 and 2. 
 */
void increasecursorspeed(void){
        //increasing the address that the pointer is referenced
      //  int *um = &speeds[4]; 
        
     
        if((*defaultspeed)>=14){
            blink(6, 50, 3);
            Serial.println(*defaultspeed);
           // printf("ERROR default speed at upper limit\n");
            //*defaultspeed = &speeds[6]; 
      
        }
        else{
          defaultspeed++; 
           Serial.println(*defaultspeed);
           
            Serial.println((long)defaultspeed, HEX);
          
            //Serial.println(speeds[4]
            blink(3, 100, 1);
            EEPROM.put(2, *defaultspeed);
            delay(25);
           
        }
 }

void decreasecursorspeed(void){
         //decreasing the address
    
        if((*defaultspeed)<=3){ //defaultspeed starts at 3 so if we decrease the value default speed is pointing 
        //to by 2 it will be 1 we dont want it to go less than 1 since this is outside our array
         blink(6, 50, 3);
       
   
        }
        
        else{
          defaultspeed--;
           Serial.println(*defaultspeed);
           
          
           EEPROM.put(2, *defaultspeed); //.just added seems to keep the lipsync from restarting once the limit is reached
        blink(3, 100, 1);
        delay(25);
       
        }
  
 }
 
void my_delay(int j){

}


/* FUNCTION: smoothmoves (created may 23)
 * simple function to filter out y values. returns a y value according to how far teh joystick is pushed in  each quadrant
 * uses zones to determine how fast to move
 */
/*TRYING SWITCH STATEMENTS TO REDUCE loop time
 * 
 */

int smoothmovesswitchypos(int y ){
      switch(y){
      case 0 ... 4: 
      return 0; 

      case 5 ... 11 :
      return 1; 
    
      case 12 ... 14:
      return *defaultspeed-2; 

      case 15 ... 17:
      return *defaultspeed -1 ; 

      default: //shouldnt go below 0 but we will see
      return *defaultspeed; 
      }

  
}

 int smoothmovesswitchyneg(int y ){
      switch(y){
      case -4 ... 0: 
      return 0; 

      case -11 ... -5:
      return -1; 

       case -14 ... -12:
       return (*defaultspeed-2)*-1; 
      
      case -17 ... -15:
      return (*defaultspeed-1)*-1 ; 

      default: //shouldnt go below 0 but we will see
      return -1*(*defaultspeed); 
      }

  
}


 
 int smoothmovesswitchxpos(int x ){
      switch(x){
      case 0 ... 4: 
      return 0; 

      case 5 ... 9:
      return 1; 

      case 10 ... 13:
      return *defaultspeed -2 ; 

       case 14 ... 16:
      return *defaultspeed -1 ; 

      default: //shouldnt go below 0 but we will see
      return *defaultspeed; 
      }
      

  
}

 int smoothmovesswitchxneg(int x ){
      switch(x){
      case -4 ... 0: 
      return 0; 
      
      case -9 ... -5:
      return -1;
      
      case -13 ... -10:
      return (*defaultspeed-2)*-1; 
      
      case -16 ... -14:
      return (*defaultspeed-1)*-1 ; 

      default: //shouldnt go below 0 but we will see
      return -1*(*defaultspeed); 
      }

  
}
/*
 int smoothmovesswitchypos(int y ){
      switch(y){
      case 0 ... 3: 
      return 0; 

      case 4 ... 9 :
      return 1; 
    
      case 10 ... 13:
      return *defaultspeed-2; 

      case 14 ... 16:
      return *defaultspeed -1 ; 

      default: //shouldnt go below 0 but we will see
      return *defaultspeed; 
      }

  
}

 int smoothmovesswitchyneg(int y ){
      switch(y){
      case -3 ... 0: 
      return 0; 

      case -9 ... -4:
      return -1; 

       case -13 ... -10:
       return (*defaultspeed-2)*-1; 
      
      case -16 ... -14:
      return (*defaultspeed-1)*-1 ; 

      default: //shouldnt go below 0 but we will see
      return -1*(*defaultspeed); 
      }

  
}*/

 


 /*
void timer(){
  static int i= 0 ; 
  //static bool cool ; 
  
 i = i+ 1; 
  Serial.println(i) ;
 

}
*/
/*FUNCTION: smoothmovestimer
 * uses a simple timer that counts up when the joystick is moved. If the joystick is moved long enough it will speed up the cursor to allow the user to reach different parts of the screen
 * PROs: no zones, more fluid. CONS: timing can get quite annoying, harder to draw circles since the chnage in direction restarts the counter
 */
 
/*FUNCTION: smoothmovesexp 
 * uses an exponential function to determine how fast the cursor should move depending on the direction vector (x is actually the number of spaces the cursor will move at a time)
 * PROS: smooth moves, acts like a mouse, allows for faster gradual movement. REACTS to increase/decrease buttons CONS: diagonal can be a little unpredictable 
 */
 /*
 int smoothmovesexpx(int x){


if ( x >=4){
  x = (int)(round((x *(exp(1)))/10)); //simple exponential function allows for gradual movement rather than linear
return x; 
}
else if ( x <=  -4) {
   x = (1*((int)(round((x *(exp(1)))/10) )));   
return x; 
}

else {
 
  return 0;
}

  
 }

 int smoothmovesexpy(int y){

//Serial.println("hi"); 
if ( y >=3){
  y = (int)(round((y *(exp(1)))/10)); 
return y; 
}
else if ( y <=  -3) {
   y  =(1* (int)(round((y *(exp(1)))/10) )); 
return y ; 
}

else {
 
  return 0;
}

  
 }
*/
/*
 
 * semi working function for increasign cursor movement exponentially
 delay (1);
    y = (int)(round((y*(exp(1)))/10)); 
    Serial.println(y);*/
