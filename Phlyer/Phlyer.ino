/*********************
Program for Physics Phlyer
Input from Force Plate and Rotary Motion Sensors
Output to Bar Graph Displays
**********************/
#include <Wire.h>  // For the I2C communication used by 
#include <Adafruit_GFX.h> // Graphics Library used by LEDBackpack routines
#include <Adafruit_LEDBackpack.h>  // Bar graph control

Adafruit_24bargraph barVNeg = Adafruit_24bargraph();
Adafruit_24bargraph barVPos = Adafruit_24bargraph();
Adafruit_24bargraph barAcc = Adafruit_24bargraph();
Adafruit_BicolorMatrix matrixFPos = Adafruit_BicolorMatrix();
Adafruit_BicolorMatrix matrixFNeg = Adafruit_BicolorMatrix();

// initializing variables
byte mainopt = 1;  // Main Menu Option
int n = 5;  // A Number we wish to adjust for testing
int level;  // 
int BTN1pin = 3;  // UP button
int BTN2pin = 2;  // DN button
int Forcepin = 0;  // Force Plate analog input pin
byte buttons = 0;
int zeroF = 230;  // Froceplate zero leve
int CWPin = 5;  //  Rotary Motion CW signal
int CCWPin = 6;  //  Rotary Motion CCW signal


void setup() {
  Serial.begin(9600);  // set serial communication for the LCD display:
  Serial.write(12);  // clear display
  delay(6000);
  Serial.print("Hello world");
  delay(1000);
  
  pinMode(CWPin,INPUT);  
  pinMode(CCWPin,INPUT);
  
  barVNeg.begin(0x70);
  barVPos.begin(0x71);
  barAcc.begin(0x72);
  matrixFPos.begin(0x75);
  matrixFNeg.begin(0x74);
  
  // Clear all the bar and matrix displays
  for (int i=0; i<24; i++)  {
    barVNeg.setBar(i,LED_OFF);
    barVPos.setBar(i,LED_OFF);
    barAcc.setBar(i,LED_OFF);
  }
  barVNeg.writeDisplay();
  barVPos.writeDisplay();
  barAcc.writeDisplay();
  matrixFPos.fillRect(0,0, 7,7, LED_OFF);
  matrixFNeg.fillRect(0,0, 7,7, LED_OFF);
  matrixFPos.writeDisplay();
  matrixFNeg.writeDisplay();
  
}
  
// =====================  MAIN LOOP ==================================================
void loop() {
  Serial.write(128);  //set cursor to home position
  delay(100);
  if (mainopt == 1) {Serial.print("Welcome to Menu ");}
  if (mainopt == 2) {Serial.print("Set Value       ");}
  if (mainopt == 3) {Serial.print("Show Value      ");}
  if (mainopt == 4) {Serial.print("Show Force      ");}
  if (mainopt == 5) {Serial.print("Zero Force      ");}
  if (mainopt == 6) {Serial.print("Show Velocity   ");}

  delay(100);

  #define TOP 6  // This must be set to the number of menu options
  
// check the buttons and respond by changing the menu
  buttons = checkButtons();  // 'buttons' value depends on what was presssed
  if (buttons) {                              // Anything but but zero does next section
    if (buttons == 1) {mainopt++;}    // Up button increments menu option
    if (buttons == 2) {mainopt--;}   // Down button decrements menu option
    if (mainopt > TOP) {mainopt = 1;}         // rollover menu options
    if (mainopt == 0)   {mainopt = TOP;}      // rollunder menu options
     
// if select is pressed, then call the function of that menu item
    if (buttons == 3) {
      if (mainopt == 1) welcome();
      if (mainopt == 2) setValue();
      if (mainopt == 3) showValue();
      if (mainopt == 4) showForce();
      if (mainopt == 5) zeroForce();
      if (mainopt == 6) showVelocity();

    }
    Serial.write(12);  // button press has been handled so clear before rewriting
    delay(500);    // Wait until button is no longer pressed
  }
}

//// --------------------------------------- functions called by the menu -------------------------

// Flash one LED then other LED then return
void welcome() {
  Serial.write(12);
  Serial.print("I am Arduino Uno");   
  delay(2000);
}

// Adjust a number using the up and down buttons
void setValue() {
  Serial.write(12);  // clear display
  while(1) {
    Serial.write(128);  //set cursor to home position
    Serial.print("N = ");
    Serial.print(n);
    Serial.print("  ");
    delay(100);
    byte buttons = checkButtons();
    if (buttons == 1) {n++;}  // Up button increments number
    if (buttons == 2) {n--;}   // Down button decrements number
    if (buttons == 3) {return;}
  }
}

// Adjust a number using the up and down buttons and show it on bargraphs
void showValue() {
  Serial.write(12);  // clear display
  while(1) {
    graphV(n);
    graphA(n);
    graphF(float(n));
    Serial.write(128);  //set cursor to home position
    Serial.print("N = ");
    Serial.print(n);
    Serial.print("  ");
    delay(100);
    byte buttons = checkButtons();
    if (buttons == 1) {n++;}  // Up button increments number
    if (buttons == 2) {n--;}   // Down button decrements number
    if (buttons == 3) {return;}
  }
}

// Read the Force Plate and display on wide graph continuously
void showForce() {
  int force ;
  long int totalf;
  float xforce;
  Serial.write(12);  // clear display
  while( checkButtons()==0 ) {
    totalf = 0;
    for (int i=0; i<20*n; i++) {
      force = analogRead(Forcepin) - zeroF;
      totalf = totalf + force;
    }
    xforce = 1.0 * totalf / (20*n);
    Serial.write(128);  //set cursor to home position
    Serial.print("F = ");
    Serial.print(force);
    Serial.print("  ");
    graphF(xforce);
//    delay(300);
  }

}  

//Zero the Force Plate
void zeroForce() {
  Serial.write(12);  // clear display
  long int total = 0;
  for (int i=0; i<20*n; i++) {
    total = total + analogRead(Forcepin);
  }
  zeroF = total / (20*n);
  Serial.print("zero = ");
  Serial.print(zeroF);
  Serial.print("  ");
  delay(2000);
}

// Show the velocity from the rotary motion sensor
void showVelocity() {
  int countCW = 0;
  int countCCW = 0;
  Serial.write(12);  // clear display
  delay(1000);
  
  while( checkButtons()==0 ) {
    Serial.write(128);  //set cursor to home position
    pulseIn(CWPin,HIGH);
    countCW++;
    Serial.print(countCW);
//    }
//    else {
//      Serial.print("0 ");
//    }
//    if (digitalRead(CCWPin)==HIGH) {
//      Serial.print("1 ");
//    }
//    else {
//      Serial.print("0 ");
//    }
//
  }
}
  
  
// ------------------------------- basic functions ------------------------------------------------

//Velocity bar graph ...................
void graphV(int v) {
  for (int i=0; i<24; i++) { 
    barVPos.setBar(i,LED_OFF);
    if (v>i) {barVPos.setBar( i, LED_GREEN);};
    barVNeg.setBar(i,LED_OFF);
    if (-v>i) {barVNeg.setBar(i,LED_RED);};
  }
  barVNeg.writeDisplay();
  barVPos.writeDisplay();
}  

//Acceleration bar graph ....................
void graphA(int v) {
  for (int i=0; i<12; i++) { 
    barAcc.setBar(i+12,LED_OFF);
    if (v>i) {barAcc.setBar( i+12, LED_GREEN);};
    barAcc.setBar(11-i,LED_OFF);
    if (-v>i) {barAcc.setBar(11-i,LED_RED);};
  }
  barAcc.writeDisplay();
}  

//Force bar graph ......................
void graphF(float f) {

  matrixFPos.fillRect(0,0, 8,8, LED_OFF);
  matrixFNeg.fillRect(0,0, 8,8, LED_OFF);
  
  for (int i=0; i<8; i++) { 
    if (f>i) { matrixFPos.drawLine(0,7-i, 1,7-i, LED_RED);}
//    else { matrixFPos.drawLine(0,7-i, 7,7-i, LED_OFF) ;}
    if (-f>i) {matrixFNeg.drawLine(0,i, 1,i, LED_RED);}
//    else { matrixFNeg.drawLine(0,i, 7,i, LED_OFF) ;}
  }
  matrixFPos.writeDisplay();
  matrixFNeg.writeDisplay();
}

byte  checkButtons() {   // Return value: 0=Notpressed 1=Up 2=Down 3=Go
   static byte  BN;   //  "Static" so it keeps its value from one call to the next 
   if (BN != 0) {delay(200);}  // If button was pressed last time, pause to slow repeat speed
   BN = digitalRead(BTN1pin) + 2*digitalRead(BTN2pin);   // Read the 2 button lines and set ButN
   if (BN !=0) {            // Something is pressed this time so...
      delay(20);           // wait a bit for switch bounce to settle
      BN = digitalRead(BTN1pin) + 2*digitalRead(BTN2pin); // read again and set ButN
   }
   return (BN);
}

// -----  Interupt Service Routines  -----------------------------------------------

// Handler for CCW Pin pulse 
void ISR_CW() {
  

