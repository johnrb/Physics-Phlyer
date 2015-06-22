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
//
int BTN1pin = 6;  // UP button input pin
int BTN2pin = 5;  // DN button
int Forcepin = 0;  // Force Plate analog input pin
int CWPin = 2;  //  Rotary Motion CW signal input pin
int CCWPin = 3;  //  Rotary Motion CCW signal

// initializing variables
byte buttons = 0;
byte mainopt = 1;  // Main Menu Option
int n = 5;  // A number to adjust for testing
float veloScale = 1.0;
float accelScale = 1.0;
float forceScale = 1.0;
int zeroF = 230;  // Froceplate zero leve
long int time0, time1;   // time of this and last interrupt
float vel0, vel1;  // this velocity and the last velocity
float accel0;
float distance = 50000.0;  // This scales the velocity
boolean accelTrue;


void setup() {
  Serial.begin(9600);  // set serial communication for the LCD display:
  delay(1000);
  Serial.write(12);  // clear display
  delay(1000);
  Serial.print("setup bars");
  delay(1000);

  pinMode(CWPin,INPUT);  
  pinMode(CCWPin,INPUT);

  Serial.write(148);  // cursor to Line 2
  Serial.print("finding batt 2");
  delay(500);
  
  // Start all the bar and matrix routines
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
  
  // Zero the force Plate
  zeroForce();
  
}
  
// =====================  MAIN LOOP ==================================================
void loop() {
  Serial.write(128);  //set cursor to home position
  delay(100);
  if (mainopt == 1) {Serial.print("Main Program    ");}
  if (mainopt == 2) {Serial.print("Toggle Accel    ");}
  if (mainopt == 3) {Serial.print("Scale Velocity  ");}
  if (mainopt == 4) {Serial.print("Scale Accelerate");}
  if (mainopt == 5) {Serial.print("Scale Force     ");}
  if (mainopt == 6) {Serial.print("Zero Force      ");}
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
      if (mainopt == 1) everything();
      if (mainopt == 2) toggle(accelTrue, "accel");
      if (mainopt == 3) adjust(veloScale, "Veloc");
      if (mainopt == 4) adjust(accelScale,"Accel");
      if (mainopt == 5) adjust(forceScale,"Force");
      if (mainopt == 6) zeroForce();
    }
    Serial.write(12);  // button press has been handled so clear before rewriting
    delay(200);    // Wait until button is no longer pressed
  }
}

//// --------------------------------------- functions called by the menu -------------------------

// toggle the boolean value of ctrl
void toggle(boolean &ctrl, char* label ) {
  Serial.write(12);
  delay(1000);
  while(1) {
    Serial.write(128);
    Serial.print(label);   
    Serial.print(" is ");   
    Serial.print(ctrl);   
    byte buttons = checkButtons();
    if ((buttons == 1) || (buttons == 2)) {                // Up button increases scale
      ctrl = !ctrl;
    }
    if (buttons == 3) {return;}
  }
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

// Adjust a floating value using the up and down buttons with a log scale
void adjust(float &scale, char* label ) {
  float cycle[20] = {1.0,1.2,1.5,1.8,2.2,2.7,3.3,3.9,4.7,5.6,6.8,7.5,10};
  int cyclesize=12;
  float decade = 1.0;
  float factor = scale;

  Serial.write(12);  // clear display

  // find decade and factor that equal scale
  while (factor>9.9) {
    factor = factor/10;
    decade = decade*10;
  }
  while (factor<1.0) {
    factor = factor*10;
    decade = decade/10;
  }
  int i;
  for (i=0; cycle[i]<factor; i++) {}  // find index for cycle to equal current scale 
  
  
  while(1) {
    Serial.write(128);  //set cursor to home position
    Serial.print(label);
    Serial.print(" = ");
    Serial.print(scale);
    Serial.print("   ");
    delay(100);
    byte buttons = checkButtons();
    if (buttons == 1) {                // Up button increases scale
      i++;
      if (i >=cyclesize) {  // factor rolls over and increments the decade
        i=0;
        decade = decade*10;
      }
      factor = cycle[i];
    }
      
    if (buttons == 2) {                // Down button decreases scale
      i--;
      if (i<0) {   // factor rolls under and decrements the decade
        i=cyclesize-1;
        decade = decade/10;
      }
      factor = cycle[i];
    }
     
    if (buttons == 3) {return;}
    
    scale = factor * decade;
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


// Everything !! Bar Graph of velocity and Acceleration and Force
void everything() {
  float xforce; 
  int force ;
  long int totalf;
  int shift = 10;  // Time increment for velocities in ms
  long int time, timePoint;
  float velOld, velNew;
  timePoint = millis() >> shift + 1 << shift; // round off and add one to get new timepoint

  attachInterrupt(0, ISR_CW, RISING);
  attachInterrupt(1, ISR_CCW, FALLING);
  Serial.write(12);  // clear display
  delay(1000);
  
  while( checkButtons()==0 ) {
  
    // update velocity and acceleration if we've reached the next timepoint to do so
    if (millis() >= timePoint) {
      velOld = velNew;
      velNew = vel0;
      accel0 = accelScale * (velNew - velOld);
      timePoint = millis() + 50;
//      timePoint = millis() >> shift + 1 << shift; // round off and add one to get new timepoint
    }    

    graphV(int(velNew));
    if (accelTrue) {
      graphA(int(accel0));
    }

    // show force bar
    totalf = 0;
    for (int i=0; i<20*n; i++) {
      force = analogRead(Forcepin) - zeroF;
      totalf = totalf + force;
    }
    xforce = forceScale * totalf / (20*n);    
    graphF2(xforce);
    
  }
 
  detachInterrupt(0);
  detachInterrupt(1);
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
//Force bar graph ......................
void graphF2(float f) {

  matrixFPos.fillRect(0,0, 8,8, LED_OFF);  // Clear force matrix
  matrixFNeg.fillRect(0,0, 8,8, LED_OFF);
  int fint = int(f);
  matrixFPos.fillRect(0,8,8,1-fint , LED_RED);  // Draw full width bars for integer part of f
  matrixFNeg.fillRect(0,-1,8,1-fint, LED_RED);
  
  int ifrac;  // draw partial bars for fractions
  ifrac = int( (f - float(fint)) * 4);
  if      (ifrac>0) { matrixFPos.drawLine(4-ifrac, 7-fint, 3+ifrac, 7-fint, LED_RED); } 
  else if (ifrac<0) { matrixFNeg.drawLine(4+ifrac, -fint, 3-ifrac, -fint, LED_RED); }

  
  Serial.write(128);  //set cursor to home position
  Serial.print("F=");
  Serial.print(f);
  Serial.write(148);  //set cursor to home position
  Serial.print(fint);
  Serial.print("=fint frac=");
  Serial.print(ifrac);
 
  matrixFPos.writeDisplay();
  matrixFNeg.writeDisplay();  
}  


byte  checkButtons() {   // Return value: 0=Notpressed 1=Up 2=Down 3=Go
   static byte  butn;   //  "Static" so it keeps its value from one call to the next 
   if (butn != 0) {delay(200);}  // If button was pressed last time, pause to slow repeat speed
   butn = digitalRead(BTN1pin) + 2*digitalRead(BTN2pin);   // Read the 2 button lines and set butn
   if (butn !=0) {            // Something is pressed this time so...
      delay(20);           // wait a bit for switch bounce to settle
      butn = digitalRead(BTN1pin) + 2*digitalRead(BTN2pin); // read again and set butn
   }
   return (butn);
}

// -----  Interupt Service Routines  -----------------------------------------------

// Handler for CW Pin pulse ..................
void ISR_CW() {
  time0 = micros();
  vel0 = veloScale * distance/(time0 - time1);
  time1 = time0;
}
  
  
// Handler for CCW Pin pulse ..................
void ISR_CCW() {
  time0 = micros();
  vel0 = veloScale * -distance/(time0 - time1);
  time1 = time0;
}

