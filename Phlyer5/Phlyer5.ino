/*********************
Program for Physics Phlyer
Input from Force Plate and Rotary Motion Sensors
Output to Bar Graph Displays
**********************/
#include <Wire.h>  // For the I2C communication used by 
#include <Adafruit_GFX.h> // Graphics Library used by LEDBackpack routines
#include <Adafruit_LEDBackpack.h>  // Bar graph control
#include <EEPROM.h>   // To keep scale values when turned off 

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
int ArdBattpin = 1;  // Arduino Battery Voltage input pin
int DisBattpin = 2;  // Display Battery Voltage input pin


// initializing variables
byte buttons = 0;
byte mainopt = 1;  // Main Menu Option
int nReads = 100;  // Number of force readings to average
int zeroF = 230;  // Forceplate zero level
long int t0, t1;   // Last time and current time
long int currentTime; // Current time kept by interrupt routintes in usec
long int currentPos; // Current position kept by interrupt routines in 0.1 mm
long  x0, x1 = 0;  // last position adn this position
float v0, v1 = 0;  // last velocity and this velocity
float a0, a1 = 0;  // last acceleration and this acceleration
boolean accelTrue = 1;  // Show the acceleration graph
float ardBattVoltage;
float disBattVoltage;

float veloScale = 1.0;
float accelScale = 1.0;
float forceScale = 1.0;
int eeAddrVelo = 0;
int eeAddrAccel = 10;
int eeAddrForce = 20;

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

  // Display the Battery Voltages
  ardBattVoltage = analogRead(ArdBattpin) / 102.4;
  disBattVoltage = analogRead(DisBattpin) / 102.4;
  Serial.write(12);  // clear display
  delay(1000);
  Serial.print("Dis Bat = ");
  Serial.print(disBattVoltage);
  Serial.write(148);  // cursor to Line 2
  Serial.print("Ard Bat = ");
  Serial.print(ardBattVoltage);
  delay(2500);
   
  // get scale values from eeprom memory
  EEPROM.get( eeAddrVelo, veloScale );
  EEPROM.get( eeAddrAccel, accelScale );
  EEPROM.get( eeAddrForce, forceScale );

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
  if (mainopt == 7) {Serial.print("Reset Scaling   ");}  

  delay(100);

  #define TOP 7  // This must be set to the number of menu options
  
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
      if (mainopt == 7) resetScaling();
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


// Adjust a floating value using the up and down buttons with a log scale
void adjust(float &scale, char* label ) {
  float cycle[20] = {1.0,1.2,1.5,1.8,2.2,2.7,3.3,3.9,4.7,5.6,6.8,7.5,10};
  int cyclesize=12;
  float decade = 1.0;
  float factor = scale;

  Serial.write(12);  // clear display

  // find decade and factor whose product equals scale
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
     
    if (buttons == 3) {
      EEPROM.put( eeAddrVelo, veloScale );
      EEPROM.put( eeAddrAccel, accelScale );
      EEPROM.put( eeAddrForce, forceScale );
      return;
    }
    scale = factor * decade;
  }
}

//Zero the Force Plate
void zeroForce() {
  Serial.write(12);  // clear display
  long int total = 0;
  for (int i=0; i<nReads; i++) {
    total = total + analogRead(Forcepin);
  }
  zeroF = total / nReads;
  Serial.print("zero = ");
  Serial.print(zeroF);
  Serial.print("  ");
  delay(2000);
}

// Reset scale values to original values
void resetScaling() {
  veloScale = 10.0;
  accelScale = 2.0;
  forceScale = 1.0;

  // put scale values into eeprom memory
  EEPROM.put( eeAddrVelo, veloScale );
  EEPROM.put( eeAddrAccel, accelScale );
  EEPROM.put( eeAddrForce, forceScale );
  
}

// Everything !! Bar Graph of velocity and Acceleration and Force
void everything() {
  float xforce; 
  int force ;
  long int totalf;
  int shift = 10;  // Time increment for velocities in ms
  long int time, timePoint, dt;
  float velOld, velNew;
  timePoint = millis() >> shift + 1 << shift; // round off and add one to get new timepoint

  attachInterrupt(0, ISR_CW, RISING);
  attachInterrupt(1, ISR_CCW, FALLING);
  Serial.write(12);  // clear display
  delay(1000);
  
  while( checkButtons()==0 ) {
  
    // update values of t,x,v,a if we've reached the next timepoint to do so
    if (millis() >= timePoint) {
      // determine measured values of t,x,v,a
      t1 = currentTime;   // in microseconds
      x1 = currentPos;    // in 0.1 mm
      dt = t1-t0;   //  this will be close to 50,000 usec
      if(dt>0) {
        v1 = veloScale * 100 * (x1-x0)/dt;  // in m/s if scale is 1.0
        a1 = accelScale * 1e6 * (v1-v0)/dt;  // in m/s2 if scale is 1.0
      }
      else {
        v1 = 0;
        a1 = 0;
      }
      Serial.write(128);  //set cursor to home position
      Serial.print("a=");
      Serial.print(a1);
      Serial.write(148);  //set cursor to 2nd line
      Serial.print("v=");
      Serial.print(v1);
 
      
      // determine predicted values of x,v,a

      // weighted average of measured and predicted
      t0 = t1;
      x0 = x1;
      v0 = v1;
      a0 = a1;

      timePoint = millis() + 50;
    }    

    graphV(int(v0));
    if (accelTrue) {
      graphA(int(a0));
    }

    // show force bar ................    Force plate switch to left for max range then xforce is in units of 5 N 
    totalf = 0;
    for (int i=0; i<nReads; i++) {
      force = analogRead(Forcepin) - zeroF;
      totalf = totalf + force;
    }
    xforce = forceScale * totalf / nReads;    
    graphF2(xforce);
    
  }
 
  detachInterrupt(0);
  detachInterrupt(1);
}

  
  
// ------------------------------- basic functions ------------------------------------------------

//Velocity bar graph ............draws a velocity bar graph on 4 LED bar graph units. Range on v is -24 to +24
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

//Acceleration bar graph ............draws an acceleration bar graph on 2 LED bar graph units. Range on v is -12 to +12
void graphA(int v) {
  for (int i=0; i<12; i++) { 
    barAcc.setBar(i+12,LED_OFF);
    if (v>i) {barAcc.setBar( i+12, LED_GREEN);};
    barAcc.setBar(11-i,LED_OFF);
    if (-v>i) {barAcc.setBar(11-i,LED_RED);};
  }
  barAcc.writeDisplay();
}  

//Force fraction bar graph ...................... Makes a wide bar graph. Range on f is -8 to +8.  Also makes part width bars to show fractional values by quarters
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

  matrixFPos.writeDisplay();
  matrixFNeg.writeDisplay();  
}  

// Check the buttons.............   // Return value: 0=Notpressed 1=Up 2=Down 3=Go
byte  checkButtons() {  
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
  currentTime = micros();  // store the current time
  currentPos--;  // subtract one unit from position.  Each unit is about 0.1 mm
}
  
  
// Handler for CCW Pin pulse ..................
void ISR_CCW() {
  currentTime = micros();   // store the current time
  currentPos++;  // add one unit to position
}

