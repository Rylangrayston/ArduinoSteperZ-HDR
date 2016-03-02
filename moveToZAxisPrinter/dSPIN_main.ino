float maxHeight = 43; ////////////////////////////////////////////////// GAVIN CHANG THIS
float layerHeight = .16;        // bigDip: 0.1   , bounceyDip
float dipPrintHeight = 5;    // bigDip: 6.0   , bounceyDip
int liftPrintHeight = 3.0;       // bigDip: 3.    , bounceyDip
int delayAfterLift = 4000;      // bigDip: 2000  , bounceyDip
int delayAtSurface = 3000;        // bigDip: 3000     , bounceyDip
int secondDipHeight = .2;


int longestLayerTime = 16000;   
float resinHeight = 4;

int layerCount = 0;
int layersBeforWipeingStarts = 10;

int pause = 1;
int startup = 1;
// pins:
int lightPin = 9;
int dimmerNobPin = 14;
int upperLimitPin = 2;
int lowerLimitPin = 3;
int errorPin = 5;
int shutterPin = 8;
int lightOffOverRidePin = 15;
int ACPowerPin = 16;
int PrintHeight = 1;
int stepCount = 0;
boolean firstDrip = true;

float mmPerStep = 0.003927;
float resinHeightSteps = resinHeight / mmPerStep;
float stepsPerLayer = layerHeight/mmPerStep;
float dripsPermm = (1.0 / layerHeight) * 2.0; 
float maxSteps = maxHeight / mmPerStep;


int dripPin = A3;
int lightBrightness = 0;
int maxSpeedRise = 0 ;
int accelerationRate = 20;

int dripDelay = 500;
int pulsesPerDrip = 20;

float dipPrintSteps = dipPrintHeight / mmPerStep;
int liftPrintSteps = liftPrintHeight/ mmPerStep;
int stirDeapth = 40;
int secondDipSteps = secondDipHeight/mmPerStep;



///#############################################

/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

int advanceWiperDelaySpeed = 15;
int delaySpeedRetracting =15;
int retractedPosition = 60;
int forwardPosition = 10;







void retractWiper(){
    for (pos = forwardPosition; pos <= retractedPosition; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(delaySpeedRetracting);                       // waits 15ms for the servo to reach the position
  }
}


void advanceWiper(){
    for (pos = retractedPosition; pos >= forwardPosition; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(advanceWiperDelaySpeed);                       // waits 15ms for the servo to reach the position
  }
}


void dipPrint(){
  
   dSPIN_Move(FWD, dipPrintSteps * 128.0);
         while (digitalRead(dSPIN_BUSYN) == LOW);  // wait Until the movement completes, the
      dSPIN_Move(REV, (dipPrintSteps) * 128.0);
         while (digitalRead(dSPIN_BUSYN) == LOW);  // wait Until the movement completes, the
  
  
}

void wipe(){
  advanceWiper();
  retractWiper();
}

///########################################################@@@@@@@@@@@@@@@@@@@@@





void stir(){
  for (int i = 0; i <= stirDeapth; ++i)
    {
            dSPIN_Move(FWD, (1/mmPerStep) * 128.0);
           while (digitalRead(dSPIN_BUSYN) == LOW);  // wait Until the movement completes, the
    }
    
  Serial.println("hit the space key when your done stiring");
  while (Serial.available() == false);
    
     for (int i = 0; i <= stirDeapth; ++i)
    {
            dSPIN_Move(REV, (1/mmPerStep) * 128.0);
           while (digitalRead(dSPIN_BUSYN) == LOW);  // wait Until the movement completes, the
    }
         
}

void dipAndLiftPrint(){
      dSPIN_Move(FWD, dipPrintSteps * 128.0);
         while (digitalRead(dSPIN_BUSYN) == LOW);  // wait Until the movement completes, the
      dSPIN_Move(REV, (dipPrintSteps + liftPrintSteps) * 128.0);
         while (digitalRead(dSPIN_BUSYN) == LOW);  // wait Until the movement completes, the
         delay(delayAfterLift);
      dSPIN_Move(FWD, (liftPrintSteps + stepsPerLayer) * 128.0);
         while (digitalRead(dSPIN_BUSYN) == LOW);  // wait Until the movement completes, the
      //dSPIN_Move(FWD, secondDipSteps * 128.0);
        // while (digitalRead(dSPIN_BUSYN) == LOW);  // wait Until the movement completes, the         
         
     // dSPIN_Move(REV, secondDipSteps * 128.0);
       //  while (digitalRead(dSPIN_BUSYN) == LOW);  // wait Until the movement completes, the
         
      
         
      delay(delayAtSurface);
      
      
      
      
         
}


void sendDrip(){
  
  delay(dripDelay);
  Serial.println(stepCount);
   for (int i=0; i < pulsesPerDrip; i++) {
  digitalWrite(dripPin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(0);               // wait for a second
  digitalWrite(dripPin, LOW);    // turn the LED off by making the voltage LOW
  delay(1);               // wait for a second
   }
}



void nextLayer() {
  // dSPIN_Move(FWD, stepsPerLayer * 128.0);
 // while (digitalRead(dSPIN_BUSYN) == LOW);  // wait Until the movement completes, the
  stepCount += stepsPerLayer;
  
//  if (layerCount > layersBeforWipeingStarts){
//  wipe();
 // }
  sendDrip();
  sendDrip(); 
  layerCount += 1; 
   

}










//dSPIN_main.ino - Contains the setup() and loop() functions.


void setup() 
{

  Serial.begin(9600);
 
   pinMode(lightPin, OUTPUT);
  pinMode(upperLimitPin, INPUT);
  pinMode(lowerLimitPin, INPUT);
  pinMode(errorPin, OUTPUT);
  pinMode(shutterPin, OUTPUT);
  pinMode(lightOffOverRidePin, INPUT);
  pinMode(ACPowerPin, OUTPUT);
  pinMode(dripPin, OUTPUT);
  
   myservo.attach(A4);  // attaches the servo on pin 9 to the servo object
   myservo.write(retractedPosition);

  
 
  
  // dSPIN_init() is implemented in the dSPIN_support.ino file. It includes
  //  all the necessary port setup and SPI setup to allow the Arduino to
  //  control the dSPIN chip and relies entirely upon the pin redefinitions
  //  in dSPIN_example.ino
  dSPIN_init();
  
  // The following function calls are for this demo application- you will
  //  need to adjust them for your particular application, and you may need
  //  to configure additional registers.
  
  // First, let's set the step mode register:
  //   - dSPIN_SYNC_EN controls whether the BUSY/SYNC pin reflects the step
  //     frequency or the BUSY status of the chip. We want it to be the BUSY
  //     status.
  //   - dSPIN_STEP_SEL_x is the microstepping rate- we'll go full step.
  //   - dSPIN_SYNC_SEL_x is the ratio of (micro)steps to toggles on the
  //     BUSY/SYNC pin (when that pin is used for SYNC). Make it 1:1, despite
  //     not using that pin.
  dSPIN_SetParam(dSPIN_STEP_MODE, !dSPIN_SYNC_EN | dSPIN_STEP_SEL_1_128 | dSPIN_SYNC_SEL_1);

  // Configure the FS_SPD register- this is the speed at which the driver ceases
  //  microstepping and goes to full stepping. FSCalc() converts a value in steps/s
  //  to a value suitable for this register; to disable full-step switching, you
  //  can pass 0x3FF to this register.
  dSPIN_SetParam(dSPIN_FS_SPD, FSCalc(100000));
  // Configure the acceleration rate, in steps/tick/tick. There is also a DEC register;
  //  both of them have a function (AccCalc() and DecCalc() respectively) that convert
  //  from steps/s/s into the appropriate value for the register. Writing ACC to 0xfff
  //  sets the acceleration and deceleration to 'infinite' (or as near as the driver can
  //  manage). If ACC is set to 0xfff, DEC is ignored. To get infinite deceleration
  //  without infinite acceleration, only hard stop will work.
  //dSPIN_SetParam(dSPIN_ACC, 0x009);
  //dSPIN_SetParam(dSPIN_DEC, 0x009);
  
  dSPIN_SetParam(dSPIN_ACC, AccCalc(5000));
  dSPIN_SetParam(dSPIN_DEC, DecCalc(5000));
  // Configure the overcurrent detection threshold. The constants for this are defined
  //  in the dSPIN_example.ino file.
  dSPIN_SetParam(dSPIN_OCD_TH, dSPIN_OCD_TH_6000mA);
  // Set up the CONFIG register as follows:
  //  PWM frequency divisor = 1
  //  PWM frequency multiplier = 2 (62.5kHz PWM frequency)
  //  Slew rate is 290V/us
  //  Do shut down bridges on overcurrent
  //  Disable motor voltage compensation
  //  Hard stop on switch low
  //  16MHz internal oscillator, nothing on output
  dSPIN_SetParam(dSPIN_CONFIG, 
                   dSPIN_CONFIG_PWM_DIV_1 | dSPIN_CONFIG_PWM_MUL_2 | dSPIN_CONFIG_SR_290V_us
                 | dSPIN_CONFIG_OC_SD_ENABLE | dSPIN_CONFIG_VS_COMP_DISABLE 
                 | dSPIN_CONFIG_SW_HARD_STOP | dSPIN_CONFIG_INT_16MHZ);
  // Configure the RUN KVAL. This defines the duty cycle of the PWM of the bridges
  //  during running. 0xFF means that they are essentially NOT PWMed during run; this
  //  MAY result in more power being dissipated than you actually need for the task.
  //  Setting this value too low may result in failure to turn.
  //  There are ACC, DEC, and HOLD KVAL registers as well; you may need to play with
  //  those values to get acceptable performance for a given application.
  
  // 1e = 12 %
  // 7D = 50 %
  // 3F = 25 %
  // BE = 75%
  // FF  = 100%
  
  
  
  
  dSPIN_SetParam(dSPIN_KVAL_RUN, 0xff);
  dSPIN_SetParam(dSPIN_KVAL_ACC, 0x3f);
  dSPIN_SetParam(dSPIN_KVAL_DEC, 0x3f);
  dSPIN_SetParam(dSPIN_KVAL_HOLD,0x1e);
  
  
    // Configure the MAX_SPEED register- this is the maximum number of (micro)steps per
  //  second allowed. You'll want to mess around with your desired application to see
  //  how far you can push it before the motor starts to slip. The ACTUAL parameter
  //  passed to this function is in steps/tick; MaxSpdCalc() will convert a number of
  //  steps/s into an appropriate value for this function. Note that for any move or
  //  goto type function where no speed is specified, this value will be used.
  dSPIN_SetParam(dSPIN_MAX_SPEED, MaxSpdCalc(680));
  // Calling GetStatus() clears the UVLO bit in the status register, which is set by
  //  default on power-up. The driver may not run without that bit cleared by this
  //  read operation.
  dSPIN_GetStatus();
  
  dSPIN_Move(FWD, 100*128);
}



// Continually turn one revolution forward, then back again, stopping in between each turn.




void oporateShutter() {
      //cammera settings:
  int longExposureTime = 2000;
  int shortExposureTime = 500;
  int longExposureRecoveryTime = 1000;
  int shortExposureRecoveryTime = 500;
  int photoCycleTime = shortExposureRecoveryTime + longExposureRecoveryTime + shortExposureTime + longExposureTime;
  unsigned long time = millis();
  
  unsigned long cycleCount = time/ photoCycleTime ;
  unsigned long timeAtStartOfCurrentCycle = cycleCount * photoCycleTime;
  
    if (time > timeAtStartOfCurrentCycle + 0 & time < timeAtStartOfCurrentCycle + longExposureTime ) {
    digitalWrite(shutterPin, HIGH); 
    //Serial.println("start long exposure");
    }
  
  if (time > timeAtStartOfCurrentCycle + longExposureTime & time < timeAtStartOfCurrentCycle + longExposureTime + longExposureRecoveryTime) {
    digitalWrite(shutterPin, LOW); 
    //Serial.println("start long exposure recovery"); 
  } 
    
  if (time > timeAtStartOfCurrentCycle + longExposureTime + longExposureRecoveryTime & time < timeAtStartOfCurrentCycle + longExposureTime + longExposureRecoveryTime + shortExposureTime) {
    digitalWrite(shutterPin, HIGH); 
    setBrightness(1);
    //Serial.println("start short exposure");    
  }
    
  if (time > timeAtStartOfCurrentCycle + longExposureTime + longExposureRecoveryTime + shortExposureTime ) {
    digitalWrite(shutterPin, LOW); 
    setBrightness(0);
    //Serial.println("start short exposure recovery ");    
        
    
  }
  }
  
  //Serial.println(cycleCount);
  //Serial.println(time);
  
void checkOverRideLights() {
  if (digitalRead(lightOffOverRidePin) == 1) {
   setBrightness(1); 
  }
}
  


void setBrightness(int factor) {
  if (digitalRead(lightOffOverRidePin) == 1) {
   factor = 1; 
  }
    int dimmerNob = analogRead(dimmerNobPin);
    int lightBrightness = (dimmerNob * (254 / 1023.0) * factor );
    analogWrite(lightPin, lightBrightness);
    if (factor > 0){
    digitalWrite(ACPowerPin, HIGH);
    }
    else{
      digitalWrite(ACPowerPin, LOW);
    }
}

void abortIfLimitSwitch(){
       if (digitalRead(upperLimitPin) == LOW | digitalRead(lowerLimitPin) == LOW){
       //hard stop here
       setBrightness(0);
       digitalWrite(shutterPin, LOW);
       while(true){
         digitalWrite(errorPin, HIGH);
         delay(200);
         digitalWrite(errorPin, LOW);
         delay(200);
       }
}
}

void findUpperLimit() {
  
  while (digitalRead(upperLimitPin) == HIGH) {
     dSPIN_Move(REV, 40 * 128.0);
     while (digitalRead(dSPIN_BUSYN) == LOW);  // wait Until the movement completes, the
     stepCount = 0;
     pause = 1;
     firstDrip = true;
     
    // if (maxSpeedRise < 399) {
     //dSPIN_SetParam(dSPIN_MAX_SPEED, MaxSpdCalc(maxSpeedRise)); 
    // }
    // maxSpeedRise += accelerationRate; 
    // if (maxSpeedRise > 400){
     // maxSpeedRise = 400;
     //}
     }
     
  dSPIN_Move(FWD, resinHeightSteps * 128.0);
  while (digitalRead(dSPIN_BUSYN) == LOW);
 //delay(100);
}

void findLowerLimit() {
  
  while (digitalRead(lowerLimitPin) == HIGH) {
     dSPIN_Move(FWD, 40 * 128.0);
     while (digitalRead(dSPIN_BUSYN) == LOW);  // wait Until the movement completes, the
     //dSPIN_SetParam(dSPIN_MAX_SPEED, MaxSpdCalc(max_speed_rise)); 
     //max_speed_rise += accelerationRate; 
     //if (max_ > 100){
     // maxSpeedRise = 100;
     }
     
  dSPIN_Move(REV, 700.0 * 128.0);
  while (digitalRead(dSPIN_BUSYN) == LOW);
 delay(100);
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 

      if (inChar == 'd'){
        dSPIN_Move(FWD, 200.0 * 128.0);
        Serial.println(inChar);
        inChar = '`';
      }
      
      if (inChar == 'u'){
        dSPIN_Move(REV, 10000.0 * 128.0);
        Serial.println(inChar);
        inChar = '`';
      }      
      
            if (inChar == 't'){
        sendDrip();
        Serial.println(inChar);
        inChar = '`';
      }   
     
                 if (inChar == 'f'){
        dripDelay -= 100;
        if (dripDelay < 1){
          dripDelay = 100;
        }
          
        
        Serial.println(inChar);
        inChar = '`';
      }    
     
                 if (inChar == 's'){
        dripDelay += 100;
        Serial.println(inChar);
        inChar = '`';
      }  
   
                  if (inChar == 'r'){
        startup = 1;
        Serial.println(inChar);
        inChar = '`';
      }      
      
                  if (inChar == 'p'){
        pause = 1;
        Serial.println(inChar);
        inChar = '`';
      }  
      
                 if (inChar == 'g'){
        pause = 0;
        Serial.println(inChar);
        inChar = '`';
      }  
   
   
                  if (inChar == 'z'){
        Serial.println(inChar);
        inChar = '`';
        stir();
      }  
  }
}




void loop()
{

  if (startup == 1) {
    Serial.println("254 drips per mm");
    Serial.println("f = faster... s = slower ... p = pause ... g = go/unpause ....  r = restart...z = stir....");
    Serial.println("layerHeight:");
    Serial.println(layerHeight);
    Serial.println("mmPerStep");
    Serial.println(mmPerStep,8);
    Serial.println("stepsPerLayer");
    Serial.println(stepsPerLayer);
    Serial.println("dripsPermm:");
    Serial.println(dripsPermm);
    Serial.println("maxSteps");
    Serial.println(maxSteps);
 //   Serial.println("dipPrintSteps")
  //  Serial.println(dipPrintSteps);
    findUpperLimit();
    //findLowerLimit();
    startup = 0;
  }

if (stepCount < maxSteps && pause == 0){
  
    if (firstDrip){
      sendDrip();
      firstDrip = false;
      Serial.println("send first stager drip");
    }
  
  //dipPrint();
  dipAndLiftPrint();
  nextLayer();
  delay(longestLayerTime);
}

if (stepCount > maxSteps && pause == 0){
    Serial.println("FINISHED PRINT");
  findUpperLimit();
  pause = 1;

}
  serialEvent();
//  abortIfLimitSwitch();
 // oporateShutter();
//  checkOverRideLights();
 
  
}




