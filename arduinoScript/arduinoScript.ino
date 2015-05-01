// pins:
int lightPin = 9;
int dimmerNobPin = 14;
int upperLimitPin = 3;
int lowerLimitPin = 2;
int errorPin = 5;
int shutterPin = 8;
int lightOffOverRidePin = 15;
int ACPowerPin = 16;





int lightBrightness = 0;

void setup() {
  Serial.begin(9600);
  pinMode(lightPin, OUTPUT);
  pinMode(upperLimitPin, INPUT);
  pinMode(lowerLimitPin, INPUT);
  pinMode(errorPin, OUTPUT);
  pinMode(shutterPin, OUTPUT);
  pinMode(lightOffOverRidePin, INPUT);
  pinMode(ACPowerPin, OUTPUT);
  
}


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
    Serial.println("start long exposure");
    }
  
  if (time > timeAtStartOfCurrentCycle + longExposureTime & time < timeAtStartOfCurrentCycle + longExposureTime + longExposureRecoveryTime) {
    digitalWrite(shutterPin, LOW); 
    Serial.println("start long exposure recovery"); 
  } 
    
  if (time > timeAtStartOfCurrentCycle + longExposureTime + longExposureRecoveryTime & time < timeAtStartOfCurrentCycle + longExposureTime + longExposureRecoveryTime + shortExposureTime) {
    digitalWrite(shutterPin, HIGH); 
    setBrightness(1);
    Serial.println("start short exposure");    
  }
    
  if (time > timeAtStartOfCurrentCycle + longExposureTime + longExposureRecoveryTime + shortExposureTime ) {
    digitalWrite(shutterPin, LOW); 
    setBrightness(0);
    Serial.println("start short exposure recovery ");    
        
    
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
       digitalWrite(shutterPin, LOW);
       while(true){
         digitalWrite(errorPin, HIGH);
         delay(200);
         digitalWrite(errorPin, LOW);
         delay(200);
       }
}
}





void loop() {

  
  abortIfLimitSwitch();
  oporateShutter();
  checkOverRideLights();
 
}
