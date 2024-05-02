
/*
BEHOLD, AND DESPAIR THAT YE HAVE IT NOT:
Cael's Amazing Code for smoker Brains - FSM CODE BASE DESIGN
4.19.2024
Features:
Two Temperature Inputs: Ambient and Meat (smoothed with alpha filter)
Rotary Encoder Dial to control the set temperature (with max and min vals)
Button to lock Temperature setting
LCD screen displaying temperature and state information
A Relay to allow us to control a smoker augar motor (default closed) (now includes hysterisis protection threshold)

//futre future plans - control this all with a raspberry pi

//ambient temperature looks fine, meat temperature numbers still require calibration.

*** To acccess raspberry pi we type: 
ssh smokerpi@192.168.1.197

*/

//=====//  LIBRARIES  //=====//

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "max6675.h"

//=====//  PINS  //=====//
#define RotaryEncoderCLK 9
#define RotaryEncoderDT 10
#define RotaryEncoderSW 11
#define thermoCLK 13
#define thermoDO 12
#define thermoCS1 8
#define thermoCS2 7
#define MOTORPIN 6

//=====//  SPECIAL VALUES  //=====//
//temp max and min for set temp
#define TEMPMIN 0
#define TEMPMAX 500
//hysterisis threshold
#define HYSTTHRESH 5
//alpha filter value
#define ALPHA 0.8
//how long between we sample the temperature
#define TEMP_SAMPLING_PERIOD 1000

//=====// FSM VALUES //=====//
//Screen Display FSM
#define SYSTEM_STARTING 0
#define SET_TEMPERATURE 1
#define CYBER_SMOKER 2


//=====//  INTIALIZE GLOBAL VARIABLES  //=====//
LiquidCrystal_I2C lcd(0x3f, 20, 4);
MAX6675 thermocoupleMeat(thermoCLK, thermoCS1, thermoDO);
MAX6675 thermocoupleAMB(thermoCLK, thermoCS2, thermoDO);

int setTemperature = 100;
int currentSmokerTemperature = 0;
int lastSmokerTemperature = currentSmokerTemperature;
int currentMeatTemperature = 0;
int lastMeatTemperature = currentMeatTemperature;
int currentStateCLK;
int lastStateCLK;
unsigned long lastButtonPress = 0;
int lastTempMeasurement = 0;
bool motorActive = false;
bool lock = false;


void setup() {
  //INITIALIZE INPUTS
  pinMode(RotaryEncoderCLK,INPUT);
	pinMode(RotaryEncoderDT,INPUT);
	pinMode(RotaryEncoderSW, INPUT_PULLUP);

  lastStateCLK = digitalRead(RotaryEncoderCLK);

  //INITIALIZE OUTPUTS
  pinMode(MOTORPIN, OUTPUT);
  digitalWrite(MOTORPIN, LOW);

  //INTIALIZE LCD SCREEN
  lcd.init();
  lcd.backlight();
  delay(10);
  screenStateDisplayFSM(SYSTEM_STARTING);
  delay(490);
}

void loop() {
  perceive();
  act();
}

//===============// PERCEIVE //===============//
void perceive(){
  //perceive thermometers
  if(lastTempMeasurement > TEMP_SAMPLING_PERIOD){
    lastMeatTemperature = currentMeatTemperature;
    currentMeatTemperature = temperatureAlphaFilter(measureMeatTemperature(), lastMeatTemperature);
    lastSmokerTemperature = currentSmokerTemperature;
    currentSmokerTemperature = temperatureAlphaFilter(measureAmbientTemperature(), lastSmokerTemperature);
    updateScreenValues();
    lastTempMeasurement = 0;
    
    if(motorActive && (currentSmokerTemperature > (setTemperature + HYSTTHRESH))){
      motorActive = false;
    }
    if(!motorActive && (currentSmokerTemperature < setTemperature)){
      motorActive = true;
    }
  }
  lastTempMeasurement++;

  //perceive rotary encoder
  readRotaryDial();
  lock = readRotarySwitch();
}

//===============// ACT //===============//
void act(){
  //act screen
  if(lock){
    screenStateDisplayFSM(SET_TEMPERATURE);
  }else{
    screenStateDisplayFSM(CYBER_SMOKER);
  }
  updateScreenValues();

  //act motor
  if(motorActive){  ////THESE MAY BE BACKWARD!
      digitalWrite(MOTORPIN, HIGH);
    }else{
      digitalWrite(MOTORPIN, LOW);
    }
}


//========// TEMPERATURE SIGNAL PROCESSING FUNCTIONS//========//
float measureMeatTemperature(){
  Serial.println("Meat Thm: ");
  Serial.print("F = ");
  Serial.println(convertMeatThermometer(thermocoupleMeat.readFahrenheit()));
  return(convertMeatThermometer(thermocoupleMeat.readFahrenheit()));
}
float measureAmbientTemperature(){
  Serial.println("Ambi Thm: ");
  Serial.print("F = ");
  Serial.println(thermocoupleAMB.readFahrenheit());
  return(thermocoupleAMB.readFahrenheit());
}
float temperatureAlphaFilter(float inputTemp, float lastFilteredValue){
  float filteredValue = (ALPHA * lastFilteredValue) + ((1-ALPHA) * inputTemp);
  return filteredValue;
}
float convertMeatThermometer(int input){
  return(2.76*input - 526.5);
}

//========// ROTARY ENCODER PROCESSING FUNCTIONS//========//
void readRotaryDial(){
  currentStateCLK = digitalRead(RotaryEncoderCLK);

	// If last and current state of CLK are different, then pulse occurred
	// React to only 1 state change to avoid double count
	if (currentStateCLK != lastStateCLK  && currentStateCLK == 1 && !lock){

		// If the DT state is different than the CLK state then
		// the encoder is rotating CCW so decrement
		if (digitalRead(RotaryEncoderDT) != currentStateCLK) {
			setTemperature = setTemperature - 5;
      updateScreenValues();
      if(setTemperature < TEMPMIN){
        setTemperature = TEMPMIN;
      }
		} else {
			// Encoder is rotating CW so increment
			setTemperature = setTemperature + 5;
      updateScreenValues();
      if(setTemperature > TEMPMAX){
        setTemperature = TEMPMAX;
      }
		}
		Serial.print("SetTemperature: ");
		Serial.println(setTemperature);
	}
	// Remember last CLK state
	lastStateCLK = currentStateCLK;
}
bool readRotarySwitch(){
  static bool switchOn = true;
  int btnState = digitalRead(RotaryEncoderSW);
	//If we detect LOW signal, button is pressed
	if (btnState == LOW) {
		//if 50ms have passed since last LOW pulse, it means that the
		//button has been pressed, released and pressed again
		if (millis() - lastButtonPress > 50) {
        if(lock){
        switchOn = false;
      }else{
        switchOn = true;
      }
		}
		// Remember last button press event
		lastButtonPress = millis();
	}
  return switchOn;
}

//========// SCREEN CONTROL FUCNTIONS //========//
void screenStateDisplayFSM(int caseInput){
  lcd.setCursor(0, 0); 
  switch(caseInput){
    case SYSTEM_STARTING:   
    lcd.print("  SYSTEM  STARTING  "); 
    break;
    case SET_TEMPERATURE:     
    lcd.print("  SET  TEMPERATURE  ");
    break;
    case CYBER_SMOKER:
    lcd.print("    CYBER SMOKER    ");
    break;
  }  
}
void printTemperature(int tempVal, int row){
  if(tempVal < 100){
    lcd.setCursor(15, row);     
    lcd.print(" ");
    lcd.setCursor(16, row);     
    lcd.print(tempVal);   
  }else{
    lcd.setCursor(15, row);     
    lcd.print(tempVal);  
  }
}
void updateScreenValues(){
  printTemperature(setTemperature,1);  
  printTemperature(currentSmokerTemperature,2); 
  printTemperature(currentMeatTemperature,3);      
}