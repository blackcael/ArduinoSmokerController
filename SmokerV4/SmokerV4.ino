



// UNFINISHED



/*
BEHOLD, AND DESPAIR THAT YE HAVE IT NOT:
Cael's Amazing Code for smoker Brains
4.10.2024
Features:
Two Temperature Inputs: Ambient and Meat (smoothed with alpha filter)
Rotary Encoder Dial to control the set temperature (with max and min vals)
Button to lock Temperature setting
LCD screen displaying temperature and state information
A Relay to allow us to control a smoker augar motor (default closed) (now includes hysterisis protection threshold)

//futre future plans - control this all with a raspberry pi instead

//ambient temperature looks fine, meat temperature numbers still require calibration.

*** To acccess raspberry pi we type: 
ssh smokerpi@192.168.1.197

*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "max6675.h"
//PINS
#define RotaryEncoderCLK 9
#define RotaryEncoderDT 10
#define RotaryEncoderSW 11
#define thermoCLK 13
#define thermoDO 12
#define thermoCS1 8
#define thermoCS2 7
#define MOTORPIN 6

//SPECIAL VALUES
//temp max and min for set temp
#define TEMPMIN 0
#define TEMPMAX 500
//hysterisis threshold
#define HYSTTHRESH 5
//alpha filter value
#define ALPHA 0.8


LiquidCrystal_I2C lcd(0x3f, 20, 4);  
int setTemperature = 100;
int currentSmokerTemperature = 0;
int currentMeatTemperature = 0;
int currentStateCLK;
int lastStateCLK;
unsigned long lastButtonPress = 0;
int lastTempMeasurement = 0;
bool motorActive = false;
bool lock = false;
unsigned long cookstartTime = 0;

MAX6675 thermocoupleMeat(thermoCLK, thermoCS1, thermoDO);
MAX6675 thermocoupleAMB(thermoCLK, thermoCS2, thermoDO);

void setup() {
  Serial.begin(9600);

  pinMode(RotaryEncoderCLK,INPUT);
	pinMode(RotaryEncoderDT,INPUT);
	pinMode(RotaryEncoderSW, INPUT_PULLUP);
  pinMode(MOTORPIN, OUTPUT);

  digitalWrite(MOTORPIN, LOW);

  lastStateCLK = digitalRead(RotaryEncoderCLK);

  lcd.init();              // Initialize the LCD
  lcd.backlight();         // Turn on the backlight
  
  // Set cursor to the top left corner and print the string on the first row
  lcd.setCursor(0, 0);     
  lcd.print("  SYSTEM  STARTING  "); 
  
  // Move to the second row and print the string
  lcd.setCursor(0, 1);     
  lcd.print("Set Temp: "); 
  
  // Move to the third row and print the string
  lcd.setCursor(0, 2);     
  lcd.print("Smoker Temp:    "); 
  
  // Move to the fourth row and print the string
  lcd.setCursor(0, 3);     
  lcd.print("Meat Temp: ");  
  // wait for MAX chip to stabilize
  delay(500);
  lcd.setCursor(0, 0);     
  lcd.print("  SET  TEMPERATURE  ");
}

void loop() {
  // basic readout test, just print the current temp
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

	// Read the button state
	int btnState = digitalRead(RotaryEncoderSW);

	//If we detect LOW signal, button is pressed
	if (btnState == LOW) {
		//if 50ms have passed since last LOW pulse, it means that the
		//button has been pressed, released and pressed again
		if (millis() - lastButtonPress > 50) {
        if(lock){
        lock = false;
      }else{
        lock = true;
      }
		}
    if(lock){
      lcd.setCursor(0, 0);     
      lcd.print("    CYBER SMOKER    "); 
    }else{
      lcd.setCursor(0, 0);     
      lcd.print("  SET  TEMPERATURE  ");
    }
		// Remember last button press event
		lastButtonPress = millis();
	}

   // UPDATE Measured Temps - For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  if(lastTempMeasurement > 999){
    float lastMeatTemperature = currentMeatTemperature;
    currentMeatTemperature = temperatureAlphaFilter(measureMeatTemperature(), lastMeatTemperature);
    float lastSmokerTemperature = currentSmokerTemperature;
    currentSmokerTemperature = temperatureAlphaFilter(measureAmbientTemperature(), lastSmokerTemperature);
    updateScreenValues();
    lastTempMeasurement = 0;
    
    if(motorActive && (currentSmokerTemperature > (setTemperature + HYSTTHRESH))){
      motorActive = false;
    }
    if(!motorActive && (currentSmokerTemperature < setTemperature)){
      motorActive = true;
    }

    if(motorActive){  ////THESE MAY BE BACKWARD!
      digitalWrite(MOTORPIN, LOW);
    }else{
      digitalWrite(MOTORPIN, HIGH);
    }
  }
  lastTempMeasurement++;

  delay(1);
  
}
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

float convertMeatThermometer(int input){
  return(2.76*input - 546.5);
  //added 20 4.14.2024 -- still requires better calibration
l;;;;;;;
}

void updateScreenValues(){
  printTemperature(setTemperature,1);  

  printTemperature(currentSmokerTemperature,2); 
  
  printTemperature(currentMeatTemperature,3);      
  //lcd.print();  
  
}

float temperatureAlphaFilter(float inputTemp, float lastFilteredValue){
  float filteredValue = (ALPHA * lastFilteredValue) + ((1-ALPHA) * inputTemp);
  return filteredValue;
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