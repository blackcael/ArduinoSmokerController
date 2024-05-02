/*
BEHOLD, AND DESPAIR THAT YE HAVE IT NOT:
Cael's Amazing Code for smoker Brains
3.23.2024
Features:
Two Temperature Inputs: Ambient and Meat
Rotary Encoder Dial to control the set temperature (with max and min vals)
A Relay to allow us to control a smoker augar motor

*** To acccess raspberry pi we type: 
ssh smokerpi@192.168.1.197

*/


#include "max6675.h"
//PINS
#define RotaryEncoderCLK 2
#define RotaryEncoderDT 3
#define RotaryEncoderSW 4
#define thermoCLK 13
#define thermoDO 12
#define thermoCS1 8
#define thermoCS2 7
#define MOTORPIN 6

//SPECIAL VALUES
#define TEMPMIN 0
#define TEMPMAX 400



int setTemperature = 100;
int currentSmokerTemperature = 0;
int currentStateCLK;
int lastStateCLK;
unsigned long lastButtonPress = 0;
int lastTempMeasurement = 0;
bool motorActive = false;

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

  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize
  delay(500);
}

void loop() {
  // basic readout test, just print the current temp
  currentStateCLK = digitalRead(RotaryEncoderCLK);

	// If last and current state of CLK are different, then pulse occurred
	// React to only 1 state change to avoid double count
	if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

		// If the DT state is different than the CLK state then
		// the encoder is rotating CCW so decrement
		if (digitalRead(RotaryEncoderDT) != currentStateCLK) {
			setTemperature = setTemperature - 5;
      if(setTemperature < TEMPMIN){
        setTemperature = TEMPMIN;
      }
		} else {
			// Encoder is rotating CW so increment
			setTemperature = setTemperature + 5;
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
			Serial.println("Button pressed!");
		}

		// Remember last button press event
		lastButtonPress = millis();
	}
   // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  if(lastTempMeasurement > 999){
    currentSmokerTemperature = measureTemperature();
    lastTempMeasurement = 0;
    
    if(currentSmokerTemperature < setTemperature){
      motorActive = false;
    }else{
      motorActive = true;
    }
    if(motorActive){
      digitalWrite(MOTORPIN, HIGH);
    }else{
      digitalWrite(MOTORPIN, LOW);
    }
  }
  lastTempMeasurement++;
  delay(1);
}

float measureTemperature(){
  Serial.println("Meat Thm: ");
  Serial.print("F = ");
  Serial.println(convertMeatThermometer(thermocoupleMeat.readFahrenheit()));

  Serial.println("Ambi Thm: ");
  Serial.print("F = ");
  Serial.println(thermocoupleAMB.readFahrenheit());
  return(thermocoupleAMB.readFahrenheit());
}

float convertMeatThermometer(int input){
  return(2.76*input - 526.5);
}


