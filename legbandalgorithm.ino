#include <PID_v1.h>

const int numDelay = 200; //Constant for setting the delay

//Define Variables we'll be connecting to
double Setpoint_up, Setpoint_low, Input, Output1, Output2;

//Specify the links and initial tuning parameters for PID
PID myPIDRise(&Input, &Output1, &Setpoint_up, 30, 3, 0, DIRECT);
PID myPIDFall(&Input, &Output2, &Setpoint_low, 30, 3, 0, REVERSE);

//Defining the variables for changing the tuning parameters during run-time
//First set to the initial values
double Kp = 30, Ki = 3, Kd = 0;

//motor speed. Maximum could be 255
double cwMotor = 64;  //clockwise speed
double ccwMotor = 64; //anti-clockwise speed

bool isTight; //The flag for compression state of the bandage

unsigned long intervalStartTime; // To store the start time for the delay interval
unsigned long intervalOne = 2000; //the time we need to wait when motor has rotated clockwise, high pressure interval
unsigned long intervalTwo = 2000; //the time we need to wait when motor has rotated anticlockwise, low pressure interval

// FSR Variables
int fsrPin5 = A1; // the FSR 5 and 10K resistor are connected to A1
int fsrPin6 = A3; // the FSR 6 and 10K resistor are connected to A3
int fsrPin7 = A4; // the FSR 7 and 10K resistor are connected to A4

double fsrReading5; //the analog reading from the FSR resistor divider
double fsrReading6;
double fsrReading7;

double fsrVoltage5; //the analog reading converted to voltage
double fsrVoltage6;
double fsrVoltage7;

double fsrForce_mmHg_5; //Finally the voltage converted to force (mmHg) using the formula provided in the thesis
double fsrForce_mmHg_6;
double fsrForce_mmHg_7;
double fsrForce_mmHgAvg; //Average force from fsr sensors

//String variables to store the values and controls coming in from serial port
String inputString = "";
String controlCode = "";
String stringValue = "";

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
	// Required for Serial on Zero based boards
	#define Serial SERIAL_PORT_USBVIRTUAL
#endif

void setup()
{ 
	//initialize the variables we're linked to
	Setpoint_up = 70;
	Setpoint_low = 15;

	myPIDRise.SetOutputLimits(0, cwMotor);
	myPIDFall.SetOutputLimits(0, ccwMotor);
	//turn the PID on
	myPIDRise.SetMode(AUTOMATIC);
	myPIDFall.SetMode(AUTOMATIC);
	
	//Calculations for translating the voltage reading at the Arduino pins into pressure values
	fsrReading5 = analogRead(fsrPin5);
	fsrReading6 = analogRead(fsrPin6);
	fsrReading7 = analogRead(fsrPin7);
	
	fsrVoltage5 = fsrReading5 * (5.0/1023.0);
	fsrVoltage6 = fsrReading6 * (5.0/1023.0);
	fsrVoltage7 = fsrReading7 * (5.0/1023.0);
	
	fsrForce_mmHg_5 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage5)));
	fsrForce_mmHg_6 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage6)));
	fsrForce_mmHg_7 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage7)));
	fsrForce_mmHgAvg = (fsrForce_mmHg_5 + fsrForce_mmHg_6 + fsrForce_mmHg_7) / 3;
	
	
	//Figuring out the initial state of the bandage
	if(fsrForce_mmHgAvg > (Setpoint_up + Setpoint_low) / 2){
		isTight = true;
	}
	else{ //fsrForce_mmHgAvg < (Setpoint_up + Setpoint_low) / 2
		isTight = false;
	}
	Input = fsrForce_mmHgAvg;
	Serial.begin(9600);   //Baudrate
}

void loop(){
	while(Serial.available()){
        inputString = Serial.readStringUntil('\n'); //The string sent through the serial port read until the end of the line
        controlCode = String(inputString.substring(0,2)); //The prefix before the string to determine which variable is being updated
        stringValue = String(inputString.substring(2)); //The actual value for the variable
    
        if(controlCode == "Kp"){
          Kp = stringValue.toFloat();
          //Serial.println(Kp);
        }
        else if(controlCode == "Ki"){
          Ki = stringValue.toFloat();
          //Serial.println(Ki);
        }
        else if(controlCode == "Kd"){
          Kd = stringValue.toFloat();
          //Serial.println(Kd);
        }
        else if(controlCode == "Pu"){
          Setpoint_up = stringValue.toFloat();
          //Serial.println(Setpoint_up);
        }
        else if(controlCode == "Pl"){
          Setpoint_low = stringValue.toFloat();
          //Serial.println(Setpoint_low);
        }
        else if(controlCode == "Iu"){//High
          intervalOne = stringValue.toFloat();
          //Serial.println(intervalOne);
        }
        else if(controlCode == "Il"){//Low
          intervalTwo = stringValue.toFloat();
          //Serial.println(intervalTwo);
        }
        //Handles the case when the user clicks the reset button, corresponds to "default"
        //the desired pressure parameters are not reset
        else{
          Kp = 30;
          Ki = 3;
          Kd = 0;
        }
    }

  myPIDRise.SetTunings(Kp, Ki, Kd);
  myPIDFall.SetTunings(Kp, Ki, Kd);
  
	if(isTight){ //Loosening bandage
    Serial.println("Loosening");
		while(fsrForce_mmHgAvg > Setpoint_low){
			Serial.println("Loosening Loop");
			myPIDFall.Compute();
			analogWrite(6, Output2);
			analogWrite(10, 0);
			
			fsrReading5 = analogRead(fsrPin5);
			fsrReading6 = analogRead(fsrPin6);
			fsrReading7 = analogRead(fsrPin7);
			
			fsrVoltage5 = fsrReading5 * (5.0/1023.0);
			fsrVoltage6 = fsrReading6 * (5.0/1023.0);
			fsrVoltage7 = fsrReading7 * (5.0/1023.0);
			
			fsrForce_mmHg_5 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage5)));
			fsrForce_mmHg_6 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage6)));
			fsrForce_mmHg_7 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage7)));
			fsrForce_mmHgAvg = (fsrForce_mmHg_5 + fsrForce_mmHg_6 + fsrForce_mmHg_7) / 3;
						
			delay(numDelay);
			Serial.println(fsrForce_mmHgAvg);
			
			Input = fsrForce_mmHgAvg;
		}
		
		intervalStartTime = millis();
		while((millis() - intervalStartTime) < intervalTwo){
			Serial.println("WaitLoosen");
			analogWrite(6, 0);
			analogWrite(10, 0);
			
			fsrReading5 = analogRead(fsrPin5);
			fsrReading6 = analogRead(fsrPin6);
			fsrReading7 = analogRead(fsrPin7);
			
			fsrVoltage5 = fsrReading5 * (5.0/1023.0);
			fsrVoltage6 = fsrReading6 * (5.0/1023.0);
			fsrVoltage7 = fsrReading7 * (5.0/1023.0);
			
			fsrForce_mmHg_5 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage5)));
			fsrForce_mmHg_6 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage6)));
			fsrForce_mmHg_7 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage7)));
			fsrForce_mmHgAvg = (fsrForce_mmHg_5 + fsrForce_mmHg_6 + fsrForce_mmHg_7) / 3;
			
			delay(numDelay);
			Serial.println(fsrForce_mmHgAvg);
		}
		
		isTight = false;
		Input = fsrForce_mmHgAvg;
	}
	
	else{ //isTight = false, tightening the bandage
    Serial.println("Tightening");
		while(fsrForce_mmHgAvg < Setpoint_up){
			Serial.println("Tightening Loop");
			myPIDRise.Compute();
			analogWrite(6, 0);
			analogWrite(10, Output1);
			
			fsrReading5 = analogRead(fsrPin5);
			fsrReading6 = analogRead(fsrPin6);
			fsrReading7 = analogRead(fsrPin7);
			
			fsrVoltage5 = fsrReading5 * (5.0/1023.0);
			fsrVoltage6 = fsrReading6 * (5.0/1023.0);
			fsrVoltage7 = fsrReading7 * (5.0/1023.0);
			
			fsrForce_mmHg_5 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage5)));
			fsrForce_mmHg_6 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage6)));
			fsrForce_mmHg_7 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage7)));
			fsrForce_mmHgAvg = (fsrForce_mmHg_5 + fsrForce_mmHg_6 + fsrForce_mmHg_7) / 3;
						
			delay(numDelay);
			Serial.println(fsrForce_mmHgAvg);
			
			Input = fsrForce_mmHgAvg;
		}
		
		intervalStartTime = millis();
		while((millis() - intervalStartTime) < intervalOne){
			Serial.println("WaitTight");
			analogWrite(6, 0);
			analogWrite(10, 0);
      
			fsrReading5 = analogRead(fsrPin5);
			fsrReading6 = analogRead(fsrPin6);
			fsrReading7 = analogRead(fsrPin7);
			
			fsrVoltage5 = fsrReading5 * (5.0/1023.0);
			fsrVoltage6 = fsrReading6 * (5.0/1023.0);
			fsrVoltage7 = fsrReading7 * (5.0/1023.0);
			
			fsrForce_mmHg_5 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage5)));
			fsrForce_mmHg_6 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage6)));
			fsrForce_mmHg_7 = 5.286*(pow(2.7183,(0.7559 * fsrVoltage7)));
			fsrForce_mmHgAvg = (fsrForce_mmHg_5 + fsrForce_mmHg_6 + fsrForce_mmHg_7) / 3;
						
			delay(numDelay);
			Serial.println(fsrForce_mmHgAvg);
		}
		
		isTight = true;
		Input = fsrForce_mmHgAvg;
	}
}
