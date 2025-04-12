
#include <PID_v1.h>
#include "thermocouples.h"
#include "pulse_control.h"

const int tempSensor = A0; // Temperature Sensor input
const double setTemp = 22;//178.176; // Set Temperature Value of 37 degrees Celcius
const int pwmOut = 9; // PWM Output
double tempLevel; //variable that stores the temperature level

// Tuning parameters
float kp=0; //Initial Proportional Gain 
float ki=0; //Initial Integral Gain 
float kd=0;  //Initial Differential Gain 

double Setpoint, Input, Output;  //These are just variables for storing values
PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT); // This sets up our PDID Loop
//Input is our PV
//Output is our u(t)
//Setpoint is our SP
const int sampleRate = 1; // Variable that determines how fast our PID loop runs

// Communication setup
const long serialPing = 1000; //This determines how often we ping our loop
// Serial pingback interval in milliseconds
unsigned long now = 0; //This variable is used to keep track of time placeholder for current timestamp
unsigned long lastMessage = 0; //This keeps track of when our loop last spoke to serial last message timestamp.

#define WindowSize 200
#define WindowSize_cooler 200
unsigned long nextTime, startTime, actualTime, windowStartTime; 

double tiempo;
double temperatura;

void setup(){
  readThermocouple(&tempLevel);
  Input = tempLevel;
  Setpoint = setTemp;  //set @ 22
  
  Serial.begin(115200); //Start a serial session
  myPID.SetMode(AUTOMATIC);  //Turn on the PID loop
  myPID.SetSampleTime(sampleRate); //Sets the sample rate
  
  Serial.println("Begin"); // Hello World!
  lastMessage = millis(); // timestamp
}

void loop(){
  Setpoint = setTemp; //set temperature at 37 degrees Celcius
  actualTime = millis();
  if(actualTime > windowStartTime){
    readThermocouple(&tempLevel);
    Input = tempLevel;

    windowStartTime = actualTime + WindowSize;
}
  myPID.Compute();  //Run the PID loop

  setPwmPulse(Output);
  
  now = millis(); //Keep track of time
  if((now - lastMessage) > serialPing) {  //If its been long enough give us some info on serial
    if (Serial.available() > 0) { //If we sent the program a command deal with it
      for (int x = 0; x < 4; x++) {
        switch (x) {
          case 0:
            kp = Serial.parseFloat();  
            break;
          case 1:
            ki = Serial.parseFloat();
            break;
          case 2:
            kd = Serial.parseFloat();
            break;
          case 3:
            for (int y = Serial.available(); y == 0; y--) {
              Serial.read();  //Clear out any residual junk
            }
            break;
        }
      }

      Serial.printf("kp: %.3f - ki: %.3f - kd: %.3f\n", kp, ki, kd);
      myPID.SetTunings(kp, ki, kd); //Set the PID gain constants and start running
    }
    
    lastMessage = now; 
    //update the time stamp. 
  }
  
}