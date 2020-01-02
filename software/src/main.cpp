#include <Arduino.h>
#include <PID_v1.h>
#include <stdio.h>

#define PIN_INPUT PIN_A0 // Left PT100
#define PIN_OUTPUT PIN6 // Left Mosfet (heater)
#define SCL PIN3
#define SDA PIN2

double setpoint, input, output;
double alpha = 0.00385; // ohm / deg (Celius)

// Controller Gains
double K_p=1, K_i=0, K_d=0;

// Create the controller
PID myPID(&input, &output, &setpoint, K_p, K_i, K_d, DIRECT);

void setup(){
  Serial.begin(9600);
  Serial.println("==================== Ultimaker Print Core cleaner ====================");
  Serial.println("--------------------     By Isak Åslund, CASE     --------------------");
  //initialize the variables we're linked to
  input = analogRead(PIN_INPUT);
  setpoint = 100; // Celcius

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop(){
  input = analogRead(PIN_INPUT);

  double voltage = input * 5 / 1023.0;
  double current = (5 - voltage) / 120.0;
  double res = voltage / current;
  double temp = (res/100 - 1) / alpha;


  //myPID.Compute();
  //analogWrite(PIN_OUTPUT, output);



  char buffer [200];  
  sprintf(buffer, "Input = %d\t Voltage = %0.3f\t Current = %0.3f\t Resistance = %0.3f\t Temperature = %0.3f", (int) input, voltage, current, res, temp);
  Serial.println(buffer);
  
}
