#include <Wire.h>
#include <PID_v1.h>

// I2C address of the Arduino Nano
#define NANO_ADDRESS 0x04

// ADC Pin assignments for sensors
#define SENSOR1  //Outer Left
#define SENSOR2  //Left
#define SENSOR3  //Middle Left
#define SENSOR4  //Middle Right
#define SENSOR5  //Right
#define SENSOR6  //Outer Right

int baseMotorSpeed = 150; //base motor speed of the EEEbot

//setting initial motor speeds and servo angle
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int servoPosition = 120;

//servo angle so wheels are facing forwards
int centerServoPosition = 120; 

//center of the car
int setpoint = 0; 

//motor scaling factor
float K = 0.098;

// distances from point to each of the sensors
float distanceToSensor1 = -35;
float distanceToSensor2 = -20;
float distanceToSensor3 = -5;
float distanceToSensot4 = 5;
float distanceToSensor5 = 20;
float distanceToSensor6 = 35;

// calibration variables
int black1, black2, black3, black4, black5, black6;
int white1, white2, white3, white4, white5, white6;

// PiD variables
double Setpoint, Input, Output;
double Kp= 67, Ki=0, Kd=0;


PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Start the I2C communication
  Wire.begin();
  Serial.begin(9600);

  // Set the pin modes for the sensor pairs
  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);
  pinMode(SENSOR3, INPUT);
  pinMode(SENSOR4, INPUT);
  pinMode(SENSOR5, INPUT);
  pinMode(SENSOR6, INPUT);

  //Calibrated values for sensors when car is over black line
  black1 = 521;
  black2 = 536;
  black3 = 648;
  black4 = 557;
  black5 = 664;
  black6 = 607;

  //Calibrated values for when the car is over the white background
  white1 = 4095;
  white2 = 4095;
  white3 = 4095;
  white4 = 4095;
  white5 = 4095;
  white6 = 4095;

  // Initialize PiD
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1000);
  Setpoint = 0;
}

void loop() {


  // Read the values from all 6 sensors
  int sensorValue1 = analogRead(SENSOR1);
  int sensorValue2 = analogRead(SENSOR2);
  int sensorValue3 = analogRead(SENSOR3);
  int sensorValue4 = analogRead(SENSOR4);
  int sensorValue5 = analogRead(SENSOR5);
  int sensorValue6 = analogRead(SENSOR6);

  //Constrain the values to a specified range, to remove extremes
  sensorValue1 = constrain(sensorValue1, black1, white1);
  sensorValue2 = constrain(sensorValue2, black2, white2);
  sensorValue3 = constrain(sensorValue3, black3, white3);
  sensorValue4 = constrain(sensorValue4, black4, white4);
  sensorValue5 = constrain(sensorValue5, black5, white5);
  sensorValue6 = constrain(sensorValue6, black6, white6);

  //Maps the values into the 0 to 255 range
  sensorValue1 = map(sensorValue1, black1, white1, 0, 255);
  sensorValue2 = map(sensorValue2, black2, white2, 0, 255);
  sensorValue3= map(sensorValue3, black3, white3, 0, 255);
  sensorValue4= map(sensorValue4, black4, white4, 0, 255);
  sensorValue5= map(sensorValue5, black5, white5, 0, 255);
  sensorValue6= map(sensorValue6, black6, white6, 0, 255);

  // Calculate the weighted average of the sensor values
  float numerator = (sensorValue1 * distanceToSensor1 + sensorValue2 * distanceToSensor2 + sensorValue3 * distanceToSensor3 + sensorValue4 * distanceToSensor4 + sensorValue5 * distanceToSensor5 + sensorValue6 * distanceToSensor6);
  float denominator = (sensorValue1 + sensorValue2 + sensorValue3 + sensorValue4 + sensorValue5 + sensorValue6);
  float weightedAverage = numerator/denominator;



  //Serial.print("Weighted average: ");
  //Serial.println(avg);
  
  Input = weightedAverage;
  myPID.Compute();

  //Serial.print("Output:");
  //Serial.println(Output);

  // Calculate the new speed and steering values
  //Right turns are minus values
  //Left turns are positive values
  
  leftMotorSpeed = baseMotorSpeed + (K * Output);
  rightMotorSpeed = baseMotorSpeed - (K * Output);
  servoPosition = centerServoPosition + Output;

  // Constrain the values to the appropriate range
  leftMotorSpeed = constrain(leftSpeed, 0, 255);
  rightMotorSpeed = constrain(rightSpeed, 0, 255);
  servoPosition = constrain(servoPosition, 60, 180);

  transmitArduino(leftMotorSpeed, rightMotorSpeed, servoPosition);
}

  // Send the values to the Arduino Nano

void transmitArduino(int leftMotorSpeed, int rightMotorSpeed, int servoPosition)

{

  Wire.beginTransmission(NANO_ADDRESS);  // transmit to device #4

  Wire.write((byte)((leftMotorSpeed & 0x0000FF00) >> 8));  // first byte of leftMotor, containing bits 16 to 9

  Wire.write((byte)(leftMotorSpeed & 0x000000FF));  // second byte of leftMotor, containing the 8 LSB - bits 8 to 1

  Wire.write((byte)((rightMotorSpeed & 0x0000FF00) >> 8));  // first byte of rightMotor, containing bits 16 to 9

  Wire.write((byte)(righMotorSpeed & 0x000000FF));  // second byte of rightMotor, containing the 8 LSB - bits 8 to 1

  Wire.write((byte)((servoPosition & 0x0000FF00) >> 8));  // first byte of rightMotor, containing bits 16 to 9

  Wire.write((byte)(servoPosition & 0x000000FF));  // second byte of rightMotor, containing the 8 LSB - bits 8 to 1

  Wire.endTransmission();  // stop transmitting
}