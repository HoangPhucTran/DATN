#include <Arduino.h>

///////////////  MODE MOTOR  ///////////////
int mode = 0;
#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2
// uart
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
String command = "";         // a String to hold incoming data

////////////////////////////////////////////
///////////  FOR DRIVER MOTER  ///////////
//MOTOR1 : LEFT, MORTOR2: RIGHT
const int IN1=5;
const int IN2=4;
const int ENA=6;

const int IN3=10; // 8 hu
const int IN4=7;
const int ENB=9;
////////////////////////////////////////////

///////////////  FOR SENSOR  ///////////////
const int SENSOR1 = 42;
const int SENSOR2 = 44;
const int SENSOR3 = 46;
const int SENSOR4 = 48;
const int SENSOR5 = 50;
const int power_sensor = 52; // out put 5v for sensor

// const int HONGNGOAI = 49;    
// const int CONGTAC = 47;

// Initial Values of Sensors
int sensor[5] = {0, 0, 0, 0, 0};  
float error = 0 ;// result of handling data sensor  
int action = 0;
////////////////////////////////////////////

/////////////////FOR PID////////////////////
  float Kp=7.5, Kd=5, Ki=0;
  
  float P, I, D;
  float samplingTime = 0.01;
  float previous_error=0;
  float PID_value;
  int left_motor_speed = 0;
  int right_motor_speed= 0;
////////////////////////////////////////////

#define SETUP_SENSOR             \
do{                              \
  pinMode(SENSOR1, INPUT);       \
  pinMode(SENSOR2, INPUT);       \
  pinMode(SENSOR3, INPUT);       \
  pinMode(SENSOR4, INPUT);       \
  pinMode(SENSOR5, INPUT);       \
  pinMode(power_sensor, OUTPUT);       \
}while (0);

#define SETUP_MOTOR     \
do{                     \
  pinMode(IN1, OUTPUT); \
  pinMode(IN2, OUTPUT); \
  pinMode(ENA, OUTPUT); \
                        \
  pinMode(IN4, OUTPUT); \
  pinMode(IN3, OUTPUT); \
  pinMode(ENB, OUTPUT); \
}while (0);

//////////////////////////////////////////
void Forwardmottor(int Speed);
void Backward(int Speed);
void Motor_Brake();
void Turn_Right(int Speed);
void Turn_Left(int Speed);
void reverse(int Speed);
//////////////////////////////
void Motor1_Forward(int Speed);
void Motor1_Backward(int Speed);
void Motor1_Brake();
void Motor2_Forward(int Speed);
void Motor2_Backward(int Speed);
void Motor2_Brake();
/////////////////////////////
void read_sensor_values();
/////////////////////////////
void command_mode();
void calculatePID();
void motorPIDcontrol();
void dc_pump(int state);