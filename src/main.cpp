#include <Arduino.h>
// 0.5 m/s  -> 1,06v/s
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 

  SETUP_MOTOR
  SETUP_SENSOR
  digitalWrite(power_sensor, HIGH);

}

void loop() {
  sensor[0] = digitalRead(SENSOR1); 
  sensor[1] = digitalRead(SENSOR2);
  sensor[2] = digitalRead(SENSOR3);
  sensor[3] = digitalRead(SENSOR4);
  sensor[4] = digitalRead(SENSOR5);
  
  // doc duoc la 0 khong duoc la 1
  // 11111
  // 01111
  // 00111
  // 10111
  // 10011
  // 11011 //
  // 11001
  // 11101
  // 11100
  // 11110
  // 00000

  // if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))         { error = 0;} 
  if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    { error = -4; action =1;}
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    { error = -3; action =1;}
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    { error = -2; action =1;}
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))    { error = -1; action =1;}
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))    {  error = 0; action =1;}
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))    { error = 1; action =1;}
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 1))    { error = 2; action =1;}
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))    { error = 3; action =1;}
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))    { error = 4; action =1;}
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))    {action =0; Motor_Brake();}
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    {action =0; Motor_Brake();}

  // Serial.print("acction= ");  Serial.println(action);



  // error = 0 - error;
  // P = error * Kp;
  // I += Ki*error*samplingTime;
  // D = (Kd *(error - previous_error)) / samplingTime;
  // PID_value = P + I + D;
  // previous_error = error;

  if (action == 1){
  error = error;
  P = error;
  I += error;
  D = (error - previous_error) ;
  PID_value = Kp*P + Ki*I + Kd*D;
  previous_error = error;

  // Calculating the effective motor speed:
   left_motor_speed  = 18 + PID_value;   // dau -
   right_motor_speed = 18 - PID_value;   // dau +
  // Serial.print("PID= ");  Serial.println(PID_value);
  
  // The motor speed should not exceed the max PWM value    
  // constrain(left_motor_speed, 0, 127);
  // constrain(right_motor_speed, 0, 127);
    if (left_motor_speed > 255)
        left_motor_speed = 127;
    else
    if (left_motor_speed < 0)
        left_motor_speed = 0;
    
    if (right_motor_speed > 255)
        right_motor_speed = 127;
    else
    if (right_motor_speed < 0 )
        right_motor_speed = 0;

  // nen <127 de tiet kiem dien   / truot dai
  Motor1_Forward(left_motor_speed);
  Motor2_Forward(right_motor_speed);
  action = 0;
 }
}


// ////////////////////////////////////////////////////////
void Forwardmottor(int Speed)
{
  Motor1_Forward(Speed);
  Motor2_Forward(Speed);
}

void Backward(int Speed)
{
  Motor1_Backward(Speed);
  Motor2_Backward(Speed);
}

void Motor_Brake()
{
  Motor1_Brake();
  Motor2_Brake();
}

void Turn_Right(int Speed)
{
  Motor1_Forward(Speed);
  Motor2_Brake();
}

void Turn_Left(int Speed)
{
  Motor1_Brake();
  Motor2_Forward(Speed);
}

void reverse(int Speed)
{
  Motor2_Forward(Speed);
  Motor1_Backward(Speed);
}
//////////////////////////////////////////////////////////

void Motor1_Forward(int Speed)
{
 digitalWrite(IN1,HIGH);
 digitalWrite(IN2,LOW);
 analogWrite(ENA,Speed);
}

void Motor1_Backward(int Speed)
{
 digitalWrite(IN1,LOW);
 digitalWrite(IN2,HIGH);
 analogWrite(ENA,Speed);
}

void Motor1_Brake()
{
 digitalWrite(IN1,LOW);
 digitalWrite(IN2,LOW);
}

void Motor2_Forward(int Speed)
{
 digitalWrite(IN3,HIGH);
 digitalWrite(IN4,LOW);
 analogWrite(ENB,Speed);
}

void Motor2_Backward(int Speed)
{
 digitalWrite(IN3,LOW);
 digitalWrite(IN4,HIGH);
 analogWrite(ENB,Speed);
}

void Motor2_Brake()
{
 digitalWrite(IN3,LOW);
 digitalWrite(IN4,LOW);
}
//////////////////////////////////////////////////////////
void read_sensor_values()
{
  sensor[0] = digitalRead(SENSOR1); 
  sensor[1] = digitalRead(SENSOR2);
  sensor[2] = digitalRead(SENSOR3);
  sensor[3] = digitalRead(SENSOR4);
  sensor[4] = digitalRead(SENSOR5);

  // if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))         error = -3; 
  // else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))    error = -2;
  // else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))    error = -1;
  // else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))    error =  0;
  // else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    error =  1;
  // else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))    error =  2;
  // else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))    error =  3;
  // else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))    error =  10;
  // academia.edu/15513253/Robot_do_line

  // doc duoc la 0 khong duoc la 1
  // 11111
  // 01111
  // 00111
  // 10111
  // 10011
  // 11011
  // 11001
  // 11001
  // 11101
  // 11100
  // 00000
  // if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))         { Motor_Brake();} 
  // else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    { Motor1_Backward(25);  Motor2_Forward(50);}
  // else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    { Motor1_Backward(15);  Motor2_Forward(35); }
  // else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    { Motor1_Backward(15);  Motor2_Forward(25); }
  // else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))    { Motor1_Backward(10);  Motor2_Forward(25);}
  // else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))    { Forwardmottor(30);}
  // else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))    { Motor1_Forward(25); Motor2_Backward(10);}
  // else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 1))    { Motor1_Forward(25); Motor2_Backward(15);}
  // else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))    { Motor1_Forward(35); Motor2_Backward(15);}
  // else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))    {  Motor1_Forward(50); Motor2_Backward(25);}
  // else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))    { 
    // while (1)
    // { 
    //   reverse(25);
    //   if ((sensor[0] | sensor[1] | sensor[2] | sensor[3] | sensor[4]) == 1) break;
    // }
  //   Motor_Brake();
    
  // }

}




/*
  driver cho dong co phun
  int ENA_pump = 9; // tu chon chan
  int IN1_pump = 8;
  int IN2_pump = 7;

void setup() 
{
  pinMode(ENA_pump, OUTPUT);
  pinMode(IN1_pump, OUTPUT);
  pinMode(IN2_pump, OUTPUT);

}
  
  digitalWrite(IN1_pump, LOW);
  digitalWrite(IN2_pump, LOW);

  void motor_pump_on()
  {
    digitalWrite(IN1_pump, HIGH);
    digitalWrite(IN2_pump, LOW);
  }

  void motor_pump_off()
  {
    digitalWrite(IN1_pump, LOW);
    digitalWrite(IN2_pump, LOW);
  }



  // void check_mode(int mode)
  // { 
    // b1 : doc uart tu esp32    
    // while ( available())
  //   {
  //     switch (inputString)
  //     {
  //     case "1":
  //       auto_mode();
  //       break;
  //     case "2":
  //       manual_mode(mode);
  //       break;     
  //     default:
  //       Motor_Brake();
  //       break;
  //     }
  //   }
  // }
    


void manual_mode(int mode)
{
    switch (mode)
    {
    case 1:
      Forwardmottor(50);  
      break;
    case 2:
      Backward(40);     
      break;
    case 3:
      Turn_Right(50);
      break;
    case 4:
      Turn_Left(50);
      break;    
    default:
      Motor_Brake();
      break;
    }
}
*/
