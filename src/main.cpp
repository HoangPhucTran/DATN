#include <Arduino.h>
#include "line_following.h"

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2); 
// 0.5 m/s  -> 1,06v/s
float PID_value_pre = 0 ;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  Serial1.begin(9600); // uart
  inputString.reserve(200);

  SETUP_MOTOR
  SETUP_SENSOR

  // digitalWrite(power_sensor, HIGH);

  pinMode(IN1_pump, OUTPUT);
  pinMode(IN2_pump, OUTPUT);
  digitalWrite(IN1_pump, LOW);
  digitalWrite(IN2_pump, LOW);

  lcd.init();                    
  lcd.backlight();
  lcd.setCursor(2,0);
  lcd.print("DHSPKT HCM");
  lcd.setCursor(0,1);
  lcd.print("Xin chao cac ban");
}

void loop() {  

/*
b1 : uart de lay du lieu
b2 : phan tach command de lay du lieu
b3 : 1 la chay do line, 2 la chay manual trong dong while 
b4 : neu chay do line thi tinh pid
b5 : 

  
*/  
  if ((stringComplete) ) {
  Serial.println("\n\n\n______test________");
  Serial.println(inputString);
  Serial.println(command[0]);
  Serial.println(command[1]);
  Serial.println(command[2]);
  Serial.println(command[3]);
  
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("aa");

  lcd.setCursor(4,0);
  lcd.print(inputString);
  lcd.setCursor(0,1); 
  lcd.print(command[0]);
  Serial.println("\n\n\n______test end");
  inputString = "";
  stringComplete = false;
  command_mode();
  command = "";  
  }
  
  if (mode == FOLLOWING_LINE)
  { 
    read_sensor_values();
    switch (mode)
    {
    case STOPPED:
      Motor_Brake();
      break;
    case NO_LINE:
      Motor_Brake();
      break;
    case FOLLOWING_LINE:
      calculatePID();
      motorPIDcontrol();
      break;
    }
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
  Motor2_Backward(0.2*Speed);
}
void Turn_Left(int Speed)
{
  Motor1_Backward(0.2*Speed);
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

  // delay(500);
  // lcd.clear();
  // lcd.setCursor(2,0);
  // lcd.print("LINE FOLLOWING MODE");
  //   lcd.setCursor(0,1);
  // lcd.print("sensor");
  // lcd.setCursor(6,1);
  // lcd.print(sensor[0]);
  // lcd.setCursor(7,1);
  // lcd.print(sensor[1]);
  // lcd.setCursor(8,1);
  // lcd.print(sensor[2]);
  // lcd.setCursor(9,1);
  // lcd.print(sensor[3]);
  // lcd.setCursor(10,1);
  // lcd.print(sensor[4]);
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

  // if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))         error = -3; 
  // else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))    error = -2;
  // else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))    error = -1;
  // else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))    error =  0;
  // else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    error =  1;
  // else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))    error =  2;
  // else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))    error =  3;
  // else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))    error =  10;
  // academia.edu/15513253/Robot_do_line

  // if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))         { error = 0;} 
  if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    { error = -4;}
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    { error = -3;}
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    { error = -2;}
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))    { error = -1;}
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))    {  error = 0;}
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))    { error = 1;}
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 1))    { error = 2;}
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))    { error = 3;}
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))    { error = 4;}
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))    {Motor_Brake(); error = 0;}
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    {Motor_Brake(); error = 0;}
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
*/

////////////////////////UART////////////////////////////////
void serialEvent() {
  while (Serial1.available() ) {
    // get the new byte: 
      delay(10);
    char inChar = (char)Serial1.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    Serial.println("______");
    Serial.println(inputString);
    Serial.println("______");
    Serial.println(inputString.length());

    Serial.println("___11111111___");
    // if (inputString == '1') {
    //   delay(20);
    //   while (Serial1.available()) {
    //      delay(20);
    //      char inChar = (char)Serial1.read();
    //      inputString += inChar;
    //       Serial.println("aaaaaaaaaaaaaaaaa");
    //       Serial.println(inputString);
    //       Serial.println("-----");
    //   }
    // }
    if (inputString.length() == 2) {
      delay(20);
      while (Serial1.available()) {
         delay(20);
         char inChar = (char)Serial1.read();
         inputString += inChar;
          Serial.println("-----");
          Serial.println(inputString);
          Serial.println("-----");
      }
      Serial.println("++++++");
      Serial.println(inputString);
      command = inputString;
      stringComplete = true;
    }
  }
}
void serialEventRun(void) {
  if (Serial1.available()) serialEvent();
}
////////////////////////////////////////////////////////////

void command_mode()
{
    switch (command[0])
    {
    case 'w':
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("wwwww ne ");
      Forwardmottor(40);
      if (command[2] == 'p') Motor_Brake();      
      break;
    case 's':
      dc_pump(0);
      Backward(40);  
      if (command[2] == 'p') Motor_Brake();      
      break;
    case 'd':
      dc_pump(0);
      Turn_Right(45); 
      if (command[2] == 'p') Motor_Brake();      
      break;
    case 'a':
      dc_pump(0);
      Turn_Left(45);
      if (command[2] == 'p') Motor_Brake();      
      break;
    case 'l':
      // (command[2] == 'b') && (command[3] == '1') ? dc_pump(1) : dc_pump(0);
      mode = FOLLOWING_LINE;
      break;
    case 'b':
      command[1] == '1' ? dc_pump(1) : dc_pump(0);
      break;    
    case 'p':
      Motor_Brake(); // stop motor
      mode = STOPPED;
      (command[2] == 'b') && (command[3] == '0') ? dc_pump(0) : dc_pump(1);
      break;
    }
}

void calculatePID()
{
  P = error;
  I += error;
  D = (error - previous_error) ;
  PID_value = Kp*P + Ki*I + Kd*D;
  previous_error = error;
}

void motorPIDcontrol()
{
  // Calculating the effective motor speed:
  left_motor_speed  = 17 + PID_value;   // dau -
  right_motor_speed = 17 - PID_value;   // dau +
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
  if(PID_value != PID_value_pre)
  {
      delay(20);
  lcd.clear();
  //   lcd.setCursor(0,0);
  // lcd.print("sensor");
  // lcd.setCursor(6,0);
  // lcd.print(sensor[0]);
  // lcd.setCursor(7,0);
  // lcd.print(sensor[1]);
  // lcd.setCursor(8,0);
  // lcd.print(sensor[2]);
  // lcd.setCursor(9,0);
  // lcd.print(sensor[3]);
  // lcd.setCursor(10,0);
  // lcd.print(sensor[4]);
      lcd.setCursor(12,0);
  lcd.print(PID_value);
      lcd.setCursor(0,1);
  lcd.print("DCLR ");
    lcd.setCursor(7,1);
  lcd.print(left_motor_speed);
    lcd.setCursor(12,1);
  lcd.print(right_motor_speed);
  }

   Motor1_Forward(left_motor_speed);
   Motor2_Forward(right_motor_speed);
}

//////////// DC PUMP ////////////
void dc_pump(int state)
{
  if (state == 1)
  {
    //on
    digitalWrite(IN1_pump, HIGH);
    digitalWrite(IN2_pump, LOW);
  }
  else
  {
    //off
    digitalWrite(IN1_pump, LOW);
    digitalWrite(IN2_pump, LOW);
  }
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
*/