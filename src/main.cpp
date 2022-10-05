#include <Arduino.h>

#include "IMUAdapter.h"

//Carrega a biblioteca PID
#include <PID_v1.h>

enum Action {F, B, S};
enum PH_pin {EL=5, LF, LB, RB, RF, ER};
const int motor_pins[6] = {5, 6, 7, 8, 9, 10};


double Setpoint, Setpoint_aux, Setpoint2, Setpoint_back=181.0, Setpoint_forward=177.0, Input, Input2, Output=0, Output2;
double Kp=40, Ki=230, Kd=0.23;// Kp=25, Ki=220, Kd=0.3
double Kp2=15, Ki2=0, Kd2=0.2;
double potencia_frente=0, potencia_tras=0, incremento=0, potencia_esquerda=0, potencia_direita=0;

char Command = 'N';
unsigned long tempo = 0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID_yaw(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

IMUAdapter imu;

float rot[3];  // [phi, theta, psi] mesuarement

void motorConversion(float pwm1, float pwm2, int state){
  
}

void Forward(){
    potencia_frente = Output*-1;
    potencia_esquerda = potencia_frente;
    potencia_direita = potencia_frente;
    if (Output2 < 0){
      incremento = Output2*-1;
      potencia_direita = potencia_direita + incremento;
    } else if (Output2 > 0){
      incremento = Output2;
      potencia_esquerda = potencia_esquerda + incremento;
    }
    
    if (potencia_esquerda > 255)
      potencia_esquerda = 255;
    if (potencia_direita > 255)
      potencia_direita = 255;
      
    analogWrite(PH_pin::EL, potencia_esquerda);
    analogWrite(PH_pin::ER, potencia_direita);
    digitalWrite(PH_pin::LF, 0);
    digitalWrite(PH_pin::LB, 1);
    digitalWrite(PH_pin::RF, 0);
    digitalWrite(PH_pin::RB, 1);

}

void Reverse(){ 
    potencia_tras = Output;
    potencia_esquerda = potencia_tras;
    potencia_direita = potencia_tras;
    if (Output2 < 0){
      incremento = Output2*-1;
      potencia_esquerda = potencia_esquerda + incremento;
    } else if (Output2 > 0){
      incremento = Output2;
      potencia_direita = potencia_direita + incremento;
    }
    
    if (potencia_esquerda > 255)
      potencia_esquerda = 255;
    if (potencia_direita > 255)
      potencia_direita = 255;
    
    analogWrite(PH_pin::EL, potencia_esquerda);
    analogWrite(PH_pin::ER, potencia_direita);
    digitalWrite(PH_pin::LF, 1);
    digitalWrite(PH_pin::LB, 0);
    digitalWrite(PH_pin::RF, 1);
    digitalWrite(PH_pin::RB, 0);
}

void Stop(){
  analogWrite(PH_pin::EL, 0);
  analogWrite(PH_pin::ER, 0);
  digitalWrite(PH_pin::LF, 0);
  digitalWrite(PH_pin::LB, 0);
  digitalWrite(PH_pin::RF, 0);
  digitalWrite(PH_pin::RB, 0);
}

void get_info(){
  /*Serial.print(" | Input = "); Serial.print(Input);
  Serial.print(" | PID = "); Serial.print(Output);
  Serial.print(" | Input2 = "); Serial.print(Input2);
  Serial.print(" | PID2 = "); Serial.print(Output2);
  //Serial.print(" | Setpoint2 = "); Serial.print(Setpoint2);
  Serial.print(" | PE: ");Serial.print(potencia_esquerda);
  Serial.print(" | PD: ");Serial.println(potencia_direita);
  //delay(500);*/
  Serial.print("Setpoint:");Serial.print(Setpoint); Serial.print("  ");
  Serial.print("Referencia:");Serial.print(Input); Serial.print("  ");
  Serial.print("Setpoint2:");Serial.print(Setpoint2); Serial.print("  ");
  Serial.print("Referencia2:");Serial.println(Input2);
}

void go_forward(){ Setpoint  = Setpoint_forward;}
void go_back()   { Setpoint  = Setpoint_back;}
void stablish()  { Setpoint  = Setpoint_aux; Command = 'N';}
void turn_left() { Setpoint2 = Setpoint2 + 0.5;}
void turn_right(){ Setpoint2 = Setpoint2 - 0.5;}

void setup() {

  for (int i = 5; i <= 10; i++)
    pinMode(motor_pins[i], OUTPUT);
  
  Serial.begin(115200);
  imu.begin();
   
  Input = 0;
  Setpoint = 180;
  Setpoint_aux = Setpoint;
  Input2 = 0;
  Setpoint2 = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  myPID.SetOutputLimits(-255,255);

  myPID_yaw.SetMode(AUTOMATIC);
  myPID_yaw.SetSampleTime(20);
  myPID_yaw.SetOutputLimits(-255,255);

  //calcula_setpoint();
}

void loop() {

  imu.update();
  imu.getRPY(rot);
  Input = rot[0];
  Input2 = rot[1];
  myPID.Compute();
  myPID_yaw.Compute();

// get_info();
// Controle de direção -----------------------------------------
  if(Serial.available() > 0)
      Command = Serial.read();

  switch(Command){
    case 'F':
      go_forward();
    break;
    case 'T':
      go_back();
    break;
    case 'E':
      turn_left();
    break;
    case 'D':
      turn_right();
    break;
    case 'A':
      Setpoint_aux = Setpoint_aux - 0.1;
    break;
    case 'B':
      Setpoint_aux = Setpoint_aux + 0.1;
    break;
    default:
      stablish();
    break;
  }
// -------------------------------------------------------------

  if (Input > 165 && Input < 195){   
     if (Output < 0)
      Forward();    
     else if (Output > 0)
      Reverse();  
  } else {
    Stop();
  }
  
 
}

