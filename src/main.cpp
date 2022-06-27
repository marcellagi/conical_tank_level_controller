#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int pot = A0;         //potenciometro
const int echoPin = A1;     //sensor ultrassonico
const int triggerPin = A2;  //sensor ultrassonico

//portas ponte h
const int pwm = 5;         
const int inB = 6;
const int inA = 7;
const int en  = 8;
const int gnd = 9;
const int vcc = 10;

const float cone_height = 70.0;   //altura fixa do cone

float distance_cm;     //distancia medida pelo sensor
float height;         // altura da agua
float setpoint;       // altura para a agua se manter

float error = 0.0, sum_error = 0.0;    //erro e somatorio do erro usado nas constantes de controle
float kp = 1.7, ki = 0.1;              //constantes de controle

float dt = 0.1;             //delay

float PI_value;            //controle PI
uint8_t pwm_value;         //valor da porta pwm

LiquidCrystal_I2C lcd(0x27,16,2);   //lcd com modulo i2c

float readUltrasonicDistance()     //calculo da distancia medida pelo sensor
{
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  distance_cm = 0.01723 * pulseIn(echoPin, HIGH);

  if (distance_cm < 0) distance_cm = 0.0;        //nao pode diminuir de zero
  else if (distance_cm > 70.0) distance_cm = 70.0;   //nao pode passar da altura maxima
  
  return distance_cm;
}

float controlMotor(float PI_value)          //controle da bomba
{
  pwm_value = PI_value;

  if(PI_value < 50) pwm_value = 0;
  else if(PI_value > 255) pwm_value = 255;

  analogWrite(pwm, pwm_value);
}


void setup()          //set das variaveis
{
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(pot, INPUT);

  pinMode(pwm, OUTPUT);
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(en,  OUTPUT);
  pinMode(gnd, OUTPUT);
  pinMode(vcc, OUTPUT);
  
  digitalWrite(inA, LOW);
  digitalWrite(inB, HIGH);
  digitalWrite(en,  HIGH);
  digitalWrite(gnd, LOW);
  digitalWrite(vcc, HIGH);
  
  lcd.init();

  Serial.begin(9600);
}

void loop()
{
  setpoint = 5.0 + 25.0;//40.0*analogRead(pot)/1023.0;   
  // 
  distance_cm = readUltrasonicDistance();    //leitura do sensor
  height = cone_height - distance_cm;       //calculo da altura da agua
  // 
  error = setpoint - height;            //calculo do erro
  sum_error += error;                   //calculo do somatorio do erro
  // 
  PI_value = kp*error + ki*sum_error*dt;       //calculo do controle PI
// 
  controlMotor(PI_value);                   //o controle PI controla a bomba
// 
  lcd.setBacklight(HIGH);          //lcd
// 
  lcd.setCursor(0,0);
  lcd.print("ERROR - ");
  lcd.print(error, 2);
// 
  lcd.setCursor(0,1);
  lcd.print("PI   - ");
  lcd.print(PI_value, 2);
// 
  Serial.print("Setpoint:" );         //serial print
  Serial.print(setpoint);
  Serial.println(" cm");
  // 
  Serial.print("Altura:" );
  Serial.print(height);
  Serial.println("cm");
  // 
// 
  Serial.print("Erro:" );
  Serial.println(error);
  // 
  Serial.print("Sum Erro:" );
  Serial.println(sum_error);
  // 
  Serial.print("PI:" );
  Serial.println(PI_value);
  // 
  Serial.println();
  
  delay(1000*dt);         //delay
}