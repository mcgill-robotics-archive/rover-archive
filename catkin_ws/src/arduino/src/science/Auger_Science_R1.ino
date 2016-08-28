#include "HX711.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

HX711 auger_loadcell(22, 23);
HX711 top_loadcell(24, 25);
byte humidity_sensor_pin = A1;
const int wind_sensor_pin = A2;
int InA1 = 26, InB1 = 27, PWM1 = 2, InA2 = 36, InB2 = 37, PWM2 = 3;

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
Servo auger_servo, rock_servo, soil_servo;
String readString = "";
int limit_switch = 0, wind_sensor_value = 0;
float wind_sensor_voltage = 0, wind_speed = 0, voltage_constant = .004882814,
voltage_min = .4, voltage_max = 2.0, wind_speed_max = 32;

void setup() {
  Serial.begin(38400);


  // initial closed state
  auger_servo.write(0);
  rock_servo.write(0);
  soil_servo.write(0);

  // Setup pins
  auger_servo.attach(7);
  rock_servo.attach(9);
  soil_servo.attach(11);

  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);

  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // Limit switches
  pinMode(8, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);

  // We need to add this functionality
  auger_loadcell.set_scale(2280.f);
  auger_loadcell.tare();
  top_loadcell.set_scale(2280.f);
  top_loadcell.tare(); // We want the driver to be able to do this when he wants


  auger_loadcell.read();
  top_loadcell.read();

  Serial.println("Ready");  // Replace with ROS log
}

void loop() {
  // Dealt with
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    return;
  }

  // The limitations verified by limit switches - Dealt with
  int topLimit = digitalRead(8), bottomLimit = digitalRead(12);
  if (limit_switch == 2 && bottomLimit == LOW){
    analogWrite(PWM2, 0);
    limit_switch = 0;
    Serial.println("bottom limit switch");
  }
  if (limit_switch == 1 && topLimit == LOW){
    analogWrite(PWM2, 0);
    limit_switch = 0;
    Serial.println("top limit switch");
  }


  if (Serial.available()) {
    char c = Serial.read();
    if (c == ',') {
      if (readString.length() >0) {
        int n = readString.toInt();
        //************************************************************************************Zero loadcells
        if(readString.indexOf('z') >0) {
          auger_loadcell.tare();
          top_loadcell.tare();
          Serial.println("Zero");}
        //*****************************************************************************Rock and soil loadcell
        if(readString.indexOf('t') >0) {
          Serial.print("Rock and soil loadcell:\t");
          float load_t = ((abs(top_loadcell.get_units(10)))*5.0552);
          Serial.println(load_t, 2);}
        //*************************************************************************************Auger loadcell
        if(readString.indexOf('b') >0) {
          Serial.print("Auger loadcell:\t");
          Serial.println(auger_loadcell.get_units(10), 2);}
        //*******************************************************************************************Humidity
        if(readString.indexOf('h') >0) {
          int humidity_value = analogRead(humidity_sensor_pin);
          Serial.print("Humidity Level (0-1023): ");
          Serial.println(1023-humidity_value);}
        //*****************************************************************************************Wind speed
        if(readString.indexOf('w') >0) {
          wind_sensor_value = analogRead(wind_sensor_pin);
          wind_sensor_voltage = wind_sensor_value * voltage_constant;
          if (wind_sensor_voltage <= voltage_min){
            wind_speed = 0;
          }
          else {
            wind_speed = (wind_sensor_voltage - voltage_min)*wind_speed_max/(voltage_max - voltage_min);
          }
          Serial.print("Wind speed: ");
          Serial.print(wind_speed);
          Serial.println("m/s");}
        //******************************************************************************************Pressure
        if(readString.indexOf('p') >0) {
          float pascals = ((baro.getPressure()/1000) + 100);
          Serial.print(pascals);
          Serial.println(" kPa");
          float altm = baro.getAltitude();
          Serial.print(altm);
          Serial.println(" m");
          float tempC = baro.getTemperature();
          Serial.print(tempC);
          Serial.println(" C");}
        //**************************************************************************************************
        if(readString.indexOf('a') >0) {
          if (n == 1) {
            Serial.println("sealing auger soil sample box");
            auger_servo.write(75);
          }
          else if(n == 0) {
            auger_servo.write(175);
            Serial.println("opening auger soil sample box");
          }
          else {
            Serial.print("writing angle for auger soil sample box: ");
            Serial.println(n);
            auger_servo.write(n);
          }
        }
        //**************************************************************************************************
        if(readString.indexOf('r') >0) {
          if (n == 1) {
            Serial.println("sealing rock sample box");
            rock_servo.write(45);
          }
          else if(n == 0) {
            rock_servo.write(145);
            Serial.println("opening rock sample box");
          }
          else {
            Serial.print("writing angle for rock sample box: ");
            Serial.println(n);
            rock_servo.write(n);
          }
        }
        //**************************************************************************************************
        if(readString.indexOf('s') >0) {
          if (n == 1) {
            Serial.println("sealing soil sample box");
            soil_servo.write(45);
          }
          else if(n == 0) {
            soil_servo.write(145);
            Serial.println("opening soil sample box");
          }
          else {
            Serial.print("writing angle for soil sample box: ");
            Serial.println(n);
            soil_servo.write(n);
          }
        }
        //**************************************************************************************************
        if(readString.indexOf('k') >0) {
          analogWrite(PWM1, 0);
          analogWrite(PWM2, 0);
          limit_switch = 0;
          Serial.println("kill auger and carage");}
        //**************************************************************************************************
        if(readString.indexOf('d') >0) {
          if(n < 0) {
            digitalWrite(InB1, HIGH);
            digitalWrite(InA1, LOW);
            Serial.print("drilling down soil: ");
            Serial.println(n);
            analogWrite(PWM1, abs(n));
          }
          else if (n > 0) {
            digitalWrite(InB1, LOW);
            digitalWrite(InA1, HIGH);
            Serial.print("drilling up soil: ");
            Serial.println(n);
            analogWrite(PWM1, abs(n));
          }
          else{
            analogWrite(PWM1, 0);
            Serial.println("drill stop");
          }
        }
        //**************************************************************************************************
        if(readString.indexOf('c') >0) {
          if(n < 0 && topLimit == HIGH) {
            digitalWrite(InB2, HIGH);
            digitalWrite(InA2, LOW);
            analogWrite(PWM2, abs(n));
            limit_switch = 1;
            Serial.print("carage up: ");
            Serial.println(n);
          }
          else if(n < 0 && topLimit == LOW) {
            analogWrite(PWM2, 0);
            limit_switch = 0;
            Serial.println("top limit switch");
          }
          else if (n > 0 && bottomLimit == HIGH) {
            digitalWrite(InB2, LOW);
            digitalWrite(InA2, HIGH);
            analogWrite(PWM2, abs(n));
            limit_switch = 2;
            Serial.print("carage down: ");
            Serial.println(n);
          }
          else if(n > 0 && bottomLimit == LOW) {
            analogWrite(PWM2, 0);
            limit_switch = 0;
            Serial.println("bottom limit switch");
          }
          else{
            analogWrite(PWM2, 0);
            limit_switch = 0;
            Serial.println("carage stop");
          }
        }
      }
      readString="";
    }
    else {
      readString += c;
    }
  }
}
