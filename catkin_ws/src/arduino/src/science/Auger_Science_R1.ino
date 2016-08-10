#include "HX711.h"
#include <Servo.h> 
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

HX711 auger_loadcell(22, 23);
HX711 top_loadcell(24, 25);
byte humidity_sensor_pin = A1;


Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
Servo auger_servo, rock_servo, soil_servo;
String readString;

const int sensorPin = A2; //Defines the pin that the anemometer output is connected to
int sensorValue = 0; //Variable stores the value direct from the analog pin
float sensorVoltage = 0; //Variable that stores the voltage (in Volts) from the anemometer being sent to the analog pin
float windSpeed = 0; // Wind speed in meters per second (m/s)
 
float voltageConversionConstant = .004882814; //This constant maps the value provided from the analog read function, which ranges from 0 to 1023, to actual voltage, which ranges from 0V to 5V
int sensorDelay = 1000; //Delay between sensor readings, measured in milliseconds (ms)
 
//Anemometer Technical Variables
//The following variables correspond to the anemometer sold by Adafruit, but could be modified to fit other anemometers.
 
float voltageMin = .4; // Mininum output voltage from anemometer in mV.
float windSpeedMin = 0; // Wind speed in meters/sec corresponding to minimum voltage
 
float voltageMax = 2.0; // Maximum output voltage from anemometer in mV.
float windSpeedMax = 32; // Wind speed in meters/sec corresponding to maximum voltage

int InA1 = 26, InB1 = 27, PWM1 = 2, InA2 = 36, InB2 = 37, PWM2 = 3, flag = 0;

void setup() {
  Serial.begin(38400);
  
  auger_servo.write(0);
  rock_servo.write(0);
  soil_servo.write(0);
  
  auger_servo.attach(7);
  rock_servo.attach(9);
  soil_servo.attach(11);
  
  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  
  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  
  pinMode(8, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  
  auger_loadcell.set_scale(2280.f);                   
  auger_loadcell.tare();				       
  top_loadcell.set_scale(2280.f);                     
  top_loadcell.tare();
  				 
  auger_loadcell.read();
  top_loadcell.read();
  
  Serial.println("Ready");
}

int read_humidity_sensor() {
  delay(500);
  int value = analogRead(humidity_sensor_pin);
  return 1023 - value;
}
void loop() { 
  int topLimit = digitalRead(8), mildeLimit = digitalRead(10), bottomLimit = digitalRead(12);
  if (flag == 2 && bottomLimit == LOW){
    analogWrite(PWM2, 0);
    flag = 0;
    Serial.println("bottom limit switch");
  }
  if (flag == 1 && topLimit == LOW){
    analogWrite(PWM2, 0);
    flag = 0;
    Serial.println("top limit switch");
  }
  if (Serial.available()) {
    char c = Serial.read();
    if (c == ',') {
      if (readString.length() >1) {
        if(readString.indexOf('z') >0) {
          auger_loadcell.tare();				                           
          top_loadcell.tare();
          Serial.println("Zero");
        }
        if(readString.indexOf('t') >0) {
          Serial.print("Rock and soil loadcell:\t");
          Serial.println(top_loadcell.get_units(10), 2);
        }
        if(readString.indexOf('b') >0) {
          Serial.print("Auger loadcell:\t");
          Serial.print(auger_loadcell.get_units(10), 2);
        }
        if(readString.indexOf('h') >0) {
          Serial.print("Humidity Level (0-1023): ");
          Serial.println(read_humidity_sensor()); 
          delay(100);
        }
        if(readString.indexOf('W') >0) {
          sensorValue = analogRead(sensorPin); //Get a value between 0 and 1023 from the analog pin connected to the anemometer
          sensorVoltage = sensorValue * voltageConversionConstant; //Convert sensor value to actual voltage
           
          //Convert voltage value to wind speed using range of max and min voltages and wind speed for the anemometer
          if (sensorVoltage <= voltageMin){
           windSpeed = 0; //Check if voltage is below minimum value. If so, set wind speed to zero.
          }
          else {
            windSpeed = (sensorVoltage - voltageMin)*windSpeedMax/(voltageMax - voltageMin); //For voltages above minimum value, use the linear relationship to calculate wind speed.
          }
            Serial.print("Voltage: ");
            Serial.print(sensorVoltage);
            Serial.print("\t"); 
            Serial.print("Wind speed: ");
            Serial.println(windSpeed); 
           delay(sensorDelay);
        }
        if(readString.indexOf('p') >0) {
          float pascals = baro.getPressure();
          Serial.print(pascals/3377); Serial.println(" Inches (Hg)");
        
          float altm = baro.getAltitude();
          Serial.print(altm); Serial.println(" meters");
        
          float tempC = baro.getTemperature();
          Serial.print(tempC); 
          Serial.println("*C");
        
          delay(250);
        }
        int n = readString.toInt(); 
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
        if(readString.indexOf('k') >0) {
          analogWrite(PWM1, 0);
          analogWrite(PWM2, 0);
          flag = 0;
          Serial.println("kill auger and carage");
        }
        if(readString.indexOf('d') >0) {
          if(n < 0) {
            digitalWrite(InB1, HIGH);
            digitalWrite(InA1, LOW);
            Serial.print("drilling down soil: ");
            Serial.println(n);
          }
          else if (n > 0) {
            digitalWrite(InB1, LOW);
            digitalWrite(InA1, HIGH);
            Serial.print("drilling up soil: ");
            Serial.println(n);
          }
          else{
            analogWrite(PWM1, 0);
            Serial.print("drill stop");
          }
          analogWrite(PWM1, abs(n));
        }
        if(readString.indexOf('c') >0) {
          if(n < 0 && topLimit == HIGH) {
            digitalWrite(InB2, HIGH);
            digitalWrite(InA2, LOW);
            analogWrite(PWM2, abs(n));
            flag = 1;
            Serial.print("carage up: ");
            Serial.println(n);
          }
          else if(n < 0 && topLimit == LOW) {
            analogWrite(PWM2, 0);
            flag = 0;
            Serial.println("top limit switch");
          }
          else if (n > 0 && bottomLimit == HIGH) {
            digitalWrite(InB2, LOW);
            digitalWrite(InA2, HIGH);
            analogWrite(PWM2, abs(n));
            flag = 2;
            Serial.print("carage down: ");
            Serial.println(n);
          }
          else if(n > 0 && bottomLimit == LOW) {
            analogWrite(PWM2, 0);
            flag = 0;
            Serial.println("bottom limit switch");
          }
          else{
            analogWrite(PWM2, 0);
            flag = 0;
            Serial.print("carage stop: ");
            Serial.println(n);
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
