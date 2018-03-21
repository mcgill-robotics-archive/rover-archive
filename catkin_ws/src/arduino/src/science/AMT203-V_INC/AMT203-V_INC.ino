#include "AMT203-V_INC.h"

AMT_INC * encoder;
int us;

void setup() {
  Serial.begin(9600);
  encoder = new AMT_INC(2, 3);
}

void loop() {
  while(Serial.available() > 0){
    us = Serial.parseInt();
    if(us == 1){
      Serial.print("angle: ");
      Serial.println(encoder->delta());
      us = 0;
    }
  }
}
