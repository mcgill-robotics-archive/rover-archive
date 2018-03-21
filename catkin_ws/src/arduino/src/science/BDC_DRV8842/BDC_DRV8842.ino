#include "BDC_DRV8842.h"

int InB_1 = 6;
int InB_2 = 7;
int nRST = 29;
int nFLT = 31;
int PWM_val = 0;

BDC * controller;

void setup() {
  Serial.begin(9600);
  controller = new BDC(9, 6, 13, 11);
  delay(300);
  Serial.println("Ready");
}
void loop() {
  if (controller->FLT()) {
    Serial.println("Fault");
    controller->RST();
  } else {
    while (Serial.available() > 1) {
      PWM_val = Serial.parseInt();
      controller->PWM(PWM_val);
      Serial.println(PWM_val);
    }
  }


  delay(2);
}

