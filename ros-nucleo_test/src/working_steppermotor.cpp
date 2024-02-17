#include <Arduino.h>

#define step_pin D12
#define dir_pin D11
#define sleep_pin D6

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(sleep_pin, OUTPUT);

  digitalWrite(dir_pin, LOW);
  digitalWrite(sleep_pin, HIGH);
}

void go_forward_x_rotation(double x, bool forward_back) {
  digitalWrite(sleep_pin, HIGH);
  digitalWrite(dir_pin, forward_back);
  // put your main code here, to run repeatedly:
  int i = 0;

  while (true) {
    digitalWrite(step_pin, HIGH);
    delay(2);
    digitalWrite(step_pin, LOW);
    delay(2);  

    if(i++ > x*200)
    {
      // digitalWrite(dir_pin, HIGH);
      
      // digitalWrite(dir_pin, !digitalRead(dir_pin));
      // digitalWrite(sleep_pin, !digitalRead(sleep_pin));
      digitalWrite(sleep_pin, LOW);
      i = 0;
      break;
    }
  }
}


void loop() {
  go_forward_x_rotation(1, false);
  delay(5000);
}