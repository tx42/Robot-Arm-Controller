#include <Arduino.h>
#include "motor.h"
 
void Motor::init(){
  pinMode(m_power_pin, OUTPUT);
  pinMode(m_forward_pin, OUTPUT);
  pinMode(m_reverse_pin, OUTPUT);
}

void Motor::setPower(int power){
  if (power >= 0){
    setRawPower(power);
    setDirection(FORWARD);
  }else{
    setRawPower(-power);
    setDirection(REVERSE);
  }
}

void Motor::setDirection(MotorDirection direction){
  switch(direction){
    case FORWARD:
      digitalWrite(m_forward_pin, HIGH);
      digitalWrite(m_reverse_pin, LOW);
      break;
    case REVERSE:
      digitalWrite(m_forward_pin, LOW);
      digitalWrite(m_reverse_pin, HIGH);
      break;
    case STOP:
      digitalWrite(m_forward_pin, LOW);
      digitalWrite(m_reverse_pin, LOW);
  }
}

void Motor::setRawPower(unsigned int power){
  analogWrite(m_power_pin, min(power, 255));
}
