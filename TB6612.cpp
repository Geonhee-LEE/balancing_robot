#include "TB6612.h"
TB6612::TB6612(uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2){
  _pinEnable = pinEnable;
  _pinIN1 = pinIN1;
  _pinIN2 = pinIN2;
  _pwmVal = 100;
  pinMode(_pinEnable, OUTPUT);
  pinMode(_pinIN1, OUTPUT);  pinMode(_pinIN2, OUTPUT);
  digitalWrite(_pinIN1, LOW); digitalWrite(_pinIN2, LOW);
}
void TB6612::setSpeed(unsigned short pwmVal){
  _pwmVal = pwmVal;
}
unsigned short TB6612::getSpeed(){
  return _pwmVal;
}
void TB6612::forward(){
  digitalWrite(_pinIN1, HIGH);
  digitalWrite(_pinIN2, LOW);
  analogWrite(_pinEnable, _pwmVal);
}
void TB6612::backward(){
  digitalWrite(_pinIN1, LOW);
  digitalWrite(_pinIN2, HIGH);
  analogWrite(_pinEnable, _pwmVal);
}
void TB6612::run(int velocity){
  int16_t vel=constrain(velocity,-255,255);    
  if (vel>=0){
      setSpeed(vel);
      forward();
  }
  else{    
      setSpeed(-vel);
      backward();
  }
}
void TB6612::stop(){
  digitalWrite(_pinIN1, LOW);
  digitalWrite(_pinIN2, LOW);
  analogWrite(_pinEnable, 255);
}

