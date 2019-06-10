#ifndef TB6612_h
#define TB6612_h
#include "Arduino.h"
class TB6612{
   public:
      TB6612(uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2);
      void setSpeed(unsigned short pwmVal);
      unsigned short getSpeed();
      void forward();
      void backward();
      void run(int velocity);
      void stop();
   private:
      byte _pinEnable;
      byte _pinIN1;
      byte _pinIN2;
      byte _pwmVal;
};
#endif
