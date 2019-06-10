#ifndef MOTORPID_h
#define MOTORPID_h
#include <Arduino.h>
class rPID {
public:
  rPID(float kp=0., float ki=0., float kd=0., int16_t imax=0) :
    _kp(kp), _ki(ki), _kd(kd), _imax(abs(imax)) {  
      _integrator=0; _last_pos=0;_last_vel=0;}
   // Overload the function call operator 
  void operator() (float kp, float ki, float kd, int16_t imax) {
    _kp = kp; _ki = ki; _kd = kd; _imax = abs(imax);}  
  int16_t imax() const      { return _imax; }
  void  imax(const int16_t v) { _imax=abs(v); }
  float get_integrator() const  { return _integrator; }
  float get_vel() const  { return _vel; }
  void setIntegrator(float i) { _integrator = i; }
  void setP(float kp){ _kp=kp;}
  void setI(float ki){ _ki=ki;}
  void setD(float kd){ _kd=kd;}
  float getP(){ return _kp;}
  float getI(){ return _ki;}
  float getD(){ return _kd;}  
  float wheelControl(float xRef, float x, float dt);
  float speedControl(float vel_d, float pos, float dt, float& vel);
  float balanceControl(float pitchRef, float pitch, float pitchVel, float dt);
  float yawControl(float yawRef, float yaw, float yawVel, float dt);

  float yaw_err = 0;
private:
  float     _kp, _ki, _kd;
  int16_t   _imax;
  float     _integrator;    // integrator value
  int32_t   _last_pos;    // last input for derivative
  int32_t   _vel;
  float _last_vel;
};

#endif
