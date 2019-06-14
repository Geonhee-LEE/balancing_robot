#include "motorPID.h"

float rPID::wheelControl(float xRef, float x, float dt){
  float error = xRef - x;
  _integrator+=_ki*error*dt;
  _integrator = constrain(_integrator,-_imax,_imax);
  return 0.01*(_kp*error+_integrator); 
}

float rPID::speedControl(float vel_d, float pos, float dt, float& vel){
  float feedf=0.046*vel_d;//+14.0;
   vel =(pos - _last_pos)/dt; //0.5*_last_vel+0.5*(pos - _last_pos)/dt;
  _last_pos=pos; _last_vel=vel;
  float error =vel_d - vel;
  _integrator+=_ki*error*dt;
  _integrator = constrain(_integrator,-_imax,_imax);
  return _kp*error +_integrator+feedf; 
}

float rPID::balanceControl(float pitchRef,float pitch, float pitchVel, float dt){
  float err= pitchRef + pitch;
  _integrator+=_ki*err*dt;
  _integrator = constrain(_integrator,-_imax,_imax);
  return _kp*err +_kd*pitchVel+_integrator; 
}

float rPID::yawControl(float yawRef, float yaw, float dt){
  float err= yawRef - yaw;
  float yawRef_r;

  yaw_err = err;
  
  _integrator+=_ki*err*dt;                                                                    
  _integrator = constrain(_integrator,-_imax,_imax);
  return _kp*err +_integrator; 
}

/*
 * if(err > 180) 
 * {
 *    yawRef = -180+(yawRef-180);
 *    err = yawRef - yaw;
 * }
 * else if(err < -180)
 * {
 *    yawRef = 180+(yawRef+180);
 *    err = yawRef - yaw;
 * }
 * 
 */
 
