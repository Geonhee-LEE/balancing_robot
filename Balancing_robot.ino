#include <Streaming.h>  // only for simple print
#include "TB6612.h"
#include "motorPID.h"
#include "QEncoder.h"
#include "MPU9250_SPI.h"
#include "eeprom_utils.h"
#include "parameters.h"

#define BALANCING  0
#define MOVE  1
#define ROTATION  2

int reset_count = 0;
//#define CALIBRATION_MODE // For the first time

MPU9250_SPI mpu(SPI,MPU_CS,INT_PIN);
EEPROM_BACKUP eeprom;
TB6612 motorA(APWM, AIN1, AIN2);  //create a motor instance
TB6612 motorB(BPWM, BIN1, BIN2);  //create a motor instance

rPID  pidA(0.045, 0.10, 0., 255);     // kp, ki, kd,imax
rPID  pidB(0.045, 0.10, 0,  255);     // kp, ki, kd,imax
rPID  static_bala(130, 2040, 7.5, 4800);     // Only Balancing: bala(130, 2035,7,4800);
rPID  dyn_bala(130, 2040, 0, 4800);     // Only Balancing: bala(130, 2035,7,4800);
rPID  movePID(0.02, 0.005, 0);        // forward movePID Default: 0.025,0.001, 4
rPID  yawPID(5, 0, 0, 4800);

uint8_t mode = 0;
float pre_yaw = 0, cur_deg = 0, yaw_diff = 0;

Encoders encoderA(ENA_A,ENA_B), encoderB(ENB_A,ENB_B);      // Create an Encoder instance (2,3) (1,0)
uint32_t prevTime=0,intVal=2000, pT=0;
int32_t ref=0, desiredPosA=0,desiredPosB=0,cnt=0;
int32_t desiredVelA=0,desiredVelB=0;
String inStr = "";

float velA=0, velB=0,yaw=0, dt=DT;
int Desired_vel_A=0,Desired_vel_B=0;
bool second_num=false;

void setup() {
 Serial.begin(115200);
 //while(!Serial) {}
 inStr.reserve(20);
 pinMode(LED_PIN,OUTPUT);
 motorA.stop(); motorB.stop();
 encoderA.setEncoderCount(0);
 encoderB.setEncoderCount(0);
  mpu.setup();
  mpu.setMagneticDeclination(8.5);
  mpu.setSampleRate(SR_100HZ);
  mpu.setGyroRange(GYRO_RANGE_2000DPS);
  mpu.setAccelRange(ACCEL_RANGE_16G);
  mpu.setDlpfBandwidth( DLPF_BANDWIDTH_184HZ); 
  mpu.enableDataReadyInterrupt();  
#ifdef CALIBRATION_MODE 
  calibrationProcess();
#endif    
  eeprom.loadCalibration();  // calibration data
 prevTime=micros(); pT=micros();
}

void loop() {
 if (mpu.isDataReady()){
    getDt(); 
  #ifndef RAW_DATA
    mpu.update(COMPLEMENTARY); //  MAGDWICK  /COMPLEMENTARY
  #else
     Vect3  a, g, m;  // acc/gyro/mag vectors
     mpu.update(a,g,m);
  #endif   
    Vect3 gyro=mpu.getGyroVect();
    float pitchDeg=mpu.getPitch()*RAD_TO_DEG;
    int32_t curPosA=encoderA.getEncoderCount();
    int32_t curPosB=encoderB.getEncoderCount();

    float desired_deg = 0;
    float yawDeg = mpu.getYaw()*RAD_TO_DEG;
    
    // Degree update
    if( yawDeg >= 0 && pre_yaw < - 150)
    {
      yaw_diff = -yawDeg + pre_yaw + 360;
    }    
    else if(yawDeg <= 0 && pre_yaw > 150 )
    {
      yaw_diff = -yawDeg + pre_yaw - 360;
    }    
    else
      yaw_diff = yawDeg - pre_yaw;
    cur_deg += yaw_diff;
    pre_yaw = yawDeg;
    
    float yaw_Command = yawPID.yawControl(desired_deg, cur_deg, dt);
    yaw_Command=constrain(yaw_Command, -50, 50);

    int32_t desired_x = 0;
    float curPos=(curPosA+curPosB)/2;
    
    float pitchRef=movePID.wheelControl(desired_x, curPos, dt);
    pitchRef=constrain(pitchRef, -0.5, 0.5);
    
    float mCommand= static_bala.balanceControl(-pitchRef, pitchDeg, gyro.x, dt);
    if(pitchRef > 1 || pitchRef < -1)
      mCommand= dyn_bala.balanceControl(-pitchRef, pitchDeg, gyro.x, dt);
    mCommand = constrain(mCommand, -5000, 5000);
    desiredVelA = mCommand;
    desiredVelB = mCommand;
    
    int32_t uA= pidA.speedControl(desiredVelA,curPosA, dt, velA);
    int32_t uB= pidB.speedControl(desiredVelB,curPosB, dt, velB);
        

    if ((pitchDeg < ANGLE_LIMIT)&&(pitchDeg > -ANGLE_LIMIT))
    {  // fail

       if( pitchDeg < 1 && pitchDeg > -1)
       {     
         motorA.run(-yaw_Command); 
         motorB.run(yaw_Command);   
       }
       else
       {  
         motorA.run(uA); 
         motorB.run(uB);        
       }   
       reset_count++; 
          
       if(reset_count < 100)
       {
         //static_bala.setIntegrator(0);
         reset_count = 0;
        }
     }
     else{ motorA.run(0);motorB.run(0); static_bala.setIntegrator(0); movePID.setIntegrator(0); reset_count = 0; }
   
    uint32_t curTime=micros();

    if (curTime-prevTime>500000){
      mode++;

      if(mode >= 100)
      {
        mode = 0;
        Serial << "desired_deg: " << desired_deg <<  ", yawDeg: " << yawDeg << ", cur_deg: " << cur_deg << ", yaw_Command: " << yaw_Command <<endl;
        Serial << "curPosA: " << curPosA <<  " curPosB: " << curPosB <<endl;
        Serial << "desired_x: " << desired_x <<  " curPos: " << curPos <<endl;
        Serial <<" pitchRef: "<< pitchRef << " pitchDeg: " << pitchDeg <<endl;
      }
    }

    /*
    if (curTime-prevTime>20000){
         prevTime=curTime;
        Serial <<static_bala.getP()<<","<<static_bala.getI() <<","<<static_bala.getD()<<endl;
        Serial <<mpu.getPitch()*RAD_TO_DEG<<","<<gyro.x <<","<<mCommand/10.<<endl;
       Serial << "YPR "<<mpu.getYaw()*RAD_TO_DEG<<" "<<mpu.getPitch()*RAD_TO_DEG<<" "<<mpu.getRoll()*RAD_TO_DEG<<endl;
      Serial.print(desiredVelA); Serial.print(",");Serial.print(velA);Serial.print(",");Serial.println(5*uA);   
   }
   Serial.print(mpu.getYaw()*RAD_TO_DEG);
   Serial.print(',');
   Serial.print(mpu.getPitch()*RAD_TO_DEG);
   Serial.print(',');
   Serial.print(pitchRef);
   Serial.print(',');
   Serial.print(curPosA);
   Serial.print(',');
   Serial.print(curPosB);
   Serial.print(',');
   Serial.print(yawPID.yaw_err);
   Serial.print(',');
   Serial.println(yaw_data);
    */
   
 }
 // serialEvents();
}
void makeDt(float dt){
  uint32_t cT=micros();
  while (uint32_t(cT-pT) < dt*1000000.){ cT=micros();}
  pT=cT;
}
void getDt(){
  uint32_t cT=micros();
  dt=uint32_t(cT - pT)/1000000.0;
  pT=cT;
}
void serialEvents(){
  while (Serial.available()){
    char c= (char)Serial.read();
    if (c=='\n'){
       static_bala.setD(inStr.toFloat());
       inStr="";
    }
    else if (c==','){
      if (second_num){ static_bala.setI(inStr.toFloat());second_num=false;}
      else { static_bala.setP(inStr.toFloat());second_num=true;}
      inStr="";
    }
    else
     inStr+=c;   
  }
}

int compensateDZ(int input,int epsilon,int minusStart, int plusStart){
    if (abs(input)<epsilon)
      return 0;
   else
      return (input>=0)? input+epsilon: input-epsilon; 
}
float map(float value, float istart, float istop, float ostart, float ostop){
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}
void calibrationProcess(){
  mpu.calibrateGyro();   
  mpu.calibrateMag();
  eeprom.saveCalibration();
  Serial.println("Calibration Completed !!!!!!!!");  
    while(1);
}
