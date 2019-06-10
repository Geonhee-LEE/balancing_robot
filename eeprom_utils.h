#include "EEPROMex.h"
// MPU9250 with SPI interface library Ver. 0.98
// Made by HeeJae Park 
// 2019.05.27
extern MPU9250_SPI mpu;
const uint8_t EEPROM_SIZE = 1 + sizeof(float) * 3 * 4;
class EEPROM_BACKUP{
  enum EEP_ADDR{
      EEP_CALIB_FLAG = 0x00,
      EEP_ACC_BIAS = 0x01,
      EEP_GYRO_BIAS = 0x0D,
      EEP_MAG_BIAS = 0x19,
      EEP_MAG_SCALE = 0x25
  };
  public:
  void saveCalibration(bool b_save = true);
  void loadCalibration();
  void clearCalibration();
  bool isCalibrated();
  void printBytes();
  void printCalibration();
  void setupEEPROM();
}; 
void EEPROM_BACKUP::saveCalibration(bool b_save = true) {
    EEPROM.updateByte(EEP_CALIB_FLAG, 1);
    EEPROM.updateFloat(EEP_ACC_BIAS + 0, mpu.getAccBias().x);
    EEPROM.updateFloat(EEP_ACC_BIAS + 4, mpu.getAccBias().y);
    EEPROM.updateFloat(EEP_ACC_BIAS + 8, mpu.getAccBias().z);
    EEPROM.updateFloat(EEP_GYRO_BIAS + 0, mpu.getGyroBias().x);
    EEPROM.updateFloat(EEP_GYRO_BIAS + 4, mpu.getGyroBias().y);
    EEPROM.updateFloat(EEP_GYRO_BIAS + 8, mpu.getGyroBias().z);
    EEPROM.updateFloat(EEP_MAG_BIAS + 0, mpu.getMagBias().x);
    EEPROM.updateFloat(EEP_MAG_BIAS + 4, mpu.getMagBias().y);
    EEPROM.updateFloat(EEP_MAG_BIAS + 8, mpu.getMagBias().z);
    EEPROM.updateFloat(EEP_MAG_SCALE + 0, mpu.getMagScale().x);
    EEPROM.updateFloat(EEP_MAG_SCALE + 4, mpu.getMagScale().y);
    EEPROM.updateFloat(EEP_MAG_SCALE + 8, mpu.getMagScale().z);
    //if (b_save) EEPROM.commit();
}

void EEPROM_BACKUP::loadCalibration() {
    if (isCalibrated())    {
       // Serial.println("calibrated? : YES");
       // Serial.println("load calibrated values");
       Vect3 temp;
       temp.x= EEPROM.readFloat(EEP_ACC_BIAS + 0);
       temp.y= EEPROM.readFloat(EEP_ACC_BIAS + 4);
       temp.z= EEPROM.readFloat(EEP_ACC_BIAS + 8);
       mpu.setAccBias(temp);
       temp.x= EEPROM.readFloat(EEP_GYRO_BIAS + 0);
       temp.y= EEPROM.readFloat(EEP_GYRO_BIAS + 4);
       temp.z= EEPROM.readFloat(EEP_GYRO_BIAS + 8);
       mpu.setGyroBias(temp); 
       temp.x= EEPROM.readFloat(EEP_MAG_BIAS + 0);
       temp.y= EEPROM.readFloat(EEP_MAG_BIAS + 4);
       temp.z= EEPROM.readFloat(EEP_MAG_BIAS + 8);
       mpu.setMagBias(temp);
       temp.x= EEPROM.readFloat(EEP_MAG_SCALE + 0);
       temp.y= EEPROM.readFloat(EEP_MAG_SCALE + 4);
       temp.z= EEPROM.readFloat(EEP_MAG_SCALE + 8);
       mpu.setMagScale(temp);                      
    }
    else    {
        Serial.println("calibrated? : NO");
        Serial.println("load default values");
        Vect3 temp; temp.x=0;temp.y=0;temp.z=0;
        mpu.setAccBias(temp);
        mpu.setGyroBias(temp);
        mpu.setMagBias(temp);
        temp.x=1;temp.y=1;temp.z=1;
        mpu.setMagScale(temp);
    }
}

void EEPROM_BACKUP::clearCalibration()
{
    for (size_t i = 0; i < EEPROM_SIZE; ++i)
        EEPROM.writeByte(i, 0xFF);
 //   EEPROM.commit();
}

bool EEPROM_BACKUP::isCalibrated(){
    return (EEPROM.readByte(EEP_CALIB_FLAG) == 0x01);
}

void EEPROM_BACKUP::printCalibration() {
    Serial.println("< calibration parameters >");
    Serial.print("calibrated? : ");
    Serial.println(EEPROM.readByte(EEP_CALIB_FLAG) ? "YES" : "NO");
    Serial.print("acc bias x  : ");
    Serial.println(EEPROM.readFloat(EEP_ACC_BIAS + 0) * 1000.f);
    Serial.print("acc bias y  : ");
    Serial.println(EEPROM.readFloat(EEP_ACC_BIAS + 4) * 1000.f);
    Serial.print("acc bias z  : ");
    Serial.println(EEPROM.readFloat(EEP_ACC_BIAS + 8) * 1000.f);
    Serial.print("gyro bias x : ");
    Serial.println(EEPROM.readFloat(EEP_GYRO_BIAS + 0));
    Serial.print("gyro bias y : ");
    Serial.println(EEPROM.readFloat(EEP_GYRO_BIAS + 4));
    Serial.print("gyro bias z : ");
    Serial.println(EEPROM.readFloat(EEP_GYRO_BIAS + 8));
    Serial.print("mag bias x  : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_BIAS + 0));
    Serial.print("mag bias y  : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_BIAS + 4));
    Serial.print("mag bias z  : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_BIAS + 8));
    Serial.print("mag scale x : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_SCALE + 0));
    Serial.print("mag scale y : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_SCALE + 4));
    Serial.print("mag scale z : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_SCALE + 8));
}

void EEPROM_BACKUP::printBytes() {
    for (size_t i = 0; i < EEPROM_SIZE; ++i)
        Serial.println(EEPROM.readByte(i), HEX);
}

void EEPROM_BACKUP::setupEEPROM() {
    Serial.println("EEPROM start");
//    if (!EEPROM.begin(EEPROM_SIZE))
//    {
//        Serial.println("EEPROM start failed");
//    }
   // b_calibrated = isCalibrated();
    if ((EEPROM.readByte(EEP_CALIB_FLAG) != 0x01))    {
        Serial.println("Need Calibration!!");
    }
    Serial.println("EEPROM calibration value is : ");
    printCalibration();
    Serial.println("Loaded calibration value is : ");
    loadCalibration();
}
