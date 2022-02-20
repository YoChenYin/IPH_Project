/**************************************************************************************************************
  connecting three MPU9250MPU9250 sensors to an I2C multiplexer (TCA9548A)
 **************************************************************************************************************/
#include "MPU9250.h"
#include <Wire.h>

MPU9250 mpu1, mpu2;
int16_t ADx[2] = {2,3};
int mpuno = 0;
const char *spacer = ", ";
void setup() {
    
    Wire.begin();
    
    pinMode(ADx[0], OUTPUT);
    pinMode(ADx[1],OUTPUT);
    digitalWrite(ADx[0],HIGH);
    digitalWrite(ADx[1],HIGH);
    
    Serial.begin(9600);
    
    while(!Serial) {
    ; //  wait for serial port to connect
    }
    delay(1000);
    Serial.println("start");
    delay(5000);
    // MPU9250 setting
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
    
    setMPU(1);
    if (!mpu1.setup(0x68, setting)) {  // change to your own address
        while (1) {
          Serial.println("MPU 1");
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
        }
    }
    // calibration
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu1.verbose(true);
    delay(5000);
    mpu1.calibrateAccelGyro();
    mpu1.verbose(false);// calibration
    resetMPU(1);
    
    setMPU(2);
    if (!mpu2.setup(0x68, setting)) {  // change to your own address
        while (1) {
          Serial.println("MPU 1");
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
        }
    }
   // calibration
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu2.verbose(true);
    delay(5000);
    mpu2.calibrateAccelGyro();
    mpu2.verbose(false); // calibration
    resetMPU(2);
    
}

void loop() {
    mpuno = 1;
    
    setMPU(mpuno);
    if (mpu1.update()) {
        Serial.print("MPU1");
        get_roll_pitch_yaw(1);
        get_accel(1);
        get_gyro(1);   
    }
    resetMPU(mpuno);
    
    mpuno = 2;
    setMPU(mpuno);
    if (mpu2.update()) {
        Serial.print("MPU2");
        get_roll_pitch_yaw(2);
        get_accel(1);
        get_gyro(1);   
    }
    resetMPU(mpuno);
    
}

void get_roll_pitch_yaw(int i) {
  if (i == 1){
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu1.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu1.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu1.getRoll(), 2); 
  }
  else{
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu2.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu2.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu2.getRoll(), 2); 
  }
     
}

void get_accel(int i){
  if (i == 1){
    Serial.println("accel bias [g]: ");
    Serial.print(mpu1.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu1.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu1.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
  }
  else{
    Serial.println("accel bias [g]: ");
    Serial.print(mpu2.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu2.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu2.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println(); 
  }
}
void get_gyro(int i){
  if(i == 1) {
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu1.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu1.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu1.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
  }
  else {
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu2.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu2.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu2.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
  }
}
void setMPU(int16_t mpuno){
  digitalWrite(ADx[mpuno-1], LOW);
}
void resetMPU(int16_t mpuno){
  digitalWrite(ADx[mpuno-1], HIGH);  
}
