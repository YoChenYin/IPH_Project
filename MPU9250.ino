/**************************************************************************************************************
  connecting three MPU9250MPU9250 sensors to an I2C multiplexer (TCA9548A)
 **************************************************************************************************************/
#include "MPU9250.h"
#include <Wire.h>
#include <Arduino_LSM6DS3.h>
MPU9250 mpu1, mpu2;
int16_t ADx[2] = {2,3};
int mpuno = 0;
const char *spacer = ", ";
float data[27] = {};
float accelX,            accelY,             accelZ,            // units m/s/s i.e. accelZ if often 9.8 (gravity)
      gyroX,             gyroY,              gyroZ,             // units dps (degrees per second)
      gyroDriftX,        gyroDriftY,         gyroDriftZ,        // units dps
      gyroRoll,          gyroPitch,          gyroYaw,           // units degrees (expect major drift)
      gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
      accRoll,           accPitch,           accYaw,            // units degrees (roll and pitch noisy, yaw not possible)
      complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)

long lastTime;
long lastInterval;

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
    if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (true); // halt program
    } 
    Serial.println("nano IMU initialized!");
    
    calibrateIMU(250, 250); 
    lastTime = micros();
    
}
void calibrateIMU(int delayMillis, int calibrationMillis) {

  int calibrationCount = 0;

  delay(delayMillis); // to avoid shakes after pressing reset button

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    if (readIMU()) {
      // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;

      calibrationCount++;
    }
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;

}
bool readIMU() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    return true;
  }
  return false;
}
void loop() {
    float aX, aY, aZ;
    float gX, gY, gZ;
    float data[27] = {};
    const char * spacer = ", ";
    mpuno = 1;
    
    setMPU(mpuno);
    if (mpu1.update()) {
        get_roll_pitch_yaw(1);
        get_accel(1);
        get_gyro(1);   
    }
    resetMPU(mpuno);
    
    mpuno = 2;
    setMPU(mpuno);
    if (mpu2.update()) {
        get_roll_pitch_yaw(2);
        get_accel(2);
        get_gyro(2);   
    }
    resetMPU(mpuno);

    if (
      IMU.accelerationAvailable() 
      && IMU.gyroscopeAvailable()
    ) {      
      IMU.readAcceleration(aX, aY, aZ);
      IMU.readGyroscope(gX, gY, gZ);
      
      Serial.print("nanoIMU_accX:");
      Serial.print(aX,2); 
      Serial.print(", ");
      Serial.print("nanoIMU_accY:");
      Serial.print(aY,2);
      Serial.print(", ");
      Serial.print("nanoIMU_accZ:");
      Serial.print(aZ,2); 
      Serial.print(", ");
      Serial.print("nanoIMU_gyroX:"); 
      Serial.print(gX,2); 
      Serial.print(", "); 
      Serial.print("nanoIMU_gyroY:"); 
      Serial.print(gY,2); 
      Serial.print(", "); 
      Serial.print("nanoIMU_gyroZ:"); 
      Serial.println(gZ,2);
      long currentTime = micros();
      lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
      lastTime = currentTime;
  
      doCalculations();
    } 
 
    
}

void get_roll_pitch_yaw(int i) {
  if (i == 1){
    
    Serial.print("mpu1_yaw:");
    Serial.print(mpu1.getYaw(), 2);
    Serial.print(", ");
    Serial.print("mpu1_pitch:");
    Serial.print(mpu1.getPitch(), 2);
    Serial.print(", ");
    Serial.print("mpu1_roll:");
    Serial.print(mpu1.getRoll(), 2); 
    Serial.print(", ");
  }
  else{
    Serial.print("mpu2_yaw:"); 
    Serial.print(mpu2.getYaw(), 2);
    Serial.print(", ");
    Serial.print("mpu2_pitch:"); 
    Serial.print(mpu2.getPitch(), 2);
    Serial.print(", ");
    Serial.print("mpu2_roll:"); 
    Serial.print(mpu2.getRoll(), 2); 
    Serial.print(", ");
   
  }
     
}

void get_accel(int i){
  if (i == 1){
    
    Serial.print("mpu1_accX:"); 
    Serial.print(mpu1.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print("mpu1_accY:"); 
    Serial.print(mpu1.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print("mpu1_accZ:"); 
    Serial.print(mpu1.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
   
  }
  else{
    Serial.print("mpu2_accX:"); 
    Serial.print(mpu2.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print("mpu2_accY:"); 
    Serial.print(mpu2.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print("mpu2_accZ:");  
    Serial.print(mpu2.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", "); 
    
  }
}
void get_gyro(int i){
  if(i == 1) {
    Serial.print("mpu1_gyroX:");
    Serial.print(mpu1.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print("mpu1_gyroY:"); 
    Serial.print(mpu1.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print("mpu1_gyroZ:"); 
    Serial.print(mpu1.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    
  }
  else {
    Serial.print("mpu2_gyroX:"); 
    Serial.print(mpu2.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print("mpu2_gyroY:");  
    Serial.print(mpu2.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print("mpu2_gyroZ:");  
    Serial.print(mpu2.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
   
  }
}
void setMPU(int16_t mpuno){
  digitalWrite(ADx[mpuno-1], LOW);
}
void resetMPU(int16_t mpuno){
  digitalWrite(ADx[mpuno-1], HIGH);  
}


void doCalculations() {
  accRoll = atan2(accelY, accelZ) * 180 / M_PI;
  accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;

  float lastFrequency = (float) 1000000.0 / lastInterval;
  gyroRoll = gyroRoll + (gyroX / lastFrequency);
  gyroPitch = gyroPitch + (gyroY / lastFrequency);
  gyroYaw = gyroYaw + (gyroZ / lastFrequency);

  gyroCorrectedRoll = gyroCorrectedRoll + ((gyroX - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((gyroY - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = complementaryRoll + ((gyroX - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((gyroY - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
  complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;
  Serial.print("nanoIMU_yaw:");
  Serial.print(complementaryYaw);
  Serial.print(", ");
  Serial.print("nanoIMU_pitch:");
  Serial.print(complementaryPitch);
  Serial.print(", ");
  Serial.print("nanoIMU_roll:");
  Serial.print(complementaryRoll);
  Serial.print(", ");
}
