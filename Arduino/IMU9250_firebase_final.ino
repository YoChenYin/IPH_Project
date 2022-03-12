/**************************************************************************************************************
  connecting three MPU9250MPU9250 sensors to an I2C multiplexer (TCA9548A)
 **************************************************************************************************************/
/*
   Created by K. Suwatchai (Mobizt)

   Email: k_suwatchai@hotmail.com

   Github: https://github.com/mobizt

   Copyright (c) 2021 mobizt

*/

//Example shows how to connect to Firebase RTDB and perform basic operation for set, get, push and update data to database

//Required WiFiNINA Library for Arduino from https://github.com/arduino-libraries/WiFiNINA

#include "Firebase_Arduino_WiFiNINA.h"

#define DATABASE_URL "msti-514-project-default-rtdb.firebaseio.com" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
#define DATABASE_SECRET "k8fE5Fw0AYh842MiVuHax6RCcTMW0JkcfFfJQo5C"

#define WIFI_SSID "AndroidAP" //"BasFam" 
#define WIFI_PASSWORD "gixwifi123" //"MoSuMaMu1984"

//Define Firebase data object
FirebaseData fbdo;
String path="/imu_data";

#include "MPU9250.h"
#include <Wire.h>
#include <Arduino_LSM6DS3.h>
MPU9250 mpu1, mpu2;
int16_t ADx[2] = {2, 3};
int mpuno = 0;
const char *spacer = ", ";

const int dataNum = 9;
float data[dataNum];
float accelX,            accelY,             accelZ,            // units m/s/s i.e. accelZ if often 9.8 (gravity)
      gyroX,             gyroY,              gyroZ,             // units dps (degrees per second)
      gyroDriftX,        gyroDriftY,         gyroDriftZ,        // units dps
      gyroRoll,          gyroPitch,          gyroYaw,           // units degrees (expect major drift)
      gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
      accRoll,           accPitch,           accYaw,            // units degrees (roll and pitch noisy, yaw not possible)
      complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)

long lastTime;
long lastInterval;
int count = 0;
String rawData = "";
/*
float[] get_roll_pitch_yaw(int i) {
  if (i == 1) {
    float yaw1=mpu1.getYaw();
    float pitch1=mpu1.getPitch();
    float roll1=mpu1.getRoll();
  
    Serial.print("mpu1_yaw:");
    Serial.print(yaw1, 2);
    Serial.print(", ");
    Serial.print("mpu1_pitch:");
    Serial.print(pitch1, 2);
    Serial.print(", ");
    Serial.print("mpu1_roll:");
    Serial.print(roll1, 2);
    Serial.print(", ");

    return {yaw1, pitch1, roll1};
  }
  else {
    float yaw2=mpu2.getYaw();
    float pitch2=mpu2.getPitch();
    float roll2=mpu2.getRoll();
    Serial.print("mpu2_yaw:");
    Serial.print(mpu2.getYaw(), 2);
    Serial.print(", ");
    Serial.print("mpu2_pitch:");
    Serial.print(mpu2.getPitch(), 2);
    Serial.print(", ");
    Serial.print("mpu2_roll:");
    Serial.print(mpu2.getRoll(), 2);
    Serial.print(", ");
    return {yaw2, pitch2, roll2};
  }
}
*/
void get_accel(int i) {
  if (i == 1) {
    float accX1 = mpu1.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY;
    float accY1 = mpu1.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY;
    float accZ1 = mpu1.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY;
    
    Serial.print("mpu1_accX:");
    Serial.print(mpu1.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print("mpu1_accY:");
    Serial.print(mpu1.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print("mpu1_accZ:");
    Serial.print(mpu1.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    String path="/imuReadings";
  
  }
  else {
    float accX2 = mpu2.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY;
    float accY2 = mpu2.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY;
    float accZ2 = mpu2.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY;
    
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
void get_gyro(int i) {
  if (i == 1) {
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
void setMPU(int16_t mpuno) {
  digitalWrite(ADx[mpuno - 1], LOW);
}
void resetMPU(int16_t mpuno) {
  digitalWrite(ADx[mpuno - 1], HIGH);
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
/*
float[] concateAry(float[] ary1, float ary2, int start) {
    j=0;
    for (int i=start ; j<3 ; i++){
        ary1[i] = ary2[j];
        j++;
    }
    return ary1;
}
*/
/*
float[][] resetAry(float[][] ary){
    for(int i=0;i<ary.length;i++){
        for(int j=0;j<ary[0].length;i++){
            ary[i][j]=0;
        }
    }
    return ary;
}
*/
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
void setup() {

  pinMode(ADx[0], OUTPUT);
  pinMode(ADx[1], OUTPUT);
  digitalWrite(ADx[0], HIGH);
  digitalWrite(ADx[1], HIGH);

  Serial.begin(9600);

  delay(100);
  Serial.println();

  Serial.print("Connecting to Wi-Fi");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED)
  {
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Wire.begin();

//  while (!Serial) {
//    ; //  wait for serial port to connect
//  }
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
  
  Firebase.begin(DATABASE_URL, DATABASE_SECRET, WIFI_SSID, WIFI_PASSWORD);
  Firebase.reconnectWiFi(true);
}

void loop() {
    
    if (count < 20) {

        float aX, aY, aZ;
        float gX, gY, gZ;
        float temp_imu1[3], temp_imu2[3], temp_nano[3]; 
        const char * spacer = ", ";
    
        // nano imu
        if (IMU.accelerationAvailable()&& IMU.gyroscopeAvailable()) {
            IMU.readAcceleration(aX, aY, aZ);
            IMU.readGyroscope(gX, gY, gZ);

            Serial.print("nanoIMU_accX:");
            Serial.print(aX, 2);
            Serial.print(", ");
            Serial.print("nanoIMU_accY:");
            Serial.print(aY, 2);
            Serial.print(", ");
            Serial.print("nanoIMU_accZ:");
            Serial.print(aZ, 2);
            Serial.print(", ");
            Serial.print("nanoIMU_gyroX:");
            Serial.print(gX, 2);
            Serial.print(", ");
            Serial.print("nanoIMU_gyroY:");
            Serial.print(gY, 2);
            Serial.print(", ");
            Serial.print("nanoIMU_gyroZ:");
            Serial.println(gZ, 2);
            long currentTime = micros();
            lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
            lastTime = currentTime;

            // do calculation
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
            //doCalculations(); 
            data[0] = complementaryYaw ;
            data[1] = complementaryPitch ;
            data[2] = complementaryRoll; 
            //data[count] = concateAry(data[count], temp_nano, 0); 
        }
        mpuno = 1;
        setMPU(mpuno);
        if (mpu1.update()) {
        //temp_imu1 = get_roll_pitch_yaw(1);
        float yaw1=mpu1.getYaw();
        float pitch1=mpu1.getPitch();
        float roll1=mpu1.getRoll();
    
        Serial.print("mpu1_yaw:");
        Serial.print(yaw1, 2);
        Serial.print(", ");
        Serial.print("mpu1_pitch:");
        Serial.print(pitch1, 2);
        Serial.print(", ");
        Serial.print("mpu1_roll:");
        Serial.print(roll1, 2);
        Serial.print(", ");
        data[3] = yaw1 ;
        data[4] = pitch1 ;
        data[5] = roll1; 
        get_accel(1);
        get_gyro(1);
        }
        resetMPU(mpuno);

        mpuno = 2;
        setMPU(mpuno);
        if (mpu2.update()) {
        //temp_imu2 = get_roll_pitch_yaw(2);
        //data[count] = concateAry(data[count], temp_imu2,6);
        float yaw2=mpu2.getYaw();
        float pitch2=mpu2.getPitch();
        float roll2=mpu2.getRoll();
        Serial.print("mpu2_yaw:");
        Serial.print(mpu2.getYaw(), 2);
        Serial.print(", ");
        Serial.print("mpu2_pitch:");
        Serial.print(mpu2.getPitch(), 2);
        Serial.print(", ");
        Serial.print("mpu2_roll:");
        Serial.print(mpu2.getRoll(), 2);
        Serial.print(", ");
        data[6] = yaw2 ;
        data[7] = pitch2 ;
        data[8] = roll2; 
        get_accel(2);
        get_gyro(2);
        }
        resetMPU(mpuno);
   
        count++;
    }
    else {
          for (int k =0; k<dataNum;k++){
            rawData += data[k];
            if (k!= dataNum-1){
              rawData += ',';  
            }
          }
          Serial.println("sending data to firebase...");
          if(Firebase.pushString(fbdo, path ,rawData)){
              Serial.println("send data to firebase");
          } 
          else {
              Serial.println("error, " + fbdo.errorReason());
              
          }  
          rawData = "";
          
          for(int j=0;j<9;j++){
              data[j]=0;
          }
          count = 0;
    }
}
