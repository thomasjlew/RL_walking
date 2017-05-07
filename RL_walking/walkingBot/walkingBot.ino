// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include <Wire.h>
#include <Servo.h>

#define SERIAL_BAUD_RATE 115200
#define SERIAL_TIMEOUT_TIME 5
#define SERIAL_IN_OUT_DELAY 7

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t offset_gx, offset_gy, offset_gz;
double ax, ay, az;
double gx, gy, gz;
int counter = 0;
unsigned long time;

Servo s1;
Servo s2;
Servo s3;
Servo s4;
int angle1 = 90;
int angle2 = 80;
int angle3 = 90;
int angle4 = 80;

void setup(){
  //  Initialize servomotors
  s1.attach(3);
  s2.attach(5);
  s3.attach(6);
  s4.attach(9);
  s1.write(angle1); 
  s2.write(angle2);
  s3.write(angle3);
  s4.write(angle4);

  //  Initialize transmission
  //  With MPU6050 (IMU)
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //  Serial communication with computer
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT_TIME);

  //  Initialize gyroscope offsets
  offset_gx = 0; offset_gy = 0; offset_gz = 0;
  int NB_DATA_INIT_GYRO_OFFSET = 10;
  for(int i=0; i<NB_DATA_INIT_GYRO_OFFSET; i++)
  {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    offset_gx += GyX;
    offset_gy += GyY;
    offset_gz += GyZ;
  }
  offset_gx = (int16_t)(double(offset_gx)/NB_DATA_INIT_GYRO_OFFSET);
  offset_gy = (int16_t)(double(offset_gy)/NB_DATA_INIT_GYRO_OFFSET);
  offset_gz = (int16_t)(double(offset_gz)/NB_DATA_INIT_GYRO_OFFSET);
}

void loop(){
  counter++;
  
  //  Read IMU data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //  Convert raw data into appropriate values
  ax = double(AcX)/16384;
  ay = double(AcY)/16384;
  az = double(AcZ)/16384;
  gx = double(GyX-offset_gx)/131;
  gy = double(GyY-offset_gy)/131;
  gz = double(GyZ-offset_gz)/131;
  
  /*  Send Data to serial port. Format:
        counter ax ay az gx gy gz time RockNRoll!\n
      example:
        6162 -0.84 0.18 -0.53 0.27 -0.02 -0.35 105554 RockNRoll!
  */
  Serial.print(counter); Serial.print(" "); 
  Serial.print(ax); Serial.print(" "); 
  Serial.print(ay); Serial.print(" "); 
  Serial.print(az); Serial.print(" "); 
  Serial.print(gx); Serial.print(" "); 
  Serial.print(gy); Serial.print(" "); 
  Serial.print(gz); Serial.print(" "); 
  time = millis();
  Serial.print(time); Serial.print(" ");
  Serial.print("RockNRoll!"); Serial.print("\n");
  
  //  Read Angles commands
  delay(SERIAL_IN_OUT_DELAY);
  while (Serial.available() > 0) {
    // look for the next valid integer in the incoming serial stream:
    int angle1 = Serial.parseInt(); 
    int angle2 = Serial.parseInt(); 
    int angle3 = Serial.parseInt(); 
    int angle4 = Serial.parseInt();
    
    Serial.print("Received torque commands: angles 1-2-3-4: ");
    Serial.print(angle1); Serial.print(" "); 
    Serial.print(angle2); Serial.print(" "); 
    Serial.print(angle3); Serial.print(" "); 
    Serial.print(angle4); Serial.println(" "); 
    
    //  Set angles to commands
    //  ! WARNING! Commands need to be well designed, no security is included here
    s1.write(angle1);
    s2.write(angle2);
    s3.write(angle3);
    s4.write(angle4);
    
    break;
  }
  
}
