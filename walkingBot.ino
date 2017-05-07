// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
#include <Servo.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t offset_gx, offset_gy, offset_gz;
double ax, ay, az;
double gx, gy, gz;

Servo s1;
Servo s2;
Servo s3;
Servo s4;
int angle1 = 90;
int angle2 = 80;
int angle3 = 90;
int angle4 = 90;

void setup(){
  //  Initialize servomotors
  s1.attach(3); //  angle == 75 is the middle (vertical) but not accessible
  s2.attach(5);
  s3.attach(6);
  s4.attach(9);
  s1.write(angle1); 
  s2.write(angle2);
  s3.write(angle3);
  s4.write(angle4);

  //  Initialize transmission
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

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
  /*Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);*/

  //  Convert raw data into appropriate values
  ax = double(AcX)/16384;
  ay = double(AcY)/16384;
  az = double(AcZ)/16384;
  gx = double(GyX-offset_gx)/131;
  gy = double(GyY-offset_gy)/131;
  gz = double(GyZ-offset_gz)/131;
  Serial.print("ax = "); Serial.print(ax);
  Serial.print(" | ay = "); Serial.print(ay);
  Serial.print(" | az = "); Serial.print(az);
  Serial.print(" | gx = "); Serial.print(gx);
  Serial.print(" | gy = "); Serial.print(gy);
  Serial.print(" | gz = "); Serial.println(gz);

  //  test servomotor n°1 & 2 1:
  Serial.println("90");
  s1.write(90);
  delay(1000);
  Serial.println("100");
  s1.write(100);
  Serial.println("105");
  s1.write(105);
  
  /*Serial.println("60");
  s2.write(60);
  delay(1000);
  Serial.println("80");
  s2.write(80);
  delay(1000);
  Serial.println("100");
  s2.write(100);
  delay(1000);*/
  
  //  test servomotor n°3 & 4 3: 90+++
  Serial.println("70");
  s3.write(90);
  Serial.println("90");
  s3.write(90);
  delay(1000);
  Serial.println("130");
  s3.write(110);
  delay(1000);
  
  /*Serial.println("60");
  s4.write(60);
  delay(1000);
  Serial.println("90");
  s4.write(90);
  delay(1000);
  Serial.println("100");
  s4.write(110);
  delay(1000);*/
  
}
