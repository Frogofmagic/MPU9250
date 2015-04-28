#include <Wire.h>
 
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
 
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
 
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18
 
 
 
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
 
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}
 
 
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}
 
 
// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);
 
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
 
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
 
 
}
 
 
long int cpt=0;
// Main loop, read and display data
void loop()
{
 
  // _______________
  // ::: Counter :::
 
  // Display data counter
  Serial.print (cpt++,DEC);
  Serial.print ("\n");
 
 
 
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 
 
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
 
 
  // Create 16 bits values from 8 bits data
 
  // Accelerometer
  int16_t ax=Buf[0]<<8 | Buf[1];
  int16_t ay=Buf[2]<<8 | Buf[3];
  int16_t az=Buf[4]<<8 | Buf[5];
 
  // Gyroscope
  int16_t gx=Buf[8]<<8 | Buf[9];
  int16_t gy=Buf[10]<<8 | Buf[11];
  int16_t gz=Buf[12]<<8 | Buf[13];
 
    // Display values
 
  // Accelerometer
  Serial.print ("Accelerometer: ax="); 
  Serial.print (ax,DEC); 
  Serial.print (" ay=");
  Serial.print (ay,DEC);
  Serial.print (" az=");
  Serial.print (az,DEC);  
  Serial.print ("\n");
 
  // Gyroscope
  Serial.print ("Gyroscope: x="); 
  Serial.print (gx,DEC); 
  Serial.print (" y=");
  Serial.print (gy,DEC);
  Serial.print (" z=");
  Serial.print (gz,DEC);  
  Serial.print ("\n");
 
 
  // _____________________
  // :::  Magnetometer ::: 
 
 
  // Read register Status 1 and wait for the DRDY: Data Ready
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));
 
  // Read magnetometer data  
  uint8_t Mag[6];  
  I2Cread(MAG_ADDRESS,0x03,6,Mag);
 
  // Request next magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
 
 
 
  // Create 16 bits values from 8 bits data
 
  // Magnetometer
  int16_t mx=Mag[1]<<8 | Mag[0];
  int16_t my=Mag[3]<<8 | Mag[2];
  int16_t mz=Mag[5]<<8 | Mag[4];
 
 
  // Magnetometer
  Serial.print("Magnetometer: mx=");
  Serial.print (mx,DEC); 
  Serial.print (" my=");
  Serial.print (my,DEC);
  Serial.print (" mz=");
  Serial.print (mz,DEC);  
  Serial.print ("\n");
 
  // End of line
  Serial.println("");
  delay(1000);    
}
