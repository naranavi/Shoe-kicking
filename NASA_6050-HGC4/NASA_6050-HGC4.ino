//Shoe kicking
//感度低下バージョン
//CAPセンサー付き
//Yoshimi Sugimoto

#include <SoftwareSerial.h>
// MPU-6050 Accelerometer + Gyro
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "CapSense.h"

#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_ACCEL_XOUT_H       0x3B   // R  
#define MPU6050_WHO_AM_I           0x75   // R
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_I2C_ADDRESS 0x68


typedef union accel_t_gyro_union{
  struct{
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } 
  reg;
  struct{
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } 
  value;
};

long total1=0;

#define BT_RX 11
#define BT_TX 10

CapSense   cs_7_6 = CapSense(7,6);  // ピン7-6間に10MΩ、ピン6にアルミホイル

#define TP 3

#define LEDPIN 4      //シリアルLED
#define LEDS 3        //LED数
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDS, LEDPIN, NEO_GRB + NEO_KHZ800);

 #define PORATE 115200
//#define PORATE 9600

SoftwareSerial btSerial(BT_RX, BT_TX);

//Gyro
#define N_ss 20
long ss[N_ss];
int sp=0;
//accelerometer
#define N_ssa 8
long ssa[N_ssa];
int spa=0;


#define Ts 25
long ts;

void setup(){
  Wire.begin();
  int error;
  uint8_t c;
 Serial.begin(115200);
 btSerial.begin(PORATE);
  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);

   //ジャイロ感度
   MPU6050_write_reg ( MPU6050_RA_GYRO_CONFIG,MPU6050_GYRO_FS_2000<<MPU6050_GCONFIG_FS_SEL_BIT);
  //加速度感度
   uint8_t w[8];
   MPU6050_read(MPU6050_RA_ACCEL_CONFIG,w,1 );
   c = w[0];
   c = c & ~(3<<MPU6050_ACONFIG_AFS_SEL_BIT);
   c = c | (MPU6050_ACCEL_FS_16<<MPU6050_ACONFIG_AFS_SEL_BIT);
   MPU6050_write_reg (MPU6050_RA_ACCEL_CONFIG,c);

  strip.begin();
}
 
void loop(){
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_angle_x;
    float acc_angle_y;
    float acc_angle_z;

  int error,i;
  boolean outf = true;
  float dT;
  long t =millis()+1000;
  ts = millis()+ Ts;

  for(;;){
    while(ts>millis());
    ts = millis()+Ts;
    
    accel_t_gyro_union accel_t_gyro;
    error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
    //Serial.print(error,DEC);
    //Serial.print("\t");
       //capSense
    total1 =  cs_7_6.capSense(30);
    if (total1<80){
      //離れた

      if(outf == false){
       Serial.println("kick");
       //new off Tx
        outf = true;
        long a =0;
        for(int i= 0;i<N_ss ;i++){
          //if(ss[i]>0)a = a +ss[i];
          a = a +abs(ss[i]);
         //Serial.println(ss[i]);
        }
        #define Speed 200
        if(a>Speed){    //足から外れたとき足を振っているか？
          //送信
          Serial.println("acc");
          long b =0;
          for(int i= 0;i<N_ssa ;i++){
            if(ssa[i]>0) b += ssa[i];
           //Serial.println(ssa[i]);
          }
          b = b /N_ssa;
          //b = -b;             //acc mean

          gyro_z = -gyro_z;
          stripDisp(strip.Color(0, 0, 128)); // blue
          btSerial.print(a);
          btSerial.print("\t");
          btSerial.print(b);
          btSerial.print("\t");
          btSerial.print(gyro_z, 0);
          btSerial.println("");       
          
          Serial.print(a);
          Serial.print("\t");
          Serial.print(b);
          Serial.print("\t");
          Serial.print(gyro_z, 0);
          Serial.println("");       
          delay(100);
          stripDisp(strip.Color(128, 0, 0)); // red
          delay(2000);
          stripDisp(strip.Color(128, 0, 0)); // 
          for(i=128;i>10;i--){
            stripDisp(strip.Color(i, 0, 0)); // red
            delay(10);
          }
        }
          delay(300);
        
          stripDisp(strip.Color(128, 0, 0)); // 
          delay(100);
        stripDisp(strip.Color(0, 0, 0)); // off
      }else{
        //履いている
        stripDisp(strip.Color(0, 0, 0)); // off
      }
    }else{
      //履いてる
      outf = false;
      stripDisp(strip.Color(0, 128, 0)); // green

    }
    

    uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap
    SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
    SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
    SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
    SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
    SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
    SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
    SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
   
    dT = ( (float) accel_t_gyro.value.temperature + 12412.0) / 340.0;
  /*  
   *   
    Serial.print(dT, 1);
    Serial.print("\t");
    
   */
   
    float acc_x = accel_t_gyro.value.x_accel / 16384.0; //FS_SEL_0 16,384 LSB / g
    float acc_y = accel_t_gyro.value.y_accel / 16384.0;
    float acc_z = accel_t_gyro.value.z_accel / 16384.0;
   
    acc_angle_x = atan2(acc_x, acc_z) * 360 / 2.0 / PI;
    acc_angle_y = atan2(acc_y, acc_z) * 360 / 2.0 / PI;
    acc_angle_z = atan2(acc_x, acc_y) * 360 / 2.0 / PI;
   
  
    gyro_x = accel_t_gyro.value.x_gyro / 131.0;  //FS_SEL_0 131 LSB / (°/s)
    gyro_y = accel_t_gyro.value.y_gyro / 131.0;
    gyro_z = accel_t_gyro.value.z_gyro / 131.0;

  //Serial.print(accel_t_gyro.value.x_accel );
  //Serial.print("\t");
/*
  Serial.print(acc_angle_x, 2); Serial.print("\t");
  Serial.print(acc_angle_y, 2); Serial.print("\t");
  Serial.println(acc_angle_z, 2);
*/

    //gyro log
    ss[sp] = (long)gyro_x;
    sp++;
    if(sp>=N_ss)sp=0;
    //acc log
    ssa[spa] = (long)acc_angle_z;
    spa++;
    if(spa>=N_ssa)spa=0;

    
  }

}
 
// MPU6050_read
int MPU6050_read(int start, uint8_t *buffer, int size){
  int i, n, error;
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);
  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);
  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size){
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);
  return (0);  // return : no error
}
 
// MPU6050_write
int MPU6050_write(int start, const uint8_t *pData, int size){
  int n, error;
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);
  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);
  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);
  return (0);         // return : no error
}
 
// MPU6050_write_reg
int MPU6050_write_reg(int reg, uint8_t data){
  int error;
  error = MPU6050_write(reg, &data, 1);
  return (error);
}

// Fill the dots one after the other with a color
void stripDisp(uint32_t c) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
  }
  strip.show();
}

