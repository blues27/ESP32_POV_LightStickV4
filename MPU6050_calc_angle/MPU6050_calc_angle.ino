#include "MPU6050.h"
#include "CharaData.h"
#include <Wire.h>
#include <rom/ets_sys.h>

#define LED_PIN 32

#define RET_RANGE   (-429)

#define GYRO_PERIOD (1000L)
#define LOG_PERIOD  (100000L)

#define AVE_LEN   9
#define HIS_LEN   81

#define INTERRUPT_PIN 35 

void gyro_int(void);

void log_print(void *arg);

volatile float angle = 0.0; 
volatile int32_t z_data;
volatile float z_bias; 
volatile uint32_t tick = 0 ;

volatile uint32_t del_micros ; 

uint16_t tz_data[AVE_LEN];
float angvel[HIS_LEN];
uint32_t dtime[HIS_LEN];

void gyro_write(uint8_t addr,uint8_t dat)
{
  Wire.beginTransmission(MPU6050_ADDRESS_AD0_HIGH);
  Wire.write(addr);   
  Wire.write(dat);
  Wire.endTransmission();
}
 
uint8_t gyro_read(uint8_t addr)
{
  Wire.beginTransmission(MPU6050_ADDRESS_AD0_HIGH);
  Wire.write(addr);   
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDRESS_AD0_HIGH,(uint8_t)1);
  if (! Wire.available()) 
    return -1;
  return (Wire.read());      
}

#ifndef I_AM_MPU6050
  #define I_AM_MPU6050  0x68
#endif 

#ifndef D2D_MPU
  #define D2D_MPU  (0.06097560975609756097560975609756)
#endif 

void mpu_setup()
{
  int ret ;
  uint8_t mpu_adr = 0;
  uint8_t mpu_reg = 0;
  char str[64];
  Wire.setClock(400000L);
  Wire.begin(21, 22); // ESP32 SDA,SCL

  Serial.println(F("Initializing I2C devices..."));
  pinMode(INTERRUPT_PIN, INPUT);

  ret = gyro_read(MPU6050_RA_WHO_AM_I);

  sprintf(str,"WHOAMI register = 0x%X \n\r", ret);
  Serial.print(str);

  if(ret == I_AM_MPU6050) {
    Serial.print("MPU6050 is found \n\r");
  }else{
    Serial.print("no MPU6050 is found \n\r");
  }
  
  do{ 
      mpu_adr = MPU6050_RA_PWR_MGMT_1 ;
      mpu_reg = MPU6050_PWR1_DEVICE_RESET_BIT ;
      gyro_write(mpu_adr,mpu_reg);
      delay(10);

      mpu_adr = MPU6050_RA_PWR_MGMT_1 ;
      mpu_reg = 0x00 ;
      gyro_write(mpu_adr,mpu_reg);

      mpu_adr = MPU6050_RA_PWR_MGMT_1 ;
      ret=gyro_read(mpu_adr);

  }while(ret != mpu_reg);

  gyro_write(MPU6050_RA_SMPLRT_DIV, 0x00);
  
  do{ 
      mpu_adr = MPU6050_RA_CONFIG ;
      mpu_reg = 0x00 ;
      gyro_write(mpu_adr,mpu_reg);

      mpu_adr = MPU6050_RA_CONFIG ;
      ret=gyro_read(mpu_adr);

  }while(ret != mpu_reg);

  do{ 
      mpu_adr = MPU6050_RA_GYRO_CONFIG ;    
      ret = gyro_read(mpu_adr);
      ret = ret & ~0x18;
      mpu_reg = ret | (MPU6050_GYRO_FS_2000<<3); 
      gyro_write(mpu_adr,mpu_reg);
      ret = gyro_read(mpu_adr);

  }while(ret != mpu_reg);
  delay(10);

  Serial.print("I did it \n\r");
}

#define BIAS_NUM 32

void gryo_bias(void) 
{
  uint16_t i;
  uint16_t tz_data;
  int32_t z_data;
  uint8_t dmy; 

  z_bias = 0.0;

  for(i=0;i<BIAS_NUM;i++){
    dmy=gyro_read(MPU6050_RA_GYRO_ZOUT_L);
    tz_data = (0x00ff&dmy);
    dmy=gyro_read(MPU6050_RA_GYRO_ZOUT_H);
    tz_data |= (0xff00&(dmy<<8));    
    z_data = -1*(int16_t)tz_data;             
    z_bias += (float)z_data*D2D_MPU;
    delay(10);
  }
  z_bias = z_bias / BIAS_NUM;    

  Serial.print("zbias = ");
  Serial.println(z_bias);
} 

#define HIS_LEN   9 
#define ZEROSMP 15 
#define ZEROCHK 10

void mpu_gyro(void *arg)
{
  static uint16_t tz_data[HIS_LEN]={0,0,0,0,0,0,0,0,0};
  static uint32_t pre_micros = 0 ; 
  uint32_t cur_micros;
  int32_t t_data;
  int32_t a_data;
  int32_t b_data;
  uint16_t idx ; 
  
  int16_t i;
  while(1){  
    for (i = HIS_LEN - 1; i > 0; i--) {
      angvel[i] = angvel[i - 1];
      dtime[i] = dtime[i - 1];
    }
  
    for (i = AVE_LEN - 1; i > 0; i--) {
      tz_data[i] = tz_data[i - 1] ;
    }
  
    tz_data[0] = (uint16_t)gyro_read(MPU6050_RA_GYRO_ZOUT_H);
    tz_data[0] = tz_data[0] << 8 ;
    tz_data[0] |= gyro_read(MPU6050_RA_GYRO_ZOUT_L);
  
    t_data = (int16_t)tz_data[0];
  
    for (i = 1; i < AVE_LEN; i++) {
      t_data += (int16_t)tz_data[i];
    }
    z_data = - t_data / AVE_LEN;
  
    cur_micros = micros();
    del_micros = cur_micros - pre_micros ;
    pre_micros = cur_micros ;
    z_bias = 0.0;
    dtime[0] = del_micros ; 
    angvel[0] = ((float)z_data * D2D_MPU - 1.0 * z_bias) ;
  
    uint8_t cnt_pos=0,cnt_neg=0;
    cnt_pos=0 ;
    cnt_neg=0 ;
    
    for(i=0;i<ZEROSMP;i++){
      if(angvel[i] > 0)
        cnt_pos ++ ;
      if(angvel[ZEROSMP*2+i] < 0)      
        cnt_neg ++ ; 
    }
    
    if((cnt_pos > ZEROCHK) && (cnt_neg > ZEROCHK)){
      angle = 0 ;
    }
    
    angle += angvel[0] * 0.000001 * (float)del_micros ;
  
    if(angle < 0 ) 
      angle = 0 ;
    if(angle > 180 ) 
      angle = 180 ;
  
  }
}

void log_print(void *arg)
{
  uint32_t t_tick ;
  float t_angle ;
  while(1){
    t_tick = tick ; 
    t_angle = angle ;
  
    Serial.print(" time = "); 
    Serial.print(millis(),DEC); 
    Serial.print(" del_t = "); 
    Serial.print(del_micros,DEC);     
    Serial.print(" angle ="); 
    Serial.println(t_angle,2);
    delay(10);
  }
}

void setup() 
{
  int i ; 
  
  Serial.begin(115200);
  Serial.println("\n Light Stick Demo");

  mpu_setup();
  gryo_bias();

  pinMode(LED_PIN, OUTPUT);
  // ウォッチドッグ停止
  //disableCore0WDT();
  //disableCore1WDT(); 
  xTaskCreatePinnedToCore(  mpu_gyro, "I2C_TASK", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(  log_print, "LOG_TASK", 4096, NULL, 1, NULL, 1);
}

void loop() {

  delay(100);
}
