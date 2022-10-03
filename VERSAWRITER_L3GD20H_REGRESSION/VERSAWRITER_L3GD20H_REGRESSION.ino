
#include "l3gd20.h"
#include "CharaData.h"
#include <Wire.h>
#include <WiFi.h>
#include <string.h>

#define NETON

#define COL_BK  0    // 000 BLACK
#define COL_BL  1    // 001 BLUE
#define COL_RD  2    // 010 RED
#define COL_PR  3    // 011 PURPLE
#define COL_GR  4    // 100 GREEN
#define COL_SB  5    // 101 SKY BLUE
#define COL_YL  6    // 110 YELLOW
#define COL_WH  7    // 111 WHITE

//#define LED_PIN 32
#define LED_CH  0
#define LED_PIN A4  // GPIO32
#define LED_L1  5  
#define LED_L2  18
#define LED_L3  23
#define LED_L4  4
#define LED_L5  15
#define LED_L6  19
#define LED_H1  33
#define LED_H2  25
#define LED_H3  14
#define LED_H4  26
#define LED_H5  27
#define LED_R   2
#define LED_G   13
#define LED_B   12

#define AVE_LEN   9
//#define HIS_LEN   81

#define HIS_LEN   27 
#define ZEROSMP (HIS_LEN/2) 
#define ZEROCHK (ZEROSMP*2/3)
  
#define MAX_SRV_CLIENTS 1

const char* ssid = "HORNET";
const char* password = "password";

WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

void gyro_int(void);
void led_blink_int(void);
void log_print(void *arg);
void telnetserver(void *arg);
void sendmessage(uint8_t sbuf,size_t len);

volatile float angle = 0.0; 
volatile int32_t z_data;
volatile float z_bias; 
volatile uint32_t tick = 0 ;
volatile uint8_t on_led[DOTSUU]; 
volatile uint32_t del_micros ; 

static uint16_t tz_data[AVE_LEN];
float angvel[HIS_LEN];
uint32_t dtime[HIS_LEN];

void gyro_write(uint8_t addr,uint8_t dat)
{
  Wire.beginTransmission(L3GD20_I2C_ADDR);
  Wire.write(addr);   
  Wire.write(dat);
  Wire.endTransmission();
}
 
uint8_t gyro_read(uint8_t addr)
{
  Wire.beginTransmission(L3GD20_I2C_ADDR);
  Wire.write(addr);   
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)L3GD20_I2C_ADDR,(uint8_t)1);
  if (! Wire.available()) 
    return -1;
  return (Wire.read());      
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
    dmy=gyro_read(L3GD20_OUT_Z_L_ADDR);
    tz_data = (0x00ff&dmy);
    dmy=gyro_read(L3GD20_OUT_Z_H_ADDR);
    tz_data |= (0xff00&(dmy<<8));    
    z_data = -1*(int16_t)tz_data;             
    z_bias += (float)z_data*D2D_L3GD20;
    delay(10);
  }
  z_bias = z_bias / BIAS_NUM;    
} 
 
void gyro_init(void)
{
  int ret ;
  uint8_t l3gd_adr = 0;
  uint8_t l3gd_reg = 0;
    
  Wire.setClock(400000L);
  Wire.begin(21, 22); // ESP32 SDA,SCL
  delay(40);
 
  ret = gyro_read(L3GD20_WHO_AM_I_ADDR);
  if(Serial){
    Serial.print("WHOAMI register = 0x");
    Serial.println(ret,HEX);
  }
  if((ret == I_AM_L3GD20)||(ret == I_AM_L3GD20H)) {
    if(Serial)
      Serial.println("L3GD20/H is found ");
  }else{
    if(Serial)
      Serial.println("no L3GD20/H is found ");
    while(1);
    return ; 
  }
  do{
    l3gd_adr = L3GD20_CTRL_REG1_ADDR ;
    l3gd_reg = L3GD20_Z_ENABLE|L3GD20_MODE_ACTIVE
              |L3GD20_OUTPUT_DATARATE_4|L3GD20_BANDWIDTH_4;
    gyro_write(l3gd_adr,l3gd_reg);
    delay(10);

    l3gd_adr = L3GD20_CTRL_REG1_ADDR ;
    ret=gyro_read(l3gd_adr);
  }while(ret != l3gd_reg);
  do{
      l3gd_adr = L3GD20_CTRL_REG2_ADDR ;
      ret = gyro_read(l3gd_adr);

      l3gd_adr = L3GD20_CTRL_REG2_ADDR ;
      l3gd_reg = ret | L3GD20_HPM_NORMAL_MODE_RES|L3GD20_HPFCF_0;
      gyro_write(l3gd_adr,l3gd_reg);

      l3gd_adr = L3GD20_CTRL_REG2_ADDR ;
      ret=gyro_read(l3gd_adr);

  }while(ret != l3gd_reg);
  do{
      l3gd_adr = L3GD20_CTRL_REG4_ADDR ;
      l3gd_reg = L3GD20_BlockDataUpdate_Continous|L3GD20_BLE_LSB
      |L3GD20_FULLSCALE_2000;
      gyro_write(l3gd_adr,l3gd_reg);

      l3gd_adr = L3GD20_CTRL_REG4_ADDR ;
      ret=gyro_read(l3gd_adr);

  }while(ret != l3gd_reg);
  if(Serial)
    Serial.printf("I did it ");
}

void gyro_i2c(void *arg)
{
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
  
    tz_data[0] = (uint16_t)gyro_read(L3GD20_OUT_Z_H_ADDR);
    tz_data[0] = tz_data[0] << 8 ;
    tz_data[0] |= gyro_read(L3GD20_OUT_Z_L_ADDR);
  
    t_data = (int16_t)tz_data[0];
  
    for (i = 1; i < AVE_LEN; i++) {
      t_data += (int16_t)tz_data[i];
    }
    z_data = - t_data / AVE_LEN;
  
    cur_micros = micros();
    del_micros = cur_micros - pre_micros ;
    pre_micros = cur_micros ;
    
    dtime[0] = del_micros ; 
    angvel[0] = ((float)z_data * D2D_L3GD20 - 0.0 * z_bias) ;

    uint8_t cnt_pos=0,cnt_neg=0;
    cnt_pos=0 ;
    cnt_neg=0 ;
    
    for(i=0;i<ZEROSMP;i++){
      if(angvel[i] > 0)
        cnt_pos ++ ;
      if(angvel[HIS_LEN-1-i] < 0)      
        cnt_neg ++ ; 
    }
    
    if((cnt_pos > ZEROCHK) && (cnt_neg > ZEROCHK)){
      float sum = 0.0 ; 
      for(i=0;i<(HIS_LEN/2);i++){
        sum += angvel[i]* 0.000001*dtime[i]; 
      }
      angle = sum ;
    }else{  
      angle += angvel[0] * 0.000001 * (float)del_micros ;
    }

//    if(angle < 0 ) 
//      angle = 0 ;

    if((angle > (float)MOJISUU_OFS)&&(angle < (float)(MOJISUU+MOJISUU_OFS))){
  //        idx = MOJISUU - (uint16_t)(angle - (float)MOJISUU_OFS);
      idx = (uint16_t)(angle - (float)MOJISUU_OFS);
      if(idx>MOJISUU)
        idx=MOJISUU-1;
        
      for(i=0;i<DOTSUU;i++){
        on_led[i] = moji[i][idx]; 
      }
      
    }else{
      for(i=0;i<DOTSUU;i++){
        on_led[i] =  0; 
      }
    } 
  }
}

void IRAM_ATTR onTimerA()//void gyro_int(void)
{
  tick++;
}

void LED_proc_sq(int lnum,int nc)
{
//  led_proc(lnum%5+1,lnum/5+1,nc);

  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
  
  digitalWrite(LED_H1, LOW );
  digitalWrite(LED_H2, LOW );
  digitalWrite(LED_H3, LOW );
  digitalWrite(LED_H4, LOW );
  digitalWrite(LED_H5, LOW );

  digitalWrite(LED_L1, LOW );
  digitalWrite(LED_L2, LOW );
  digitalWrite(LED_L3, LOW );
  digitalWrite(LED_L4, LOW );
  digitalWrite(LED_L5, LOW );
  digitalWrite(LED_L6, LOW ); 
  
  ets_delay_us(10);
 
  switch(lnum){
    case 1 : digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, HIGH);digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 2 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, HIGH);digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 3 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, HIGH);digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 4 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, HIGH);digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 5 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);digitalWrite(LED_L1, HIGH);digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 

    case 6 : digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, HIGH);digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 7 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, HIGH);digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 8 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, HIGH);digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 9 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, HIGH);digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 10: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, HIGH);digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
            
    case 11: digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, HIGH);digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 12: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, HIGH);digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 13: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, HIGH);digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 14: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, HIGH);digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 15: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, HIGH);digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 

    case 16: digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, HIGH);digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 17: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, HIGH);digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 18: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, HIGH);digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 19: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, HIGH);digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 20: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, HIGH);digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 

    case 21: digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, HIGH);digitalWrite(LED_L6, LOW );break; 
    case 22: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, HIGH);digitalWrite(LED_L6, LOW );break; 
    case 23: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, HIGH);digitalWrite(LED_L6, LOW );break; 
    case 24: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, HIGH);digitalWrite(LED_L6, LOW );break; 
    case 25: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, HIGH);digitalWrite(LED_L6, LOW );break; 

    case 26: digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, HIGH);break; 
    case 27: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, HIGH);break; 
    case 28: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, HIGH);break; 
    case 29: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, HIGH);break; 
    case 30: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, HIGH);break; 

    default: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW ); 
  }

  ets_delay_us(5);

  switch(nc){
    case COL_BK: digitalWrite(LED_R, HIGH);digitalWrite(LED_G, HIGH);digitalWrite(LED_B, HIGH);break; 
    case COL_RD: digitalWrite(LED_R, LOW );digitalWrite(LED_G, HIGH);digitalWrite(LED_B, HIGH);break; 
    case COL_YL: digitalWrite(LED_R, LOW );digitalWrite(LED_G, LOW );digitalWrite(LED_B, HIGH);break; 
    case COL_GR: digitalWrite(LED_R, HIGH);digitalWrite(LED_G, LOW );digitalWrite(LED_B, HIGH);break; 
    case COL_SB: digitalWrite(LED_R, HIGH);digitalWrite(LED_G, LOW );digitalWrite(LED_B, LOW );break; 
    case COL_WH: digitalWrite(LED_R, LOW );digitalWrite(LED_G, LOW );digitalWrite(LED_B, LOW );break; 
    case COL_PR: digitalWrite(LED_R, LOW );digitalWrite(LED_G, HIGH);digitalWrite(LED_B, LOW );break; 
    case COL_BL: digitalWrite(LED_R, HIGH);digitalWrite(LED_G, HIGH);digitalWrite(LED_B, LOW );break; 
    default:digitalWrite(LED_R, HIGH);digitalWrite(LED_G, HIGH);digitalWrite(LED_B, HIGH);
  }  

  ets_delay_us(10);
  
}

void led_proc(int nh,int nl,int nc)
{
  switch(nc){
    case COL_BK: digitalWrite(LED_R, HIGH);digitalWrite(LED_G, HIGH);digitalWrite(LED_B, HIGH);break; 
    case COL_RD: digitalWrite(LED_R, LOW );digitalWrite(LED_G, HIGH);digitalWrite(LED_B, HIGH);break; 
    case COL_YL: digitalWrite(LED_R, LOW );digitalWrite(LED_G, LOW );digitalWrite(LED_B, HIGH);break; 
    case COL_GR: digitalWrite(LED_R, HIGH);digitalWrite(LED_G, LOW );digitalWrite(LED_B, HIGH);break; 
    case COL_SB: digitalWrite(LED_R, HIGH);digitalWrite(LED_G, LOW );digitalWrite(LED_B, LOW );break; 
    case COL_WH: digitalWrite(LED_R, LOW );digitalWrite(LED_G, LOW );digitalWrite(LED_B, LOW );break; 
    case COL_PR: digitalWrite(LED_R, LOW );digitalWrite(LED_G, HIGH);digitalWrite(LED_B, LOW );break; 
    case COL_BL: digitalWrite(LED_R, HIGH);digitalWrite(LED_G, HIGH);digitalWrite(LED_B, LOW );break; 

    default:digitalWrite(LED_R, HIGH);digitalWrite(LED_G, HIGH);digitalWrite(LED_B, HIGH);
  }  
  
  switch(nl){
    case 0: digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 1: digitalWrite(LED_L1, HIGH);digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 2: digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, HIGH);digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 3: digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, HIGH);digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 4: digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, HIGH);digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 5: digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, HIGH);digitalWrite(LED_L6, LOW );break; 
    case 6: digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, HIGH);break; 
    default:digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW ); 
  }
  switch(nh){
    case 0: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );break; 
    case 1: digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );break; 
    case 2: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );break; 
    case 3: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );break; 
    case 4: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );break; 
    case 5: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);break; 
    default:digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );
  }
}

void led_blink_int(void *arg)
{
  float t_angle ;
  uint16_t idx ; 
  int i ; 
  int premic ; 
  
  while(1){  
    for(i=0;i<DOTSUU;i++){
      LED_proc_sq(i+1,on_led[i]);   
    }
  }
}

// the setup function runs once when you press reset or power the board
void setup() 
{
  int i ; 
  
  Serial.begin(115200);
  //while (!Serial);             // Leonardo: wait for serial monitor
//  Serial.println("\n Light Stick Demo");

  for(i=0;i<DOTSUU;i++){
      on_led[i] = 0 ; 
  }
  on_led[0] = COL_BL ;
  gyro_init();
  gryo_bias();

  // initialize digital pin LED_BUILTIN as an output.
//  pinMode(LED_PIN, OUTPUT);// gpio_set_pull_mode(LED_PIN,GPIO_FLOATING); 
  pinMode(LED_L1, OUTPUT);//  gpio_set_pull_mode(LED_L1,GPIO_FLOATING); 
  pinMode(LED_L2, OUTPUT);//  gpio_set_pull_mode(LED_L2 ,GPIO_FLOATING);
  pinMode(LED_L3, OUTPUT);//  gpio_set_pull_mode(LED_L3,GPIO_FLOATING); 
  pinMode(LED_L4, OUTPUT);//  gpio_set_pull_mode(LED_L4,GPIO_FLOATING); 
  pinMode(LED_L5, OUTPUT);//  gpio_set_pull_mode(LED_L5,GPIO_FLOATING); 
  pinMode(LED_L6, OUTPUT);//  gpio_set_pull_mode(LED_L6,GPIO_FLOATING); 
  pinMode(LED_H1, OUTPUT);//  gpio_set_pull_mode(LED_H1,GPIO_FLOATING); 
  pinMode(LED_H2, OUTPUT);//  gpio_set_pull_mode(LED_H2 ,GPIO_FLOATING);
  pinMode(LED_H3, OUTPUT);//  gpio_set_pull_mode(LED_H3 ,GPIO_FLOATING);
  pinMode(LED_H4, OUTPUT);//  gpio_set_pull_mode(LED_H4,GPIO_FLOATING); 
  pinMode(LED_H5, OUTPUT);//  gpio_set_pull_mode(LED_H5,GPIO_FLOATING); 
  pinMode(LED_R, OUTPUT);//   gpio_set_pull_mode(LED_R,GPIO_FLOATING); 
  pinMode(LED_G, OUTPUT);//   gpio_set_pull_mode(LED_G,GPIO_FLOATING); 
  pinMode(LED_B, OUTPUT);//   gpio_set_pull_mode(LED_B,GPIO_FLOATING); 

  ledcSetup(LED_CH,12800,8);
  ledcAttachPin(LED_PIN,LED_CH);

#ifdef NETON  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      if(Serial)
        Serial.print(".");
  }
  server.begin();
  server.setNoDelay(true);

  if(Serial){
    Serial.print("Ready! Use 'telnet ");
    Serial.print(WiFi.localIP());
    Serial.println(" 23' to connect");
  }
  xTaskCreatePinnedToCore(  telnetserver, "TELNET_TASK", 4096, NULL, 1, NULL, 0);
#endif 

  // ウォッチドッグ停止
  disableCore0WDT();
//  disableCore1WDT(); 
  xTaskCreatePinnedToCore(  gyro_i2c, "I2C_TASK", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(  led_blink_int, "LED_TASK", 4096, NULL, 1, NULL, 0);

//  digitalWrite( LED_PIN, LOW);
  ledcWrite(LED_CH,128);
}  


void sendmessage(uint8_t *sbuf,size_t len)
{
  int i ;
  for(i = 0; i < MAX_SRV_CLIENTS; i++){
    if (serverClients[i] && serverClients[i].connected()){
      serverClients[i].write(sbuf, len);
    }
  }
}
    
void telnetserver(void *arg)
{
  int i ;
  while(1){
    if(WiFi.status() == WL_CONNECTED){
      if (server.hasClient()){
        for(i = 0; i < MAX_SRV_CLIENTS; i++){
          //find free/disconnected spot
          if (!serverClients[i] || !serverClients[i].connected()){
            if(serverClients[i]) 
              serverClients[i].stop();
            serverClients[i] = server.available();
            if(Serial){
              if (!serverClients[i]) 
                Serial.println("available broken");
              Serial.print("New client: ");
              Serial.print(i); 
              Serial.print(' ');
              Serial.println(serverClients[i].remoteIP());
            }
            break;
          }
        }
        if (i >= MAX_SRV_CLIENTS) {
          //no free/disconnected spot so reject
          server.available().stop();
        }
      }
    }else{
      if(Serial)
        Serial.println("WiFi not connected!");
      for(i = 0; i < MAX_SRV_CLIENTS; i++) {
        if (serverClients[i]) serverClients[i].stop();
      }
      delay(1000);    
    }
    delay(50);
  }
}

void log_print(void *arg)
{
  uint32_t t_tick ;
  float t_angle ;
  int i ;

  t_tick = tick ; 
  t_angle = angle ;
  if(Serial){
    Serial.print(" tick = "); 
    Serial.print(t_tick,DEC); 
    Serial.print(" del_t = "); 
    Serial.print(del_micros,DEC); 
    Serial.print(" angle ="); 
    Serial.print(t_angle,2);
      
    Serial.print(" angvel = "); 
    for(i=0;i<HIS_LEN;i++){
      Serial.print(angvel[i],3);
      Serial.print(" "); 
    }
    
    Serial.println();
  }
}


// the loop function runs over and over again forever
void loop() {
#ifdef NETON  
  int i;
  char sstr[4096];
  char stmp[256];
  sprintf(sstr, "del_t, %d, angle, %lf, angvel, ", del_micros, angle);
  
  for (i = 0; i < HIS_LEN; i++) {
    sprintf(stmp, "%.2f,", angvel[i]);
    strcat(sstr, stmp);
  }

  strcat(sstr, "\n\r");
  sendmessage((uint8_t*)sstr, strlen(sstr));
  sprintf(sstr,"");
//  Serial.print(sstr);
  
#else    
  log_print(NULL);
#endif 
  delay(10);
  static int pre_millis = 0 ;
  static int blinkcnt = 0 ; 
  int cur_millis = millis();
  if( (cur_millis - pre_millis) > 200 ){
    pre_millis = cur_millis ; 
    blinkcnt++; 
//    Serial.println(blinkcnt%2);
    
    if((blinkcnt % 2) == 0 )
//      digitalWrite( LED_PIN, HIGH);
      ledcWrite(LED_CH,128);
    else
//      digitalWrite( LED_PIN, LOW);
      ledcWrite(LED_CH,255);
  }
}
