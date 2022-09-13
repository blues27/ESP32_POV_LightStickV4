#include "l3gd20.h"
#include "CharaData.h"
#include <Wire.h>

#define COL_BK  0    // 000 BLACK
#define COL_BL  1    // 001 BLUE
#define COL_RD  2    // 010 RED
#define COL_PR  3    // 011 PURPLE
#define COL_GR  4    // 100 GREEN
#define COL_SB  5    // 101 SKY BLUE
#define COL_YL  6    // 110 YELLOW
#define COL_WH  7    // 111 WHITE

#define LED_PIN 32
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

#define RET_RANGE   (-429)  // -30 dps

#define LED_PERIOD  (100L)
#define GYRO_PERIOD (1000L)
#define LOG_PERIOD  (100000L)

hw_timer_t *timerA = NULL;    // For calc angle 
hw_timer_t *timerL = NULL;    // For led control
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void gyro_int(void);
void led_blink_int(void);
void log_print(void *arg);

volatile float angle = 0.0; 
volatile int32_t z_data;
volatile float z_bias; 
volatile uint32_t tick = 0 ;
volatile uint8_t on_led[DOTSUU]; 
volatile uint32_t del_micros ; 

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
  Serial.print("WHOAMI register = 0x");Serial.println(ret,HEX);
  
  if((ret == I_AM_L3GD20)||(ret == I_AM_L3GD20H)) {
    Serial.println("L3GD20/H is found ");
    digitalWrite( LED_PIN, LOW);
  }else{
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
  Serial.printf("I did it ");
}

#define HIS_LEN   9 

float calc_inc(uint16_t *t_data)
{
  int16_t xx[HIS_LEN],yy[HIS_LEN];
  int32_t ave_xx,ave_yy, sum ; 
  for(i;i<HIS_LEN;
  
  
}

void gyro_i2c(void *arg)
{
  static uint16_t tz_data[HIS_LEN]={0,0,0,0,0,0,0,0,0};
  static uint32_t pre_micros = 0 ; 
  uint32_t cur_micros;
//  uint8_t dmy;
//  uint16_t dmy16;
  int32_t t_data;
  int32_t a_data;
  int32_t b_data;
  uint16_t idx ; 
  
  int16_t i;
  while(1){
    
    for(i=HIS_LEN-1;i>0;i--){
      tz_data[i] = tz_data[i-1] ;
    }
  
    tz_data[0] = (uint16_t)gyro_read(L3GD20_OUT_Z_H_ADDR);
    tz_data[0] = tz_data[0]<<8 ;
    tz_data[0] |= gyro_read(L3GD20_OUT_Z_L_ADDR);

    t_data = (int16_t)tz_data[0];

    for(i=1;i<HIS_LEN;i++){
      t_data += (int16_t)tz_data[i];
    }

    cur_micros = micros();
    del_micros = cur_micros - pre_micros ;
    pre_micros = cur_micros ; 

    portENTER_CRITICAL_ISR(&timerMux);  
    z_data = - t_data / HIS_LEN;
    
    angle += ((float)z_data*D2D_L3GD20-0.0 * z_bias)*0.000001*(float)del_micros ;

    if(

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
      
    portEXIT_CRITICAL_ISR(&timerMux);
  }

}

void IRAM_ATTR onTimerA()//void gyro_int(void)
{
  portENTER_CRITICAL_ISR(&timerMux);
  tick++;
  portEXIT_CRITICAL_ISR(&timerMux);
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
  //  portENTER_CRITICAL_ISR(&timerMux);
  //    t_angle = angle ;
  //  portEXIT_CRITICAL_ISR(&timerMux);  
    for(i=0;i<DOTSUU;i++){
//      premic = micros();
      LED_proc_sq(i+1,on_led[i]);   
//      del_micros = micros() - premic; 
    }
  }
}

#if 0 
/*****************************************************************************
 *                          Interrupt Service Routin                         *
 *****************************************************************************/
void IRAM_ATTR onTimerA(){
  // Increment the counter and set the time of ISR
  gyro_int();
}
 
void IRAM_ATTR onTimerL(){
  // Increment the counter and set the time of ISR
  led_blink_int();
}
#endif 

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
  pinMode(LED_PIN, OUTPUT);// gpio_set_pull_mode(LED_PIN,GPIO_FLOATING); 
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

  // ウォッチドッグ停止
  disableCore0WDT();
  //disableCore1WDT(); 
  xTaskCreatePinnedToCore(  gyro_i2c, "I2C_TASK", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(  led_blink_int, "LED_TASK", 4096, NULL, 1, NULL, 0);
//  xTaskCreatePinnedToCore(  log_print, "LOG_TASK", 4096, NULL, 1, NULL, 1);

  // Timer: interrupt time and event setting.
//  timerA = timerBegin(2, 80, true);
//  timerL = timerBegin(1, 80, true);

  // Attach onTimer function.
//  timerAttachInterrupt(timerA, &onTimerA, true);
//  timerAttachInterrupt(timerL, &onTimerL, true);

  // Set alarm to call onTimer function every second (value in microseconds).
//  timerAlarmWrite(timerA, 1000, true);// 1ms
//  timerAlarmWrite(timerL, 10000, true);// 1ms

  // Start an alarm
//  timerAlarmEnable(timerA);
//  timerAlarmEnable(timerL);
}

void log_print(void *arg)
{
  uint32_t t_tick ;
  float t_angle ;
  while(1){
//  portENTER_CRITICAL_ISR(&timerMux);
    t_tick = tick ; 
    t_angle = angle ;
//  portEXIT_CRITICAL_ISR(&timerMux);  
  
    Serial.print(" tick = "); 
    Serial.print(t_tick,DEC); 
    Serial.print(" angle ="); 
    Serial.println(t_angle,2);
    delay(10);
  }
}

// the loop function runs over and over again forever
void loop() {
  static uint8_t l_ledcnt = 0 ;
  static uint8_t l_ledcol = 0 ; 
  
  uint32_t t_tick ;
  float t_angle ;
  t_tick = tick ; 
  t_angle = angle ;

  int i ;

//  for(i=0;i<DOTSUU;i++){
//      if(i==l_ledcnt)
//        on_led[i] = l_ledcol+1 ;
//      else
//        on_led[i] = 0 ; 
//  }
 
//  l_ledcnt ++; 
//  if(l_ledcnt == DOTSUU) {
//    l_ledcol++; 
//    l_ledcol %= 7 ;
//  }
//  l_ledcnt = l_ledcnt % DOTSUU;  

  Serial.print(" l_ledcnt = "); 
  Serial.print(l_ledcnt,DEC); 
  Serial.print(" l_ledcol = "); 
  Serial.print(l_ledcol,DEC); 

  Serial.print(" tick = "); 
  Serial.print(t_tick,DEC); 
  Serial.print(" del_t = "); 
  Serial.print(del_micros,DEC); 
  Serial.print(" angle ="); 
  Serial.print(t_angle,2);
  Serial.print(" on_led ="); 
  
  for(i=0;i<DOTSUU;i++)
    Serial.print(on_led[i],DEC);
  
  Serial.println();
    
  delay(100);

#if 0 
  static uint8_t ledcnt = 0 ;

  LED_proc_sq(ledcnt+1,7);
  ledcnt ++; 
  ledcnt = ledcnt % DOTSUU;  
#endif 

#if 0
  uint8_t dmy ; 
  uint16_t dmy16 ; 
  uint16_t t_data ; 
  int16_t z_data ;
  float yawang = 0.0 ; 
  
  dmy=gyro_read(L3GD20_OUT_Z_L_ADDR);
  t_data = (0x00ff&dmy);
  Serial.printf("tdata = %04x \t",t_data ) ;
  dmy=gyro_read(L3GD20_OUT_Z_H_ADDR);
  dmy16 = dmy ;
  t_data |= (0xff00&(dmy16<<8));
  Serial.printf("tdata = %04x \t",t_data ) ;
  z_data = t_data; 
  Serial.printf("zdata = %d (%04x) \t",z_data,z_data ) ;

  yawang = z_data * D2D_L3GD20 ; 
  Serial.printf("yawang = %f \n\r",yawang) ;
#endif 
}
