// 20250909 Light Stick with ESP32/MPU6050 V4.3 
#include "mpu6050.h"
#include "CharaData.h"
#include <Wire.h>
#include <rom/ets_sys.h>

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

#define LED_PIN_NUM GPIO_NUM_0

#define LED_L1_NUM  GPIO_NUM_5  
#define LED_L2_NUM  GPIO_NUM_18
#define LED_L3_NUM  GPIO_NUM_23
#define LED_L4_NUM  GPIO_NUM_4
#define LED_L5_NUM  GPIO_NUM_15
#define LED_L6_NUM  GPIO_NUM_19

#define LED_H1_NUM  GPIO_NUM_33
#define LED_H2_NUM  GPIO_NUM_25
#define LED_H3_NUM  GPIO_NUM_14
#define LED_H4_NUM  GPIO_NUM_26
#define LED_H5_NUM  GPIO_NUM_27

#define LED_R_NUM   GPIO_NUM_2
#define LED_G_NUM   GPIO_NUM_13
#define LED_B_NUM   GPIO_NUM_12

#define RET_RANGE   (-429)  // -30 dps

#define LED_PERIOD  (100L)
#define GYRO_PERIOD (1000L)
#define LOG_PERIOD  (100000L)

#define AVE_LEN   9
#define HIS_LEN   81

#define INTERRUPT_PIN 35 

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

void gyro_int(void);
void led_blink_int(void);
void log_print(void *arg);

volatile float angle = 0.0; 
volatile int32_t z_data;
volatile float z_bias; 
volatile uint32_t tick = 0 ;
volatile uint8_t on_led[DOTSUU]; 
volatile uint32_t del_micros ; 

uint16_t tz_data[AVE_LEN];
float angvel[HIS_LEN];
uint32_t dtime[HIS_LEN];

void gyro_write(uint8_t addr,uint8_t dat)
{
  Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
  Wire.write(addr);   
  Wire.write(dat);
  Wire.endTransmission();
}
 
uint8_t gyro_read(uint8_t addr)
{
  Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
  Wire.write(addr);   
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_DEFAULT_ADDRESS,(uint8_t)1);
  if (! Wire.available()) 
    return -1;
  return (Wire.read());      
}

void mpu_setup()
{
  int ret ;
  uint8_t mpu_adr = 0;
  uint8_t mpu_reg = 0;
  char str[64];
  Wire.setClock(400000L);
  Wire.begin(21, 22); // ESP32 SDA,SCL

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  pinMode(INTERRUPT_PIN, INPUT);

    ret = gyro_read(MPU6050_RA_WHO_AM_I);

    sprintf(str,"WHOAMI register = 0x%X \n\r", ret);
    Serial.print(str);

    if(ret == I_AM_MPU6050) {
      Serial.print("MPU6050 is found \n\r");
    }else{
      Serial.print("no MPU9250 is found \n\r");
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
        ret = ret & ~0x18; // Clear FS bits [4:3]
        mpu_reg = ret | (MPU6050_GYRO_FS_2000<<3); // Set full scale range for the gyro
        gyro_write(mpu_adr,mpu_reg);
        ret = gyro_read(mpu_adr);

    }while(ret != mpu_reg);
    delay(10);
 
    Serial.print("I did it \n\r");
}

#define BIAS_NUM 32
#if 1 
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
#endif  

#if 1
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
#endif 

void LED_proc_sq(int lnum,int nc)
{

  digitalWrite(LED_L1, LOW );
  digitalWrite(LED_L2, LOW );
  digitalWrite(LED_L3, LOW );
  digitalWrite(LED_L4, LOW );
  digitalWrite(LED_L5, LOW );
  digitalWrite(LED_L6, LOW );
  
  digitalWrite(LED_H1, LOW );
  digitalWrite(LED_H2, LOW );
  digitalWrite(LED_H3, LOW );
  digitalWrite(LED_H4, LOW );
  digitalWrite(LED_H5, LOW );

  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
 
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


void log_print(void *arg)
{
  uint32_t t_tick ;
  float t_angle ;
  while(1){
    t_tick = tick ; 
    t_angle = angle ;
  
    Serial.print(" tick = "); 
    Serial.print(t_tick,DEC); 
    Serial.print(" angle ="); 
    Serial.println(t_angle,2);
    delay(10);
  }
}

// the setup function runs once when you press reset or power the board
void setup() 
{
  int i ; 
  
  Serial.begin(115200);
//  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\n Light Stick Demo");
  
  for(i=0;i<DOTSUU;i++){
      on_led[i] = 0 ; 
  }
  on_led[0] = COL_BL ;
  mpu_setup();
  gryo_bias();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_PIN, OUTPUT);// gpio_set_pull_mode(LED_PIN_NUM,GPIO_PULLDOWN_ONLY); 

  pinMode(LED_L1, OUTPUT);//  gpio_set_pull_mode(LED_L1_NUM,GPIO_PULLDOWN_ONLY); 
  pinMode(LED_L2, OUTPUT);//  gpio_set_pull_mode(LED_L2_NUM,GPIO_PULLDOWN_ONLY);
  pinMode(LED_L3, OUTPUT);//  gpio_set_pull_mode(LED_L3_NUM,GPIO_PULLDOWN_ONLY); 
  pinMode(LED_L4, OUTPUT);//  gpio_set_pull_mode(LED_L4_NUM,GPIO_PULLDOWN_ONLY); 
  pinMode(LED_L5, OUTPUT);//  gpio_set_pull_mode(LED_L5_NUM,GPIO_PULLDOWN_ONLY); 
  pinMode(LED_L6, OUTPUT);//  gpio_set_pull_mode(LED_L6_NUM,GPIO_PULLDOWN_ONLY); 
  
  pinMode(LED_H1, OUTPUT);//  gpio_set_pull_mode(LED_H1_NUM,GPIO_PULLDOWN_ONLY); 
  pinMode(LED_H2, OUTPUT);//  gpio_set_pull_mode(LED_H2_NUM,GPIO_PULLDOWN_ONLY);
  pinMode(LED_H3, OUTPUT);//  gpio_set_pull_mode(LED_H3_NUM,GPIO_PULLDOWN_ONLY);
  pinMode(LED_H4, OUTPUT);//  gpio_set_pull_mode(LED_H4_NUM,GPIO_PULLDOWN_ONLY); 
  pinMode(LED_H5, OUTPUT);//  gpio_set_pull_mode(LED_H5_NUM,GPIO_PULLDOWN_ONLY); 

  pinMode(LED_R, OUTPUT);//   gpio_set_pull_mode(LED_R_NUM,GPIO_PULLDOWN_ONLY); 
  pinMode(LED_G, OUTPUT);//   gpio_set_pull_mode(LED_G_NUM,GPIO_PULLDOWN_ONLY); 
  pinMode(LED_B, OUTPUT);//   gpio_set_pull_mode(LED_B_NUM,GPIO_PULLDOWN_ONLY); 
  
  // ウォッチドッグ停止
  disableCore0WDT();
  disableCore1WDT(); 
  xTaskCreatePinnedToCore(  mpu_gyro, "I2C_TASK", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(  led_blink_int, "LED_TASK", 4096, NULL, 1, NULL, 0);
//  xTaskCreatePinnedToCore(  log_print, "LOG_TASK", 4096, NULL, 1, NULL, 1);
}

void loop() {

  static uint8_t l_ledcnt = 0 ;
  static uint8_t l_ledcol = 0 ; 
  
  uint32_t t_tick ;
  float t_angle ;
  t_tick = tick ; 
  t_angle = angle ;

  int i ;

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
}
