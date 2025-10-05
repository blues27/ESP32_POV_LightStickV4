
#include "l3gd20.h"
#include "CharaData.h"
#include <Wire.h>
#include <WiFi.h>
#include <string.h>

#define RET_RANGE   (-429)  // -30 dps

#define LED_PERIOD  (100L)
#define GYRO_PERIOD (1000L)
#define LOG_PERIOD  (100000L)

#define AVE_LEN   9
#define HIS_LEN   81

#define MAX_SRV_CLIENTS 1

const char* ssid = "HORNET";
const char* password = "password";

WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void gyro_int(void);
void log_print(void *arg);
void telnetserver(void *arg);
void sendmessage(uint8_t sbuf, size_t len);

volatile float angle = 0.0;
volatile int32_t z_data;
volatile float z_bias;
volatile uint32_t tick = 0 ;
volatile uint8_t on_led[DOTSUU];
volatile uint32_t del_micros ;

uint16_t tz_data[AVE_LEN];
float angvel[HIS_LEN];
uint32_t dtime[HIS_LEN];

void gyro_write(uint8_t addr, uint8_t dat)
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
  Wire.requestFrom((uint8_t)L3GD20_I2C_ADDR, (uint8_t)1);
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

  for (i = 0; i < BIAS_NUM; i++) {
    dmy = gyro_read(L3GD20_OUT_Z_L_ADDR);
    tz_data = (0x00ff & dmy);
    dmy = gyro_read(L3GD20_OUT_Z_H_ADDR);
    tz_data |= (0xff00 & (dmy << 8));
    z_data = -1 * (int16_t)tz_data;
    z_bias += (float)z_data * D2D_L3GD20;
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
  Serial.print("WHOAMI register = 0x"); Serial.println(ret, HEX);

  if ((ret == I_AM_L3GD20) || (ret == I_AM_L3GD20H)) {
    Serial.println("L3GD20/H is found ");
  } else {
    Serial.println("no L3GD20/H is found ");
    while (1);
    return ;
  }
  do {
    l3gd_adr = L3GD20_CTRL_REG1_ADDR ;
    l3gd_reg = L3GD20_Z_ENABLE | L3GD20_MODE_ACTIVE
               | L3GD20_OUTPUT_DATARATE_4 | L3GD20_BANDWIDTH_4;
    gyro_write(l3gd_adr, l3gd_reg);
    delay(10);

    l3gd_adr = L3GD20_CTRL_REG1_ADDR ;
    ret = gyro_read(l3gd_adr);
  } while (ret != l3gd_reg);
  do {
    l3gd_adr = L3GD20_CTRL_REG2_ADDR ;
    ret = gyro_read(l3gd_adr);

    l3gd_adr = L3GD20_CTRL_REG2_ADDR ;
    l3gd_reg = ret | L3GD20_HPM_NORMAL_MODE_RES | L3GD20_HPFCF_0;
    gyro_write(l3gd_adr, l3gd_reg);

    l3gd_adr = L3GD20_CTRL_REG2_ADDR ;
    ret = gyro_read(l3gd_adr);

  } while (ret != l3gd_reg);
  do {
    l3gd_adr = L3GD20_CTRL_REG4_ADDR ;
    l3gd_reg = L3GD20_BlockDataUpdate_Continous | L3GD20_BLE_LSB
               | L3GD20_FULLSCALE_2000;
    gyro_write(l3gd_adr, l3gd_reg);

    l3gd_adr = L3GD20_CTRL_REG4_ADDR ;
    ret = gyro_read(l3gd_adr);

  } while (ret != l3gd_reg);
  Serial.printf("I did it ");
}

float calc_inc(float *dat, int16_t n)
{
  float sum_u = 0.0, sum_l = 0.0 ;
  float ave_x, ave_y ;
  int i ;
  for (i = 0; i < n; i++) {
    sum_u += (float)dat[i] ;
    sum_l += (float)i ;
  }
  ave_x = sum_l / n ;
  ave_y = sum_u / n ;
  sum_l = 0.0 ; sum_u = 0.0 ;
  for (i = 0; i < n; i++) {
    sum_u += ((float)i - ave_x) * (dat[i] - ave_y) ;
    sum_l += ((float)i - ave_x) * ((float)i - ave_x) ;
  }
  return ((float)sum_u) / ((float)sum_l) ;
}
float inc1, inc2, inc3;

uint8_t calc_reg(float *av)
{
  float sec1[HIS_LEN / 3], sec2[HIS_LEN / 3], sec3[HIS_LEN / 3];
  //  float inc1, inc2, inc3;
  int i ;
  for (i = 0; i < HIS_LEN / 3; i++) {
    sec1[i] = (int16_t)av[i];
    sec2[i] = (int16_t)av[HIS_LEN / 3 + i];
    sec3[i] = (int16_t)av[2 * HIS_LEN / 3 + i];
  }
  inc1 = calc_inc(sec1, HIS_LEN / 3);
  inc2 = calc_inc(sec2, HIS_LEN / 3);
  inc3 = calc_inc(sec3, HIS_LEN / 3);
  if ((inc1 > 0) && (inc2 == 0) && (inc3 < 0)) {
    return 1 ;
  } else {
    return 0 ;
  }
}

// the setup function runs once when you press reset or power the board
void setup()
{
  int i ;
  for (i = 0; i < AVE_LEN; i++) {
    tz_data[i] = 0;
  }

  Serial.begin(115200);
  //while (!Serial);             // Leonardo: wait for serial monitor
  //  Serial.println("\n Light Stick Demo");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  server.begin();
  server.setNoDelay(true);

  Serial.print("Ready! Use 'telnet ");
  Serial.print(WiFi.localIP());
  Serial.println(" 23' to connect");

  gyro_init();
  gryo_bias();

  // ウォッチドッグ停止
  disableCore0WDT();
  //  disableCore1WDT();

  xTaskCreatePinnedToCore(  telnetserver, "TELNET_TASK", 4096, NULL, 1, NULL, 0);

}

void sendmessage(uint8_t *sbuf, size_t len)
{
  int i ;
  for (i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClients[i] && serverClients[i].connected()) {
      serverClients[i].write(sbuf, len);
    }
  }
}

void telnetserver(void *arg)
{
  int i ;
  while (1) {
    if (WiFi.status() == WL_CONNECTED) {
      if (server.hasClient()) {
        for (i = 0; i < MAX_SRV_CLIENTS; i++) {
          //find free/disconnected spot
          if (!serverClients[i] || !serverClients[i].connected()) {
            if (serverClients[i]) serverClients[i].stop();
            serverClients[i] = server.available();
            if (!serverClients[i]) Serial.println("available broken");
            Serial.print("New client: ");
            Serial.print(i); Serial.print(' ');
            Serial.println(serverClients[i].remoteIP());
            break;
          }
        }
        if (i >= MAX_SRV_CLIENTS) {
          //no free/disconnected spot so reject
          server.available().stop();
        }
      }
    } else {
      Serial.println("WiFi not connected!");
      for (i = 0; i < MAX_SRV_CLIENTS; i++) {
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

  Serial.print(" tick = ");
  Serial.print(t_tick, DEC);
  Serial.print(" del_t = ");
  Serial.print(del_micros, DEC);
  Serial.print(" angle =");
  Serial.print(t_angle, 2);
  /*
    Serial.print(" tz_data = ");
    for(i=0;i<HIS_LEN;i++){
      Serial.print((int16_t)tz_data[i],DEC);
      Serial.print(" ");
    }
  */
  Serial.println();

}

// the loop function runs over and over again forever
void loop() {
  static uint32_t pre_micros = 0 ;
  uint32_t cur_micros;
  int32_t t_data;
  int32_t a_data;
  int32_t b_data;
  uint16_t idx ;

  int16_t i;

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


#define ZEROSMP 15 
#define ZEROCHK 10

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

  char sstr[4096];
  char stmp[256];

  sprintf(sstr, "del_t, %d, angle, %lf, angvel, ", del_micros, angle);
  /*
  for (i = 0; i < HIS_LEN; i++) {
    sprintf(stmp, "%.2f,", angvel[i]);
    strcat(sstr, stmp);
  }
*/
  strcat(sstr, "\n\r");
  sendmessage((uint8_t*)sstr, strlen(sstr));
}
