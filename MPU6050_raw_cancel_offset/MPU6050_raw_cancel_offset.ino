/*
  MPU6050 Raw

  A code for obtaining raw data from the MPU6050 module with the option to
  modify the data output format.

  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki
*/
#include "I2Cdev.h"
#include "MPU6050.h"

/* MPU6050 default I2C address is 0x68*/
//MPU6050 mpu;
MPU6050 mpu(0x69);         //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/* OUTPUT FORMAT DEFINITION----------------------------------------------------------------------------------
- Use "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated list of the accel 
X/Y/Z and gyro X/Y/Z values in decimal. Easy to read, but not so easy to parse, and slower over UART.

- Use "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit binary, one right after the other. 
As fast as possible without compression or data loss, easy to parse, but impossible to read for a human. 
This output format is used as an output.
--------------------------------------------------------------------------------------------------------------*/ 
#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO
#define LED_PIN 32

int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t o_ax, o_ay, o_az;
int16_t o_gx, o_gy, o_gz;

bool blinkState;

void setup() {
  /*--Start I2C interface--*/
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(21,22); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200); //Initializate Serial wo work well at 8MHz/16MHz

  /*Initialize device and check connection*/ 
  Serial.println("Initializing MPU...");
  mpu.initialize();
  Serial.println("Testing MPU6050 connection...");
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else{
    Serial.println("MPU6050 connection successful");
  }

  /* Use the code below to change accel/gyro offset values. Use MPU6050_Zero to obtain the recommended offsets */ 
  int32_t ax_sum = 0,ay_sum, az_sum=0; 
  int32_t gx_sum = 0,gy_sum, gz_sum=0; 
  for(int i=0;i<100;i++){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_sum += ax ;ay_sum += ay ;az_sum += az ;
    gx_sum += gx ;gy_sum += gy ;gz_sum += gz ;
  }
  o_ax = ax_sum / 100 ;   o_ay = ay_sum / 100 ;   o_az = az_sum / 100 ; 
  o_gx = gx_sum / 100 ;   o_gy = gy_sum / 100 ;   o_gz = gz_sum / 100 ; 

  /*Print the defined offsets*/
  Serial.print("OFFSET begin\n");

  Serial.print("\t");

  Serial.print(o_ax);
  Serial.print("\t");
  Serial.print(o_ay); 
  Serial.print("\t");
  Serial.print(o_az);
  Serial.print("\t");
  Serial.print(o_gx); 
  Serial.print("\t");
  Serial.print(o_gy);
  Serial.print("\t");
  Serial.print(o_gz);
  Serial.print("\n");
  
  Serial.print("OFFSET end\n");

  /*Configure board LED pin for output*/ 
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  /* Read raw accel/gyro data from the module. Other methods commented*/
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //mpu.getAcceleration(&ax, &ay, &az);
  //mpu.getRotation(&gx, &gy, &gz);

  /*Print the obtained data on the defined format*/
  #ifdef OUTPUT_READABLE_ACCELGYRO
    Serial.print("a/g:\t");
    Serial.print(ax-o_ax); Serial.print("\t");
    Serial.print(ay-o_ay); Serial.print("\t");
    Serial.print(az-o_az); Serial.print("\t");
    Serial.print(gx-o_gx); Serial.print("\t");
    Serial.print(gy-o_gy); Serial.print("\t");
    Serial.println(gz-o_gz);
  #endif

  #ifdef OUTPUT_BINARY_ACCELGYRO
    Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
    Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
    Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
    Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
    Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
    Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
  #endif

  /*Blink LED to indicate activity*/
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
