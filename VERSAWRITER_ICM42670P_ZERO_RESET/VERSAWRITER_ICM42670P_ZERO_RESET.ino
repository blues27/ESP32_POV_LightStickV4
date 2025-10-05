#include "ICM42670P.h"
#include "CharaData.h"
#include <SPI.h>
#include <APA102.h>
#include <esp_task_wdt.h>  // ウォッチドッグ関連の関数用

// SK9822 LED strip configuration using APA102 library
const uint8_t dataPin = 7;
const uint8_t clockPin = 9;
const uint16_t ledCount = 65;  // Half of 130 LEDs

// Create an object for writing to the LED strip
APA102<dataPin, clockPin> ledStrip;

// Color definitions for SK9822 using APA102 library
const rgb_color colors[] = {
  rgb_color(0, 0, 0),        // COL_BK - BLACK
  rgb_color(0, 0, 255),      // COL_BL - BLUE
  rgb_color(255, 0, 0),      // COL_RD - RED
  rgb_color(255, 0, 255),    // COL_PR - PURPLE
  rgb_color(0, 255, 0),      // COL_GR - GREEN
  rgb_color(0, 255, 255),    // COL_SB - SKY BLUE
  rgb_color(255, 255, 0),    // COL_YL - YELLOW
  rgb_color(255, 255, 255)   // COL_WH - WHITE
};

// Create a buffer for holding the LED colors
rgb_color ledColors[ledCount];

// Set the brightness (0-31)
const uint8_t brightness = 8;

// ICM42670P SPI configuration
#define ICM_CS_PIN      10
#define ICM_MOSI_PIN    11
#define ICM_MISO_PIN    13
#define ICM_SCLK_PIN    12
#define ICM_INT1_PIN    17
#define ICM_INT2_PIN    18
#define ICM_SPI_FREQ    1000000  // 1MHz

// ICM42670P instance
ICM42670 icm42670p(SPI,ICM_CS_PIN,ICM_SPI_FREQ);

#define ICM_SENS_SCALE_FACTOR 16.4

// Motion variables (protected by mutex)
volatile float angle = 0.0;
volatile float gyro_z = 0.0;
volatile uint32_t last_update_time = 0;
volatile uint8_t on_led[DOTSUU];
volatile bool led_update_flag = false;

// ターニングポイント検出用の変数
volatile float prev_gyro_z = 0.0;
volatile int direction_state = 0;  // -1: 左回転, 0: 停止, 1: 右回転
volatile int prev_direction_state = 0;
volatile uint32_t direction_change_time = 0;
volatile uint32_t last_direction_change_time = 0;
volatile bool direction_stable = false;
volatile int stable_count = 0;

// 調整可能なパラメータ
const float GYRO_THRESHOLD = 10.0;      // 回転検出の閾値 (dps)
const int STABLE_COUNT_THRESHOLD = 50;   // 方向が安定したと判断するサンプル数
const uint32_t MIN_DIRECTION_TIME = 200000; // 最小方向継続時間 (microseconds, 200ms)
const float RESET_ANGLE_THRESHOLD = 5.0; // リセット時の角度閾値

// Calibration variables
float gyro_z_bias = 0.0;
bool calibration_done = false;

// Task handles
TaskHandle_t gyroTaskHandle = NULL;

// Mutex for shared data protection
SemaphoreHandle_t dataMutex = NULL;

// Function declarations
void gyroUpdateTask(void *parameter);
void detectTurningPoint();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("VersaWriter ICM42670P Demo - Multi-thread version with Turning Point Reset");
  
  // Create mutex for data protection
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    Serial.println("Failed to create mutex");
    while (1);
  }
  
  // Initialize LED array
  for (int i = 0; i < DOTSUU; i++) {
    on_led[i] = 0;
  }
  
  // Initialize LED color buffer
  for (int i = 0; i < ledCount; i++) {
    ledColors[i] = rgb_color(0, 0, 0);
  }
  
  // Configure ICM42670P pins
  pinMode(ICM_CS_PIN, OUTPUT);
  pinMode(ICM_INT1_PIN, INPUT);
  pinMode(ICM_INT2_PIN, INPUT);
  
  // Initialize SPI for ICM42670P
  SPI.begin(ICM_SCLK_PIN, ICM_MISO_PIN, ICM_MOSI_PIN, ICM_CS_PIN);
  
  // Initialize ICM42670P
  if (icm42670p.begin() != 0) {
    Serial.println("ICM42670P initialization failed!");
    while (1);
  }
  
  Serial.println("ICM42670P initialized successfully");
  
  // Start & Configure accelerometer  
  icm42670p.startAccel(800,16);
  
  // Start & Configure gyroscope
  icm42670p.startGyro(800,2000);
  
  // Calibrate gyroscope
  calibrateGyro();
  
  // Clear all LEDs
  clearAllLEDs();
  
  last_update_time = micros();
  direction_change_time = micros();
  last_direction_change_time = micros();

  // Watchdog timerを完全に無効化
  esp_task_wdt_deinit();

  // Create gyro update task (medium priority, Core 1)
  xTaskCreatePinnedToCore(
    gyroUpdateTask,     // Task function
    "GYRO_UPDATE",      // Task name
    4096,              // Stack size
    NULL,              // Parameter
    2,                 // Priority (medium)
    &gyroTaskHandle,   // Task handle
    0                  // Core 1
  );
  
  if (gyroTaskHandle == NULL) {
    Serial.println("Failed to create tasks");
    while (1);
  }
  
  Serial.println("Tasks created successfully");
  Serial.println("Setup complete");
}

void calibrateGyro() {
  Serial.println("Calibrating gyroscope... Keep device stationary");
  
  const int num_samples = 1000;
  float sum_z = 0.0;
  
  for (int i = 0; i < num_samples; i++) {
    inv_imu_sensor_event_t imu_event;
    
    if (icm42670p.getDataFromRegisters(imu_event) == 0) {
      sum_z += imu_event.gyro[2];
    }
    
    delay(5);
    
    if (i % 100 == 0) {
      Serial.print(".");
    }
  }
  
  gyro_z_bias = sum_z / num_samples;
  calibration_done = true;
  
  Serial.println();
  Serial.print("Gyro Z bias: ");
  Serial.println(gyro_z_bias);
  Serial.println("Calibration complete");
}

void detectTurningPoint() {
  uint32_t current_time = micros();
  float current_gyro = gyro_z;
  
  // 現在の回転方向を判定
  int current_direction = 0;
  if (current_gyro > GYRO_THRESHOLD) {
    current_direction = 1;  // 右回転
  } else if (current_gyro < -GYRO_THRESHOLD) {
    current_direction = -1; // 左回転
  } else {
    current_direction = 0;  // 停止
  }
  
  // 方向の変化を検出
  if (current_direction != direction_state) {
    // 方向が変わった場合
    if (current_direction != 0) {
      // 新しい方向に変化
      stable_count = 1;
      direction_state = current_direction;
      direction_stable = false;
    } else {
      // 停止状態に変化
      stable_count = 0;
      direction_stable = false;
    }
  } else if (current_direction != 0) {
    // 同じ方向が継続
    stable_count++;
    
    // 方向が安定したと判断
    if (!direction_stable && stable_count >= STABLE_COUNT_THRESHOLD) {
      direction_stable = true;
      
      // 前回と異なる方向で安定した場合（ターニングポイント検出）
      if (prev_direction_state != 0 && 
          direction_state != prev_direction_state && 
          (current_time - last_direction_change_time) > MIN_DIRECTION_TIME) {
        
        // 角度リセット条件をチェック
        float abs_angle = fabs(angle);
        if (abs_angle > RESET_ANGLE_THRESHOLD) {
          Serial.print("Turning point detected! Direction: ");
          Serial.print(prev_direction_state);
          Serial.print(" -> ");
          Serial.print(direction_state);
          Serial.print(", Angle before reset: ");
          Serial.print(angle);
          
          // 角度をリセット
          angle = 0.0;
          
          Serial.println(", Angle reset to 0");
        }
        
        last_direction_change_time = current_time;
      }
      
      prev_direction_state = direction_state;
    }
  }
  
  prev_gyro_z = current_gyro;
}

void updateLEDs() {
  // Clear all LEDs first
  for (int i = 0; i < ledCount; i++) {
    ledColors[i] = rgb_color(0, 0, 0);
  }
  
  // Map character data to LED positions (thread-safe copy)
  uint8_t local_on_led[DOTSUU];
  
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(local_on_led, (void*)on_led, DOTSUU);
    xSemaphoreGive(dataMutex);
  }
  
  for (int i = 0; i < DOTSUU && i < ledCount; i++) {
    uint8_t color_index = local_on_led[i];
    
    if (color_index > 0 && color_index < 8) {
      ledColors[i] = colors[color_index];
    }
  }
  
  // Write to LED strip
  ledStrip.write(ledColors, ledCount, brightness);
}

void clearAllLEDs() {
  for (int i = 0; i < ledCount; i++) {
    ledColors[i] = rgb_color(0, 0, 0);
  }
  ledStrip.write(ledColors, ledCount, brightness);
}

void updateAngleFromGyro() {
  if (!calibration_done) return;
  
  inv_imu_sensor_event_t imu_event;
  
  if (icm42670p.getDataFromRegisters(imu_event) == 0) {
    uint32_t current_time = micros();
    float dt = (current_time - last_update_time) / 1000000.0;  // Convert to seconds
    
    if (dt > 0.0001) {  // Minimum time threshold
      // Apply bias correction
      float current_gyro_z = imu_event.gyro[2] - gyro_z_bias;
      
      // Update shared variables with mutex protection
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        gyro_z = -(current_gyro_z / ICM_SENS_SCALE_FACTOR);  // 符号を反転
        angle += gyro_z * dt;
        last_update_time = current_time;
        
        // ターニングポイント検出
        detectTurningPoint();
        
        xSemaphoreGive(dataMutex);
      }
    }
  }
}

void updateLEDPattern() {
  float local_angle;
  
  // Get current angle in thread-safe way
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
    local_angle = angle;
    xSemaphoreGive(dataMutex);
  } else {
    return; // Skip update if mutex not available
  }
  
  // Calculate new LED pattern
  uint8_t new_on_led[DOTSUU];
  for (int i = 0; i < DOTSUU; i++) {
    new_on_led[i] = 0;
  }
  
  // Map angle to character data column
  if ( (local_angle >= (float)MOJISUU_OFS) && (local_angle < (float)(MOJISUU + MOJISUU_OFS)) ) {
    uint16_t idx = (uint16_t)(local_angle - (float)MOJISUU_OFS);
    
    if (idx >= MOJISUU) {
      idx = MOJISUU - 1;
    }
    
    // Copy column data to LED array
    for (int i = 0; i < DOTSUU; i++) {
      new_on_led[i] = moji[(DOTSUU-1)-i][idx];
    }
  }
  
  // Update shared LED pattern with mutex protection
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
    memcpy((void*)on_led, new_on_led, DOTSUU);
    led_update_flag = true;
    xSemaphoreGive(dataMutex);
  }
}

// Gyro update task - runs every 1ms on Core 1
void gyroUpdateTask(void *parameter) {
  Serial.println("Gyro update task started on Core 1");
  
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(1);  // 1ms = 1000Hz
  
  while (1) {
    // Update angle from gyroscope
    updateAngleFromGyro();
    
    // Update LED pattern based on new angle
    updateLEDPattern();
    bool should_update = false;
    
    // Check if LED update is needed
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      should_update = led_update_flag;
      if (should_update) {
        led_update_flag = false;
      }
      xSemaphoreGive(dataMutex);
    }
    
    if (should_update) {
      updateLEDs();
    }

    // Wait for next cycle (1ms interval)
    vTaskDelayUntil(&lastWakeTime, frequency);
  }
}

void loop() {
  static uint32_t last_serial_output = 0;
  uint32_t current_time = millis();
  
  // Serial output for debugging (10Hz)
  if (current_time - last_serial_output >= 100) {
    float local_angle, local_gyro_z;
    int local_direction_state;
    bool local_direction_stable;
    uint8_t local_on_led[DOTSUU];
    
    // Get current values in thread-safe way
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      local_angle = angle;
      local_gyro_z = gyro_z;
      local_direction_state = direction_state;
      local_direction_stable = direction_stable;
      memcpy(local_on_led, (void*)on_led, DOTSUU);
      xSemaphoreGive(dataMutex);
      
      Serial.print("Angle: ");
      Serial.print(local_angle, 2);
      Serial.print("°, Gyro Z: ");
      Serial.print(local_gyro_z, 2);
      Serial.print(" dps, Dir: ");
      Serial.print(local_direction_state);
      Serial.print(" (");
      Serial.print(local_direction_stable ? "Stable" : "Unstable");
      Serial.print("), LEDs: ");
      
      for (int i = 0; i < DOTSUU; i++) {
        Serial.print(local_on_led[i]);
      }
      
      Serial.println();
    }
    
    last_serial_output = current_time;
  }
  
  // Main loop can do other tasks or just delay
  delay(10);
}