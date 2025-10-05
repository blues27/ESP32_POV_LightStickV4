#include "ICM42670P.h"
#include "CharaData.h"
#include <SPI.h>
#include <APA102.h>
#include <esp_task_wdt.h>  // ウォッチドッグ関連の関数用

#define HI_SPEED_UPDATE 1

// SK9822 LED strip configuration using APA102 library
const uint8_t dataPin = 7;
const uint8_t clockPin = 9;
const uint16_t ledCount = DOTSUU;  // Half of 130 LEDs

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
#define ICM_SPI_FREQ    10000000  // 10MHz

// ICM42670P instance
ICM42670 icm42670p(SPI,ICM_CS_PIN,ICM_SPI_FREQ);

#define ICM_SENS_SCALE_FACTOR 16.4

// Motion variables (protected by mutex)
volatile float angle = 90.0;  // 初期角度を90度に設定（右腕を前方に向けた状態）
volatile float raw_angle = 90.0;  // 正規化前の生の角度
volatile float angle_offset = 0.0;  // 角度オフセット（キャリブレーション用）
volatile float gyro_z = 0.0;
volatile float gyro_drift_compensation = 0.0;  // ドリフト補正値
volatile uint32_t last_update_time = 0;
volatile uint32_t last_drift_update = 0;  // ドリフト補正の更新時間
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

volatile bool stroke_active = false;
volatile float stroke_start_time = 0;

// 双方向対応のための新しい変数を追加
volatile bool swing_direction_right_to_left = true;  // true: 右→左, false: 左→右
volatile float swing_start_angle = 0.0;  // 振り開始時の角度
volatile bool swing_in_progress = false;  // 振り動作中フラグ
volatile uint32_t last_swing_change = 0;  // 最後の振り方向変更時刻

volatile uint16_t last_column_index = 0;  // 前回の振りで到達した最後の列インデックス
volatile uint16_t current_column_index = 0;  // 現在の列インデックス

// 定数定義
const float SWING_DIRECTION_THRESHOLD = 10.0;  // 振り方向変更の閾値(dps)
const uint32_t MIN_SWING_DURATION = 200000;    // 最小振り継続時間(200ms)
const float ANGLE_RESET_THRESHOLD = 5.0;       // 角度リセット閾値

// リアルタイム追従用の変数追加
volatile float prev_angle_for_led = -999.0;  // LED更新判定用の前回角度
volatile uint32_t angle_update_count = 0;    // 角度更新カウンタ（デバッグ用）
volatile uint32_t led_update_count = 0;      // LED更新カウンタ（デバッグ用）


// 調整可能なパラメータ
const float GYRO_THRESHOLD = 20.0;      // 回転検出の閾値 (dps) - より高く設定
const int STABLE_COUNT_THRESHOLD = 30;   // 方向が安定したと判断するサンプル数 - より短く
const uint32_t MIN_DIRECTION_TIME = 100000; // 最小方向継続時間 (microseconds, 100ms) - より短く
const float RESET_ANGLE_THRESHOLD = 2.0; // リセット時の角度閾値 - より小さく
const int RESET_DIRECTION_FROM = 1;      // リセットを行う方向転換 (1: 右→左, -1: 左→右)
const int RESET_DIRECTION_TO = -1;       // リセットを行う方向転換の到達方向

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

// setup関数での変更
void setupRealtimeTask() {
  // リアルタイム追従タスクを作成
  xTaskCreatePinnedToCore(
    gyroUpdateTaskRealtime,    // または gyroUpdateTaskPrecise
    "GYRO_REALTIME",
    16384,                     // 大きなスタックサイズ
    NULL,
    5,                         // 最高優先度
    &gyroTaskHandle,
    1                          // Core 1（高性能コア）
  );
  
  // 初期値設定
  prev_angle_for_led = -999.0;
  angle_update_count = 0;
  led_update_count = 0;
}

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
  
  // 初期角度を90度に設定（右腕を前方45度の位置）
  angle = 90.0;
  raw_angle = 90.0;
  last_drift_update = micros();
  
  last_update_time = micros();
  direction_change_time = micros();
  last_direction_change_time = micros();

  // Watchdog timerを完全に無効化
  esp_task_wdt_deinit();

  setupRealtimeTask();

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

// 角度変化検出用の関数
bool shouldUpdateLED(float current_angle, float previous_angle) {
  // 角度の1の位以上が変化したかチェック
  int current_degree = (int)current_angle;
  int previous_degree = (int)previous_angle;
  
  return (current_degree != previous_degree);
}

// 超高速角度更新タスク
void gyroUpdateTaskRealtime(void *parameter) {
  Serial.println("Realtime gyro update task started on Core 1 (Ultra High Frequency)");
  
  // マイクロ秒レベルの制御用
  uint32_t lastGyroUpdate = micros();
  uint32_t lastSerialOutput = millis();
  
  // 角度更新間隔（マイクロ秒）- 可能な限り高速
  const uint32_t GYRO_UPDATE_INTERVAL = 100;  // 100μs = 10kHz
  
  // シリアル出力間隔（ミリ秒）- デバッグ用なので低速
  const uint32_t SERIAL_OUTPUT_INTERVAL = 500;  // 500ms = 2Hz
  
  while (1) {
    uint32_t currentTime = micros();
    
    // 角度更新（超高速）
    if (currentTime - lastGyroUpdate >= GYRO_UPDATE_INTERVAL) {
      // ジャイロスコープから角度を更新
      updateAngleFromGyro();
      
      // 角度が1度以上変化した場合のみLED更新
      float current_angle;
      if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {  // ノンブロッキング
        current_angle = angle;
        angle_update_count++;
        xSemaphoreGive(dataMutex);
        
        // LED更新が必要かチェック
        if (shouldUpdateLED(current_angle, prev_angle_for_led)) {
          // LEDパターンを更新
          updateLEDPattern();
          
          bool should_update = false;
          if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {
            should_update = led_update_flag;
            if (should_update) {
              led_update_flag = false;
              led_update_count++;
            }
            xSemaphoreGive(dataMutex);
          }
          
          if (should_update) {
            updateLEDs();
            prev_angle_for_led = current_angle;
          }
        }
      }
      
      lastGyroUpdate = currentTime;
    }
    
    // シリアル出力（低速・デバッグ用）
    uint32_t currentTimeMs = millis();
    if (currentTimeMs - lastSerialOutput >= SERIAL_OUTPUT_INTERVAL) {
      outputDebugInfo();
      lastSerialOutput = currentTimeMs;
    }
    
    // 他のタスクにCPU時間を短時間譲る
    taskYIELD();
  }
}

// より精密な角度変化検出版
void gyroUpdateTaskPrecise(void *parameter) {
  Serial.println("Precise angle tracking task started on Core 1");
  
  uint32_t lastUpdate = micros();
  const uint32_t UPDATE_INTERVAL = 50;  // 50μs = 20kHz
  
  // 角度変化の閾値
  const float ANGLE_CHANGE_THRESHOLD = 0.5;  // 0.5度の変化でLED更新
  
  while (1) {
    uint32_t currentTime = micros();
    
    if (currentTime - lastUpdate >= UPDATE_INTERVAL) {
      // 高速角度更新
      updateAngleFromGyro();
      
      // 角度変化をチェック
      if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {
        float current_angle = angle;
        angle_update_count++;
        xSemaphoreGive(dataMutex);
        
        // より精密な角度変化検出
        if (fabs(current_angle - prev_angle_for_led) >= ANGLE_CHANGE_THRESHOLD) {
          updateLEDPattern();
          
          bool should_update = false;
          if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {
            should_update = led_update_flag;
            if (should_update) {
              led_update_flag = false;
              led_update_count++;
            }
            xSemaphoreGive(dataMutex);
          }
          
          if (should_update) {
            updateLEDs();
            prev_angle_for_led = current_angle;
          }
        }
      }
      
      lastUpdate = currentTime;
    }
    
    // 最小限のyield
    if ((currentTime % 1000) == 0) {  // 1ms毎に1回だけyield
      taskYIELD();
    }
  }
}

// 最適化されたupdateAngleFromGyro関数
void updateAngleFromGyro() {
  if (!calibration_done) return;
  
  // 静的変数でオーバーヘッドを削減
  static inv_imu_sensor_event_t imu_event;
  static uint32_t last_sensor_read = 0;
  static uint32_t sensor_read_interval = 200;  // 200μs毎にセンサー読み取り
  
  uint32_t current_time = micros();
  
  // センサー読み取り頻度を制限（SPI通信のボトルネック対策）
  if (current_time - last_sensor_read >= sensor_read_interval) {
    if (icm42670p.getDataFromRegisters(imu_event) != 0) {
      return;  // センサー読み取り失敗時は早期リターン
    }
    last_sensor_read = current_time;
  } else {
    return;  // センサー読み取りスキップ時は早期リターン
  }
  
  float dt = (current_time - last_update_time) / 1000000.0;
  
  if (dt > 0.00001) {  // 最小時間閾値を小さく
    float current_gyro_z = imu_event.gyro[2] - gyro_z_bias;
    
    // ドリフト補正の更新頻度を下げる
    static uint32_t drift_update_counter = 0;
    if (++drift_update_counter >= 50000) {  // 約5秒毎
      if (fabs(current_gyro_z) < 5.0) {
        gyro_drift_compensation = gyro_drift_compensation * 0.99 + current_gyro_z * 0.01;
      }
      drift_update_counter = 0;
    }
    
    // ミューテックスの取得時間を最小化
    if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {  // ノンブロッキング
      gyro_z = -(current_gyro_z - gyro_drift_compensation) / ICM_SENS_SCALE_FACTOR;
      
      // 微小値のゼロクランプ
      if (fabs(gyro_z) < 0.3) {
        gyro_z = 0.0;
      }
      
      detectSwingDirection();
      
      if (swing_in_progress) {
        if (swing_direction_right_to_left) {
          if (gyro_z > 0) {
            raw_angle += gyro_z * dt;
          }
        } else {
          if (gyro_z < 0) {
            raw_angle += (-gyro_z) * dt;
          }
        }
      }
      
      // 角度範囲制限
      if (raw_angle < 0.0) raw_angle = 0.0;
      else if (raw_angle > 180.0) raw_angle = 180.0;
      

      angle = raw_angle;
      last_update_time = current_time;
      
      xSemaphoreGive(dataMutex);
    }
  }
}

// デバッグ情報出力（低速）
void outputDebugInfo() {
  static uint32_t last_output = 0;
  static uint32_t last_angle_count = 0;
  static uint32_t last_led_count = 0;
  
  uint32_t current_time = millis();
  uint32_t elapsed = current_time - last_output;
  
  if (elapsed >= 500) {  // 500ms毎
    float local_angle, local_gyro_z;
    uint32_t local_angle_count, local_led_count;
    bool local_swing_direction, local_swing_in_progress;
    uint16_t local_current_column, local_last_column;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      local_angle = angle;
      local_gyro_z = gyro_z;
      local_angle_count = angle_update_count;
      local_led_count = led_update_count;
      
      local_swing_direction = swing_direction_right_to_left;
      local_swing_in_progress = swing_in_progress;
      local_current_column = current_column_index;
      local_last_column = last_column_index;
      
      xSemaphoreGive(dataMutex);
      
      // 更新レート計算
      uint32_t angle_rate = (local_angle_count - last_angle_count) * 1000 / elapsed;
      uint32_t led_rate = (local_led_count - last_led_count) * 1000 / elapsed;
      
      Serial.print("Angle: ");
      Serial.print(local_angle, 1);
      Serial.print("°, Gyro: ");
      Serial.print(local_gyro_z, 1);
      Serial.print(" dps, Rates - Angle: ");
      Serial.print(angle_rate);
      Serial.print("Hz, LED: ");
      Serial.print(led_rate);
      Serial.print("Hz");
      
      Serial.print(", Dir: ");
      Serial.print(local_swing_direction ? "R→L" : "L→R");
      Serial.print(", Active: ");
      Serial.print(local_swing_in_progress ? "Y" : "N");
      Serial.print(", Col: ");
      Serial.print(local_current_column);
     
      Serial.println();
      
      last_output = current_time;
      last_angle_count = local_angle_count;
      last_led_count = local_led_count;
    }
  }
}

void detectSwingDirection() {
  uint32_t current_time = micros();
  float current_gyro = gyro_z;
  
  // 振り方向の検出
  bool new_direction_right_to_left = (current_gyro > 0);  // 正の角速度 = 右→左
  
  // 明確な角速度変化があった場合の振り方向変更検出
  if (fabs(current_gyro) > SWING_DIRECTION_THRESHOLD) {
    
    // 振り方向の変更を検出
    if (swing_direction_right_to_left != new_direction_right_to_left && 
        (current_time - last_swing_change) > MIN_SWING_DURATION) {
      
      Serial.print("Swing direction changed: ");
      Serial.print(swing_direction_right_to_left ? "R→L" : "L→R");
      Serial.print(" to ");
      Serial.print(new_direction_right_to_left ? "R→L" : "L→R");
      
      // 前回の振りで到達した列インデックスを保存
      if (swing_direction_right_to_left) {
        // 右→左の振りが終わった場合、現在の列インデックスを保存
        last_column_index = current_column_index;
        Serial.print(", Last column: ");
        Serial.print(last_column_index);
      }
      
      // 振り方向を更新
      swing_direction_right_to_left = new_direction_right_to_left;
      
      // 角度をリセット
      raw_angle = 0.0;
      swing_start_angle = 0.0;
      swing_in_progress = true;
      last_swing_change = current_time;
      
      Serial.print(", Reset angle to 0");
      Serial.println();
    }
  }
  
  // 振り動作の継続判定
  if (fabs(current_gyro) > 1.0) {  // 1dps以上で振り動作継続
    swing_in_progress = true;
  } else if (fabs(current_gyro) < 0.5) {  // 0.5dps以下で振り停止
    swing_in_progress = false;
  }
}

void updateLEDPattern() {
  float local_angle;
  bool local_swing_direction;
  uint16_t local_last_column_index;
  
  // スレッドセーフな値の取得
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
    local_angle = angle;
    local_swing_direction = swing_direction_right_to_left;
    local_last_column_index = last_column_index;
    xSemaphoreGive(dataMutex);
  } else {
    return;
  }
  
  // LEDパターンの計算
  uint8_t new_on_led[DOTSUU];
  for (int i = 0; i < DOTSUU; i++) {
    new_on_led[i] = 0;
  }
  
  // 角度の正規化（0-180度）
  float display_angle = local_angle;
  if (display_angle < 0.0) {
    display_angle = 0.0;
  } else if (display_angle > 180.0) {
    display_angle = 180.0;
  }
  
  // 列インデックスの計算
  uint16_t idx = 0;
  
  if (local_swing_direction) {
    // 右→左の振り：0列目から順番に増加
    if (((uint16_t)(display_angle) >= MOJISUU_OFS) && 
        ((uint16_t)(display_angle) < (MOJISUU + MOJISUU_OFS))) {
      
      idx = (uint16_t)(display_angle) - MOJISUU_OFS;
      
      // 現在の列インデックスを更新
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        current_column_index = idx;
        xSemaphoreGive(dataMutex);
      }
      
      // 通常の順序でLEDパターンを設定
      for (int i = 0; i < DOTSUU; i++) {
        new_on_led[i] = moji[moji_idx[i]][idx];
      }
    }
  } else {
    // 左→右の振り：前回の最後の列から0列目に向かって減少
    if (((uint16_t)(display_angle) >= MOJISUU_OFS) && 
        ((uint16_t)(display_angle) < (MOJISUU + MOJISUU_OFS))) {
      
      // 角度を列インデックスにマッピング
      uint16_t angle_based_idx = (uint16_t)(display_angle) - MOJISUU_OFS;
      
      // 前回の最後の列から現在の角度に基づいて逆方向にマッピング
      if (local_last_column_index < MOJISUU) {
        // 進行度を計算（0.0〜1.0）
        float progress = (float)angle_based_idx / (float)(MOJISUU - 1);
        
        // 前回の最後の列から0列目に向かって減少
        idx = local_last_column_index - (uint16_t)(progress * local_last_column_index);
        
        // 範囲チェック
        if (idx >= MOJISUU) {
          idx = 0;
        }
      } else {
        idx = 0;
      }
      
      // 現在の列インデックスを更新
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        current_column_index = idx;
        xSemaphoreGive(dataMutex);
      }
      
      // LEDパターンを設定
      for (int i = 0; i < DOTSUU; i++) {
        new_on_led[i] = moji[moji_idx[i]][idx];
      }
    }
  }
  
  // 共有LEDパターンの更新
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
    memcpy((void*)on_led, new_on_led, DOTSUU);
    led_update_flag = true;
    xSemaphoreGive(dataMutex);
  }
}

void clearAllLEDs() {
  for (int i = 0; i < ledCount; i++) {
    ledColors[i] = rgb_color(0, 0, 0);
  }
  ledStrip.write(ledColors, ledCount, brightness);
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

// Gyro update task - runs every 1ms on Core 1
void gyroUpdateTask(void *parameter) {
  Serial.println("Gyro update task started on Core 1");
  
  TickType_t lastWakeTime = xTaskGetTickCount();
//  const TickType_t frequency = pdMS_TO_TICKS(1);  // 1ms = 1000Hz
  const TickType_t frequency = pdMS_TO_TICKS(0.5);  // 0.5ms = 2000Hz  
  
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

// デバッグ用のloop関数内の表示部分も更新
void loop() {
  static uint32_t last_serial_output = 0;
  uint32_t current_time = millis();
  
  if (current_time - last_serial_output >= 100) {
    float local_angle, local_gyro_z;
    bool local_swing_direction, local_swing_in_progress;
    uint16_t local_current_column, local_last_column;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      local_angle = angle;
      local_gyro_z = gyro_z;
      local_swing_direction = swing_direction_right_to_left;
      local_swing_in_progress = swing_in_progress;
      local_current_column = current_column_index;
      local_last_column = last_column_index;
      xSemaphoreGive(dataMutex);
      
      Serial.print("Angle: ");
      Serial.print(local_angle, 2);
      Serial.print("°, Gyro Z: ");
      Serial.print(local_gyro_z, 2);
      Serial.print(" dps, Direction: ");
      Serial.print(local_swing_direction ? "R→L" : "L→R");
      Serial.print(", Active: ");
      Serial.print(local_swing_in_progress ? "Yes" : "No");
      Serial.print(", Current Col: ");
      Serial.print(local_current_column);
      Serial.print(", Last Col: ");
      Serial.print(local_last_column);
      Serial.println();
    }
    
    last_serial_output = current_time;
  }
  
  delay(10);
}
