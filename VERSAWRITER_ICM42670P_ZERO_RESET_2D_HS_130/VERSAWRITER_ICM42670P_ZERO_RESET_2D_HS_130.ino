#include "ICM42670P.h"
#include "CharaData.h"
#include <SPI.h>
#include <APA102.h>
#include <esp_task_wdt.h>  // ウォッチドッグ関連の関数用

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
// SPI設定の最適化
#define ICM_SPI_FREQ    20000000  // 20MHzに増加
#define LED_SPI_FREQ    20000000  // LEDも高速化

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
const float SWING_DIRECTION_THRESHOLD = 5.0;  // 振り方向変更の閾値(dps)
const uint32_t MIN_SWING_DURATION = 100000;    // 最小振り継続時間(200ms)
const float ANGLE_RESET_THRESHOLD = 5.0;       // 角度リセット閾値
const float SWING_ACTIVE_THRESHOLD = 0.5;      // 振り動作継続の閾値を緩く

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


// セットアップ関数の修正
void setupRealtimeTaskFixed() {
  Serial.println("Setting up fixed realtime task");
  
  // CPUクロックを最大に設定
  setCpuFrequencyMhz(240);
  
  // SPI速度設定
  SPI.setFrequency(10000000);  // 安定性重視で10MHz
  
  // タスク作成
  xTaskCreatePinnedToCore(
    gyroUpdateTaskOptimized,
    "GYRO_FIXED",
    32768,
    NULL,
    configMAX_PRIORITIES - 1,
    &gyroTaskHandle,
    1
  );
  
  // 初期値設定
  prev_angle_for_led = -999.0;
  angle_update_count = 0;
  led_update_count = 0;
  
  Serial.println("Fixed realtime task setup complete");
}
// DMA使用でCPU負荷軽減
void setupDMA() {
  // ESP32のDMA機能を使用してSPI転送を高速化
  SPI.setHwCs(true);
  SPI.setFrequency(20000000);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("VersaWriter ICM42670P Demo - Multi-thread version with Turning Point Reset");
  
  setupDMA();


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

  setupRealtimeTaskFixed();  // この行を変更


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



// より細かい角度変化でLED更新をトリガー
bool shouldUpdateLED(float current_angle, float previous_angle) {
  // 0.2度の変化でもLED更新（より滑らかな表示）
  return (fabs(current_angle - previous_angle) >= 0.2);
}

// 修正されたセンサー読み取りとLED更新タスク
void gyroUpdateTaskOptimized(void *parameter) {
  Serial.println("Fixed optimized gyro update task started on Core 1");
  
  uint32_t lastSensorRead = micros();
  uint32_t lastLEDUpdate = micros();
  uint32_t lastDebugOutput = millis();
  
  // タイミング調整
  const uint32_t SENSOR_READ_INTERVAL = 200;  // 200μs = 5kHz（より安定）
  const uint32_t LED_UPDATE_INTERVAL = 100;   // 100μs = 10kHz
  
  float interpolated_angle = 90.0;
  float angle_velocity = 0.0;
  float prev_sensor_angle = 90.0;
  
  // デバッグ用カウンタ
  uint32_t sensor_read_count = 0;
  uint32_t led_update_count = 0;
  uint32_t led_write_count = 0;
  
  while (1) {
    uint32_t currentTime = micros();
    
    // センサー読み取り（5kHz）
    if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
      updateAngleFromGyroFixed();
      sensor_read_count++;
      
      // 角度変化速度を計算して補間角度を同期
      if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {
        float current_sensor_angle = angle;
        float dt = (currentTime - lastSensorRead) / 1000000.0;
        
        if (dt > 0) {
          angle_velocity = (current_sensor_angle - prev_sensor_angle) / dt;
          
          // 補間角度を実際の角度に近づける（同期処理）
          float angle_diff = current_sensor_angle - interpolated_angle;
          if (fabs(angle_diff) > 5.0) {  // 5度以上の差がある場合は同期
            interpolated_angle = current_sensor_angle;
            Serial.println("Interpolated angle synchronized");
          }
        }
        
        prev_sensor_angle = current_sensor_angle;
        xSemaphoreGive(dataMutex);
      }
      
      lastSensorRead = currentTime;
    }
    
    // LED更新（10kHz）
    if (currentTime - lastLEDUpdate >= LED_UPDATE_INTERVAL) {
      led_update_count++;
      
      // 角度の線形補間
      float dt = (currentTime - lastLEDUpdate) / 1000000.0;
      interpolated_angle += angle_velocity * dt * 0.8;  // 0.8倍で安定化
      
      // 範囲制限
      if (interpolated_angle < 0.0) interpolated_angle = 0.0;
      else if (interpolated_angle > 180.0) interpolated_angle = 180.0;
      
      // LED更新判定を緩く
      static float prev_led_angle = -999.0;
      if (fabs(interpolated_angle - prev_led_angle) >= 0.1) {  // 0.1度の変化で更新
        updateLEDPatternFixed(interpolated_angle);
        
        bool should_update = false;
        if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {
          should_update = led_update_flag;
          if (should_update) {
            led_update_flag = false;
          }
          xSemaphoreGive(dataMutex);
        }
        
        if (should_update) {
          updateLEDsFast();
          prev_led_angle = interpolated_angle;
          led_write_count++;
        }
      }
      
      lastLEDUpdate = currentTime;
    }
    
    // デバッグ出力（1秒毎）
    uint32_t currentTimeMs = millis();
    if (currentTimeMs - lastDebugOutput >= 1000) {
      Serial.print("Rates - Sensor: ");
      Serial.print(sensor_read_count);
      Serial.print("Hz, LED Update: ");
      Serial.print(led_update_count);
      Serial.print("Hz, LED Write: ");
      Serial.print(led_write_count);
      Serial.print("Hz, Interp Angle: ");
      Serial.print(interpolated_angle, 1);
      Serial.print("°, Velocity: ");
      Serial.print(angle_velocity, 1);
      Serial.println(" dps");
      
      sensor_read_count = 0;
      led_update_count = 0;
      led_write_count = 0;
      lastDebugOutput = currentTimeMs;
    }
    
    // 最小限のyield
    taskYIELD();
  }
}

// 修正されたセンサー読み取り関数
void updateAngleFromGyroFixed() {
  if (!calibration_done) return;
  
  static inv_imu_sensor_event_t imu_event;
  static uint32_t sensor_error_count = 0;
  static uint32_t last_successful_read = 0;
  
  uint32_t current_time = micros();
  
  // センサー読み取り
  if (icm42670p.getDataFromRegisters(imu_event) != 0) {
    sensor_error_count++;
    return;
  }
  
  last_successful_read = current_time;
  float dt = (current_time - last_update_time) / 1000000.0;
  
  if (dt > 0.00001) {  // 10μs以上の間隔
    float current_gyro_z = imu_event.gyro[2] - gyro_z_bias;
    
    if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {
      gyro_z = -(current_gyro_z - gyro_drift_compensation) / ICM_SENS_SCALE_FACTOR;
      
      // より緩いノイズフィルタリング
      if (fabs(gyro_z) < 0.1) {
        gyro_z = 0.0;
      }
      
      detectSwingDirectionFixed();
      
      // 振り動作中の角度積分
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
      } else {
        // 振り動作していない時も小さな動きを検出
        if (fabs(gyro_z) > 0.5) {
          raw_angle += fabs(gyro_z) * dt * 0.5;  // 半分の感度で継続
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


// 修正された振り方向検出
void detectSwingDirectionFixed() {
  static uint32_t last_direction_change = 0;
  static float gyro_history[5] = {0};  // 角速度履歴
  static int history_index = 0;
  
  uint32_t current_time = micros();
  float current_gyro = gyro_z;
  
  // 角速度履歴を更新
  gyro_history[history_index] = current_gyro;
  history_index = (history_index + 1) % 5;
  
  // 平均角速度を計算（ノイズ軽減）
  float avg_gyro = 0.0;
  for (int i = 0; i < 5; i++) {
    avg_gyro += gyro_history[i];
  }
  avg_gyro /= 5.0;
  
  // 振り方向の検出（より緩い条件）
  bool new_direction_right_to_left = (avg_gyro > 0);
  
  // 振り方向変更の検出（閾値を緩く）
  if (fabs(avg_gyro) > SWING_DIRECTION_THRESHOLD) {
    
    if (swing_direction_right_to_left != new_direction_right_to_left && 
        (current_time - last_direction_change) > MIN_SWING_DURATION) {
      
      Serial.print("Direction change: ");
      Serial.print(swing_direction_right_to_left ? "R→L" : "L→R");
      Serial.print(" to ");
      Serial.print(new_direction_right_to_left ? "R→L" : "L→R");
      Serial.print(", Avg Gyro: ");
      Serial.print(avg_gyro, 2);
      
      // 前回の到達列を保存
      if (swing_direction_right_to_left) {
        last_column_index = current_column_index;
        Serial.print(", Saved column: ");
        Serial.print(last_column_index);
      }
      
      // 振り方向を更新
      swing_direction_right_to_left = new_direction_right_to_left;
      
      // 角度リセット（より緩く）
      raw_angle = raw_angle * 0.1;  // 完全リセットではなく10%に減衰
      swing_in_progress = true;
      last_direction_change = current_time;
      
      Serial.println();
    }
  }
  
  // 振り動作の継続判定（より緩く）
  if (fabs(avg_gyro) > SWING_ACTIVE_THRESHOLD) {
    swing_in_progress = true;
  } else if (fabs(avg_gyro) < 0.2) {
    // すぐに停止させず、徐々に減衰
    static int stop_counter = 0;
    stop_counter++;
    if (stop_counter > 10) {  // 10回連続で低い値の場合のみ停止
      swing_in_progress = false;
      stop_counter = 0;
    }
  } else {
    static int stop_counter = 0;
    stop_counter = 0;  // カウンタリセット
  }
}
// 修正されたLEDパターン更新
void updateLEDPatternFixed(float display_angle) {
  static uint8_t prev_on_led[DOTSUU] = {0};  // 前回のパターンを保存
  uint8_t new_on_led[DOTSUU];
  memset(new_on_led, 0, DOTSUU);
  
  // 角度範囲の確認（デバッグ情報付き）
  if (display_angle >= MOJISUU_OFS && display_angle < (MOJISUU + MOJISUU_OFS)) {
    uint16_t idx = 0;
    bool local_swing_direction;
    uint16_t local_last_column_index;
    
    if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {
      local_swing_direction = swing_direction_right_to_left;
      local_last_column_index = last_column_index;
      xSemaphoreGive(dataMutex);
    } else {
      return;
    }
    
    if (local_swing_direction) {
      // 右→左の振り
      idx = (uint16_t)(display_angle) - MOJISUU_OFS;
    } else {
      // 左→右の振り
      uint16_t angle_based_idx = (uint16_t)(display_angle) - MOJISUU_OFS;
      if (local_last_column_index > 0) {
        float progress = (float)angle_based_idx / (float)(MOJISUU - 1);
        idx = local_last_column_index - (uint16_t)(progress * local_last_column_index);
      } else {
        idx = MOJISUU - 1 - angle_based_idx;  // 代替方法
      }
      if (idx >= MOJISUU) idx = 0;
    }
    
    // 範囲チェック
    if (idx < MOJISUU) {
      // LEDパターン設定
      for (int i = 0; i < DOTSUU; i++) {
        if (i < sizeof(moji_idx) && moji_idx[i] < DOTSUU && idx < sizeof(moji[0])) {
          new_on_led[moji_idx[i]] = moji[i][idx];
        }
      }
      
      // 現在列インデックス更新
      if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {
        current_column_index = idx;
        xSemaphoreGive(dataMutex);
      }
    }
  } else {
    // 角度範囲外でも何かしら表示（デバッグ用）
    static uint32_t out_of_range_count = 0;
    out_of_range_count++;
    if (out_of_range_count % 1000 == 0) {
      Serial.print("Angle out of range: ");
      Serial.print(display_angle);
      Serial.print(", Range: ");
      Serial.print(MOJISUU_OFS);
      Serial.print("-");
      Serial.println(MOJISUU + MOJISUU_OFS);
    }
  }
  
  // パターンが変更された場合のみ更新
  bool pattern_changed = false;
  for (int i = 0; i < DOTSUU; i++) {
    if (new_on_led[i] != prev_on_led[i]) {
      pattern_changed = true;
      break;
    }
  }
  
  if (pattern_changed) {
    memcpy(prev_on_led, new_on_led, DOTSUU);
    
    if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {
      memcpy((void*)on_led, new_on_led, DOTSUU);
      led_update_flag = true;
      xSemaphoreGive(dataMutex);
    }
  }
}

// 高速LED更新関数
void updateLEDsFast() {
  static rgb_color led_buffer[ledCount];  // 静的バッファで高速化
  
  // LEDバッファクリア
  memset(led_buffer, 0, sizeof(led_buffer));
  
  // ローカルコピー
  uint8_t local_on_led[DOTSUU];
  if (xSemaphoreTake(dataMutex, 0) == pdTRUE) {
    memcpy(local_on_led, (void*)on_led, DOTSUU);
    xSemaphoreGive(dataMutex);
  } else {
    return;
  }
  
  // LEDカラー設定（ループ展開）
  for (int i = 0; i < DOTSUU && i < ledCount; i++) {
    uint8_t color_index = local_on_led[i];
    if (color_index > 0 && color_index < 8) {
      led_buffer[i] = colors[color_index];
    }
  }
  
  // LED出力
  ledStrip.write(led_buffer, ledCount, brightness);
}

// 動作モード設定関数
void setHighSpeedMode() {
  // CPUクロックを最大に設定
  setCpuFrequencyMhz(240);
  
  // SPIクロック速度を上げる
  SPI.setFrequency(20000000);  // 20MHz
  
  // タスク優先度を最適化
  vTaskPrioritySet(gyroTaskHandle, configMAX_PRIORITIES - 1);
}

// setup関数での初期化追加
void setupOptimized() {
  // 高速モード設定
  setHighSpeedMode();
  
  // 最適化されたタスク作成
  xTaskCreatePinnedToCore(
    gyroUpdateTaskOptimized,
    "GYRO_OPTIMIZED",
    32768,                     // より大きなスタックサイズ
    NULL,
    configMAX_PRIORITIES - 1,  // 最高優先度
    &gyroTaskHandle,
    1                          // Core 1
  );
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

void clearAllLEDs() {
  for (int i = 0; i < ledCount; i++) {
    ledColors[i] = rgb_color(0, 0, 0);
  }
  ledStrip.write(ledColors, ledCount, brightness);
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

#if 0 

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
    updateAngleFromGyroFast();
    
    // Update LED pattern based on new angle
    updateLEDPatternOptimized();
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
      updateLEDsFast();
    }

    // Wait for next cycle (1ms interval)
    vTaskDelayUntil(&lastWakeTime, frequency);
  }
}
#endif 

