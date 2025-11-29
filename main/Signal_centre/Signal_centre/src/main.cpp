#include <Arduino.h>
#include <driver/i2s.h>
#include "esp_timer.h"
#include "SignalDetector.h"
#include "TDOALocator.h"

// ====== I2S引脚配置 ======
// I2S0: Mic1(左) + Mic2(右)
#define I2S0_SD   32
#define I2S0_SCK  33
#define I2S0_WS   25

// I2S1: Mic3(左) + Mic4(右)
#define I2S1_SD   26
#define I2S1_SCK  27
#define I2S1_WS   14

// ====== 参数 ======
#define SAMPLE_RATE    48000
#define BUFFER_SIZE    64      // 小缓冲区 = 低延迟
#define CARRIER_FREQ   17500.0f
#define BANDWIDTH      2000.0f
#define THRESHOLD      0.05f

// ====== 全局对象 ======
SignalDetector detector[4] = {
  SignalDetector(CARRIER_FREQ, BANDWIDTH, SAMPLE_RATE),
  SignalDetector(CARRIER_FREQ, BANDWIDTH, SAMPLE_RATE),
  SignalDetector(CARRIER_FREQ, BANDWIDTH, SAMPLE_RATE),
  SignalDetector(CARRIER_FREQ, BANDWIDTH, SAMPLE_RATE)
};

TDOALocator locator;

int32_t i2s0_buffer[BUFFER_SIZE * 2];
int32_t i2s1_buffer[BUFFER_SIZE * 2];

// 检测结果
int64_t detect_time[4] = {0, 0, 0, 0};
float precise_time[4] = {0, 0, 0, 0};
bool detected[4] = {false, false, false, false};

// ====== 提取单声道数据 ======
void extractChannel(int32_t* stereo_buffer, float* mono_out, int count, int channel) {
  for (int i = 0; i < count; i++) {
    mono_out[i] = (float)stereo_buffer[i * 2 + channel] / 2147483648.0f;
  }
}

// ====== I2S初始化 ======
void initI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  // I2S0
  i2s_pin_config_t i2s0_pins = {
    .bck_io_num = I2S0_SCK,
    .ws_io_num = I2S0_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S0_SD
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s0_pins);

  // I2S1
  i2s_pin_config_t i2s1_pins = {
    .bck_io_num = I2S1_SCK,
    .ws_io_num = I2S1_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S1_SD
  };
  i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &i2s1_pins);
  
  Serial.println("✅ I2S初始化完成");
}

// ====== 重置检测 ======
void resetDetection() {
  for (int i = 0; i < 4; i++) {
    detected[i] = false;
    detect_time[i] = 0;
    precise_time[i] = 0;
    detector[i].reset();
  }
}

// ====== 检查全部检测到 ======
bool allDetected() {
  return detected[0] && detected[1] && detected[2] && detected[3];
}

// ====== 设置 ======
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   4麦克风 TDOA 3D定位系统 (完整版)    ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  // 初始化I2S
  initI2S();
  
  // 初始化4个信号检测器
  for (int i = 0; i < 4; i++) {
    if (!detector[i].begin()) {
      Serial.printf("❌ 检测器%d初始化失败!\n", i+1);
      while(1);
    }
    detector[i].setThreshold(THRESHOLD);
    detector[i].setHoldTime(10);
  }
  Serial.println("✅ 信号检测器初始化完成");
  
  // 配置麦克风位置（单位：米）
  locator.setMicPositions(
    0.0,   0.0,   0.0,     // Mic1
    0.30,  0.0,   0.0,     // Mic2
    0.30,  0.30,  0.0,     // Mic3
    0.0,   0.30,  0.10     // Mic4 (抬高10cm)
  );
  
  // 声速
  float temperature = 25.0;
  float sound_speed = 331.3 + 0.606 * temperature;
  locator.setSoundSpeed(sound_speed);
  locator.setForcePositiveZ(true);
  
  // 显示配置
  float sample_period = 1000000.0f / SAMPLE_RATE;
  Serial.println("\n┌─────────────────────────────────┐");
  Serial.println("│          系统参数               │");
  Serial.println("├─────────────────────────────────┤");
  Serial.printf("│ 采样率:     %d Hz\n", SAMPLE_RATE);
  Serial.printf("│ 样本周期:   %.2f μs\n", sample_period);
  Serial.printf("│ 载波频率:   %.0f Hz\n", CARRIER_FREQ);
  Serial.printf("│ 声速:       %.1f m/s\n", sound_speed);
  Serial.printf("│ 理论精度:   ~%.1f mm\n", sample_period * 0.343);
  Serial.println("└─────────────────────────────────┘");
  
  Serial.println("\n🎧 等待17.5kHz信号...\n");
}

// ====== 临时缓冲区（单声道） ======
float mono_buffer[BUFFER_SIZE];

// ====== 主循环 ======
void loop() {
  size_t bytes_read0 = 0, bytes_read1 = 0;
  
  // 记录缓冲区开始时间
  int64_t buffer_time = esp_timer_get_time();
  
  // 读取I2S数据
  i2s_read(I2S_NUM_0, i2s0_buffer, sizeof(i2s0_buffer), &bytes_read0, 10);
  i2s_read(I2S_NUM_1, i2s1_buffer, sizeof(i2s1_buffer), &bytes_read1, 10);
  
  int sample_count = bytes_read0 / (2 * sizeof(int32_t));
  
  // ====== 处理4个麦克风 ======
  
  // Mic1: I2S0 左声道
  if (!detected[0]) {
    extractChannel(i2s0_buffer, mono_buffer, sample_count, 0);
    DetectResult r = detector[0].processMono(mono_buffer, sample_count, buffer_time);
    if (r.detected) {
      detect_time[0] = r.timestamp_us;
      precise_time[0] = r.precise_time_us;
      detected[0] = true;
    }
  }
  
  // Mic2: I2S0 右声道
  if (!detected[1]) {
    extractChannel(i2s0_buffer, mono_buffer, sample_count, 1);
    DetectResult r = detector[1].processMono(mono_buffer, sample_count, buffer_time);
    if (r.detected) {
      detect_time[1] = r.timestamp_us;
      precise_time[1] = r.precise_time_us;
      detected[1] = true;
    }
  }
  
  // Mic3: I2S1 左声道
  if (!detected[2]) {
    extractChannel(i2s1_buffer, mono_buffer, sample_count, 0);
    DetectResult r = detector[2].processMono(mono_buffer, sample_count, buffer_time);
    if (r.detected) {
      detect_time[2] = r.timestamp_us;
      precise_time[2] = r.precise_time_us;
      detected[2] = true;
    }
  }
  
  // Mic4: I2S1 右声道
  if (!detected[3]) {
    extractChannel(i2s1_buffer, mono_buffer, sample_count, 1);
    DetectResult r = detector[3].processMono(mono_buffer, sample_count, buffer_time);
    if (r.detected) {
      detect_time[3] = r.timestamp_us;
      precise_time[3] = r.precise_time_us;
      detected[3] = true;
    }
  }
  
  // ====== 所有麦克风都检测到 ======
  if (allDetected()) {
    Serial.println("\n════════════════════════════════════════");
    Serial.println("🔔 信号检测到!");
    
    // 显示时间戳
    Serial.println("\n【时间戳】");
    for (int i = 0; i < 4; i++) {
      Serial.printf("  Mic%d: %lld μs", i+1, detect_time[i]);
      if (i > 0) {
        int64_t dt = detect_time[i] - detect_time[0];
        float dp = precise_time[i] - precise_time[0];
        Serial.printf("  (Δ%lld μs, 精确Δ%.2f μs)", dt, dp);
      }
      Serial.println();
    }
    
    // TDOA定位
    LocationResult result = locator.locate(
      detect_time[0],
      detect_time[1], 
      detect_time[2],
      detect_time[3]
    );
    
    if (result.success) {
      Serial.println("\n【定位结果】");
      Serial.printf("  ✅ 位置: (%.3f, %.3f, %.3f) m\n",
                    result.position.x,
                    result.position.y,
                    result.position.z);
      
      // 方向和距离
      float distance = sqrt(result.position.x * result.position.x +
                           result.position.y * result.position.y +
                           result.position.z * result.position.z);
      float horizontal = sqrt(result.position.x * result.position.x +
                             result.position.y * result.position.y);
      float angle = atan2(result.position.y, result.position.x) * 180.0 / M_PI;
      
      Serial.printf("  方向:     %.1f°\n", angle);
      Serial.printf("  水平距离: %.3f m\n", horizontal);
      Serial.printf("  高度:     %.3f m\n", result.position.z);
      Serial.printf("  3D距离:   %.3f m\n", distance);
      Serial.printf("  残差:     %.2e\n", result.error);
      Serial.printf("  迭代:     %d\n", result.iterations);
    } else {
      Serial.println("\n  ❌ 定位失败");
      Serial.printf("  残差: %.2e\n", result.error);
    }
    
    Serial.println("════════════════════════════════════════\n");
    
    // 等待信号消失
    delay(200);
    
    // 等到所有麦克风信号消失
    bool signal_present = true;
    while (signal_present) {
      i2s_read(I2S_NUM_0, i2s0_buffer, sizeof(i2s0_buffer), &bytes_read0, 10);
      i2s_read(I2S_NUM_1, i2s1_buffer, sizeof(i2s1_buffer), &bytes_read1, 10);
      
      signal_present = false;
      for (int i = 0; i < 4; i++) {
        if (detector[i].isSignalPresent()) {
          signal_present = true;
        }
      }
      delay(10);
    }
    
    // 重置
    resetDetection();
    Serial.println("🎧 等待下一个信号...\n");
  }
  
  // ====== 实时状态显示 ======
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 100) {
    lastDisplay = millis();
    
    Serial.print("\r");
    for (int i = 0; i < 4; i++) {
      float env = detector[i].getEnvelope();
      int bars = constrain((int)(env * 200), 0, 8);
      
      Serial.printf("M%d%s[", i+1, detected[i] ? "✓" : " ");
      for (int j = 0; j < 8; j++) {
        Serial.print(j < bars ? "█" : "·");
      }
      Serial.print("] ");
    }
  }
}