#include <Arduino.h>
#include <driver/ledc.h>
#include "esp_timer.h"

// ====== 硬件配置 ======
#define BUZZER_PIN    25        // 压电陶瓷片连接引脚
#define CARRIER_FREQ  17500     // 载波频率 17.5kHz
#define PWM_CHANNEL   0         // LEDC通道
#define PWM_RESOLUTION 10       // 10-bit分辨率 (0-1023)
#define PWM_DUTY      512       // 50%占空比（可调节音量）

// ====== OOK调制参数 ======
#define BIT_TIME_MS   20        // 每bit时长（与接收端一致）
#define SYNC_PATTERN  0xAA      // 同步头 10101010

// ====== 全局变量 ======
bool carrier_on = false;

// ====== 初始化PWM ======
void initCarrier() {
  // LEDC定时器配置
  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = CARRIER_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);
  
  // LEDC通道配置
  ledc_channel_config_t channel_conf = {
    .gpio_num = BUZZER_PIN,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .channel = (ledc_channel_t)PWM_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,  // 初始关闭
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf);
  
  Serial.println("✅ PWM初始化完成");
}

// ====== 载波控制 ======
void carrierOn() {
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL, PWM_DUTY);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL);
  carrier_on = true;
}

void carrierOff() {
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL);
  carrier_on = false;
}

// ====== OOK调制：发送一个bit ======
void sendBit(bool bit) {
  if (bit) {
    carrierOn();
  } else {
    carrierOff();
  }
  delayMicroseconds(BIT_TIME_MS * 1000);  // 精确延时
}

// ====== 发送一个字节 ======
void sendByte(uint8_t data) {
  for (int i = 7; i >= 0; i--) {
    bool bit = (data >> i) & 0x01;
    sendBit(bit);
  }
}

// ====== 发送数据包 ======
void sendPacket(const uint8_t* data, int length) {
  int64_t start_time = esp_timer_get_time();
  
  // 1. 发送同步头
  sendByte(SYNC_PATTERN);
  
  // 2. 发送数据
  for (int i = 0; i < length; i++) {
    sendByte(data[i]);
  }
  
  // 3. 发送结束标志
  sendByte(0x00);
  
  // 4. 关闭载波
  carrierOff();
  
  int64_t duration = esp_timer_get_time() - start_time;
  
  Serial.printf("📤 发送完成: %d字节, 耗时%.2fms\n", 
                length, duration / 1000.0);
}

// ====== 发送字符串（便捷函数）======
void sendString(const char* str) {
  sendPacket((const uint8_t*)str, strlen(str));
}

// ====== 设置 ======
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   17.5kHz 超声波发送器                ║");
  Serial.println("║   (压电陶瓷片 OOK调制)                ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  // 初始化PWM
  initCarrier();
  
  // 显示配置
  Serial.println("┌──────────────────────────────┐");
  Serial.println("│      发送参数                │");
  Serial.println("├──────────────────────────────┤");
  Serial.printf("│ 载波频率:   %d Hz\n", CARRIER_FREQ);
  Serial.printf("│ 调制方式:   OOK\n");
  Serial.printf("│ Bit时长:    %d ms\n", BIT_TIME_MS);
  Serial.printf("│ 比特率:     %d bps\n", 1000 / BIT_TIME_MS);
  Serial.printf("│ 占空比:     %d%%\n", (PWM_DUTY * 100) / 1024);
  Serial.println("└──────────────────────────────┘\n");
  
  // 测试音
  Serial.println("🔊 发送测试音（3秒）...");
  carrierOn();
  delay(3000);
  carrierOff();
  Serial.println("测试完成\n");
  
  delay(1000);
}

// ====== 主循环 ======
void loop() {
  // 示例1: 发送字符串
  Serial.println("发送: Hello");
  sendString("Hello");
  delay(2000);
  
  // 示例2: 发送自定义数据
  uint8_t data[] = {0x12, 0x34, 0x56, 0x78};
  Serial.println("发送: 自定义数据");
  sendPacket(data, 4);
  delay(2000);
  
  // 示例3: 发送计数
  static int counter = 0;
  char buffer[16];
  sprintf(buffer, "N%d", counter++);
  Serial.printf("发送: %s\n", buffer);
  sendString(buffer);
  delay(2000);
}