#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ============ WiFi 配置 ============
const char* WIFI_SSID = "310";
const char* WIFI_PASS = "202520062007";

// ============ MQTT 配置 ============
const char* MQTT_SERVER = "221.7.223.136";
const int   MQTT_PORT = 18883;
const char* MQTT_USER = "";
const char* MQTT_PASS = "";
const char* VEHICLE_ID = "esp32_001";
char MQTT_TOPIC[64];
const char* MQTT_CLIENT_ID = "ESP32_SoundLocator";

// ============ 引脚配置 ============
#define I2S0_SCK  21
#define I2S0_WS   19
#define I2S0_SD   18
#define I2S1_SCK  14
#define I2S1_WS   12
#define I2S1_SD   13

#define SAMPLE_RATE     96000
#define BUFFER_SIZE     64

// ============ 麦克风配置 ============
const float MIC_SPACING = 0.10f;
const float SOUND_SPEED = 343.0f;
const double SAMPLE_PERIOD_US = 1000000.0 / SAMPLE_RATE;
const double MAX_DELAY_US = (MIC_SPACING * 1.414f / SOUND_SPEED) * 1e6 + 100.0;  // ~512μs
const int MAX_DELAY_SAMPLES = (int)(MAX_DELAY_US / SAMPLE_PERIOD_US) + 20;

const float HALF_SP = MIC_SPACING / 2.0f;
const float MIC_X[4] = { HALF_SP, -HALF_SP,  HALF_SP, -HALF_SP};
const float MIC_Y[4] = { HALF_SP,  HALF_SP, -HALF_SP, -HALF_SP};

// ============ 缓冲区 ============
int32_t i2s0_buffer[BUFFER_SIZE * 2];
int32_t i2s1_buffer[BUFFER_SIZE * 2];

// ============ 历史缓冲区 (用于峰值检测) ============
#define HISTORY_SIZE 512
float history[4][HISTORY_SIZE];
int history_idx = 0;

// ============ 双核同步 ============
static SemaphoreHandle_t i2s_start_sem;
static SemaphoreHandle_t i2s_done_sem;
static volatile size_t i2s1_bytes_read = 0;

// ============ 检测状态 ============
uint32_t sample_counter = 0;
bool triggered = false;
int post_trigger_count = 0;
int trigger_start_idx = 0;

// ============ 噪声基线 ============
float noise_rms[4] = {0.003f, 0.003f, 0.003f, 0.003f};
float THRESHOLD_MULT = 8.0f;  // 提高阈值

// ============ 统计 ============
uint32_t event_count = 0;
uint32_t valid_count = 0;

// ============ 历史记录 ============
float last_x = 0, last_y = 0;
bool has_history = false;
bool debug_mode = false;
bool monitor_mode = false;

// ============ 实时音量 ============
float current_amp[4] = {0};
float max_amp[4] = {0};
unsigned long last_monitor_print = 0;

// ============ MQTT 相关 ============
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

struct LocationData {
    float x, y, distance, angle, error;
    const char* direction;
    bool valid;
    unsigned long timestamp;
} currentLocation = {0, 0, 0, 0, 0, "未知", false, 0};

unsigned long lastMqttSend = 0;
unsigned long lastMqttReconnect = 0;
const unsigned long MQTT_SEND_INTERVAL = 1000;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;

bool wifiConnected = false;
bool mqttConnected = false;

// ============================================================
//                      辅助函数
// ============================================================

inline float convertSample(int32_t raw) {
    return (float)(raw >> 8) / 8388608.0f;
}

// ============================================================
//                   网络连接
// ============================================================

void checkWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiConnected) {
            wifiConnected = true;
            Serial.printf("\n✓ WiFi已连接 IP: %s\n", WiFi.localIP().toString().c_str());
        }
    } else {
        if (wifiConnected) {
            wifiConnected = false;
            mqttConnected = false;
            Serial.println("\n✗ WiFi断开");
        }
    }
}

void checkMQTT() {
    if (!wifiConnected) return;
    if (mqttClient.connected()) { mqttConnected = true; return; }
    mqttConnected = false;
    
    unsigned long now = millis();
    if (now - lastMqttReconnect < MQTT_RECONNECT_INTERVAL) return;
    lastMqttReconnect = now;
    
    Serial.printf("MQTT连接 %s:%d ...", MQTT_SERVER, MQTT_PORT);
    
    bool ok = (strlen(MQTT_USER) > 0) ? 
              mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS) :
              mqttClient.connect(MQTT_CLIENT_ID);
    
    Serial.println(ok ? "成功!" : "失败");
    if (ok) mqttConnected = true;
}

void sendMQTTData() {
    if (!mqttConnected) return;
    
    StaticJsonDocument<300> doc;
    doc["vehicle_id"] = VEHICLE_ID;
    doc["valid"] = currentLocation.valid;
    doc["x"] = round(currentLocation.x * 100) / 100.0;
    doc["y"] = round(currentLocation.y * 100) / 100.0;
    doc["distance"] = round(currentLocation.distance * 10) / 10.0;
    doc["angle"] = round(currentLocation.angle);
    doc["direction"] = currentLocation.direction;
    doc["timestamp"] = millis();
    doc["events"] = valid_count;
    
    char buf[256];
    serializeJson(doc, buf);
    
    if (mqttClient.publish(MQTT_TOPIC, buf)) {
        if (debug_mode) Serial.printf("📤 %s\n", buf);
    }
}

// ============================================================
//                   I2S 任务
// ============================================================

void i2s1ReadTask(void* param) {
    size_t bytes_read;
    while (true) {
        xSemaphoreTake(i2s_start_sem, portMAX_DELAY);
        i2s_read(I2S_NUM_1, i2s1_buffer, sizeof(i2s1_buffer), &bytes_read, portMAX_DELAY);
        i2s1_bytes_read = bytes_read;
        xSemaphoreGive(i2s_done_sem);
    }
}

int syncReadI2S(size_t* br0, size_t* br1) {
    xSemaphoreGive(i2s_start_sem);
    i2s_read(I2S_NUM_0, i2s0_buffer, sizeof(i2s0_buffer), br0, portMAX_DELAY);
    xSemaphoreTake(i2s_done_sem, portMAX_DELAY);
    *br1 = i2s1_bytes_read;
    return min(*br0, *br1) / (2 * sizeof(int32_t));
}

// ============================================================
//            峰值检测 - 找到真正的声音起始点
// ============================================================

int findPeakPosition(int mic, int start_idx, int window) {
    float max_val = 0;
    int max_pos = start_idx;
    
    for (int i = 0; i < window; i++) {
        int idx = (start_idx + i) % HISTORY_SIZE;
        float val = fabsf(history[mic][idx]);
        if (val > max_val) {
            max_val = val;
            max_pos = start_idx + i;
        }
    }
    return max_pos;
}

// 找到信号上升沿 (从峰值往前找)
int findOnsetPosition(int mic, int peak_idx, float threshold) {
    int onset = peak_idx;
    
    for (int i = 0; i < 50; i++) {  // 最多往前找50个采样
        int idx = (peak_idx - i + HISTORY_SIZE) % HISTORY_SIZE;
        if (fabsf(history[mic][idx]) < threshold) {
            onset = peak_idx - i + 1;
            break;
        }
    }
    return onset;
}

// ============================================================
//                       处理采样
// ============================================================

void processSamples(int32_t* buf0, int32_t* buf1, int count) {
    for (int i = 0; i < count; i++) {
        float s[4];
        s[0] = convertSample(buf0[i * 2 + 0]);
        s[1] = convertSample(buf0[i * 2 + 1]);
        s[2] = convertSample(buf1[i * 2 + 0]);
        s[3] = convertSample(buf1[i * 2 + 1]);
        
        // 存入历史缓冲区
        for (int m = 0; m < 4; m++) {
            history[m][history_idx] = s[m];
        }
        history_idx = (history_idx + 1) % HISTORY_SIZE;
        
        // 更新实时音量
        for (int m = 0; m < 4; m++) {
            float amp = fabsf(s[m]);
            current_amp[m] = current_amp[m] * 0.99f + amp * 0.01f;
            if (amp > max_amp[m]) max_amp[m] = amp;
        }
        
        sample_counter++;
        
        if (!triggered) {
            // 检测是否有强信号
            float max_s = 0;
            int max_m = 0;
            for (int m = 0; m < 4; m++) {
                if (fabsf(s[m]) > max_s) {
                    max_s = fabsf(s[m]);
                    max_m = m;
                }
            }
            
            float thresh = noise_rms[max_m] * THRESHOLD_MULT;
            
            if (max_s > thresh) {
                triggered = true;
                post_trigger_count = MAX_DELAY_SAMPLES + 30;  // 多等一些
                trigger_start_idx = history_idx;
                
                if (debug_mode) {
                    Serial.printf("🎤 触发! M%d amp=%.5f thresh=%.5f\n", 
                                  max_m+1, max_s, thresh);
                }
            }
        } else {
            post_trigger_count--;
            if (post_trigger_count <= 0) break;
        }
    }
}

void resetDetection() {
    triggered = false;
    post_trigger_count = 0;
    sample_counter = 0;
}

// ============================================================
//                       定位
// ============================================================

struct Result { float x, y, dist, angle, error; bool valid; const char* dir; };

Result locateSource(float dt[4], int first, int trig_cnt) {
    Result r = {0, 0, 0, 0, 9999, false, "?"};
    float best_err = 9999;
    
    // 全范围搜索
    for (float dist = 0.15f; dist <= 2.5f; dist += 0.05f) {
        for (float a = -M_PI; a <= M_PI; a += 0.05f) {
            float x = dist * sinf(a);
            float y = dist * cosf(a);
            
            float d[4];
            for (int m = 0; m < 4; m++) {
                float dx = x - MIC_X[m];
                float dy = y - MIC_Y[m];
                d[m] = sqrtf(dx*dx + dy*dy);
            }
            
            float err = 0;
            int cnt = 0;
            for (int m = 0; m < 4; m++) {
                if (dt[m] < 9000) {
                    float pred = (d[m] - d[first]) / SOUND_SPEED * 1e6f;
                    err += (pred - dt[m]) * (pred - dt[m]);
                    cnt++;
                }
            }
            
            if (cnt >= 3) {
                err = sqrtf(err / cnt);
                if (err < best_err) {
                    best_err = err;
                    r.x = x;
                    r.y = y;
                }
            }
        }
    }
    
    // 精细搜索
    if (best_err < 500) {
        float cx = r.x, cy = r.y;
        for (float x = cx - 0.15f; x <= cx + 0.15f; x += 0.01f) {
            for (float y = cy - 0.15f; y <= cy + 0.15f; y += 0.01f) {
                float d2 = x*x + y*y;
                if (d2 < 0.02f || d2 > 6.25f) continue;
                
                float d[4];
                for (int m = 0; m < 4; m++) {
                    float dx = x - MIC_X[m];
                    float dy = y - MIC_Y[m];
                    d[m] = sqrtf(dx*dx + dy*dy);
                }
                
                float err = 0;
                int cnt = 0;
                for (int m = 0; m < 4; m++) {
                    if (dt[m] < 9000) {
                        float pred = (d[m] - d[first]) / SOUND_SPEED * 1e6f;
                        err += (pred - dt[m]) * (pred - dt[m]);
                        cnt++;
                    }
                }
                
                if (cnt >= 3) {
                    err = sqrtf(err / cnt);
                    if (err < best_err) {
                        best_err = err;
                        r.x = x;
                        r.y = y;
                    }
                }
            }
        }
    }
    
    r.error = best_err;
    r.dist = sqrtf(r.x * r.x + r.y * r.y);
    r.angle = atan2f(r.x, r.y) * 180.0f / M_PI;
    r.valid = (best_err < 150);  // 放宽误差限制
    
    if (r.valid) {
        float th = 0.08f;
        bool front = r.y > th, back = r.y < -th, left = r.x < -th, right = r.x > th;
        if (front && right) r.dir = "右前";
        else if (front && left) r.dir = "左前";
        else if (back && right) r.dir = "右后";
        else if (back && left) r.dir = "左后";
        else if (front) r.dir = "正前";
        else if (back) r.dir = "正后";
        else if (right) r.dir = "正右";
        else if (left) r.dir = "正左";
        else r.dir = "中心";
    }
    return r;
}

// ============================================================
//                       分析事件
// ============================================================

void analyzeEvent() {
    event_count++;
    
    // 使用峰值检测找到各麦克风的声音到达时间
    int peak_pos[4];
    int onset_pos[4];
    float peak_amp[4];
    int trig_cnt = 0;
    
    // 找每个麦克风的峰值
    int search_start = (trigger_start_idx - 20 + HISTORY_SIZE) % HISTORY_SIZE;
    
    for (int m = 0; m < 4; m++) {
        // 找峰值
        float max_val = 0;
        int max_idx = 0;
        
        for (int i = 0; i < MAX_DELAY_SAMPLES + 50; i++) {
            int idx = (search_start + i) % HISTORY_SIZE;
            float val = fabsf(history[m][idx]);
            if (val > max_val) {
                max_val = val;
                max_idx = i;
            }
        }
        
        peak_pos[m] = max_idx;
        peak_amp[m] = max_val;
        
        // 找上升沿 (从峰值往前找到低于阈值的点)
        float onset_thresh = noise_rms[m] * 3.0f;
        int onset = max_idx;
        
        for (int i = 0; i < 40; i++) {
            int idx = (search_start + max_idx - i + HISTORY_SIZE) % HISTORY_SIZE;
            if (fabsf(history[m][idx]) < onset_thresh) {
                onset = max_idx - i + 1;
                break;
            }
        }
        onset_pos[m] = onset;
        
        if (peak_amp[m] > noise_rms[m] * 4.0f) {
            trig_cnt++;
        }
    }
    
    if (debug_mode) {
        Serial.printf("事件#%u: 检测到%d个有效麦克风\n", event_count, trig_cnt);
        Serial.printf("  峰值位置: [%d, %d, %d, %d]\n", peak_pos[0], peak_pos[1], peak_pos[2], peak_pos[3]);
        Serial.printf("  起始位置: [%d, %d, %d, %d]\n", onset_pos[0], onset_pos[1], onset_pos[2], onset_pos[3]);
        Serial.printf("  峰值幅度: [%.4f, %.4f, %.4f, %.4f]\n", peak_amp[0], peak_amp[1], peak_amp[2], peak_amp[3]);
    }
    
    if (trig_cnt < 3) {
        if (debug_mode) Serial.println("  ❌ 有效麦克风不足3个");
        return;
    }
    
    // 找最早到达的麦克风 (使用onset位置)
    int first = 0;
    int min_onset = onset_pos[0];
    for (int m = 1; m < 4; m++) {
        if (peak_amp[m] > noise_rms[m] * 4.0f && onset_pos[m] < min_onset) {
            min_onset = onset_pos[m];
            first = m;
        }
    }
    
    // 计算 TDOA
    float dt[4];
    for (int m = 0; m < 4; m++) {
        if (peak_amp[m] > noise_rms[m] * 4.0f) {
            dt[m] = (onset_pos[m] - min_onset) * SAMPLE_PERIOD_US;
        } else {
            dt[m] = 9999;
        }
    }
    
    if (debug_mode) {
        Serial.printf("  首个: M%d, TDOA: [%.1f, %.1f, %.1f, %.1f] μs\n", 
                      first+1, dt[0], dt[1], dt[2], dt[3]);
    }
    
    // 检查 TDOA 合理性
    float max_tdoa = 0;
    for (int m = 0; m < 4; m++) {
        if (dt[m] < 9000 && dt[m] > max_tdoa) max_tdoa = dt[m];
    }
    
    if (max_tdoa > MAX_DELAY_US * 1.5f) {
        if (debug_mode) Serial.printf("  ❌ TDOA过大: %.1f > %.1f\n", max_tdoa, MAX_DELAY_US * 1.5f);
        return;
    }
    
    // 定位
    Result r = locateSource(dt, first, trig_cnt);
    
    if (debug_mode) {
        Serial.printf("  定位: (%.2f, %.2f) err=%.1f valid=%d\n", r.x, r.y, r.error, r.valid);
    }
    
    if (!r.valid) return;
    
    // 平滑
    if (has_history) {
        float jump = sqrtf((r.x-last_x)*(r.x-last_x) + (r.y-last_y)*(r.y-last_y));
        if (jump > 1.0f) {
            r.x = 0.5f*r.x + 0.5f*last_x;
            r.y = 0.5f*r.y + 0.5f*last_y;
            r.dist = sqrtf(r.x*r.x + r.y*r.y);
            r.angle = atan2f(r.x, r.y) * 180.0f / M_PI;
        }
    }
    last_x = r.x; last_y = r.y; has_history = true;
    valid_count++;
    
    currentLocation.x = r.x * 100;
    currentLocation.y = r.y * 100;
    currentLocation.distance = r.dist * 100;
    currentLocation.angle = r.angle;
    currentLocation.direction = r.dir;
    currentLocation.valid = true;
    currentLocation.timestamp = millis();
    
    Serial.printf("\n🔊 #%u %s %+.0f° | X:%+.0fcm Y:%+.0fcm | %.0fcm (err:%.0f)\n",
                  valid_count, r.dir, r.angle, r.x*100, r.y*100, r.dist*100, r.error);
}

// ============================================================
//                       校准
// ============================================================

void calibrate() {
    Serial.println("\n=== 校准中(请安静2秒) ===");
    resetDetection();
    
    for (int m = 0; m < 4; m++) max_amp[m] = 0;
    
    float sum_sq[4] = {0};
    int count = 0;
    unsigned long start = millis();
    
    while (millis() - start < 2000) {
        size_t br0, br1;
        int n = syncReadI2S(&br0, &br1);
        for (int i = 0; i < n; i++) {
            float s[4];
            s[0] = convertSample(i2s0_buffer[i*2]);
            s[1] = convertSample(i2s0_buffer[i*2+1]);
            s[2] = convertSample(i2s1_buffer[i*2]);
            s[3] = convertSample(i2s1_buffer[i*2+1]);
            for (int m = 0; m < 4; m++) sum_sq[m] += s[m] * s[m];
            count++;
        }
    }
    
    Serial.println("噪声RMS / 阈值:");
    for (int m = 0; m < 4; m++) {
        noise_rms[m] = sqrtf(sum_sq[m] / count);
        if (noise_rms[m] < 0.0003f) noise_rms[m] = 0.0005f;
        Serial.printf("  M%d: RMS=%.6f 阈值=%.6f\n", m+1, noise_rms[m], noise_rms[m] * THRESHOLD_MULT);
    }
    
    event_count = valid_count = 0;
    has_history = false;
    currentLocation.valid = false;
    resetDetection();
    
    // 清空历史
    memset(history, 0, sizeof(history));
    history_idx = 0;
    
    Serial.printf("阈值倍数: %.1f\n", THRESHOLD_MULT);
    Serial.println("校准完成!\n");
}

// ============================================================
//                   音量监控
// ============================================================

void printMonitor() {
    Serial.printf("\r");
    for (int m = 0; m < 4; m++) {
        float amp = current_amp[m];
        int bars = (int)(amp / 0.002f);
        bars = constrain(bars, 0, 15);
        
        Serial.printf("M%d[", m+1);
        for (int b = 0; b < 15; b++) {
            Serial.print(b < bars ? "|" : " ");
        }
        Serial.printf("]%.3f ", amp);
    }
}

// ============================================================
//                        Setup
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=============================");
    Serial.println("  ESP32 声源定位 v2.3");
    Serial.println("  (峰值检测算法)");
    Serial.println("=============================\n");
    
    snprintf(MQTT_TOPIC, sizeof(MQTT_TOPIC), "vehicle/%s/info", VEHICLE_ID);
    Serial.printf("MQTT主题: %s\n", MQTT_TOPIC);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("WiFi连接中: %s\n", WIFI_SSID);
    
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setKeepAlive(60);
    mqttClient.setSocketTimeout(3);
    
    i2s_start_sem = xSemaphoreCreateBinary();
    i2s_done_sem = xSemaphoreCreateBinary();
    
    i2s_config_t cfg = {
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
    
    i2s_pin_config_t pins0 = {.bck_io_num=I2S0_SCK, .ws_io_num=I2S0_WS, 
                               .data_out_num=I2S_PIN_NO_CHANGE, .data_in_num=I2S0_SD};
    i2s_pin_config_t pins1 = {.bck_io_num=I2S1_SCK, .ws_io_num=I2S1_WS,
                               .data_out_num=I2S_PIN_NO_CHANGE, .data_in_num=I2S1_SD};
    
    i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pins0);
    i2s_driver_install(I2S_NUM_1, &cfg, 0, NULL);
    i2s_set_pin(I2S_NUM_1, &pins1);
    
    xTaskCreatePinnedToCore(i2s1ReadTask, "I2S1", 4096, NULL, 5, NULL, 0);
    Serial.println("I2S初始化完成");
    
    for (int i = 0; i < 50; i++) {
        size_t br0, br1;
        syncReadI2S(&br0, &br1);
    }
    
    calibrate();
    
    Serial.println("命令: c=校准 d=调试 v=音量 m=MQTT +=阈值+ -=阈值-\n");
}

// ============================================================
//                         Loop
// ============================================================

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        while (Serial.available()) Serial.read();
        switch (cmd) {
            case 'c': case 'C': monitor_mode = false; calibrate(); break;
            case 'd': case 'D': debug_mode = !debug_mode; Serial.printf("\n调试: %s\n", debug_mode?"开":"关"); break;
            case 'v': case 'V': monitor_mode = !monitor_mode; Serial.printf("\n音量监控: %s\n", monitor_mode?"开":"关"); break;
            case 'm': case 'M': Serial.printf("\nWiFi:%s MQTT:%s\n", wifiConnected?"✓":"✗", mqttConnected?"✓":"✗"); break;
            case '+': case '=': THRESHOLD_MULT = min(20.0f, THRESHOLD_MULT + 1); Serial.printf("\n阈值: %.0f\n", THRESHOLD_MULT); break;
            case '-': case '_': THRESHOLD_MULT = max(3.0f, THRESHOLD_MULT - 1); Serial.printf("\n阈值: %.0f\n", THRESHOLD_MULT); break;
            case 'r': case 'R': for(int m=0;m<4;m++) max_amp[m]=0; Serial.println("\n重置max"); break;
        }
    }
    
    checkWiFi();
    checkMQTT();
    if (mqttConnected) mqttClient.loop();
    
    unsigned long now = millis();
    if (now - lastMqttSend >= MQTT_SEND_INTERVAL) {
        lastMqttSend = now;
        sendMQTTData();
    }
    
    size_t br0, br1;
    int count = syncReadI2S(&br0, &br1);
    processSamples(i2s0_buffer, i2s1_buffer, count);
    
    if (monitor_mode && now - last_monitor_print > 100) {
        last_monitor_print = now;
        printMonitor();
    }
    
    if (triggered && post_trigger_count <= 0) {
        if (monitor_mode) Serial.println();
        analyzeEvent();
        delay(300);
        resetDetection();
    }
}