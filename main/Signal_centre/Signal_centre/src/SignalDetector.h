#ifndef SIGNAL_DETECTOR_H
#define SIGNAL_DETECTOR_H

#include <Arduino.h>
#include "esp_timer.h"

// 检测结果
struct DetectResult {
  bool detected;              // 是否检测到信号
  int64_t timestamp_us;       // 信号到达时间（微秒）
  int sample_index;           // 在缓冲区中的位置
  float amplitude;            // 信号幅度
  float precise_time_us;      // 插值后的精确时间
};

// 带通滤波器（二阶IIR）
class BandpassFilter {
public:
  // 中心频率，带宽，采样率
  void init(float center_freq, float bandwidth, float sample_rate);
  float process(float input);
  void reset();

private:
  float b0, b1, b2;
  float a1, a2;
  float x1, x2, y1, y2;
};

// 高精度信号检测器
class SignalDetector {
public:
  SignalDetector(float center_freq = 17500,    // 载波频率
                 float bandwidth = 2000,        // 带宽
                 int sample_rate = 44100);      // 采样率
  
  ~SignalDetector();
  
  bool begin();
  
  // ★ 核心功能：处理样本并检测信号起始时间
  // buffer_start_time_us: 缓冲区第一个样本的时间戳
  DetectResult process(int32_t* samples,          // I2S立体声数据
                       int sample_count,           // 样本数
                       int64_t buffer_start_time); // 缓冲区起始时间(us)
  
  // 处理单声道float数据
  DetectResult processMono(float* samples,
                           int sample_count,
                           int64_t buffer_start_time);
  
  // 配置
  void setThreshold(float threshold);
  void setHoldTime(int samples);  // 信号保持时间（防抖）
  
  // 获取信息
  float getEnvelope() { return _envelope; }
  bool isSignalPresent() { return _signal_present; }
  int64_t getLastDetectTime() { return _last_detect_time; }
  
  // 重置状态
  void reset();

private:
  float _center_freq;
  float _bandwidth;
  int _sample_rate;
  float _sample_period_us;   // 每个样本的时间（微秒）
  
  BandpassFilter _filter;
  
  float _threshold;
  int _hold_samples;
  
  float _envelope;
  bool _signal_present;
  bool _prev_signal;
  int _hold_counter;
  
  int64_t _last_detect_time;
  
  // 包络检测
  float _env_attack;
  float _env_release;
  
  float processOneSample(float sample);
  int64_t findPreciseEdge(float* env_buffer, int edge_index, 
                          int64_t buffer_start_time);
};

#endif