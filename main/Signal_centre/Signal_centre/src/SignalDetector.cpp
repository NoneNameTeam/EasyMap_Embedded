#include "SignalDetector.h"
#include <math.h>

// ====== 带通滤波器 ======
void BandpassFilter::init(float center_freq, float bandwidth, float sample_rate) {
  // 二阶带通滤波器系数计算
  float w0 = 2.0f * M_PI * center_freq / sample_rate;
  float Q = center_freq / bandwidth;
  float alpha = sinf(w0) / (2.0f * Q);
  
  float a0 = 1.0f + alpha;
  b0 = alpha / a0;
  b1 = 0;
  b2 = -alpha / a0;
  a1 = -2.0f * cosf(w0) / a0;
  a2 = (1.0f - alpha) / a0;
  
  reset();
}

float BandpassFilter::process(float input) {
  float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
  
  x2 = x1;
  x1 = input;
  y2 = y1;
  y1 = output;
  
  return output;
}

void BandpassFilter::reset() {
  x1 = x2 = y1 = y2 = 0;
}

// ====== 信号检测器 ======
SignalDetector::SignalDetector(float center_freq, float bandwidth, int sample_rate) {
  _center_freq = center_freq;
  _bandwidth = bandwidth;
  _sample_rate = sample_rate;
  _sample_period_us = 1000000.0f / sample_rate;  // 每样本的微秒数
  
  _threshold = 0.1f;
  _hold_samples = 10;
  
  // 包络跟随器参数
  _env_attack = 0.1f;    // 快速攻击
  _env_release = 0.001f; // 慢速释放
}

SignalDetector::~SignalDetector() {
}

bool SignalDetector::begin() {
  _filter.init(_center_freq, _bandwidth, _sample_rate);
  reset();
  return true;
}

void SignalDetector::reset() {
  _filter.reset();
  _envelope = 0;
  _signal_present = false;
  _prev_signal = false;
  _hold_counter = 0;
  _last_detect_time = 0;
}

void SignalDetector::setThreshold(float threshold) {
  _threshold = threshold;
}

void SignalDetector::setHoldTime(int samples) {
  _hold_samples = samples;
}

float SignalDetector::processOneSample(float sample) {
  // 1. 带通滤波
  float filtered = _filter.process(sample);
  
  // 2. 取绝对值（全波整流）
  float rectified = fabsf(filtered);
  
  // 3. 包络跟随
  if (rectified > _envelope) {
    _envelope = _env_attack * rectified + (1.0f - _env_attack) * _envelope;
  } else {
    _envelope = _env_release * rectified + (1.0f - _env_release) * _envelope;
  }
  
  return _envelope;
}

DetectResult SignalDetector::process(int32_t* samples, 
                                      int sample_count,
                                      int64_t buffer_start_time) {
  DetectResult result = {false, 0, -1, 0, 0};
  
  // 临时存储包络用于精确定位
  float* env_buffer = (float*)malloc(sample_count * sizeof(float));
  if (!env_buffer) return result;
  
  bool edge_found = false;
  int edge_index = -1;
  
  // 处理每个样本
  for (int i = 0; i < sample_count; i++) {
    // 取左声道（偶数位置），归一化
    float sample = (float)samples[i * 2] / 2147483648.0f;
    
    // 处理并获取包络
    env_buffer[i] = processOneSample(sample);
    
    // 检测信号状态
    bool signal_now = (env_buffer[i] > _threshold);
    
    // 检测上升沿（信号刚开始）
    if (signal_now && !_prev_signal && !edge_found) {
      edge_found = true;
      edge_index = i;
    }
    
    _prev_signal = signal_now;
    
    // 更新信号存在状态（带保持）
    if (signal_now) {
      _signal_present = true;
      _hold_counter = _hold_samples;
    } else if (_hold_counter > 0) {
      _hold_counter--;
    } else {
      _signal_present = false;
    }
  }
  
  // 如果检测到上升沿，计算精确时间
  if (edge_found && edge_index >= 0) {
    result.detected = true;
    result.sample_index = edge_index;
    result.amplitude = env_buffer[edge_index];
    
    // ★ 计算精确时间戳
    // 基础时间 = 缓冲区起始 + 样本索引 × 样本周期
    result.timestamp_us = buffer_start_time + 
                          (int64_t)(edge_index * _sample_period_us);
    
    // ★ 线性插值获得更精确的时间
    if (edge_index > 0) {
      float prev_env = env_buffer[edge_index - 1];
      float curr_env = env_buffer[edge_index];
      
      // 计算阈值穿越点的分数位置
      if (curr_env > prev_env) {
        float fraction = (_threshold - prev_env) / (curr_env - prev_env);
        fraction = constrain(fraction, 0.0f, 1.0f);
        
        // 精确时间 = 前一个样本时间 + 分数 × 样本周期
        result.precise_time_us = buffer_start_time + 
                                 (edge_index - 1 + fraction) * _sample_period_us;
      } else {
        result.precise_time_us = result.timestamp_us;
      }
    } else {
      result.precise_time_us = result.timestamp_us;
    }
    
    _last_detect_time = result.timestamp_us;
  }
  
  free(env_buffer);
  return result;
}

DetectResult SignalDetector::processMono(float* samples,
                                          int sample_count,
                                          int64_t buffer_start_time) {
  DetectResult result = {false, 0, -1, 0, 0};
  
  float* env_buffer = (float*)malloc(sample_count * sizeof(float));
  if (!env_buffer) return result;
  
  bool edge_found = false;
  int edge_index = -1;
  
  for (int i = 0; i < sample_count; i++) {
    env_buffer[i] = processOneSample(samples[i]);
    
    bool signal_now = (env_buffer[i] > _threshold);
    
    if (signal_now && !_prev_signal && !edge_found) {
      edge_found = true;
      edge_index = i;
    }
    
    _prev_signal = signal_now;
    
    if (signal_now) {
      _signal_present = true;
      _hold_counter = _hold_samples;
    } else if (_hold_counter > 0) {
      _hold_counter--;
    } else {
      _signal_present = false;
    }
  }
  
  if (edge_found && edge_index >= 0) {
    result.detected = true;
    result.sample_index = edge_index;
    result.amplitude = env_buffer[edge_index];
    
    result.timestamp_us = buffer_start_time + 
                          (int64_t)(edge_index * _sample_period_us);
    
    if (edge_index > 0) {
      float prev_env = env_buffer[edge_index - 1];
      float curr_env = env_buffer[edge_index];
      
      if (curr_env > prev_env) {
        float fraction = (_threshold - prev_env) / (curr_env - prev_env);
        fraction = constrain(fraction, 0.0f, 1.0f);
        result.precise_time_us = buffer_start_time + 
                                 (edge_index - 1 + fraction) * _sample_period_us;
      } else {
        result.precise_time_us = result.timestamp_us;
      }
    } else {
      result.precise_time_us = result.timestamp_us;
    }
    
    _last_detect_time = result.timestamp_us;
  }
  
  free(env_buffer);
  return result;
}