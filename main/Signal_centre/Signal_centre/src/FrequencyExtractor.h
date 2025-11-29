#ifndef FREQUENCY_EXTRACTOR_H
#define FREQUENCY_EXTRACTOR_H

#include <Arduino.h>
#include "esp_dsp.h"

// 提取结果结构体
struct FreqResult {
  float amplitude;      // 峰值幅度
  float peak_freq;      // 峰值频率 (Hz)
  float total_energy;   // 窗口内总能量
  float snr;           // 信噪比 (dB)
  bool detected;       // 是否超过阈值
};

class FrequencyExtractor {
public:
  // 构造/析构
  FrequencyExtractor(int fft_size = 512, int sample_rate = 44100);
  ~FrequencyExtractor();
  
  // 初始化
  bool begin();
  
  // 核心功能：提取频率窗口信号
  FreqResult extract(int32_t* raw_samples,    // 原始采样（立体声交织）
                     float center_freq,        // 中心频率 (Hz)
                     float window_width,       // 窗口宽度 (Hz)
                     float threshold = 10.0);  // 检测阈值
  
  // 提取（单声道输入）
  FreqResult extractMono(float* samples,      // 单声道float数据
                         float center_freq,
                         float window_width,
                         float threshold = 10.0);
  
  // 获取完整频谱
  void getSpectrum(int32_t* raw_samples, float* spectrum_out, int max_bins);
  
  // 获取参数
  float getFreqResolution() { return (float)_sample_rate / _fft_size; }
  int getFFTSize() { return _fft_size; }
  int getSampleRate() { return _sample_rate; }
  bool isReady() { return _initialized; }

private:
  int _fft_size;
  int _sample_rate;
  float* _fft_data;
  float* _window;
  bool _initialized;
  
  void doFFT();
  float calcMagnitude(int bin);
};

#endif