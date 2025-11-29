#include "FrequencyExtractor.h"

FrequencyExtractor::FrequencyExtractor(int fft_size, int sample_rate) {
  _fft_size = fft_size;
  _sample_rate = sample_rate;
  _fft_data = nullptr;
  _window = nullptr;
  _initialized = false;
}

FrequencyExtractor::~FrequencyExtractor() {
  if (_fft_data) free(_fft_data);
  if (_window) free(_window);
}

bool FrequencyExtractor::begin() {
  // 分配对齐内存
  _fft_data = (float*)aligned_alloc(16, _fft_size * 2 * sizeof(float));
  _window = (float*)aligned_alloc(16, _fft_size * sizeof(float));
  
  if (!_fft_data || !_window) {
    return false;
  }
  
  // 初始化ESP-DSP FFT
  esp_err_t ret = dsps_fft2r_init_fc32(NULL, _fft_size);
  if (ret != ESP_OK) {
    return false;
  }
  
  // 生成汉宁窗
  dsps_wind_hann_f32(_window, _fft_size);
  
  _initialized = true;
  return true;
}

void FrequencyExtractor::doFFT() {
  dsps_fft2r_fc32(_fft_data, _fft_size);
  dsps_bit_rev_fc32(_fft_data, _fft_size);
  dsps_cplx2reC_fc32(_fft_data, _fft_size);
}

float FrequencyExtractor::calcMagnitude(int bin) {
  float real = _fft_data[bin * 2];
  float imag = _fft_data[bin * 2 + 1];
  return sqrtf(real * real + imag * imag);
}

FreqResult FrequencyExtractor::extract(int32_t* raw_samples,
                                       float center_freq,
                                       float window_width,
                                       float threshold) {
  FreqResult result = {0, 0, 0, 0, false};
  
  if (!_initialized) return result;
  
  // 填充数据（取左声道，偶数位置）
  memset(_fft_data, 0, _fft_size * 2 * sizeof(float));
  for (int i = 0; i < _fft_size; i++) {
    _fft_data[i] = ((float)raw_samples[i * 2] / 100000.0f) * _window[i];
  }
  
  // 执行FFT
  doFFT();
  
  // 计算bin范围
  float freq_res = getFreqResolution();
  int bin_low = max(1, (int)((center_freq - window_width / 2) / freq_res));
  int bin_high = min(_fft_size / 2 - 1, (int)((center_freq + window_width / 2) / freq_res));
  
  // 计算总能量（噪声基底）
  float noise_energy = 0;
  for (int i = 1; i < _fft_size / 2; i++) {
    noise_energy += calcMagnitude(i);
  }
  
  // 分析窗口内信号
  float max_mag = 0;
  int max_bin = bin_low;
  float window_energy = 0;
  
  for (int i = bin_low; i <= bin_high; i++) {
    float mag = calcMagnitude(i);
    window_energy += mag;
    
    if (mag > max_mag) {
      max_mag = mag;
      max_bin = i;
    }
  }
  
  // 填充结果
  result.amplitude = max_mag;
  result.peak_freq = max_bin * freq_res;
  result.total_energy = window_energy;
  
  float signal = window_energy;
  float noise = noise_energy - window_energy;
  result.snr = (noise > 0) ? 20.0f * log10f(signal / noise) : 0;
  
  result.detected = (max_mag > threshold);
  
  return result;
}

FreqResult FrequencyExtractor::extractMono(float* samples,
                                           float center_freq,
                                           float window_width,
                                           float threshold) {
  FreqResult result = {0, 0, 0, 0, false};
  
  if (!_initialized) return result;
  
  // 填充数据
  memset(_fft_data, 0, _fft_size * 2 * sizeof(float));
  for (int i = 0; i < _fft_size; i++) {
    _fft_data[i] = samples[i] * _window[i];
  }
  
  // 执行FFT
  doFFT();
  
  // 后续处理与extract相同
  float freq_res = getFreqResolution();
  int bin_low = max(1, (int)((center_freq - window_width / 2) / freq_res));
  int bin_high = min(_fft_size / 2 - 1, (int)((center_freq + window_width / 2) / freq_res));
  
  float max_mag = 0;
  int max_bin = bin_low;
  float window_energy = 0;
  float noise_energy = 0;
  
  for (int i = 1; i < _fft_size / 2; i++) {
    float mag = calcMagnitude(i);
    noise_energy += mag;
    
    if (i >= bin_low && i <= bin_high) {
      window_energy += mag;
      if (mag > max_mag) {
        max_mag = mag;
        max_bin = i;
      }
    }
  }
  
  result.amplitude = max_mag;
  result.peak_freq = max_bin * freq_res;
  result.total_energy = window_energy;
  
  float noise = noise_energy - window_energy;
  result.snr = (noise > 0) ? 20.0f * log10f(window_energy / noise) : 0;
  result.detected = (max_mag > threshold);
  
  return result;
}

void FrequencyExtractor::getSpectrum(int32_t* raw_samples, float* spectrum_out, int max_bins) {
  if (!_initialized) return;
  
  memset(_fft_data, 0, _fft_size * 2 * sizeof(float));
  for (int i = 0; i < _fft_size; i++) {
    _fft_data[i] = ((float)raw_samples[i * 2] / 100000.0f) * _window[i];
  }
  
  doFFT();
  
  int bins = min(max_bins, _fft_size / 2);
  for (int i = 0; i < bins; i++) {
    spectrum_out[i] = calcMagnitude(i);
  }
}