#ifndef OOK_DEMODULATOR_H
#define OOK_DEMODULATOR_H

#include <Arduino.h>
#include "esp_timer.h"  // 使用ESP32高精度定时器

// 解调器状态
enum DemodState {
  DEMOD_IDLE,
  DEMOD_SYNC,
  DEMOD_RECEIVING,
  DEMOD_COMPLETE
};

// 时间戳结构（微秒精度）
struct TimeStamp {
  int64_t signal_detected;    // 首次检测到信号的时间 (us)
  int64_t sync_complete;      // 同步完成的时间 (us)
  int64_t first_bit;          // 第一个数据bit的时间 (us)
  int64_t last_bit;           // 最后一个bit的时间 (us)
  int64_t packet_complete;    // 数据包完成的时间 (us)
};

// Bit时间戳（用于精确分析）
struct BitTiming {
  int64_t timestamp;          // 时间戳 (us)
  bool value;                 // bit值
  float amplitude;            // 信号强度
};

// 接收结果
struct DemodResult {
  bool complete;              // 是否接收完成
  uint8_t* data;              // 数据指针
  int length;                 // 数据长度
  
  // 微秒级时间戳
  TimeStamp time;             // 各阶段时间戳
  int64_t duration_us;        // 总耗时 (us)
  
  // Bit级时间（可选）
  BitTiming* bit_times;       // bit时间数组
  int bit_count;              // bit数量
};

// 回调函数类型（带时间戳）
typedef void (*DemodCallback)(uint8_t* data, int length, TimeStamp* time);

class OOKDemodulator {
public:
  OOKDemodulator(int bit_time_ms = 50,
                 float threshold_on = 15.0,
                 float threshold_off = 5.0,
                 uint8_t sync_pattern = 0xAA,
                 int buffer_size = 128);
  
  ~OOKDemodulator();
  
  bool begin();
  
  // 输入信号
  DemodResult feed(float amplitude);
  
  // 设置回调
  void onDataReceived(DemodCallback callback);
  
  // 重置
  void reset();
  
  // 获取状态
  DemodState getState() { return _state; }
  const char* getStateName();
  
  // 获取当前时间戳
  TimeStamp getTimeStamp() { return _timestamp; }
  
  // 获取信号检测时间（最关键的时间点）
  int64_t getSignalDetectedTime() { return _timestamp.signal_detected; }
  
  // 统计
  uint32_t getBytesReceived() { return _bytes_total; }
  uint32_t getPacketsReceived() { return _packets_total; }
  uint32_t getErrors() { return _errors; }
  
  // 配置
  void setThresholds(float on, float off);
  void setBitTime(int ms);
  void setSyncPattern(uint8_t pattern);
  void enableBitTiming(bool enable);  // 启用bit级时间记录

private:
  // 配置
  int _bit_time;
  float _threshold_on;
  float _threshold_off;
  uint8_t _sync_pattern;
  int _buffer_size;
  bool _bit_timing_enabled;
  
  // 状态
  DemodState _state;
  int64_t _bit_start;
  uint8_t _bit_buffer;
  int _bit_count;
  int _idle_count;
  
  // 时间戳
  TimeStamp _timestamp;
  
  // 数据缓冲
  uint8_t* _data_buffer;
  int _data_index;
  
  // Bit时间缓冲
  BitTiming* _bit_times;
  int _bit_times_index;
  int _max_bits;
  
  // 统计
  uint32_t _bytes_total;
  uint32_t _packets_total;
  uint32_t _errors;
  
  // 回调
  DemodCallback _callback;
  
  // 内部方法
  void processIdle(float amplitude);
  void processSync(float amplitude);
  void processReceiving(float amplitude);
  void finishPacket();
  void recordBit(bool value, float amplitude);
  
  // 获取微秒时间
  inline int64_t getMicros() {
    return esp_timer_get_time();  // 微秒精度
  }
};

#endif