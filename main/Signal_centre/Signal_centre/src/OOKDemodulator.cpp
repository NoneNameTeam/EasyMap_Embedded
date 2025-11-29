#include "OOKDemodulator.h"

OOKDemodulator::OOKDemodulator(int bit_time_ms,
                               float threshold_on,
                               float threshold_off,
                               uint8_t sync_pattern,
                               int buffer_size) {
  _bit_time = bit_time_ms;
  _threshold_on = threshold_on;
  _threshold_off = threshold_off;
  _sync_pattern = sync_pattern;
  _buffer_size = buffer_size;
  _bit_timing_enabled = false;
  
  _data_buffer = nullptr;
  _bit_times = nullptr;
  _callback = nullptr;
  _max_bits = buffer_size * 8 + 16;  // 数据bits + 同步bits
  
  _state = DEMOD_IDLE;
  _bytes_total = 0;
  _packets_total = 0;
  _errors = 0;
}

OOKDemodulator::~OOKDemodulator() {
  if (_data_buffer) free(_data_buffer);
  if (_bit_times) free(_bit_times);
}

bool OOKDemodulator::begin() {
  _data_buffer = (uint8_t*)malloc(_buffer_size);
  if (!_data_buffer) return false;
  
  _bit_times = (BitTiming*)malloc(_max_bits * sizeof(BitTiming));
  if (!_bit_times) {
    free(_data_buffer);
    return false;
  }
  
  reset();
  return true;
}

void OOKDemodulator::reset() {
  _state = DEMOD_IDLE;
  _bit_buffer = 0;
  _bit_count = 0;
  _data_index = 0;
  _idle_count = 0;
  _bit_start = 0;
  _bit_times_index = 0;
  
  // 清空时间戳
  memset(&_timestamp, 0, sizeof(TimeStamp));
  
  if (_data_buffer) {
    memset(_data_buffer, 0, _buffer_size);
  }
  if (_bit_times) {
    memset(_bit_times, 0, _max_bits * sizeof(BitTiming));
  }
}

void OOKDemodulator::onDataReceived(DemodCallback callback) {
  _callback = callback;
}

void OOKDemodulator::enableBitTiming(bool enable) {
  _bit_timing_enabled = enable;
}

const char* OOKDemodulator::getStateName() {
  switch (_state) {
    case DEMOD_IDLE: return "IDLE";
    case DEMOD_SYNC: return "SYNC";
    case DEMOD_RECEIVING: return "RECV";
    case DEMOD_COMPLETE: return "DONE";
    default: return "????";
  }
}

void OOKDemodulator::setThresholds(float on, float off) {
  _threshold_on = on;
  _threshold_off = off;
}

void OOKDemodulator::setBitTime(int ms) {
  _bit_time = ms;
}

void OOKDemodulator::setSyncPattern(uint8_t pattern) {
  _sync_pattern = pattern;
}

void OOKDemodulator::recordBit(bool value, float amplitude) {
  if (_bit_timing_enabled && _bit_times_index < _max_bits) {
    _bit_times[_bit_times_index].timestamp = getMicros();
    _bit_times[_bit_times_index].value = value;
    _bit_times[_bit_times_index].amplitude = amplitude;
    _bit_times_index++;
  }
}

DemodResult OOKDemodulator::feed(float amplitude) {
  DemodResult result = {0};
  result.complete = false;
  result.data = nullptr;
  result.length = 0;
  result.bit_times = nullptr;
  result.bit_count = 0;
  
  switch (_state) {
    case DEMOD_IDLE:
      processIdle(amplitude);
      break;
      
    case DEMOD_SYNC:
      processSync(amplitude);
      break;
      
    case DEMOD_RECEIVING:
      processReceiving(amplitude);
      break;
      
    case DEMOD_COMPLETE:
      // 填充结果
      result.complete = true;
      result.data = _data_buffer;
      result.length = _data_index;
      result.time = _timestamp;
      result.duration_us = _timestamp.packet_complete - _timestamp.signal_detected;
      
      if (_bit_timing_enabled) {
        result.bit_times = _bit_times;
        result.bit_count = _bit_times_index;
      }
      
      // 调用回调
      if (_callback && _data_index > 0) {
        _callback(_data_buffer, _data_index, &_timestamp);
      }
      
      _packets_total++;
      reset();
      break;
  }
  
  return result;
}

void OOKDemodulator::processIdle(float amplitude) {
  if (amplitude > _threshold_on) {
    // ★ 记录信号首次检测时间（最关键的时间点）
    _timestamp.signal_detected = getMicros();
    
    _state = DEMOD_SYNC;
    _bit_buffer = 0;
    _bit_count = 0;
    _bit_start = getMicros();
    _idle_count = 0;
    _bit_times_index = 0;
    
    recordBit(true, amplitude);
  }
}

void OOKDemodulator::processSync(float amplitude) {
  int64_t now = getMicros();
  
  // 采样时间到（转换为微秒比较）
  if (now - _bit_start >= _bit_time * 1000) {
    bool bit = (amplitude > _threshold_on) ? 1 : 0;
    
    _bit_buffer = (_bit_buffer << 1) | bit;
    _bit_count++;
    _bit_start = now;
    
    recordBit(bit, amplitude);
    
    // 检查同步模式
    if (_bit_count >= 8) {
      if (_bit_buffer == _sync_pattern) {
        // ★ 记录同步完成时间
        _timestamp.sync_complete = now;
        
        _state = DEMOD_RECEIVING;
        _data_index = 0;
        _bit_count = 0;
        _bit_buffer = 0;
      } else if (_bit_count > 32) {
        _errors++;
        reset();
      }
    }
  }
  
  // 信号丢失
  if (amplitude < _threshold_off) {
    _idle_count++;
    if (_idle_count > 10) {
      reset();
    }
  } else {
    _idle_count = 0;
  }
}

void OOKDemodulator::processReceiving(float amplitude) {
  int64_t now = getMicros();
  
  // 采样时间到
  if (now - _bit_start >= _bit_time * 1000) {
    bool bit = (amplitude > _threshold_on) ? 1 : 0;
    
    _bit_buffer = (_bit_buffer << 1) | bit;
    _bit_count++;
    _bit_start = now;
    
    recordBit(bit, amplitude);
    
    // 记录第一个数据bit时间
    if (_data_index == 0 && _bit_count == 1) {
      _timestamp.first_bit = now;
    }
    
    // 完成一个字节
    if (_bit_count == 8) {
      _data_buffer[_data_index++] = _bit_buffer;
      _bytes_total++;
      _bit_count = 0;
      _bit_buffer = 0;
      
      // ★ 更新最后bit时间
      _timestamp.last_bit = now;
      
      // 检查结束
      if (_data_buffer[_data_index - 1] == 0x00 || _data_index >= _buffer_size - 1) {
        finishPacket();
      }
    }
  }
  
  // 信号丢失
  if (amplitude < _threshold_off) {
    _idle_count++;
    if (_idle_count > 15) {
      finishPacket();
    }
  } else {
    _idle_count = 0;
  }
}

void OOKDemodulator::finishPacket() {
  // ★ 记录数据包完成时间
  _timestamp.packet_complete = getMicros();
  
  if (_data_index > 0) {
    _state = DEMOD_COMPLETE;
  } else {
    reset();
  }
}