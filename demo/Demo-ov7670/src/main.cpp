#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

const char* ap_ssid = "OV7670-Camera";
const char* ap_password = "12345678";

#define PIN_D0      12
#define PIN_D1      13
#define PIN_D2      14
#define PIN_D3      15
#define PIN_D4      16
#define PIN_D5      17
#define PIN_D6      18
#define PIN_D7      19

#define PIN_PCLK    22
#define PIN_HREF    23
#define PIN_VSYNC   25
#define PIN_XCLK    4

#define PIN_SIOD    26
#define PIN_SIOC    27
#define PIN_RESET   32
#define PIN_PWDN    33

#define DATA_SHIFT  12
#define PCLK_MASK   (1<<22)
#define HREF_MASK   (1<<23)
#define VSYNC_MASK  (1<<25)

#define OV7670_ADDR 0x21

// ========== 分辨率定义 ==========
#define RES_QQVGA       0
#define RES_QVGA_2PART  1
#define RES_QVGA_4PART  2

typedef struct {
  uint8_t id;
  uint16_t fullWidth;
  uint16_t fullHeight;
  uint8_t parts;
  uint16_t partHeight;
  const char* name;
} Resolution;

const Resolution resolutions[] = {
  {RES_QQVGA,      160, 120, 1, 120, "160x120"},
  {RES_QVGA_2PART, 320, 240, 2, 120, "320x240/2"},
  {RES_QVGA_4PART, 320, 240, 4,  60, "320x240/4"},
};
#define RES_COUNT 3

// ========== 内存 ==========
#define MAX_PART_SIZE  (320 * 120 * 2)

uint8_t* frameBuffer0 = nullptr;
uint8_t* frameBuffer1 = nullptr;
uint8_t* compBuffer = nullptr;
bool buffersAllocated = false;

volatile uint8_t* captureBuffer;
volatile uint8_t* sendBuffer;

volatile uint16_t imgWidth = 160;
volatile uint16_t imgHeight = 120;
volatile uint16_t fullWidth = 160;
volatile uint16_t fullHeight = 120;
volatile uint8_t numParts = 1;
volatile uint8_t currentPart = 0;
volatile uint8_t readyPart = 0;
volatile bool partReady = false;

volatile uint8_t currentRes = RES_QQVGA;
volatile uint32_t frameCount = 0;
volatile uint32_t captureFPS = 0;
volatile uint32_t streamFPS = 0;
volatile uint8_t compressMode = 1;
volatile bool resChangePending = false;

#define MODE_RAW      0
#define MODE_RGB332   1
#define MODE_GRAY     2
#define MODE_RLE      3

SemaphoreHandle_t bufferMutex;
WebServer server(80);
WiFiServer streamServer(81);

// ========== 内存分配 ==========
bool allocateFixedBuffers() {
  if(buffersAllocated) return true;
  
  Serial.printf("分配缓冲区: 2 x %d bytes\n", MAX_PART_SIZE);
  
  frameBuffer0 = (uint8_t*)malloc(MAX_PART_SIZE);
  if(!frameBuffer0) return false;
  
  frameBuffer1 = (uint8_t*)malloc(MAX_PART_SIZE);
  if(!frameBuffer1) {
    free(frameBuffer0);
    frameBuffer0 = nullptr;
    return false;
  }
  
  compBuffer = (uint8_t*)malloc(MAX_PART_SIZE);
  
  captureBuffer = frameBuffer0;
  sendBuffer = frameBuffer1;
  buffersAllocated = true;
  
  Serial.printf("✓ 分配成功, 剩余堆: %d\n", ESP.getFreeHeap());
  return true;
}

// ========== 摄像头驱动 ==========
void cam_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(OV7670_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delay(1);
}

uint8_t cam_read(uint8_t reg) {
  Wire.beginTransmission(OV7670_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)OV7670_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

bool initCamera() {
  pinMode(PIN_PWDN, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_PWDN, HIGH);
  digitalWrite(PIN_RESET, LOW);
  delay(100);
  
  ledcSetup(0, 20000000, 1);
  ledcAttachPin(PIN_XCLK, 0);
  ledcWrite(0, 1);
  delay(50);
  
  digitalWrite(PIN_PWDN, LOW);
  delay(50);
  digitalWrite(PIN_RESET, HIGH);
  delay(300);
  
  Wire.begin(PIN_SIOD, PIN_SIOC, 100000);
  delay(50);
  
  for(int i = 0; i < 5; i++) {
    uint8_t pid = cam_read(0x0A);
    Serial.printf("PID: 0x%02X\n", pid);
    if(pid == 0x76) return true;
    delay(100);
  }
  return false;
}

// ========== OV7670配置 - 优化QVGA设置 ==========
void configureOV7670(bool isQVGA) {
  cam_write(0x12, 0x80);  // 复位
  delay(100);
  
  // 基础设置
  cam_write(0x11, 0x01);  // CLKRC: 内部时钟预分频
  cam_write(0x3A, 0x04);  // TSLB
  cam_write(0x12, 0x04);  // COM7: RGB输出
  cam_write(0x40, 0xD0);  // COM15: RGB565, 全范围
  cam_write(0x8C, 0x00);  // RGB444
  cam_write(0x04, 0x00);  // COM1
  cam_write(0x3D, 0x80);  // COM13
  cam_write(0x15, 0x00);  // COM10: HREF不反转
  cam_write(0x6B, 0x0A);  // DBLV: PLL x4
  
  if(isQVGA) {
    // QVGA 320x240
    cam_write(0x0C, 0x04);  // COM3: 使能缩放
    cam_write(0x3E, 0x19);  // COM14: PCLK分频, 手动缩放
    cam_write(0x70, 0x3A);  // SCALING_XSC
    cam_write(0x71, 0x35);  // SCALING_YSC
    cam_write(0x72, 0x11);  // SCALING_DCWCTR: 水平和垂直2分频
    cam_write(0x73, 0xF1);  // SCALING_PCLK_DIV
    cam_write(0xA2, 0x02);  // PCLK延迟
    
    // 窗口设置 - QVGA
    cam_write(0x17, 0x16);  // HSTART
    cam_write(0x18, 0x04);  // HSTOP
    cam_write(0x32, 0xA4);  // HREF - 调整边缘
    cam_write(0x19, 0x02);  // VSTART
    cam_write(0x1A, 0x7A);  // VSTOP
    cam_write(0x03, 0x0A);  // VREF
  } else {
    // QQVGA 160x120
    cam_write(0x0C, 0x04);  // COM3: 使能缩放
    cam_write(0x3E, 0x1A);  // COM14: PCLK分频x4
    cam_write(0x70, 0x3A);
    cam_write(0x71, 0x35);
    cam_write(0x72, 0x22);  // 4分频
    cam_write(0x73, 0xF2);
    cam_write(0xA2, 0x02);
    
    cam_write(0x17, 0x16);
    cam_write(0x18, 0x04);
    cam_write(0x32, 0xA4);
    cam_write(0x19, 0x02);
    cam_write(0x1A, 0x7A);
    cam_write(0x03, 0x0A);
  }
  
  // 自动控制
  cam_write(0x13, 0xE7);  // COM8: AGC/AWB/AEC
  cam_write(0x14, 0x28);  // COM9: AGC增益上限
  cam_write(0x01, 0x80);  // BLUE
  cam_write(0x02, 0x80);  // RED
  
  // 颜色矩阵
  cam_write(0x4F, 0x80);
  cam_write(0x50, 0x80);
  cam_write(0x51, 0x00);
  cam_write(0x52, 0x22);
  cam_write(0x53, 0x5E);
  cam_write(0x54, 0x80);
  cam_write(0x58, 0x9E);
  cam_write(0xB0, 0x84);
  cam_write(0x6F, 0x9F);
  cam_write(0x55, 0x00);  // 亮度
  cam_write(0x56, 0x40);  // 对比度
  
  delay(300);
}

void switchResolution(uint8_t newRes) {
  if(newRes >= RES_COUNT) newRes = RES_QQVGA;
  
  const Resolution& r = resolutions[newRes];
  Serial.printf("\n>>> 切换到 %s <<<\n", r.name);
  
  uint32_t partSize = r.fullWidth * r.partHeight * 2;
  if(partSize > MAX_PART_SIZE) {
    Serial.printf("❌ 块大小超限\n");
    return;
  }
  
  bool isQVGA = (r.fullWidth == 320);
  configureOV7670(isQVGA);
  
  fullWidth = r.fullWidth;
  fullHeight = r.fullHeight;
  imgWidth = r.fullWidth;
  imgHeight = r.partHeight;
  numParts = r.parts;
  currentRes = newRes;
  currentPart = 0;
  partReady = false;
  
  Serial.printf("✓ %dx%d 分%d块\n", fullWidth, fullHeight, numParts);
}

// ========== 采集函数 - 修复版 ==========
// 整帧采集（用于160x120和调试）
void IRAM_ATTR captureFullFrame(volatile uint8_t* buf, uint16_t w, uint16_t h) {
  register volatile uint32_t* gpio = &GPIO.in;
  register uint32_t g;
  
  while(*gpio & VSYNC_MASK);
  while(!(*gpio & VSYNC_MASK));
  while(*gpio & VSYNC_MASK);
  
  portDISABLE_INTERRUPTS();
  
  volatile uint8_t* p = buf;
  for(int y = 0; y < h; y++) {
    while(!(*gpio & HREF_MASK));
    for(int x = 0; x < w * 2; x++) {
      while(*gpio & PCLK_MASK);
      while(!(*gpio & PCLK_MASK));
      g = *gpio;
      *p++ = (g >> DATA_SHIFT) & 0xFF;
    }
    while(*gpio & HREF_MASK);
  }
  
  portENABLE_INTERRUPTS();
}

// 分块采集 - 修复同步问题
void IRAM_ATTR capturePartToBuffer(volatile uint8_t* buf, uint16_t startLine, uint16_t numLines, uint16_t w, uint16_t totalH) {
  register volatile uint32_t* gpio = &GPIO.in;
  register uint32_t g;
  
  uint16_t endLine = startLine + numLines;
  int bytesPerLine = w * 2;
  
  // 等待新帧开始
  while(*gpio & VSYNC_MASK);      // 等待VSYNC低
  while(!(*gpio & VSYNC_MASK));   // 等待VSYNC高（帧开始）
  while(*gpio & VSYNC_MASK);      // 等待VSYNC低（有效数据开始）
  
  portDISABLE_INTERRUPTS();
  
  volatile uint8_t* p = buf;
  
  for(int y = 0; y < totalH; y++) {
    // 等待行开始
    while(!(*gpio & HREF_MASK));
    
    if(y >= startLine && y < endLine) {
      // 需要保存的行
      for(int x = 0; x < bytesPerLine; x++) {
        while(*gpio & PCLK_MASK);
        while(!(*gpio & PCLK_MASK));
        g = *gpio;
        *p++ = (g >> DATA_SHIFT) & 0xFF;
      }
    } else {
      // 跳过的行 - 仍然需要读取所有像素以保持同步
      for(int x = 0; x < bytesPerLine; x++) {
        while(*gpio & PCLK_MASK);
        while(!(*gpio & PCLK_MASK));
      }
    }
    
    // 等待行结束
    while(*gpio & HREF_MASK);
  }
  
  portENABLE_INTERRUPTS();
}

// ========== 另一种方法：按行号精确采集 ==========
// 这个版本更可靠，通过计数确保同步
void IRAM_ATTR capturePartV2(volatile uint8_t* buf, uint8_t partNum, uint8_t totalParts, uint16_t w, uint16_t totalH) {
  register volatile uint32_t* gpio = &GPIO.in;
  register uint32_t g;
  
  uint16_t linesPerPart = totalH / totalParts;
  uint16_t startLine = partNum * linesPerPart;
  uint16_t endLine = startLine + linesPerPart;
  int bytesPerLine = w * 2;
  
  // 等待新帧
  while(*gpio & VSYNC_MASK);
  while(!(*gpio & VSYNC_MASK));
  while(*gpio & VSYNC_MASK);
  
  portDISABLE_INTERRUPTS();
  
  volatile uint8_t* p = buf;
  int lineCount = 0;
  
  while(lineCount < totalH) {
    // 等待HREF高（行有效）
    uint32_t timeout = 100000;
    while(!(*gpio & HREF_MASK) && --timeout);
    if(timeout == 0) break;
    
    if(lineCount >= startLine && lineCount < endLine) {
      // 采集这行
      for(int x = 0; x < bytesPerLine; x++) {
        while(*gpio & PCLK_MASK);
        while(!(*gpio & PCLK_MASK));
        g = *gpio;
        *p++ = (g >> DATA_SHIFT) & 0xFF;
      }
    } else {
      // 跳过但保持同步
      for(int x = 0; x < bytesPerLine; x++) {
        while(*gpio & PCLK_MASK);
        while(!(*gpio & PCLK_MASK));
      }
    }
    
    // 等待行结束
    while(*gpio & HREF_MASK);
    lineCount++;
  }
  
  portENABLE_INTERRUPTS();
}

// ========== 压缩函数 ==========
size_t compressRGB332(uint8_t* src, uint8_t* dst, uint32_t srcSize) {
  size_t outIdx = 0;
  for(size_t i = 0; i < srcSize; i += 2) {
    uint16_t rgb565 = (src[i] << 8) | src[i + 1];
    uint8_t r = (rgb565 >> 13) & 0x07;
    uint8_t g = (rgb565 >> 8) & 0x07;
    uint8_t b = (rgb565 >> 3) & 0x03;
    dst[outIdx++] = (r << 5) | (g << 2) | b;
  }
  return outIdx;
}

size_t compressGray(uint8_t* src, uint8_t* dst, uint32_t srcSize) {
  size_t outIdx = 0;
  for(size_t i = 0; i < srcSize; i += 2) {
    uint16_t rgb565 = (src[i] << 8) | src[i + 1];
    uint8_t r = (rgb565 >> 11) & 0x1F;
    uint8_t g = (rgb565 >> 5) & 0x3F;
    uint8_t b = rgb565 & 0x1F;
    dst[outIdx++] = (r * 8 + g * 4 + b * 8) / 4;
  }
  return outIdx;
}

size_t compressRLE(uint8_t* src, uint8_t* dst, uint32_t srcSize, uint32_t maxOut) {
  size_t outIdx = 0;
  uint8_t lastVal = 0xFF;
  uint8_t runLen = 0;
  
  for(size_t i = 0; i < srcSize && outIdx < maxOut - 2; i += 2) {
    uint16_t rgb565 = (src[i] << 8) | src[i + 1];
    uint8_t r = (rgb565 >> 13) & 0x07;
    uint8_t g = (rgb565 >> 8) & 0x07;
    uint8_t b = (rgb565 >> 3) & 0x03;
    uint8_t val = (r << 5) | (g << 2) | b;
    
    if(val == lastVal && runLen < 255) {
      runLen++;
    } else {
      if(runLen > 0) {
        dst[outIdx++] = runLen;
        dst[outIdx++] = lastVal;
      }
      lastVal = val;
      runLen = 1;
    }
  }
  if(runLen > 0 && outIdx < maxOut - 1) {
    dst[outIdx++] = runLen;
    dst[outIdx++] = lastVal;
  }
  
  if(outIdx >= srcSize / 2) {
    return compressRGB332(src, dst, srcSize);
  }
  return outIdx;
}

size_t compressFrame(uint8_t* src, uint8_t* dst, uint8_t mode, uint32_t srcSize) {
  switch(mode) {
    case MODE_RGB332: return compressRGB332(src, dst, srcSize);
    case MODE_GRAY:   return compressGray(src, dst, srcSize);
    case MODE_RLE:    return compressRLE(src, dst, srcSize, srcSize);
    default:
      if(src != dst) memcpy(dst, src, srcSize);
      return srcSize;
  }
}

// ========== 采集任务 ==========
void captureTask(void* param) {
  Serial.printf("📹 采集任务 Core %d\n", xPortGetCoreID());
  uint32_t lastFPSTime = millis();
  uint32_t fpsCounter = 0;
  
  while(1) {
    if(resChangePending) {
      xSemaphoreTake(bufferMutex, portMAX_DELAY);
      switchResolution(currentRes);
      resChangePending = false;
      xSemaphoreGive(bufferMutex);
      delay(100);
      continue;
    }
    
    if(!captureBuffer) {
      vTaskDelay(100);
      continue;
    }
    
    // 根据分块数选择采集方式
    if(numParts == 1) {
      // 整帧采集
      captureFullFrame(captureBuffer, fullWidth, fullHeight);
    } else {
      // 分块采集 - 使用V2版本
      capturePartV2(captureBuffer, currentPart, numParts, fullWidth, fullHeight);
    }
    
    if(xSemaphoreTake(bufferMutex, 0) == pdTRUE) {
      volatile uint8_t* temp = captureBuffer;
      captureBuffer = sendBuffer;
      sendBuffer = temp;
      readyPart = currentPart;
      partReady = true;
      xSemaphoreGive(bufferMutex);
    }
    
    currentPart++;
    if(currentPart >= numParts) {
      currentPart = 0;
      frameCount++;
      fpsCounter++;
    }
    
    if(millis() - lastFPSTime >= 1000) {
      captureFPS = fpsCounter;
      fpsCounter = 0;
      lastFPSTime = millis();
    }
    
    vTaskDelay(1);
  }
}

// ========== 流处理 ==========
void handleStreamClient(WiFiClient& client) {
  Serial.printf("📺 客户端连接\n");
  client.setNoDelay(true);
  
  unsigned long timeout = millis() + 200;
  while(client.connected() && millis() < timeout) {
    if(client.available()) { client.read(); timeout = millis() + 10; }
  }
  
  client.print("HTTP/1.1 200 OK\r\n");
  client.print("Content-Type: application/octet-stream\r\n");
  client.print("Access-Control-Allow-Origin: *\r\n");
  client.print("Connection: close\r\n\r\n");
  
  uint32_t fpsCount = 0;
  uint32_t fpsTime = millis();
  
  uint8_t header[16];
  header[0] = 0xAA;
  header[1] = 0x55;
  
  while(client.connected()) {
    if(!partReady) {
      vTaskDelay(2);
      continue;
    }
    
    xSemaphoreTake(bufferMutex, portMAX_DELAY);
    
    if(!sendBuffer) {
      xSemaphoreGive(bufferMutex);
      break;
    }
    
    uint8_t mode = compressMode;
    uint16_t w = imgWidth;
    uint16_t h = imgHeight;
    uint32_t partSize = w * h * 2;
    uint8_t part = readyPart;
    uint8_t total = numParts;
    uint16_t fw = fullWidth;
    uint16_t fh = fullHeight;
    
    partReady = false;
    
    uint8_t* compDst = compBuffer ? compBuffer : (uint8_t*)sendBuffer;
    size_t compSize = compressFrame((uint8_t*)sendBuffer, compDst, mode, partSize);
    
    header[2] = mode;
    header[3] = (compSize >> 8) & 0xFF;
    header[4] = compSize & 0xFF;
    header[5] = captureFPS;
    header[6] = streamFPS;
    header[7] = (frameCount >> 8) & 0xFF;
    header[8] = frameCount & 0xFF;
    header[9] = (compSize * 100) / partSize;
    header[10] = (fw >> 8) & 0xFF;
    header[11] = fw & 0xFF;
    header[12] = (fh >> 8) & 0xFF;
    header[13] = fh & 0xFF;
    header[14] = part;
    header[15] = total;
    
    bool ok = (client.write(header, 16) == 16);
    
    if(ok) {
      size_t sent = 0;
      while(sent < compSize) {
        size_t chunk = min((size_t)2048, compSize - sent);
        size_t wr = client.write(compDst + sent, chunk);
        if(wr == 0) { ok = false; break; }
        sent += wr;
      }
    }
    
    xSemaphoreGive(bufferMutex);
    
    if(!ok) break;
    
    fpsCount++;
    if(millis() - fpsTime >= 1000) {
      streamFPS = fpsCount;
      fpsCount = 0;
      fpsTime = millis();
    }
  }
  
  Serial.printf("📺 断开\n");
}

void httpTask(void* param) {
  Serial.printf("🌐 HTTP Core %d\n", xPortGetCoreID());
  streamServer.begin();
  
  while(1) {
    server.handleClient();
    WiFiClient streamClient = streamServer.available();
    if(streamClient) handleStreamClient(streamClient);
    vTaskDelay(1);
  }
}

// ========== HTML ==========
const char htmlTemplate[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OV7670</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1a1a1a;color:#fff;font-family:system-ui;padding:10px}
.c{max-width:500px;margin:0 auto}
h1{text-align:center;color:#4af;padding:8px;font-size:16px}
.cv-wrap{width:100%;aspect-ratio:4/3;background:#000;border-radius:8px;overflow:hidden;position:relative}
canvas{width:100%;height:100%;image-rendering:pixelated;display:block}
.part-ind{position:absolute;top:5px;right:5px;background:rgba(0,0,0,0.7);padding:4px 8px;border-radius:4px;font-size:11px}
.ctrl{background:#2a2a2a;padding:10px;border-radius:8px;margin:8px 0}
.row{display:flex;gap:6px;margin-bottom:8px}
button{flex:1;padding:12px;border:none;border-radius:6px;font-size:12px;cursor:pointer;font-weight:bold}
.g{background:#2a2;color:#fff}
.r{background:#a22;color:#fff}
.b{background:#26a;color:#fff}
select{flex:1;padding:12px;border-radius:6px;background:#444;color:#fff;border:none;font-size:13px}
.info{display:grid;grid-template-columns:repeat(5,1fr);gap:4px;margin:8px 0}
.box{background:#2a2a2a;padding:6px;border-radius:6px;text-align:center}
.num{font-size:16px;font-weight:bold}
.c1{color:#4f8}.c2{color:#48f}.c3{color:#f84}.c4{color:#fa0}.c5{color:#f4a}
.lbl{font-size:9px;color:#888}
.st{background:#2a2a2a;border-radius:6px;padding:8px;text-align:center;font-size:12px}
</style>
</head><body>
<div class="c">
<h1>📷 OV7670 分块传输</h1>
<div class="cv-wrap">
<canvas id="cv" width="320" height="240"></canvas>
<div class="part-ind" id="pind">块: -/-</div>
</div>
<div class="ctrl">
<div class="row">
<select id="res" onchange="setRes()">
<option value="0">160x120 整帧</option>
<option value="1">320x240 分2块</option>
<option value="2" selected>320x240 分4块</option>
</select>
<select id="mode" onchange="setMode()">
<option value="0">原始RGB565</option>
<option value="1" selected>RGB332压缩</option>
<option value="2">灰度</option>
<option value="3">RLE压缩</option>
</select>
</div>
<div class="row">
<button class="g" onclick="start()">▶ 开始</button>
<button class="r" onclick="stop()">⏹ 停止</button>
<button class="b" onclick="getInfo()">ℹ 信息</button>
</div>
</div>
<div class="info">
<div class="box"><div class="num c1" id="cfps">-</div><div class="lbl">采集FPS</div></div>
<div class="box"><div class="num c2" id="sfps">-</div><div class="lbl">流FPS</div></div>
<div class="box"><div class="num c3" id="nfps">-</div><div class="lbl">显示FPS</div></div>
<div class="box"><div class="num c4" id="kb">-</div><div class="lbl">KB/s</div></div>
<div class="box"><div class="num c5" id="ratio">-</div><div class="lbl">压缩率</div></div>
</div>
<div class="st" id="st">就绪</div>
</div>
<script>
var run=0,reader=null,cv=document.getElementById("cv"),ctx=cv.getContext("2d");
var dispFps=0,dispTime=Date.now(),totalBytes=0;
var fullFrame=null,fullW=0,fullH=0,totalParts=1;

function setRes(){fetch("/r?v="+document.getElementById("res").value)}
function setMode(){fetch("/m?v="+document.getElementById("mode").value)}
function getInfo(){
  fetch("/info").then(r=>r.json()).then(j=>{
    document.getElementById("st").textContent=j.fw+"x"+j.fh+" 分"+j.parts+"块 | 堆:"+Math.round(j.heap/1024)+"KB";
  });
}

function upd(b){
  dispFps++;totalBytes+=b;
  var n=Date.now();
  if(n-dispTime>=1000){
    document.getElementById("nfps").textContent=dispFps;
    document.getElementById("kb").textContent=Math.round(totalBytes/1024);
    dispFps=0;totalBytes=0;dispTime=n;
  }
}

function dec565(d,img,offset,count){
  for(var p=offset*4,i=0;i<d.length-1&&i/2<count;i+=2){
    var c=(d[i]<<8)|d[i+1];
    img.data[p++]=(c>>8)&0xF8;img.data[p++]=(c>>3)&0xFC;img.data[p++]=(c<<3)&0xF8;img.data[p++]=255;
  }
}
function dec332(d,img,offset,count){
  for(var p=offset*4,i=0;i<d.length&&i<count;i++){
    var c=d[i];img.data[p++]=c&0xE0;img.data[p++]=(c&0x1C)<<3;img.data[p++]=(c&0x03)<<6;img.data[p++]=255;
  }
}
function decGray(d,img,offset,count){
  for(var p=offset*4,i=0;i<d.length&&i<count;i++){
    img.data[p++]=d[i];img.data[p++]=d[i];img.data[p++]=d[i];img.data[p++]=255;
  }
}
function decRLE(d,img,offset,maxCount){
  var p=offset*4,cnt=0;
  for(var i=0;i<d.length-1&&cnt<maxCount;i+=2){
    var n=d[i],c=d[i+1],r=c&0xE0,g=(c&0x1C)<<3,b=(c&0x03)<<6;
    for(var j=0;j<n&&cnt<maxCount;j++,cnt++){
      img.data[p++]=r;img.data[p++]=g;img.data[p++]=b;img.data[p++]=255;
    }
  }
}

function processPart(d,m,fw,fh,partNum,totalP){
  if(!fullFrame||fullW!=fw||fullH!=fh||totalParts!=totalP){
    fullW=fw;fullH=fh;totalParts=totalP;
    cv.width=fw;cv.height=fh;
    fullFrame=ctx.createImageData(fw,fh);
    for(var i=0;i<fullFrame.data.length;i+=4){
      fullFrame.data[i]=0;fullFrame.data[i+1]=0;fullFrame.data[i+2]=0;fullFrame.data[i+3]=255;
    }
  }
  var partH=fh/totalP;
  var pixelOffset=partNum*fw*partH;
  var pixelCount=fw*partH;
  if(m==0)dec565(d,fullFrame,pixelOffset,pixelCount);
  else if(m==1)dec332(d,fullFrame,pixelOffset,pixelCount);
  else if(m==2)decGray(d,fullFrame,pixelOffset,pixelCount);
  else if(m==3)decRLE(d,fullFrame,pixelOffset,pixelCount);
  document.getElementById("pind").textContent="块:"+(partNum+1)+"/"+totalP;
  if(partNum==totalP-1)ctx.putImageData(fullFrame,0,0);
}

async function start(){
  stop();await new Promise(r=>setTimeout(r,100));
  run=1;document.getElementById("st").textContent="连接中...";
  fullFrame=null;
  try{
    var res=await fetch("http://"+location.hostname+":81/");
    reader=res.body.getReader();
    document.getElementById("st").textContent="已连接";
    var buf=new Uint8Array(0);
    while(run){
      var{done,value}=await reader.read();if(done)break;
      var tmp=new Uint8Array(buf.length+value.length);tmp.set(buf);tmp.set(value,buf.length);buf=tmp;
      while(buf.length>=16){
        if(buf[0]!=0xAA||buf[1]!=0x55){
          var f=false;for(var i=1;i<buf.length-1;i++)if(buf[i]==0xAA&&buf[i+1]==0x55){buf=buf.slice(i);f=true;break;}
          if(!f)buf=new Uint8Array(0);continue;
        }
        var m=buf[2],len=(buf[3]<<8)|buf[4],cf=buf[5],sf=buf[6],rt=buf[9];
        var fw=(buf[10]<<8)|buf[11],fh=(buf[12]<<8)|buf[13];
        var partNum=buf[14],totalP=buf[15];
        if(buf.length<16+len)break;
        document.getElementById("cfps").textContent=cf;
        document.getElementById("sfps").textContent=sf;
        document.getElementById("ratio").textContent=rt+"%";
        processPart(buf.slice(16,16+len),m,fw,fh,partNum,totalP);
        upd(len);
        buf=buf.slice(16+len);
      }
      if(buf.length>200000)buf=new Uint8Array(0);
    }
  }catch(e){console.log(e);}
  document.getElementById("st").textContent="已断开";
}

function stop(){run=0;if(reader){try{reader.cancel();}catch(e){}reader=null;}document.getElementById("st").textContent="已停止";}
getInfo();
</script>
</body></html>)rawliteral";

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========== OV7670 分块传输 ==========\n");
  
  bufferMutex = xSemaphoreCreateMutex();
  
  for(int i = 12; i <= 19; i++) pinMode(i, INPUT);
  pinMode(PIN_PCLK, INPUT);
  pinMode(PIN_HREF, INPUT);
  pinMode(PIN_VSYNC, INPUT);
  
  if(!initCamera()) {
    Serial.println("❌ 摄像头初始化失败!");
    while(1) delay(1000);
  }
  Serial.println("✓ 摄像头初始化成功");
  
  if(!allocateFixedBuffers()) {
    Serial.println("❌ 内存分配失败!");
    while(1) delay(1000);
  }
  
  switchResolution(RES_QVGA_4PART);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.printf("🌐 http://%s/\n", WiFi.softAPIP().toString().c_str());
  
  server.on("/", [](){ server.send_P(200, "text/html", htmlTemplate); });
  server.on("/m", [](){
    if(server.hasArg("v")) compressMode = server.arg("v").toInt();
    server.send(200, "text/plain", "OK");
  });
  server.on("/r", [](){
    if(server.hasArg("v")) {
      uint8_t newRes = server.arg("v").toInt();
      if(newRes < RES_COUNT) {
        currentRes = newRes;
        resChangePending = true;
      }
    }
    server.send(200, "text/plain", "OK");
  });
  server.on("/info", [](){
    char json[128];
    snprintf(json, sizeof(json), 
      "{\"fw\":%d,\"fh\":%d,\"parts\":%d,\"heap\":%d}",
      fullWidth, fullHeight, numParts, ESP.getFreeHeap());
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", json);
  });
  
  server.begin();
  
  xTaskCreatePinnedToCore(httpTask, "HTTP", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(captureTask, "Cap", 4096, NULL, 2, NULL, 1);
  
  Serial.println("✓ 启动完成\n");
}

void loop() {
  static uint32_t last = 0;
  if(millis() - last > 5000) {
    Serial.printf("[%dx%d/%d] 帧:%lu 采集:%lu 流:%lu\n", 
      fullWidth, fullHeight, numParts, frameCount, captureFPS, streamFPS);
    last = millis();
  }
  vTaskDelay(100);
}