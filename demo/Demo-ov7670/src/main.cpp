#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

const char* ap_ssid = "OV7670-Camera";
const char* ap_password = "12345678";

// 你现在的新接线
#define PIN_SIOD    26   // SDA
#define PIN_SIOC    27   // SCL
#define PIN_VSYNC   25   // VS
#define PIN_HREF    23   // HS
#define PIN_XCLK    4    // MCLK
#define PIN_PCLK    22   // PCLK
#define PIN_RESET   14
#define PIN_PWDN    12

// D0-D3 在 GPIO.in (0-31)
#define PIN_D0      5
#define PIN_D1      18
#define PIN_D2      19
#define PIN_D3      21

// D4-D7 在 GPIO.in1 (32-39)
#define PIN_D4      36   // VP
#define PIN_D5      39   // VN
#define PIN_D6      34
#define PIN_D7      35

// GPIO.in 位掩码 (GPIO 0-31)
#define D0_MASK     (1<<5)
#define D1_MASK     (1<<18)
#define D2_MASK     (1<<19)
#define D3_MASK     (1<<21)
#define PCLK_MASK   (1<<22)
#define HREF_MASK   (1<<23)
#define VSYNC_MASK  (1<<25)

// GPIO.in1 位掩码 (GPIO 32-39, 需要减32)
#define D4_MASK     (1<<(36-32))  // bit 4
#define D5_MASK     (1<<(39-32))  // bit 7
#define D6_MASK     (1<<(34-32))  // bit 2
#define D7_MASK     (1<<(35-32))  // bit 3

#define OV7670_ADDR 0x21
#define IMG_W 160
#define IMG_H 120
#define IMG_SIZE (IMG_W * IMG_H * 2)

uint8_t frameBuffer[IMG_SIZE];
WebServer server(80);

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
  Serial.println("复位摄像头...");
  pinMode(PIN_PWDN, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  
  digitalWrite(PIN_PWDN, HIGH);
  digitalWrite(PIN_RESET, LOW);
  delay(100);
  
  // XCLK 10MHz
  Serial.println("启动 XCLK...");
  ledcSetup(0, 10000000, 1);
  ledcAttachPin(PIN_XCLK, 0);
  ledcWrite(0, 1);
  delay(50);
  
  digitalWrite(PIN_PWDN, LOW);
  delay(50);
  digitalWrite(PIN_RESET, HIGH);
  delay(300);
  
  // I2C
  Serial.printf("初始化 I2C (SDA=%d, SCL=%d)...\n", PIN_SIOD, PIN_SIOC);
  Wire.begin(PIN_SIOD, PIN_SIOC, 100000);
  delay(50);
  
  // 检测摄像头
  Serial.println("检测摄像头...");
  for(int i = 0; i < 5; i++) {
    uint8_t pid = cam_read(0x0A);
    Serial.printf("  尝试 %d: PID = 0x%02X\n", i+1, pid);
    if(pid == 0x76) return true;
    delay(100);
  }
  return false;
}

void configureCamera() {
  cam_write(0x12, 0x80);
  delay(300);
  
  cam_write(0x12, 0x14);  // QVGA RGB
  cam_write(0x11, 0x01);
  cam_write(0x6B, 0x0A);
  
  cam_write(0x40, 0xD0);  // RGB565
  cam_write(0x8C, 0x00);
  cam_write(0x04, 0x00);
  cam_write(0x0C, 0x04);
  cam_write(0x3E, 0x1A);
  cam_write(0x3A, 0x04);
  cam_write(0x3D, 0x80);
  cam_write(0x15, 0x00);
  
  cam_write(0x17, 0x16);
  cam_write(0x18, 0x04);
  cam_write(0x19, 0x02);
  cam_write(0x1A, 0x7A);
  cam_write(0x32, 0x80);
  cam_write(0x03, 0x0A);
  
  cam_write(0x70, 0x3A);
  cam_write(0x71, 0x35);
  cam_write(0x72, 0x22);
  cam_write(0x73, 0xF2);
  
  cam_write(0x13, 0xE7);
  cam_write(0x14, 0x28);
  
  cam_write(0x01, 0x80);
  cam_write(0x02, 0x80);
  
  cam_write(0x4F, 0x80);
  cam_write(0x50, 0x80);
  cam_write(0x51, 0x00);
  cam_write(0x52, 0x22);
  cam_write(0x53, 0x5E);
  cam_write(0x54, 0x80);
  cam_write(0x58, 0x9E);
  
  cam_write(0xB0, 0x84);
  cam_write(0x6F, 0x9F);
  
  delay(300);
}

bool captureFrame() {
  // 等待 VSYNC
  while(GPIO.in & VSYNC_MASK);
  while(!(GPIO.in & VSYNC_MASK));
  while(GPIO.in & VSYNC_MASK);
  
  int idx = 0;
  register uint32_t g0, g1;
  
  for(int y = 0; y < IMG_H && idx < IMG_SIZE; y++) {
    // ===== 关键改进：确保在 HREF 稳定后才开始读取 =====
    while(!(GPIO.in & HREF_MASK));  // 等待 HREF 高
    // 读取一行
    for(int x = 0; x < IMG_W * 2 && idx < IMG_SIZE; x++) {
      // 等 PCLK 低
      while(GPIO.in & PCLK_MASK);
      // 等 PCLK 高
      while(!(GPIO.in & PCLK_MASK));
      
      // 同时读取两个寄存器
      g0 = GPIO.in;
      g1 = GPIO.in1.val;
      
      // 组装字节
      uint8_t b = 0;
      if(g0 & D0_MASK) b |= 0x01;  // D0 = GPIO5
      if(g0 & D1_MASK) b |= 0x02;  // D1 = GPIO18
      if(g0 & D2_MASK) b |= 0x04;  // D2 = GPIO19
      if(g0 & D3_MASK) b |= 0x08;  // D3 = GPIO21
      if(g1 & D4_MASK) b |= 0x10;  // D4 = GPIO36
      if(g1 & D5_MASK) b |= 0x20;  // D5 = GPIO39
      if(g1 & D6_MASK) b |= 0x40;  // D6 = GPIO34
      if(g1 & D7_MASK) b |= 0x80;  // D7 = GPIO35
      
      frameBuffer[idx++] = b;
    }
    
    // 等待 HREF 低
    while(GPIO.in & HREF_MASK);
  }
  
  return idx >= IMG_SIZE - 100;
}

const char html[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OV7670</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1a1a1a;color:#fff;font-family:system-ui;padding:10px}
.c{max-width:480px;margin:0 auto}
h1{text-align:center;color:#4af;padding:15px}
canvas{width:100%;background:#000;border-radius:8px;image-rendering:pixelated}
.ctrl{background:#2a2a2a;padding:15px;border-radius:8px;margin:10px 0}
.btn{display:flex;gap:8px;margin:8px 0}
button{flex:1;padding:12px;border:none;border-radius:6px;font-size:14px;cursor:pointer;font-weight:bold}
.g{background:#2a2;color:#fff}
.b{background:#26a;color:#fff}
.r{background:#a22;color:#fff}
.info{background:#2a2a2a;padding:15px;border-radius:8px;margin:10px 0;text-align:center}
.fps{font-size:28px;color:#4af;font-weight:bold}
.st{font-size:12px;color:#aaa;margin-top:5px}
</style>
</head><body>
<div class="c">
<h1>📷 OV7670</h1>
<canvas id="cv" width="160" height="120"></canvas>
<div class="ctrl">
<div class="btn">
<button class="g" onclick="cap()">📸 拍摄</button>
<button class="b" onclick="go()">▶️ 流</button>
<button class="r" onclick="run=0">⏹️ 停</button>
</div>
</div>
<div class="info">
<div class="fps" id="fps">--</div>
<div class="st" id="st">就绪</div>
</div>
</div>
<script>
var run=0,d=null,n=0,lt=Date.now();
var ctx=document.getElementById("cv").getContext("2d");

async function cap(){
  try{
    document.getElementById("st").textContent="...";
    var t0=Date.now();
    var r=await fetch("/c");
    if(!r.ok)throw 0;
    d=new Uint8Array(await r.arrayBuffer());
    draw();
    n++;
    var now=Date.now();
    if(now-lt>=1000){
      document.getElementById("fps").textContent=n+" FPS";
      n=0;lt=now;
    }
    document.getElementById("st").textContent=d.length+" bytes | "+(now-t0)+"ms";
    return 1;
  }catch(e){
    document.getElementById("st").textContent="Error";
    return 0;
  }
}

function draw(){
  if(!d)return;
  var img=ctx.createImageData(160,120),p=0;
  for(var i=0;i<d.length-1&&p<img.data.length;i+=2){
    var c=(d[i]<<8)|d[i+1];
    img.data[p++]=(c&0xf800)>>8|(c&0xe000)>>13;
    img.data[p++]=(c&0x07e0)>>3|(c&0x0600)>>9;
    img.data[p++]=(c&0x001f)<<3|(c&0x001c)>>2;
    img.data[p++]=255;
  }
  ctx.putImageData(img,0,0);
}

async function go(){run=1;while(run){if(!await cap())await new Promise(r=>setTimeout(r,200));else await new Promise(r=>setTimeout(r,50));}}
</script>
</body></html>)rawliteral";

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n========================================");
  Serial.println("        OV7670 Camera");
  Serial.println("========================================\n");
  
  Serial.println("接线配置 (新):");
  Serial.println("  D0-D3: 5, 18, 19, 21");
  Serial.println("  D4-D7: 36(VP), 39(VN), 34, 35");
  Serial.println("  XCLK=4, PCLK=22, VSYNC=25, HREF=23");
  Serial.println("  SDA=26, SCL=27");
  Serial.println("  RESET=14, PWDN=12");
  Serial.println();
  
  if(!initCamera()) {
    Serial.println("❌ 摄像头未检测到!");
    Serial.println("请检查 SDA(26) 和 SCL(27) 接线!");
    while(1) delay(1000);
  }
  Serial.println("✓ 摄像头检测成功 (PID=0x76)");
  
  configureCamera();
  Serial.println("✓ 摄像头配置完成");
  
  // 配置引脚
  pinMode(PIN_D0, INPUT);
  pinMode(PIN_D1, INPUT);
  pinMode(PIN_D2, INPUT);
  pinMode(PIN_D3, INPUT);
  pinMode(PIN_D4, INPUT);
  pinMode(PIN_D5, INPUT);
  pinMode(PIN_D6, INPUT);
  pinMode(PIN_D7, INPUT);
  pinMode(PIN_VSYNC, INPUT);
  pinMode(PIN_HREF, INPUT);
  pinMode(PIN_PCLK, INPUT);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  
  Serial.println("\n========================================");
  Serial.printf("  SSID: %s\n", ap_ssid);
  Serial.printf("  密码: %s\n", ap_password);
  Serial.printf("  URL: http://%s/\n", WiFi.softAPIP().toString().c_str());
  Serial.println("========================================\n");
  
  server.on("/", [](){ server.send_P(200, "text/html", html); });
  server.on("/c", [](){
    unsigned long t0 = millis();
    if(captureFrame()) {
      Serial.printf("帧: %lu ms\n", millis()-t0);
      server.send_P(200, "application/octet-stream", (char*)frameBuffer, IMG_SIZE);
    } else {
      server.send(500, "text/plain", "Fail");
    }
  });
  
  server.begin();
  Serial.println("✓ 服务器就绪!\n");
}

void loop() {
  server.handleClient();
}