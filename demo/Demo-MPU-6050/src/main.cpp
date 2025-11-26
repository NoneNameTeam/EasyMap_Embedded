#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define SDA_PIN 17
#define SCL_PIN 16
#define AD0_PIN 22
#define INT_PIN 23

Adafruit_MPU6050 mpu;

// ========== 校准偏移 ==========
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// ========== 运动数据 ==========
float velX = 0, velY = 0, velZ = 0;
float posX = 0, posY = 0, posZ = 0;

// ========== 滤波数据 ==========
float filtAccX = 0, filtAccY = 0, filtAccZ = 0;

// ========== 时间 ==========
unsigned long lastTime = 0;
unsigned long lastPrint = 0;

// ========== 静止检测 ==========
int stillCount = 0;

// ========== 参数 ==========
const float ACC_THRESHOLD = 0.15;  // 加速度死区
const float VEL_MAX = 2.0;         // 最大速度限制 m/s
const float ACC_MAX = 5.0;         // 最大加速度限制 m/s²

void calibrate() {
    Serial.println();
    Serial.println("================================");
    Serial.println("  校准中，请保持完全静止...");
    Serial.println("  传感器可以任意姿态");
    Serial.println("================================");
    delay(2000);
    
    const int samples = 1000;
    double sumAx = 0, sumAy = 0, sumAz = 0;
    double sumGx = 0, sumGy = 0, sumGz = 0;
    
    Serial.print("采样中");
    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        sumAx += a.acceleration.x;
        sumAy += a.acceleration.y;
        sumAz += a.acceleration.z;
        sumGx += g.gyro.x;
        sumGy += g.gyro.y;
        sumGz += g.gyro.z;
        
        if (i % 200 == 0) Serial.print(".");
        delay(3);
    }
    
    // 保存静止时的加速度读数（包含重力）
    accelOffsetX = sumAx / samples;
    accelOffsetY = sumAy / samples;
    accelOffsetZ = sumAz / samples;
    
    gyroOffsetX = sumGx / samples;
    gyroOffsetY = sumGy / samples;
    gyroOffsetZ = sumGz / samples;
    
    // 重置状态
    velX = velY = velZ = 0;
    posX = posY = posZ = 0;
    filtAccX = filtAccY = filtAccZ = 0;
    stillCount = 0;
    
    Serial.println(" 完成!");
    Serial.println();
    Serial.print("重力向量: X=");
    Serial.print(accelOffsetX, 2);
    Serial.print(" Y=");
    Serial.print(accelOffsetY, 2);
    Serial.print(" Z=");
    Serial.println(accelOffsetZ, 2);
    Serial.println("================================");
    Serial.println();
    Serial.println("命令: c=校准 r=重置位置 v=清零速度");
    Serial.println();
    
    lastTime = micros();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println();
    Serial.println("====== MPU6050 位置追踪 ======");
    
    pinMode(AD0_PIN, OUTPUT);
    digitalWrite(AD0_PIN, LOW);
    
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    
    if (!mpu.begin(0x68, &Wire)) {
        Serial.println("MPU6050 未找到!");
        while (1) delay(10);
    }
    Serial.println("MPU6050 OK");
    
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);  // 最强硬件滤波
    
    calibrate();
}

void loop() {
    // 命令处理
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'c' || c == 'C') {
            calibrate();
            return;
        }
        if (c == 'r' || c == 'R') {
            posX = posY = posZ = 0;
            velX = velY = velZ = 0;
            Serial.println();
            Serial.println(">>> 位置已重置 <<<");
            Serial.println();
        }
        if (c == 'v' || c == 'V') {
            velX = velY = velZ = 0;
            Serial.println();
            Serial.println(">>> 速度已清零 <<<");
            Serial.println();
        }
    }
    
    // 时间
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    
    if (dt <= 0 || dt > 0.05) {
        dt = 0.005;
    }
    
    // 读取传感器
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // 去除重力（用校准时记录的静止读数）
    float accX = a.acceleration.x - accelOffsetX;
    float accY = a.acceleration.y - accelOffsetY;
    float accZ = a.acceleration.z - accelOffsetZ;
    
    float gx = (g.gyro.x - gyroOffsetX) * 57.2958;
    float gy = (g.gyro.y - gyroOffsetY) * 57.2958;
    float gz = (g.gyro.z - gyroOffsetZ) * 57.2958;
    
    // 低通滤波
    const float alpha = 0.2;
    filtAccX = filtAccX * (1 - alpha) + accX * alpha;
    filtAccY = filtAccY * (1 - alpha) + accY * alpha;
    filtAccZ = filtAccZ * (1 - alpha) + accZ * alpha;
    
    // 加速度限幅（防止异常值）
    if (filtAccX > ACC_MAX) filtAccX = ACC_MAX;
    if (filtAccX < -ACC_MAX) filtAccX = -ACC_MAX;
    if (filtAccY > ACC_MAX) filtAccY = ACC_MAX;
    if (filtAccY < -ACC_MAX) filtAccY = -ACC_MAX;
    if (filtAccZ > ACC_MAX) filtAccZ = ACC_MAX;
    if (filtAccZ < -ACC_MAX) filtAccZ = -ACC_MAX;
    
    // 计算总量
    float totalAcc = sqrt(filtAccX*filtAccX + filtAccY*filtAccY + filtAccZ*filtAccZ);
    float totalGyro = sqrt(gx*gx + gy*gy + gz*gz);
    
    // 运动状态判断
    bool isRotating = totalGyro > 10.0;   // 旋转中
    bool isMoving = totalAcc > ACC_THRESHOLD && !isRotating;
    
    // 速度积分
    if (isMoving) {
        stillCount = 0;
        
        // 死区处理后积分
        if (fabs(filtAccX) > ACC_THRESHOLD) velX += filtAccX * dt;
        if (fabs(filtAccY) > ACC_THRESHOLD) velY += filtAccY * dt;
        if (fabs(filtAccZ) > ACC_THRESHOLD) velZ += filtAccZ * dt;
        
        // 速度限幅
        if (velX > VEL_MAX) velX = VEL_MAX;
        if (velX < -VEL_MAX) velX = -VEL_MAX;
        if (velY > VEL_MAX) velY = VEL_MAX;
        if (velY < -VEL_MAX) velY = -VEL_MAX;
        if (velZ > VEL_MAX) velZ = VEL_MAX;
        if (velZ < -VEL_MAX) velZ = -VEL_MAX;
        
    } else {
        stillCount++;
        
        // 静止时速度衰减
        if (stillCount > 5) {
            velX *= 0.9;
            velY *= 0.9;
            velZ *= 0.9;
        }
        if (stillCount > 20) {
            velX *= 0.8;
            velY *= 0.8;
            velZ *= 0.8;
        }
        if (stillCount > 50) {
            // 彻底静止
            if (fabs(velX) < 0.01) velX = 0;
            if (fabs(velY) < 0.01) velY = 0;
            if (fabs(velZ) < 0.01) velZ = 0;
        }
    }
    
    // 位置积分
    posX += velX * dt;
    posY += velY * dt;
    posZ += velZ * dt;
    
    // 输出（每200ms一次，用换行方式）
    if (millis() - lastPrint >= 200) {
        lastPrint = millis();
        
        float totalVel = sqrt(velX*velX + velY*velY + velZ*velZ);
        float distance = sqrt(posX*posX + posY*posY + posZ*posZ);
        
        // 状态
        const char* status;
        if (isRotating) {
            status = "旋转";
        } else if (isMoving) {
            status = "移动";
        } else {
            status = "静止";
        }
        
        // 输出一行
        Serial.print("位置(cm): X=");
        Serial.print(posX * 100, 1);
        Serial.print(" Y=");
        Serial.print(posY * 100, 1);
        Serial.print(" Z=");
        Serial.print(posZ * 100, 1);
        Serial.print(" | 距原点=");
        Serial.print(distance * 100, 1);
        Serial.print("cm | 速度=");
        Serial.print(totalVel, 2);
        Serial.print("m/s | ");
        Serial.println(status);
    }
    
    delay(5);
}