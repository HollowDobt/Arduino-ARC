#ifndef LIB_H
#define LIB_H

#include <Arduino.h>
#include <NewPing.h>
#include <PID_v1.h>
#include <wit_c_sdk.h>

////////////////////////
/* Const Values Table */
// NewPing Setup
// Example
// #define TRIGGER_PIN  A14
// #define ECHO_PIN     A15
// #define MAX_DISTANCE 200
// Arduino         超声波模块
//  A14  ------>   Trig（触发脚）
//  A15  <------   Echo（回波脚）
//   5V  ------>   VCC
//  GND  ------>   GND

constexpr int TRIGGER_PIN = A14;  // Arduino 中会自行替换 A14
constexpr int ECHO_PIN = A15;
constexpr int MAX_DISTANCE = 200;

// PID Action Control

constexpr float TRACK_LENGTH = 100.0f;        // 跑道长度
constexpr float CURRENT_ACCELERATION = 3.0f;  // 加速度
constexpr float INTERVAL = 10.0f;             // 一个加速周期

// [Auto-Gen-Begin] //
constexpr float K_P = 1.0f;   // 微分参数
constexpr float K_I = 0.1f;   // 微分参数
constexpr float K_D = 0.01f;  // 微分参数
// [Auto-Gen-End] //

// WIT Gyroscope
constexpr uint8_t ACC_UPDATE = 0x01;
constexpr uint8_t GYRO_UPDATE = 0x02;
constexpr uint8_t ANGLE_UPDATE = 0x04;
constexpr uint8_t MAG_UPDATE = 0x08;
constexpr uint8_t READ_UPDATE = 0x80;
constexpr uint16_t STARTING_ANGLE = 0x5fff;

/* Self-define Functions */
float ping_distance(NewPing sonar);
void gyro_sensor_uart_send(uint8_t *uiData, uint32_t uiSize);
void gyro_sensor_data_update(uint32_t uiReg, uint32_t uiRegNum);
void gyro_delay_ms(uint16_t uiMs);
void gyro_setup(void);
void gyro_get(float *fGyro, float *fAngle);

/* Variables Initilaize */
extern volatile uint8_t g_gyroDataUpdate;

#endif