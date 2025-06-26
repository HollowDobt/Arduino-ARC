#ifndef LIB_H
#define LIB_H

#include "bits/NewPing.h"
#include "bits/PID_v1.h"
#include "bits/wit_c_sdk.h"
#include "Arduino.h"

////////////////////////
/* Const Values Table */
////////////////////////
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

const int TRIGGER_PIN = A14;  // Arduino 中会自行替换 A14
const int ECHO_PIN = A15;
const int MAX_DISTANCE = 200;

// PID Action Control

const float TRACK_LENGTH = 100.0f;        // 跑道长度
const float CURRENT_ACCELERATION = 3.0f;  // 加速度
const float INTERVAL = 10.0f;             // 一个加速周期
const float K_P = 1.0f;                   // 微分参数
const float K_I = 0.1f;                   // 微分参数
const float K_D = 0.01f;                  // 微分参数

// WIT Gyroscope
// Bitmask Define
const int ACC_UPDATE = 0x01;
const int GYRO_UPDATE = 0x02;
const int ANGLE_UPDATE = 0x04;
const int MAG_UPDATE = 0x08;
const int READ_UPDATE = 0x80;



///////////////////////////
/* Self-define Functions */
///////////////////////////
float ping_distance(NewPing sonar);
void gyro_sensor_uart_send(uint8_t *uiData, uint32_t uiSize);
void gyro_sensor_data_update(uint32_t uiReg, uint32_t uiRegNum);
void gyro_delay_ms(uint16_t uiMs);
void gyro_setup(void);

#endif

/* Variables Initilaize */