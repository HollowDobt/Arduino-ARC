#ifndef LIB_H
#define LIB_H

#include <Arduino.h>
#include <Encoder.h>
#include <NewPing.h>
#include <PID_v1.h>
#include <math.h>
#include <wit_c_sdk.h>

////////////////////////
/* Const Values Table */

// State Error Enbage
// 容许角度误差为 3 度, 每次进入 posture_change 后保证误差归到 1.0
constexpr float NORMAL_ANGLE_TOLERANCE = 3.0;
constexpr float CHANGE_ANGLE_TOLERANCE = 0.5;

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

constexpr int TRIGGER_PIN1 = A0;  // Arduino 中会自行替换 A14
constexpr int ECHO_PIN1 = A1;
constexpr int TRIGGER_PIN2 = A2;
constexpr int ECHO_PIN2 = A3;
constexpr int MAX_DISTANCE = 350;

// PID Action Control

constexpr float TRACK_LENGTH = 100.0f;  // 跑道长度
constexpr float ACCELERATION = 3.0f;    // 加速度
constexpr float TARGET_SPEED = 100.0;
constexpr float LOW_SPEED = 40.0;
constexpr float ROTATE_PWM = 50.0;
constexpr float MOVE_PWMY = 50.0;

// WIT Gyroscope
constexpr uint8_t ACC_UPDATE = 0x01;
constexpr uint8_t GYRO_UPDATE = 0x02;
constexpr uint8_t ANGLE_UPDATE = 0x04;
constexpr uint8_t MAG_UPDATE = 0x08;
constexpr uint8_t READ_UPDATE = 0x80;
constexpr uint16_t STARTING_ANGLE = 0x0000;

// Define REG
constexpr int GYRO_Z_REG = 57;
constexpr int ANGLE_Z_REG = 63;

// Define Encoder Pins
constexpr int ENCODER_1_A = 11;
constexpr int ENCODER_1_B = 10;
constexpr int ENCODER_2_A = ;
constexpr int ENCODER_2_B = ;
constexpr int ENCODER_3_A = ;
constexpr int ENCODER_3_B = ;
constexpr int ENCODER_4_A = ;
constexpr int ENCODER_4_B = ;
constexpr int ENCODER_LINES = 11;
constexpr int DEFAULT_INTERVAL_MS = 50;

// PID Aims: 分状态不同的目标转速

/* Self-define Functions */
// 超声波距离测定
inline float ping_distance(NewPing sonar);

void gyro_sensor_uart_send(uint8_t *uiData, uint32_t uiSize);

void gyro_sensor_data_update(uint32_t uiReg, uint32_t uiRegNum);

void gyro_delay_ms(uint16_t uiMs);

inline void gyro_setup(void);

// 获取偏向角度和角速度
inline void gyro_get(float *fGyro, float *fAngle);

/*
 * WARNING: Below functions may result in serious errors.
 */
// Sensors Reading
float current_rpm_fetch(unsigned int *oldPosition, unsigned int *oldTime,
                        const Encoder encoder,
                        const unsigned int intervalMs = DEFAULT_INTERVAL_MS);

// Posture Adjustment State Function: 状态转移函数(判定是否转移到POSTURE_CHANGE)
inline void posture_change(const float fAngle);

inline void all_motors_stop(void);

/*
 * WARNIG: Above functions may result in serious errors.
 */

/* Variables Initilaize */
extern volatile uint8_t g_gyroDataUpdate;

// State Definition
enum State {
    INIT,           // 初始状态
    NORMAL_DRIVE,   // 正常前进状态, 判断避障
    TURN_LEFT,      // 左转状态
    TURN_RIGHT,     // 右转状态
    FINAL_TURN,     // 最后一次逻辑处理状态
    FINISHED,       // 进入停车状态
    POSTURE_CHAGE,  // 姿态调整状态
    POSITION_CHANGE
};

#endif
