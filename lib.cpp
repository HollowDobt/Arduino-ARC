#include "lib.h"

#include <cinttypes>

/* NewPing Lib */
// 获取距离
inline float ping_distance(NewPing sonar) { return sonar.ping() / 58.82; }

/* WIT Gyroscope Lib */
// 定义全局变量(不可优化)
volatile uint8_t g_gyroDataUpdate = 0;

// 函数指针: 写入函数(不可直接调用)
void gyro_sensor_uart_send(uint8_t *uiData, uint32_t uiSize) {
    Serial1.write(uiData, uiSize);
    Serial1.flush();
}

// 函数指针: 获取更新情况(不可直接调用)
void gyro_sensor_data_update(uint32_t uiReg, uint32_t uiRegNum) {
    if (uiRegNum == 0) return;

    for (uint32_t i = 0; i < uiRegNum; i++, uiReg++) {
        switch (uiReg) {
            case GZ:
                g_gyroDataUpdate |= GYRO_UPDATE;
                break;
            case Yaw:
                g_gyroDataUpdate |= ANGLE_UPDATE;
                break;
            default:
                g_gyroDataUpdate |= READ_UPDATE;
                break;
        }
    }
}

// 函数指针: 延时(不可直接调用)
void gyro_delay_ms(uint16_t uiMs) { delay(uiMs); }

// 放在 setup(), gyro 设置
inline void gyro_setup() {
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);

    WitSerialWriteRegister(gyro_sensor_uart_send);
    WitRegisterCallBack(gyro_sensor_data_update);
    WitDelayMsRegister(gyro_delay_ms);

    WitSetUartBaud(WIT_BAUD_115200);
    delay(200);
    WitSetYAW(STARTING_ANGLE);
    delay(300);
    WitSetOutputRate(RRATE_50HZ);
    delay(300);
}

// 放在 loop(), gyro 调用. 用途是实现函数:   zGyro.getGyro();zAngle.geyAngle();
// 放入两个变量, 完成自动更新
inline void gyro_get(float *fGyro, float *fAngle) {
    while (Serial1.available()) {
        WitSerialDataIn(Serial1.read());
    }
    if (g_gyroDataUpdate != 0) {
        *fGyro = sReg[GYRO_Z_REG] / 32768.0f * 2000.0f;
        *fAngle = sReg[ANGLE_Z_REG] / 32768.0f * 180.0f;
        if (*fAngle > 180) {
            fAngle =
        }
        /* Use for debugging
        if (g_gyroDataUpdate & GYRO_UPDATE) {
            Serial.print("GyroZ: ");
            Serial.print(*fGyro, 1);
            Serial.print("\r\n");
            g_gyroDataUpdate &= ~GYRO_UPDATE;
        }
        if (g_gyroDataUpdate & ANGLE_UPDATE) {
            Serial.print("AngleZ: ");
            Serial.print(*fAngle, 3);
            Serial.print("\r\n");
            g_gyroDataUpdate &= ~ANGLE_UPDATE;
        }
        */
        g_gyroDataUpdate = 0;
    }
}

/* WARNING: Below functions may result in serious errors */
// Get current rpm
float current_rpm_fetch(int *oldPosition, unsigned int *oldTime,
                        const Encoder encoder,
                        const unsigned int intervalMs = DEFAULT_INTERVAL_MS) {
    int newPosition = encoder.read();
    unsigned long currentTime = millis();

    if (currentTime - *oldTime >= intervalMs) {
        int pulses = newPosition - *oldPosition;
        float rpm = (pulses / (float)ENCODER_LINES) * (60000.0 / inintervalMs);

        *oldPosition = newPosition;
        *oldTime = currentTime;
        return rpm;
    }

    return -1.0;
}

// Detect current fAngle to see if there's need in adjusting posture
inline void posture_change(const float fAngle, State &currentState,
                           State &lastState, const float &distance2) {
    if (fabs(fAngle) >= NORMAL_ANGLE_TOLERANCE) {
        lastState = currentState;
        currentState = POSTURE_CHAGE;
    }
    if (distance2 <= 15 || distance2 >= 25) {
        lastState = currentState;
        currentState = POSITION_CHANGE;
    }
}

// Stop all the motors
inline void all_motors_stop(robot::robot_4_wheels &myRobot) {
    myRobot.fRight.halt();
    myRobot.fLeft.halt();
    myRobot.bRight.halt();
    myRobot.bLeft.halt();
}
