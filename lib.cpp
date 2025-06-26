#include "lib.h"

////////////////////////////////////
/* NewPing Lib */
float ping_distance(NewPing sonar) {
    return sonar.ping()/58.82;
}
////////////////////////////////////



////////////////////////////////////
/* WIT Gyroscope Lib */

volatile char gyroDataUpdate = 0;

void gyro_sensor_uart_send(uint8_t *uiData, uint32_t uiSize) {
    Serial1.write(uiData, uiSize);
    Serial1.flush();
}

void gyro_sensor_data_update(uint32_t uiReg, uint32_t uiRegNum){
    int i;

    for (i = 0; i < uiRegNum; i++) {
    switch (uiReg) {
      case GZ:
        gyroDataUpdate |= GYRO_UPDATE;
        break;
      case Yaw:
        gyroDataUpdate |= ANGLE_UPDATE;
        break;
      default:
        gyroDataUpdate |= READ_UPDATE;
        break;
    }
    uiReg++;
    }
}

void gyro_delay_ms(uint16_t uiMs) {
    delay(uiMs);
}

void gyro_setup() {
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);

    WitSerialWriteRegister(gyro_sensor_uart_send);
    WitRegisterCallBack(gyro_sensor_data_update);
    WitDelayMsRegister(gyro_delay_ms);

    WitSetUartBaud(WIT_BAUD_115200);
    delay(200);
    WitSetYAW(0x5FFF);
    delay(300);
    WitSetOutputRate(RRATE_50HZ);
    delay(300);
}
//////////////////////////////////////