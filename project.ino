#include "lib.h"
/*
void setup() {
  gyro_setup();
}
*/

/*
static int turnCount = 0;
static int distance1, distance2;
*/

/* WIT Gyroscope Lib */
// 定义全局变量(不可优化)


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  gyro_setup();
}



void loop() {
  float fGyro, fAngle;

  gyro_get(&fGyro, &fAngle);

  /*
  gyro_get(fGyro, fAngle);
  */
/*
  zGyro.getGyro();
  zAngle.geyAngle();
  distance1 = sonar1.ping();
  distance2 = sonar2.ping();
  while (turnCount == 0) {
    zGyro.getGyro();
    zAngle.geyAngle();
    distance1 = sonar1.ping();
    distance2 = sonar2.ping();
    postureChange();  // 监测 z 方向角度姿态检测与调整
    if (distance1 <= 50.0) {
      speedDown();  // 后续可能会改
      if (distance2 >= 40.0) {
        turnRight();
        turnCount++;
      }
    } else {
      speedControl();
    }
  }
  // 终止条件
  while (turnCount == 6) {
    zGyro.getGyro();
    zAngle.geyAngle();
    distance1 = sonar1.ping();
    distance2 = sonar2.ping();
    if (distance1 <= 50) {
      if (distance2 >= 40) {
        // 待补充, 减速
        turnLeft();
        smalllForward();
        return;
      } else {
        speedControl();
      }
    }
  }
  if (distance1 <= 50) {
    if (distance2 >= 40) {
      // 待补充, 减速
      if (turnCount % 2 == 1) {
        turnRight();
        turnRight();
      } else {
        turnLeft();
        turnLeft();
      }
      turnCount++;
    }
  } else {
    speedControl();
  }
*/

}
