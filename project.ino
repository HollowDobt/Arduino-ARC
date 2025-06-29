/*
 * Author: HollowDobt
 * 实现思路: 有限状态机
 * 方式: 状态更新, 所有函数均无显式长 while 循环, 通过状态更新进入 while,

 * 尽可能避免进入死循环.
 * 实现: 所有函数实际上是"单步"的. 禁用函数内部循环
*/

#include "lib.h"
#include <PinChangeInt.h>

// Initial State
static State currentState = INIT;
static State lastState = INIT;

// Initial Turns
static uint8_t turnCount = 0;

// Initial Sonar Distances
static NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
static NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
static float distance1 = 0.0, distance2 = 0.0;

// Initial Omega Difitions
static float fGyro = 0.0, fAngle = 0.0;

// Register Pins Reader Coding
static Encoder encoder1(ENCODER_1_A, ENCODER_1_B);
static Encoder encoder2(ENCODER_2_A, ENCODER_2_B);
static Encoder encoder3(ENCODER_3_A, ENCODER_3_B);
static Encoder encoder4(ENCODER_4_A, ENCODER_4_B);

// Initial oldTime, oldPosition
static int oldPosition1 = 0, oldPosition2 = 0, oldPosition3 = 0,
           oldPosition4 = 0;

static unsigned long oldTime = 0;

static double rpmFR = 0.0, rpmFL = 0.0, rpmBR = 0.0, rpmBL = 0.0;
static double pwmFR = 0.0, pwmFL = 0.0, pwmBR = 0.0, pwmBL = 0.0;

// Initial robot state
static robot::robot_4_wheels myRobot(PINS);

static double pidTargetFR = TARGET_SPEED;
static double pidTargetFL = TARGET_SPEED;
static double pidTargetBR = TARGET_SPEED;
static double pidTargetBL = TARGET_SPEED;

// Initial PID algorithm
static PID pidFR(&rpmFR, &pwmFR, &pidTargetFR, 2.0, 5.0, 1.0, DIRECT);
static PID pidFL(&rpmFL, &pwmFL, &pidTargetFL, 2.0, 5.0, 1.0, DIRECT);
static PID pidBR(&rpmBR, &pwmBR, &pidTargetBR, 2.0, 5.0, 1.0, DIRECT);
static PID pidBL(&rpmBL, &pwmBL, &pidTargetBL, 2.0, 5.0, 1.0, DIRECT);

// 非阻塞 FSM 化 TURNING 状态必要全局变量
static float targetAngle = 0.0; // 目标角度
static bool turningRight = false; // 右转 or 左转?
static bool finalTurn = false; // 是否最后一次大转弯

// 中断服务函数
static volatile unsigned long lastPulseTime[4] = {0, 0, 0, 0};
static volatile unsigned long pulseInterval[4] = {0, 0, 0, 0};
static volatile float rpm[4] = {0, 0, 0, 0};
static volatile int8_t direction[4] = {0, 0, 0, 0};
static volatile bool lastALevel[4] = {0, 0, 0, 0};

/*
 * 功能集成函数定义在主文件中.
 */

void update_sensors(void);

void pid_setup(void);

void pid_update(void);

void all_motors_stop(robot::robot_4_wheels &myRobot);

void small_forward(void);

// 中断服务函数
void hallA_isr(void);
// void hallB_isr(void);
void ISR_loop(void);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  gyro_setup();
  pid_setup();

  // 中断服务初始化
  for (uint8_t i = 0; i < 4; i ++) {
    pinMode(hallA[i], INPUT_PULLUP);
    pinMode(hallB[i], INPUT_PULLUP);
    PCintPort::attachInterrupt(hallA[i], hallA_isr, CHANGE);
  }
}

void loop() {
  update_sensors();
  delay(50);

  switch (currentState) {
    case INIT:
      Serial.println("Car State Init.");
      posture_change(fAngle, currentState, lastState, distance2);
      if (distance1 > 0.1 && distance1 <= 50) {
        speed_down();
        if (distance1 < 40 && distance2 >= 40) {
            // 右转准备, 目标角度确定, 切换状态
          targetAngle = fAngle - 90.0;
          turningRight = true;
          finalTurn = false;
          currentState = TURNING;
          return;
        }
      } else {
        speed_control(TARGET_SPEED);
      }
      break;

    case NORMAL_DRIVE:
      posture_change(fAngle, currentState, lastState, distance2);
      if (distance1 > 0.1 && distance1 <= 50) {
        speed_down();
        if (distance1 < 40 && distance2 >= 40) {
          if (turnCount % 2 == 1) {
            targetAngle = fAngle - 90.0;
            turningRight = true;
            finalTurn = false;
          } else {
            targetAngle = fAngle + 90.0;
            turningRight = false;
            finalTurn = false;
          }
          currentState = TURNING;
          return;
        }
      } else {
        speed_control(TARGET_SPEED);
      }
      break;

    case TURNING:
      if (fabs(fAngle - targetAngle) > 3.0) {
        if (turningRight) {
            myRobot.move_circular(0, 100, 100, 200);
        } else {
            myRobot.move_circular(1, 100, 100, 200);
        }
      } else {
        all_motors_stop(myRobot);
        WitStartIYAWCali();
        turnCount++;
        if (finalTurn) {
            small_forward();
            currentState = FINISHED;
        } else if (turnCount == 6) {
            currentState = FINAL_TURN;
        } else {
            currentState = NORMAL_DRIVE;
        }
      }
      break;

    case FINAL_TURN:
      if (distance1 > 0.1 && distance1 <= 50) {
        speed_down();
        if (distance1 < 40 && distance2 >= 40) {
          // 最后一轮左转
          targetAngle = fAngle + 90.0;
          turningRight = false;
          finalTurn = true;
          currentState = TURNING;
        }
      } else {
        posture_change(fAngle, currentState, lastState, distance2);
        speed_control(TARGET_SPEED);
      }
      break;

    case FINISHED:
      Serial.println("Shutdown State. Keep Silence.");
      all_motors_stop(myRobot);
      while (true) {
        Serial.println("Please turn off the power manually to avoid power consumption.");
      }
      break;

    case POSTURE_CHANGE:
      if (fAngle > 0) {
        myRobot.fRight.move_cclockwise(ROTATE_PWM);  // 前右轮反转
        myRobot.bRight.move_cclockwise(ROTATE_PWM);  // 后右轮反转
        myRobot.fLeft.move_clockwise(ROTATE_PWM);    // 前左轮正转
        myRobot.bLeft.move_clockwise(ROTATE_PWM);    // 后左轮正转
      } else {
        myRobot.fRight.move_clockwise(ROTATE_PWM);  // 前右轮正转
        myRobot.bRight.move_clockwise(ROTATE_PWM);  // 后右轮正转
        myRobot.fLeft.move_cclockwise(ROTATE_PWM);  // 前左轮反转
        myRobot.bLeft.move_cclockwise(ROTATE_PWM);  // 后左轮反转
      }

      if (fabs(fAngle) <= CHANGE_ANGLE_TOLERANCE) {
        currentState = lastState;
        all_motors_stop(myRobot);
      }
      break;

    case POSITION_CHANGE:
      if (distance2 < 15) {
        // 向前移动
        myRobot.fRight.move_clockwise((uint8_t)pwmFR - MOVE_PWMY);  // 前右轮正转
        myRobot.bRight.move_clockwise((uint8_t)pwmBR + MOVE_PWMY);  // 后右轮正转
        myRobot.fLeft.move_clockwise((uint8_t)pwmFL + MOVE_PWMY);   // 前左轮正转
        myRobot.bLeft.move_clockwise((uint8_t)pwmBL - MOVE_PWMY);   // 后左轮正转
      }
      if (distance2 > 25) {
        // 向前移动
        myRobot.fRight.move_clockwise((uint8_t)pwmFR + MOVE_PWMY);  // 前右轮正转
        myRobot.bRight.move_clockwise((uint8_t)pwmBR - MOVE_PWMY);  // 后右轮正转
        myRobot.fLeft.move_clockwise((uint8_t)pwmFL - MOVE_PWMY);   // 前左轮正转
        myRobot.bLeft.move_clockwise((uint8_t)pwmBL + MOVE_PWMY);   // 后左轮正转
      }
      if (distance2 >= 15 && distance2 <= 25) {
        currentState = lastState;
        all_motors_stop(myRobot);
      }
      break;
  }
}

void update_sensors() {
  distance1 = ping_distance(sonar1);
  distance2 = ping_distance(sonar2);
  gyro_get(&fGyro, &fAngle);
  ISR_loop();
  pid_update();
}

void pid_setup() {
  pidFR.SetMode(AUTOMATIC);
  pidFL.SetMode(AUTOMATIC);
  pidBR.SetMode(AUTOMATIC);
  pidBL.SetMode(AUTOMATIC);
  pidFR.SetOutputLimits(0, 255);
  pidFL.SetOutputLimits(0, 255);
  pidBR.SetOutputLimits(0, 255);
  pidBL.SetOutputLimits(0, 255);
}

void pid_update() {
  pidFR.Compute();
  pidFL.Compute();
  pidBR.Compute();
  pidBL.Compute();
}

void speed_control(const float &desireRPM) {
  pidTargetFR = desireRPM;
  pidTargetFL = desireRPM;
  pidTargetBR = desireRPM;
  pidTargetBL = desireRPM;

  pid_update();

  myRobot.fRight.move_clockwise((uint8_t)pwmFR);
  myRobot.fLeft.move_clockwise((uint8_t)pwmFL);
  myRobot.bRight.move_clockwise((uint8_t)pwmBR);
  myRobot.bLeft.move_clockwise((uint8_t)pwmBL);
}

void speed_down() {
  speed_control(LOW_SPEED);
}

void turning(float TAngle) {
  if (TAngle > 0) {
    // 向右转
    while (fAngle >= TAngle - 3.0 && fAngle <= TAngle + 3.0) {
      myRobot.move_circular(0, 100, 100, 200);
    }
    // 使用wit_init 角度调零
    WitStartIYAWCali();
  } else {
    // 向左转
    while (fAngle >= TAngle - 3.0 && fAngle <= TAngle + 3.0) {
      myRobot.move_circular(1, 100, 100, 200);
    }
    // 使用wit_init 角度调零
    WitStartIYAWCali();
  }
}

// Stop all the motors
void all_motors_stop(robot::robot_4_wheels &myRobot) {
  myRobot.fRight.halt();
  myRobot.fLeft.halt();
  myRobot.bRight.halt();
  myRobot.bLeft.halt();
}

// 小步幅前进
void small_forward() {
  myRobot.move_up(100);
  delay(300);
}

// 中断服务函数
void hallA_isr() {
  unsigned long now = micros();
  for (uint8_t i = 0; i < 4; i ++) {
    bool aLevel = digitalRead(hallA[i]);
    if (aLevel != lastALevel[i]) {
      pulseInterval[i] = now - lastPulseTime[i];
      lastPulseTime[i] = now;

      bool bLevel = digitalRead(hallB[i]);
      bool isRising = (!lastALevel[i]) && aLevel;
      bool isFalling = lastALevel[i] && (!aLevel);
      lastALevel[i] = aLevel;

      if (isRising) direction[i] = bLevel ? 1 : -1;
      if (isFalling) direction[i] = bLevel ? -1 : 1;

    }
  }
}

void ISR_loop() {
    static unsigned long lastPrint = 0;
    unsigned long now = micros();

    // 判停转，长时间无脉冲则rpm归零
    for (uint8_t i = 0; i < 4; ++i) {
      const unsigned long timeout_us = 500000; // 0.5秒
      if (now - lastPulseTime[i] > timeout_us) {
            rpm[i] = 0.0;
            direction[i] = 0; // 方向未知
        } else if (pulseInterval[i] > 0) {
            // 在主循环安全计算浮点rpm
            float freq = 1e6 / pulseInterval[i]; // Hz
            rpm[i] = freq * 60.0 / (ENCODER_LINES * 2); // *2表示A沿全部，按实际分辨率调整
        }
    }


    rpmFR = rpm[0];
    rpmFL = rpm[1];
    rpmBR = rpm[2];
    rpmBL = rpm[3];

    // 打印
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        for (uint8_t i = 0; i < 4; ++i) {
            Serial.print("Wheel");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(rpm[i]);
            Serial.print(" rpm, Dir: ");
            if      (direction[i] == 1) Serial.print("FWD");
            else if (direction[i] == -1) Serial.print("REV");
            else    Serial.print("STILL");
            Serial.println();
        }
        Serial.println();
    }
}