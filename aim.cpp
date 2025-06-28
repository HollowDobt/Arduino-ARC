/*
 * Author: HollowDobt
 * 实现思路: 有限状态机
 * 方式: 状态更新, 所有函数均无显式长 while 循环, 通过状态更新进入 while,
 * 尽可能避免进入死循环.
 * 实现: 所有函数实际上是"单步"的. 禁用函数内部循环
 */

#include "lib.h"

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

static unsigned int oldTime = 0;

static float rpmFR = 0.0, rpmFL = 0.0, rpmBR = 0.0, rpmBL = 0.0;
static float pwmFR = 0.0, pwmFL = 0.0, pwmBR = 0.0, pwmBL = 0.0;

// Initial robot state
static robot::robot_4_wheels myRobot(/* 补充构造函数 */);

// Initial PID algorithm
static PID pidFR(&rpmFR, &pwmFR, &POSTURE_CHANGE_PID_TARGET_FR, 2.0, 5.0, 1.0,
                 DIRECT);
static PID pidFL(&rpmFL, &pwmFL, &POSTURE_CHANGE_PID_TARGET_FL, 2.0, 5.0, 1.0,
                 DIRECT);
static PID pidBR(&rpmBR, &pwmBR, &POSTURE_CHANGE_PID_TARGET_BR, 2.0, 5.0, 1.0,
                 DIRECT);
static PID pidBL(&rpmBL, &pwmBL, &POSTURE_CHANGE_PID_TARGET_BL, 2.0, 5.0, 1.0,
                 DIRECT);

/*
 * 功能集成函数定义在主文件中.
 */

void update_sensors();
void pid_setup();
void posture_change_step(const float &fAngle);

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    gyro_setup();
    pid_setup();
}

void loop() {
    update_sensors();
    delay(50);

    switch (currentState) {
        case INIT:
            Serial.println("Car State Init.");
            posture_change(fAngle, currentState, lastState);
            if (distance1 > 0.1 && distance1 <= 50) {
                speed_down();
                if (distance1 < 40 && distance2 >= 40) {
                    turn_right();
                    turnCount++;
                    currentState = NORMAL_DRIVE;
                    return;
                }
            } else {
                speed_control();
            }
            break;

        case NORMAL_DRIVE:
            posture_change(fAngle, currentState, lastState);
            if (distance1 > 0.1 && distance1 <= 50) {
                speed_down();
                if (distance1 < 40 && distance2 >= 40) {
                    if (turnCount % 2 == 1) {
                        currentState = TURN_RIGHT;
                    } else {
                        currentState = TURN_LEFT;
                    }
                    return;
                }
            } else {
                speed_control();
            }
            break;

        case TURN_RIGHT:
            turn_right();
            turnCount++;
            if (turnCount == 6) {
                currentState = FINAL_TURN;
            } else {
                currentState = NORMAL_DRIVE;
            }
            break;

        case FINAL_TURN:
            if (distance1 > 0.1 && distance1 <= 50) {
                speed_down();
                if (distance1 < 40 && distance2 >= 40) {
                    turn_left();
                    small_forward();
                    currentState = FINISHED;
                }
            } else {
                posture_change(fAngle, currentState, lastState);
                speed_control();
            }

        case TURN_LEFT:
            turn_left();
            turnCount++;
            currentState = NORMAL_DRIVE;
            break;

        case FINISHED:
            stop_motor();
            Serial.println("Finished.");
            while (true);
            break;

        case POSTURE_CHAGE:
            posture_change_step_1(fAngle, myRobot);
            if (fabs(fAngle) <= CHANGE_ANGLE_TOLERANCE) {
                currentState = lastState;
                all_motors_stop(myRobot);
            }
            break;
    }
}

inline void update_sensors() {
    distance1 = ping_distance(sonar1);
    distance2 = ping_distance(sonar2);
    gyro_get(&fGyro, &fAngle);
    rpmFR = current_rpm_fetch(oldPosition1, oldTime, encoder1);
    rpmFL = current_rpm_fetch(oldPosition2, oldTime, encoder2);
    rpmBR = current_rpm_fetch(oldPosition3, oldTime, encoder3);
    rpmBL = current_rpm_fetch(oldPosition4, oldTime, encoder4);
    pidFR.Compute();
    pidFL.Compute();
    pidBR.Compute();
    pidBL.Compute();
}

inline void pid_setup() {
    pidFR.SetMode(AUTOMATIC);
    pidFL.SetMode(AUTOMATIC);
    pidBR.SetMode(AUTOMATIC);
    pidBL.SetMode(AUTOMATIC);
    pidFR.SetOutputLimits(0, 255);
    pidFL.SetOutputLimits(0, 255);
    pidBR.SetOutputLimits(0, 255);
    pidBL.SetOutputLimits(0, 255);
}

inline void posture_change_step_1(const float &fAngle,
                                  robot::robot_4_wheels &myRobot) {
    if (fAngle > CHANGE_ANGLE_TOLERANCE) {
        // 向右偏，需要逆时针旋转纠正
        myRobot.fRight.move_cclockwise((uint8_t)(pwmFR * 0.6));
        myRobot.bRight.move_cclockwise((uint8_t)(pwmBR * 0.6));
        myRobot.fLeft.move_clockwise((uint8_t)(pwmFL * 0.6));
        myRobot.bLeft.move_clockwise((uint8_t)(pwmBL * 0.6));
    } else if (fAngle < -CHANGE_ANGLE_TOLERANCE) {
        // 向左偏，需要顺时针旋转纠正
        myRobot.fRight.move_clockwise((uint8_t)(pwmFR * 0.6));
        myRobot.bRight.move_clockwise((uint8_t)(pwmBR * 0.6));
        myRobot.fLeft.move_cclockwise((uint8_t)(pwmFL * 0.6));
        myRobot.bLeft.move_cclockwise((uint8_t)(pwmBL * 0.6));
    }
}

inline void posture_change_step_2(const float &distance1,
                                  const float &distance2,
                                  robot::robot_4_wheels &myRobot) {
    // 如果距离传感器检测到需要前后调整
    if (distance2 > 0.1 && distance2 < 20) {
        // 距离太近，需要后退
        myRobot.fRight.move_clockwise((uint8_t)(pwmFR * 0.4) - 15);
        myRobot.bRight.move_clockwise((uint8_t)(pwmBR * 0.4) + 15);
        myRobot.fLeft.move_clockwise((uint8_t)(pwmFL * 0.4) + 15);
        myRobot.bLeft.move_clockwise((uint8_t)(pwmBL * 0.4) - 15);
    } else if (distance1 > 0.1 && distance1 > 40) {
        // 距离太远，需要前进
        myRobot.fRight.move_clockwise((uint8_t)(pwmFR * 0.4) + 15);
        myRobot.bRight.move_clockwise((uint8_t)(pwmBR * 0.4) - 15);
        myRobot.fLeft.move_clockwise((uint8_t)(pwmFL * 0.4) - 15);
        myRobot.bLeft.move_clockwise((uint8_t)(pwmBL * 0.4) + 15);
    }
}
