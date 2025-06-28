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

static float pidTargetFR = TARGET_SPEED;
static float pidTargetFL = TARGET_SPEED;
static float pidTargetBR = TARGET_SPEED;
static float pidTargetBL = TARGET_SPEED;

// Initial PID algorithm
static PID pidFR(&rpmFR, &pwmFR, &pidTargetFR, 2.0, 5.0, 1.0, DIRECT);
static PID pidFL(&rpmFL, &pwmFL, &pidTargetFL, 2.0, 5.0, 1.0, DIRECT);
static PID pidBR(&rpmBR, &pwmBR, &pidTargetBR, 2.0, 5.0, 1.0, DIRECT);
static PID pidBL(&rpmBL, &pwmBL, &pidTargetBL, 2.0, 5.0, 1.0, DIRECT);

/*
 * 功能集成函数定义在主文件中.
 */

inline void update_sensors();
inline void pid_setup();
inline void pid_update(void);

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
                    turning(/* Right Tangle -90 ... */);
                    turnCount++;
                    currentState = NORMAL_DRIVE;
                    return;
                }
            } else {
                speed_control(TARGET_SPEED);
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
                speed_control(TARGET_SPEED);
            }
            break;

        case TURN_RIGHT:
            turning(/* Right Tangle -90 ... */);
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
                    turning(/* Left Tangle 90 ... */);
                    small_forward();
                    currentState = FINISHED;
                }
            } else {
                posture_change(fAngle, currentState, lastState);
                speed_control(TARGET_SPEED);
            }

        case TURN_LEFT:
            turning(/* Left Tangle 90 ... */);
            turnCount++;
            currentState = NORMAL_DRIVE;
            break;

        case FINISHED:
            Serial.println("Finished.");
            all_motors_stop(myRobot);
            while (true);
            break;

        case POSTURE_CHANGE:
            if (fAngle > 0) {
                robot.fRight.move_cclockwise(ROTATE_PWM);  // 前右轮反转
                robot.bRight.move_cclockwise(ROTATE_PWM);  // 后右轮反转
                robot.fLeft.move_clockwise(ROTATE_PWM);    // 前左轮正转
                robot.bLeft.move_clockwise(ROTATE_PWM);    // 后左轮正转
            } else {
                robot.fRight.move_clockwise(ROTATE_PWM);  // 前右轮正转
                robot.bRight.move_clockwise(ROTATE_PWM);  // 后右轮正转
                robot.fLeft.move_cclockwise(ROTATE_PWM);  // 前左轮反转
                robot.bLeft.move_cclockwise(ROTATE_PWM);  // 后左轮反转
            }

            if (fabs(fAngle) <= CHANGE_ANGLE_TOLERANCE) {
                currentState = lastState;
                all_motors_stop(myRobot);
            }
            break;

        case POSITION_CHANGE:
            if (distance2 < 15) {
                // 向前移动
                robot.fRight.move_clockwise((uint8_t)pwmFR -
                                            MOVE_PWMY);  // 前右轮正转
                robot.bRight.move_clockwise((uint8_t)pwmBR +
                                            MOVE_PWMY);  // 后右轮正转
                robot.fLeft.move_clockwise((uint8_t)pwmFL +
                                           MOVE_PWMY);  // 前左轮正转
                robot.bLeft.move_clockwise((uint8_t)pwmBL -
                                           MOVE_PWMY);  // 后左轮正转
            }
            if (distance2 > 25) {
                // 向前移动
                robot.fRight.move_clockwise((uint8_t)pwmFR +
                                            MOVE_PWMY);  // 前右轮正转
                robot.bRight.move_clockwise((uint8_t)pwmBR -
                                            MOVE_PWMY);  // 后右轮正转
                robot.fLeft.move_clockwise((uint8_t)pwmFL -
                                           MOVE_PWMY);  // 前左轮正转
                robot.bLeft.move_clockwise((uint8_t)pwmBL +
                                           MOVE_PWMY);  // 后左轮正转
            }
            if (distance2 >= 15 && distance2 <= 25) {
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
    pid_update();
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

inline void pid_update() {
    pidFR.Compute();
    pidFL.Compute();
    pidBR.Compute();
    pidBL.Compute();
}

inline void speed_control(double desiredRPM) {
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

inline void speed_down() { speed_control(LOW_SPEED); }

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
