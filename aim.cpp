#include "lib.h"

static uint8_t currentState = INIT;
static uint8_t turnCount = 0;
static float distance1 = 0.0, distance2 = 0.0;
static float fGyro = 0.0, fAngle = 0.0;
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);

void update_sensors();

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    gyro_setup();
}

void loop() {
    update_sensors();
    delay(50);

    switch (currentState) {
        case INIT:
            Serial.println("Car State Init.");
            posture_change();
            if (distance1 > 0 && distance1 <= 50) {
                speed_down();

                if (distance2 >= 40) {
                    turn_right();
                    turnCount++;
                    currentState = NORMAL_DRIVE;
                }
            } else {
                speed_control();
            }
            break;

        case NORMAL_DRIVE:
            posture_change();
            if (distance1 > 0 && distance1 <= 50) {
                speed_down();
                if (distance2 >= 40) {
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
            posture_change();
            if (turnCount == 6) {
                currentState = FINAL_TURN;
            } else {
                currentState = NORMAL_DRIVE;
            }
            break;

        case FINAL_TURN:
            if (distance1 > 0 && distance1 <= 50 && distance2 >= 40) {
                turn_left();
                small_forward();
                currentState = FINISHED;
            } else {
                posture_change();
                speed_control();
            }
            break;

        case TURN_LEFT:
            turn_left();
            turnCount++;
            posture_change();
            currentState = NORMAL_DRIVE;
            break;

        case FINISHED:
            stop_motor();
            Serial.println("Finished.");
            while (true);
            return;
    }
}

void update_sensors() {
    distance1 = ping_distance(sonar1);
    distance2 = ping_distance(sonar2);
    gyro_get(&fGyro, &fAngle);
}
