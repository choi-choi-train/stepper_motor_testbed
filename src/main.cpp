#include <Arduino.h>
#include <stepper.h>
#include <TeensyTimerTool.h>
#include <cmath>
#include <vector>

#define BUTTON_PIN 15
#define POT_PIN 14
#define LED_PIN 13
#define STEP_PIN 19
#define DIR_PIN 20

using namespace std;
TeensyTimerTool::PeriodicTimer t_main;

bool buttonHeld = false;
int startTime = 0;
int releaseTime = 0;
float maxSpeed = 7500.0;

void potentiometer_button_control();
void angle_homing();

Stepper motor_1(STEP_PIN, DIR_PIN, maxSpeed);

void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(POT_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);

    motor_1.setPullInSpeed(1000.0); //speed to which the stepper can instantly jump without ramp-up

    while(!Serial) {;}
    TeensyTimerTool::attachErrFunc(TeensyTimerTool::ErrorHandler(Serial));
    // t_main.begin(potentiometer_button_control, 1000);
    // Serial.println("STARTING PROGRAM");
    t_main.begin(angle_homing, 1000);
}

void potentiometer_button_control() {
    int read = analogRead(POT_PIN); //1023 -> 0 when wiping left to right
    float targetSpeed = ((float)(511.5-read)/511.5) * maxSpeed;

    bool button_pressed = digitalRead(BUTTON_PIN) == LOW;
    if (button_pressed) {
        digitalWrite(LED_PIN, HIGH);
        if (!buttonHeld) {
            Serial.printf("STARTING SPEED: %f\n", motor_1.currentSpeed());
            Serial.printf("ACCELERATING TO SPEED %f\n", targetSpeed);
            buttonHeld = true;
            motor_1.setStartingSpeed(motor_1.currentSpeed());
            startTime = micros();
        }
        motor_1.setTargetSpeed(targetSpeed);
        Serial.printf("^^^ %f\n", motor_1.currentSpeed());
        motor_1.bezcelerate(startTime);
    } else {
        digitalWrite(LED_PIN, LOW);
        if (buttonHeld) {
            buttonHeld = false;
            releaseTime = micros();
            motor_1.setStartingSpeed(motor_1.currentSpeed());
            Serial.println("DECELERATING....");
        }
        motor_1.setTargetSpeed(0);
        Serial.printf("vvv %f\n", motor_1.currentSpeed());
        motor_1.bezcelerate(releaseTime);
    }
}

bool event_happening = false;
void angle_homing() {
    bool buttonPressed = digitalRead(BUTTON_PIN) == LOW;
    if (event_happening) {
        digitalWrite(LED_PIN, HIGH);
        motor_1.moveBy(360, startTime);
    } else {
        digitalWrite(LED_PIN, LOW);
    }

    if (!buttonHeld && buttonPressed) {
        startTime = micros();
        motor_1.setStepCounter(0);
        motor_1.setTargetSpeed(7500);
        buttonHeld = true;
        event_happening = true;
    }
}

void loop() {
    // sanitychecking
    // bool button_pressed = digitalRead(BUTTON_PIN) == LOW;
    // if (button_pressed) {
    //     digitalWrite(LED_PIN, HIGH);
    //     digitalWrite(STEP_PIN, HIGH);
    //     delayMicroseconds(1000); // 1–2 µs minimum for TMC2209
    //     digitalWrite(STEP_PIN, LOW);
    //     delayMicroseconds(1000);
    // } else {
    //     digitalWrite(LED_PIN, LOW);
    // }
    ;
}