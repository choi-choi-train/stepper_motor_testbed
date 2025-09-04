#pragma once
#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <bezier.h>
#include <cstdlib>

using namespace std;

class Stepper {
    public:
        Stepper(int stepPin, int dirPin, float maxSpeed) {
            _maxSpeed = maxSpeed;
            _beziertable = lookup(_maxSpeed);
            _stepPin = stepPin;
            _dirPin = dirPin;
        }

        float currentSpeed() {
            return _current_speed;
        }
        void setTargetSpeed(float targetSpeed) {
            _target_speed = targetSpeed;
        }
        void setPullInSpeed(float pullInSpeed) {
            _pullin_speed = pullInSpeed;
        }
        void setStartingSpeed(float startingSpeed) {
            _starting_speed = startingSpeed;
        }
        void setStepCounter(int stepCount) {
            _stepCounter = stepCount;
        }
        
        float bezceleration(int startTime, int accelTime, float startSpeed, float endSpeed) {
            float res_speed;
            int elapsedTime = micros() - startTime;
            float t = (float)elapsedTime / (float)accelTime;
            if (t > 1.0) return -1.0; 

            int i = static_cast<int>(t * (_beziertable.size()-1));
            float scalar = _beziertable[i];

            float difference = endSpeed - startSpeed; 
            res_speed = startSpeed + difference*scalar;
            _current_speed = res_speed;
            // Serial.printf("RES_SPEED: %.2f\n", res_speed);

            return res_speed;
        }
        void stepISR() {
            digitalWriteFast(_stepPin, HIGH);
            digitalWriteFast(_stepPin, LOW);
            _stepCounter++;
        }

        void startStepping() {
            _stepTimer.begin([this] {this->stepISR();}, 1000);
            _timer_running = true;
            Serial.println("starting stepping");
        }

        void setStepSpeed(float speed) {
            if (speed < 1.0) speed = 1.0;
            float period = 1000000.0/speed;
            _stepTimer.setPeriod(period);
        }

        void moveToAngle(float angle, int startTime) {
            float steps_to_take = (float)angle/1.8;
        }

    private:
        int _stepPin, _dirPin;
        volatile int _stepCounter = 0;
        int _decel_time = 0;
        bool _pulse_high = false;
        bool _timer_running = false;
        bool _cruising = false;
        float _maxSpeed;
        float _target_speed = 0;
        float _current_speed = 0;
        float _starting_speed = 0;
        float _pullin_speed = 0;
        TeensyTimerTool::PeriodicTimer _stepTimer;
        vector<float> _beziertable;
};

// REWRITE MOVEBY ANGLE USING RESHAPED HELPER FUNCTIONS