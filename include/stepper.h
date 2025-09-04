#pragma once
#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <bezier.h>
#include <cstdlib>

using namespace std;
#define accel_time 2000000 //microseconds -> 1.0 seconds

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
        void bezcelerate(int startTime) {
            int elapsedTime = micros() - startTime;
            float t = (float) elapsedTime / accel_time;
            if (t > 1.0) t = 1.0; 
            
            int i = static_cast<int>(t * (_beziertable.size()-1));
            float scalar = _beziertable[i];
            
            float difference = _target_speed - _starting_speed; 
            _current_speed = _starting_speed + difference*scalar;

            if (_current_speed < 0) {
                digitalWrite(_dirPin, LOW);
            } else {
                digitalWrite(_dirPin, HIGH);
            }
            // Serial.printf("CURRENT SPEED: %.6f\n", _current_speed);
            
            // NEW PWM SCHEME: CHANGE TEENSYTIMERTOOL PERIOD
            if (abs(_current_speed) < 0.01) {
                if (_timer_running) {                
                    _stepTimer.stop();
                    _timer_running = false;
                }
            } else {
                float new_period = (float)(1000000/fabs(_current_speed));
                if (!_timer_running) {
                    int pin = 0 + _stepPin;
                    int* step_counter = &_stepCounter;
                    _stepTimer.begin([pin, step_counter](){
                        digitalWriteFast(pin, !digitalRead(pin));
                        if (digitalRead(pin)) {*step_counter = *step_counter+1;}
                        delayNanoseconds(2);
                    }, new_period);
                    _timer_running = true;
                } else {
                    _stepTimer.setPeriod(new_period);
                }
            } 
        }

        void moveBy(float angle, int startTime) {
            int steps_to_take = (float)angle/1.8;
            bezcelerate(startTime);
            Serial.printf("bezcelerating...")
            // if (_stepCounter <= steps_to_take) {
            //     Serial.printf("STEPS: %.d / %.2f\n", _stepCounter, steps_to_take);
            // }
        }

    private:
        int _stepPin, _dirPin;
        int _stepCounter = 0;
        int _decel_time = 0;
        bool _pulse_high = false;
        bool _timer_running = false;
        float _maxSpeed;
        float _target_speed = 0;
        float _current_speed = 0;
        float _starting_speed = 0;
        float _pullin_speed = 0;
        TeensyTimerTool::PeriodicTimer _stepTimer;
        vector<float> _beziertable;
};

// TODO: FIX MOVEBY TO COUNT STEPS INSTEAD OF TIME
    // ALSO NEED TO ADD STEP TALLYING TO BEZCELERATION
    // ALLOWS ME TO USE ALTERNATIVES TO BEZIER PROFILE FOR LOWER SPEEDS WITHOUT SACRIFICING RELIABILITY IN HOMING TO CERTAIN ANGLES
    // 