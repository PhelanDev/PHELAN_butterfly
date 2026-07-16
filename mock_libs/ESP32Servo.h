
#pragma once
class Servo {
public:
    void setPeriodHertz(int) {}
    void attach(int, int, int) {}
    void attach(int) {} // for old Servo.h
    void detach() {}
    void write(int) {}
};

namespace ESP32PWM {
    static void allocateTimer(int) {}
}
