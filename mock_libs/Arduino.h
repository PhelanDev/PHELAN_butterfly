
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#ifndef abs
#define abs(x) ((x)>0?(x):-(x))
#endif

typedef uint8_t byte;

#define LOW 0
#define HIGH 1
#define OUTPUT 1

void pinMode(int, int);
void digitalWrite(int, int);
int digitalRead(int);
unsigned long millis();

class HardwareSerial {
public:
    HardwareSerial(int) {}
    void begin(long, int=0, int=-1, int=-1, bool=false) {}
    void end() {}
    bool available() { return false; }
    int read() { return 0; }
    int availableForWrite() { return 64; }
    void print(const char*) {}
    void print(int) {}
    void println(const char*) {}
    void println(int) {}
};

extern HardwareSerial Serial;
#define SERIAL_8N1 0
#define SERIAL_8E2 0
