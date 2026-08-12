#pragma once

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef uint8_t byte;
typedef bool boolean;
typedef uint16_t word;

class __FlashStringHelper;
#define F(str) (reinterpret_cast<const __FlashStringHelper *>(str))

#ifndef HIGH
#define HIGH 0x1
#endif
#ifndef LOW
#define LOW 0x0
#endif

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define ANALOG 0x3

#define CHANGE 1
#define FALLING 2
#define RISING 3

#ifndef MSBFIRST
#define MSBFIRST 1
#endif
#ifndef LSBFIRST
#define LSBFIRST 0
#endif

#ifndef abs
#define abs(x) ((x) >= 0 ? (x) : -(x))
#endif
#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

inline bool isDigit(int c)
{
    return isdigit(c) != 0;
}

class StringSumHelper
{
public:
    explicit StringSumHelper(const char *s = "") : s_(s ? s : "") {}
    const char *c_str() const { return s_; }

private:
    const char *s_;
};

class Print
{
public:
    virtual ~Print() {}
    virtual size_t write(uint8_t c) = 0;

    virtual size_t write(const uint8_t *buffer, size_t size)
    {
        size_t written = 0;
        while (written < size)
        {
            written += write(buffer[written]);
        }
        return written;
    }

    size_t print(const char *s)
    {
        if (!s)
        {
            return 0;
        }
        return write(reinterpret_cast<const uint8_t *>(s), strlen(s));
    }

    size_t print(const __FlashStringHelper *s)
    {
        return print(reinterpret_cast<const char *>(s));
    }

    size_t print(char c)
    {
        return write(static_cast<uint8_t>(c));
    }

    size_t print(int n)
    {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", n);
        return print(buf);
    }

    size_t print(unsigned int n)
    {
        char buf[16];
        snprintf(buf, sizeof(buf), "%u", n);
        return print(buf);
    }

    size_t print(long n)
    {
        char buf[24];
        snprintf(buf, sizeof(buf), "%ld", n);
        return print(buf);
    }

    size_t print(unsigned long n)
    {
        char buf[24];
        snprintf(buf, sizeof(buf), "%lu", n);
        return print(buf);
    }

    size_t print(float n, int digits = 2)
    {
        char buf[40];
        if (digits < 0)
        {
            digits = 2;
        }
        if (digits > 8)
        {
            digits = 8;
        }
        snprintf(buf, sizeof(buf), "%.*f", digits, static_cast<double>(n));
        return print(buf);
    }

    size_t print(double n, int digits = 2)
    {
        return print(static_cast<float>(n), digits);
    }

    size_t println()
    {
        return print("\r\n");
    }

    size_t println(const char *s)
    {
        size_t n = print(s);
        return n + println();
    }

    size_t println(const __FlashStringHelper *s)
    {
        size_t n = print(s);
        return n + println();
    }

    size_t println(char c)
    {
        size_t n = print(c);
        return n + println();
    }

    size_t println(int n)
    {
        size_t r = print(n);
        return r + println();
    }

    size_t println(unsigned int n)
    {
        size_t r = print(n);
        return r + println();
    }

    size_t println(long n)
    {
        size_t r = print(n);
        return r + println();
    }

    size_t println(unsigned long n)
    {
        size_t r = print(n);
        return r + println();
    }

    size_t println(float n, int digits = 2)
    {
        size_t r = print(n, digits);
        return r + println();
    }
};

class Stream : public Print
{
public:
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return -1; }
    virtual void flush() {}
};

class HardwareSerial : public Stream
{
public:
    void begin(unsigned long baud);
    int available() override;
    int read() override;
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
};

extern HardwareSerial Serial;

void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
uint32_t analogRead(int pin);
void analogWrite(int pin, int value);
int digitalPinToInterrupt(int pin);
void attachInterrupt(int interruptNum, void (*userFunc)(void), int mode);
void noInterrupts(void);
void interrupts(void);

unsigned long micros(void);
unsigned long millis(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000UL);
