#include <stdio.h>
#include <stdlib.h>
#include <lgpio.h>
#include <unistd.h>

#define MONITOR_CHARGING_PIN 23
#define VENT_MONITOR_PIN 26
#define VENT_CASE_PIN 7
#define MONITOR_ON_PIN 17

int main()
{
    int pi = lgGpiochipOpen(0);
    if (pi < 0)
    {
        perror("Can't open gpiochip");
        exit(100);
    }

    if (lgGpioClaimOutput(pi, 0, MONITOR_CHARGING_PIN, 0) < 0)
    {
        perror("Can't claim monitor charging pin");
        exit(100);
    }

    if (lgGpioClaimOutput(pi, 0, VENT_MONITOR_PIN, 0) < 0)
    {
        perror("Can't claim monitor ventilator pin");
        exit(100);
    }

    if (lgGpioClaimOutput(pi, 0, VENT_CASE_PIN, 0) < 0)
    {
        perror("Can't claim case ventilator pin");
        exit(100);
    }

    if (lgGpioClaimOutput(pi, 0, MONITOR_ON_PIN, 0) < 0)
    {
        perror("Can't claim monitor on pin");
        exit(100);
    }

    usleep(10e3);

    lgGpioWrite(pi, MONITOR_CHARGING_PIN, 1);

    usleep(10e3);

    lgGpioWrite(pi, VENT_MONITOR_PIN, 1);

    usleep(10e3);

    lgGpioWrite(pi, VENT_CASE_PIN, 1);

    usleep(10e3);

    lgGpioWrite(pi, MONITOR_ON_PIN, 0);
    usleep(2e6);
    lgGpioWrite(pi, MONITOR_ON_PIN, 1);

    usleep(10e3);

    lgGpiochipClose(pi);
    return 0;
}