#ifndef FEEDER_CONSOLE_H
#define FEEDER_CONSOLE_H
#include <stddef.h>
#include <stdint.h>
uint8_t FeederConsole_Handle(const char *line, char *response, size_t capacity);
void FeederConsole_Telemetry(char *response, size_t capacity);
void FeederConsole_SpeedTelemetry(char *response, size_t capacity);
#endif
