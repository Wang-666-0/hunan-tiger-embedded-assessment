/* Pitch 串口命令 */
#ifndef PITCH_CONSOLE_H
#define PITCH_CONSOLE_H
#include "app_config.h"
#include <stdint.h>
#include <stddef.h>
void PitchConsole_Init(void);
uint8_t PitchConsole_Handle(const char *line, char *response, size_t capacity);
uint8_t PitchConsole_Poll(char *response, size_t capacity);
void PitchConsole_CancelPreparation(void);
#endif
