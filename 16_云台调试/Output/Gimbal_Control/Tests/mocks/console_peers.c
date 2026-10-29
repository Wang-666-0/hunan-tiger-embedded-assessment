/* 单元测试的外围模块替身。被测模块的真实实现或夹具会覆盖 weak 定义。 */
#include "pitch_backend.h"
#include "pitch_console.h"
#include "feeder_console.h"
#include <string.h>

__attribute__((weak)) void PitchBackend_GetState(PitchBackendState *out)
{ memset(out, 0, sizeof(*out)); }
__attribute__((weak)) void PitchBackend_GetStep(PitchStep *out)
{ memset(out, 0, sizeof(*out)); }
__attribute__((weak)) void PitchBackend_PrepareViewHold(void) { }
__attribute__((weak)) void PitchConsole_Init(void) { }
__attribute__((weak)) void PitchConsole_CancelPreparation(void) { }
__attribute__((weak)) uint8_t PitchConsole_Handle(const char *line, char *out, size_t size)
{ (void)line; (void)out; (void)size; return 0U; }
__attribute__((weak)) uint8_t PitchConsole_Poll(char *out, size_t size)
{ (void)out; (void)size; return 0U; }
__attribute__((weak)) uint8_t FeederConsole_Handle(const char *line, char *out, size_t size)
{ (void)line; (void)out; (void)size; return 0U; }
__attribute__((weak)) void FeederConsole_Telemetry(char *out, size_t size)
{ if (size) { out[0] = '\0'; } }
__attribute__((weak)) void FeederConsole_SpeedTelemetry(char *out, size_t size)
{ if (size) { out[0] = '\0'; } }
