#ifndef REMOTE_APP_H
#define REMOTE_APP_H
#include "remote_control.h"
void RemoteApp_Init(void);
void RemoteApp_Step(void);
void RemoteApp_Stop(RemoteControl_Fault fault);
void RemoteApp_GetState(RemoteControl *out);
#endif
