#ifndef BREATH_LED_H
#define BREATH_LED_H

/* 任务入口：启动 PWM，以 5 tick 间隔推进亮度，不返回。 */
void BreathLed_Task(void *argument);

#endif /* BREATH_LED_H */
