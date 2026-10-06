#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

/* 输入不含 CR/LF、以 '\0' 结尾的完整行；B0..1000，T200..10000。
 * 这里只解析并投递命令，硬件更新由 PWM 任务负责。
 */
void CommandParser_Process(const char *line);

#endif /* COMMAND_PARSER_H */
