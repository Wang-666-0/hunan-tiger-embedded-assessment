# 27 电软正式批考核


各考核任务分别存放在独立文件夹中，任务编号用于整理，不代表官方评分顺序。

| 任务 | 入口 |
| --- | --- |
| 01 · STM32 基础与最小系统板说明 | [查看任务目录说明](01_STM32基础与最小系统板说明/README.md) |
| 02 · CubeMX 配置与 Keil 使用 | [查看任务目录说明](02_CubeMX配置与Keil使用/README.md) |
| 03 · 基于 PWM 实现呼吸灯 | [查看任务目录说明](03_PWM呼吸灯/README.md) |
| 04 · 常用通信协议认识 | [查看任务目录说明](04_常用通信协议认识/README.md) |
| 05 · 串口通信实现 | [查看任务目录说明](05_串口通信实现/README.md) |
| 06 · Git 学习与版本管理 | [查看任务目录说明](06_Git/README.md) |
| 07 · VOFA+ 学习和使用 | [查看任务目录说明](07_VOFA学习和使用/README.md) |
| 08 · FreeRTOS 基础 | [查看任务目录说明](08_FreeRTOS基础/README.md) |
| 09 · IMU 读取任务 | [查看学习笔记](09_IMU读取任务/01_学习笔记/IMU读取与互补滤波学习笔记.md) |
| 10 · FreeRTOS 进阶 | [查看任务目录说明](10_FreeRTOS进阶/README.md) |
| 11 · CAN 通信 | [查看任务目录说明](11_CAN通信/README.md) |

任务 01～10 的学习记录已整理，任务 11_CAN通信 已建立目录并归档云台板资料，正在开展.

## 使用说明

后续学习笔记的图片引用统一为：图 01 · 图片主题 + GitHub 图片直达链接. 不保留本地图片地址、重复文件名链接或图片嵌入语法. 图片实际保存到对应任务的过程记录目录，整理完成后填写真实链接并提交推送. 既有旧笔记不追溯修改.

## 同步到 GitHub

在本文件夹空白处右键打开终端，依次执行：

```powershell
git status
git add -A
git status
git commit -m "docs: 更新本次学习内容"
git pull --rebase
git push
```

- 第一次 `git status` 用于确认发生了哪些变化。
- `git add -A` 会暂存新增、修改和删除的文件；第二次 `git status` 用于在提交前复查。
- 提交信息应简要说明本次改动，例如 `docs: 补充 GPIO 学习笔记` 或 `feat: 添加 PWM 呼吸灯工程`。
- `git pull --rebase` 用于先取得 GitHub 上可能存在的新提交，再执行 `git push` 上传本地提交。
- 如果没有任何修改，`git commit` 会提示 `nothing to commit`，此时不需要推送。

不要提交账号密码、访问令牌、私人信息或体积很大的编译产物。Keil 的 `Objects`、`Listings` 等输出目录已写入 `.gitignore`。
