# STM32G431 PMSM FOC 实验记录

本系列记录基于 STM32G431 的永磁同步电机（PMSM）有感 FOC 控制系统开发过程。

从 FOC 理论基础出发，逐步完成硬件选型与检查、三相 PWM 输出、编码器反馈、电流同步采样、坐标变换、电流 PI 控制，以及速度环和位置环控制验证。

## 一、理论基础与硬件准备

1. [P0. FOC基础知识](https://zhuanlan.zhihu.com/p/2086862123133572398)
2. [P1-1. 硬件选材](https://zhuanlan.zhihu.com/p/2088256698574291280)
3. [P1-2. 硬件检查](https://zhuanlan.zhihu.com/p/2089489576817583207)

## 二、PWM与电机位置反馈

1. [实验：验证六路互补PWM](https://zhuanlan.zhihu.com/p/2089496073614185917)
2. [实验：产生一个旋转的电压矢量](https://zhuanlan.zhihu.com/p/2090131839415538927)
3. [实验：编码器读取链路验证](https://zhuanlan.zhihu.com/p/2091650671690961471)

## 三、电流采样与坐标变换

1. [实验：三路电流采样零点校准与电流换算](https://zhuanlan.zhihu.com/p/2092268916248687715)
2. [实验：PWM同步电流采样链路验证](https://zhuanlan.zhihu.com/p/2093367720058406605)
3. [实验：ADC采样窗口分析与TIM1 Break保护验证](https://zhuanlan.zhihu.com/p/2094900903023331044)
4. [实验：三相电流采样链路搭建与Park变换模长守恒验证](https://zhuanlan.zhihu.com/p/2098803208881693415)
5. [实验：编码器电角度零点校准与dq电流行为验证](https://zhuanlan.zhihu.com/p/2098807111996262143)

## 四、闭环控制与实验验证

1. [FOC波形分析上位机推荐——VOFA+](https://zhuanlan.zhihu.com/p/2100289543144583943)
2. [实验：Id/Iq双电流环PI闭环控制](https://zhuanlan.zhihu.com/p/2103514479803813996)
3. [实验：FOC三环控制实现与验证](https://zhuanlan.zhihu.com/p/2105325971461222992)

## 项目硬件平台

- 主控：STM32G431RB（NUCLEO-G431RB）
- 功率驱动板：X-NUCLEO-IHM08M1
- 电机：ACM4010V24H-B5
- 位置反馈：增量式编码器
- 调试工具：STM32CubeIDE、VOFA+、逻辑分析仪

## 主要实验内容

- TIM1 中心对齐互补 PWM
- 空间矢量脉宽调制（SVPWM）
- TIM2 编码器位置反馈
- TIM1 与 ADC 注入转换同步
- 电流采样零点校准
- Clarke/Park 坐标变换
- 编码器电角度零点校准
- Id/Iq 双电流环 PI 调节
- 速度环与位置环闭环控制
- ADC 采样窗口检查与 PWM 故障保护

本系列文章主要记录实际实验中的原理分析、参数配置、测试过程、异常现象与问题排查。