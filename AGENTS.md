# foc_motor 开发规则

## 当前阶段

- 当前处于 M1 收尾：允许 Zephyr 工程骨架、CI、无母线安全启动和诊断验证。进入 M2 外设实现前需确认阶段门；再次明确确认进入 M3 前，不实现或接入 Clarke、Park、PI、SVPWM 或闭环 FOC 算法。
- 硬件事实和证据编号见 `docs/hardware_reference.md`，架构基线见 `docs/zephyr_architecture.md`，阶段门见 `docs/development_roadmap.md`，M1 构建证据见 `docs/m1_build_evidence.md`，未关闭风险见 `docs/risk_list.md`。

## Zephyr 与实时性

- 禁止在 FOC ISR 中调用任何 Zephyr kernel API、Zephyr 设备 API、日志、队列、信号量、互斥量、内存分配或 CAN/SPI API。
- FOC ISR 只允许访问预配置寄存器/固定缓冲区、读取周期计数器、调用无 RTOS 依赖的 FOC Kernel，并写入 PWM 影子寄存器。
- FOC 数学模块必须是纯 C，不得包含 Zephyr、CMSIS、STM32 HAL/LL 或板级头文件；host 与 ARM 构建必须复用同一份源码。
- 禁止动态内存：不得使用 `malloc/calloc/realloc/free`、`new/delete`、`k_malloc/k_calloc/k_heap`。线程栈、CANopen 对象字典、消息缓冲区和遥测缓冲区全部静态分配。
- 所有周期任务和 ISR 必须可测量：记录开始、结束、周期、WCET、最大抖动和超期计数。FOC ISR 使用 DWT CYCCNT；实验构建可在 SD4 保持低电平时复用 D 相信号做示波器测量。
- 中断优先级数字越小优先级越高。FOC 采样 ISR 是唯一计划使用的零延迟中断；普通线程和普通驱动不得提升到同级。
- FOC ISR 的目标周期为 20 kHz（可校准范围 10–25 kHz），50 us 截止期内 WCET 不得超过 20 us，连续运行不得出现超期。

## 分层与依赖

- 依赖只能向下：Application -> Control -> FOC Kernel -> BSP -> Hardware。
- BSP 负责引脚、时钟、TIM1、ADC2/DMA、TIM3 编码器、关断和周期计数；应用层不得直接写外设寄存器。
- FOC Kernel 只接收数值输入并返回数值输出，不拥有设备、线程、时钟、日志或通信。
- ISR 与线程之间只使用固定大小、单写者的快照；不得用锁阻塞 FOC ISR。
- 引脚、外设实例和时钟选择放在 devicetree/Kconfig 或 BSP，不在 Control/Application 中硬编码。

## 安全默认值

- 复位后立即保持 SD1/SD2/SD3/SD4 为低；D 相在本项目中始终禁用。
- PWM 只能在 ADC 校准、编码器检查、故障检查和控制状态机全部通过后使能。
- 软件故障处理不是功率级保护。带电闭环测试前必须具备独立于 CPU 的硬件关断路径，并验证其关断时间。
- 看门狗只能由健康监督逻辑喂狗；不得在 FOC ISR 或无条件周期任务中喂狗。
- 不得在没有限流电源、急停、目标板身份确认和恢复方案时执行功率测试或烧录。

## 构建与测试

- Host 构建必须在没有 Zephyr 环境的普通 CMake/CTest 环境中编译 FOC 数学模块，并以 warnings-as-errors 和 ASan/UBSan 运行。
- ARM 构建使用本机 `/home/gtc/zephyrproject` 当前基线：Zephyr `4.4.99/main`（commit `8dafb9a897da0f79a1a3724a108b21fc28719915`）、Zephyr SDK 1.0.1 和 `west build`。升级该 checkout 前必须先更新并重新生成 M1 证据；CI 只构建/测试，不自动烧录。
- Clarke、Park、PI、SVPWM 的每个行为变更必须先有 host 测试；PWM、ADC、DMA、编码器的行为必须由目标板/HIL 证据确认。
- Hardware 测试必须记录板卡序列号、固件提交、工具版本、接线、供电限制、原始日志和示波器/逻辑分析仪证据。
- Host 测试不能替代 ARM 构建，ARM 构建不能替代 HIL。没有硬件证据时不得声称硬件已验证。

## 实现原则

- 使用 C11 和 `float32` 作为初始数值基线；只有 WCET 或数值证据表明不足时才引入定点实现。
- 当前不开发 CAN/CANopen；重新启动该范围时优先使用 Zephyr 已有子系统和已集成的 CANopenNode，不自研 RTOS、协议栈、测试框架或内存管理器。
- 不为未验证需求增加抽象、配置项或依赖；最小可验证实现优先。
