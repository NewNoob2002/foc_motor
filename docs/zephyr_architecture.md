# STM32G4 + Zephyr + FOC + CANopen 架构设计

状态：M1 实现基线。本文不包含 FOC 算法实现。

## 1. 范围与关键决策

平台目标是 STM32G431RBT6 上的三相 FOC 电机控制器，Zephyr 负责设备管理、控制线程和诊断；硬实时电流环由定时器触发的 ADC/DMA ISR 执行。CANopen 保留为未来目标，当前延期。现有第四桥臂不参与三相 FOC，始终通过 SD4 保持关闭。

按用户确认，M1 直接使用本机 `/home/gtc/zephyrproject`：Zephyr `4.4.99/main`（`v4.4.0-12700-g8dafb9a897d`，commit `8dafb9a897da0f79a1a3724a108b21fc28719915`）和 Zephyr SDK 1.0.1。该 checkout 不是 stable release；升级环境必须显式更新本文并重新生成构建与硬件证据。

初始控制参数是可校准基线，不是最终电机参数：

| 项目 | 基线 | 说明 |
|---|---:|---|
| PWM / FOC 周期 | 20 kHz | 允许在 10–25 kHz 内根据功率级、噪声和 WCET 调整 |
| 数值格式 | IEEE-754 `float32` | Cortex-M4F 原生支持；定点只在测量证明必要时增加 |
| 电流采样 | A/C 两低侧分流 | 第三相由电流和为零重构；极端占空比存在采样窗口限制 |
| 快速位置输入 | KTH7823 AB -> TIM3 编码器模式 | ISR 只读计数器；Z 用于索引/一致性检查 |
| 编码器 SPI | 停机配置和诊断 | 不在 FOC ISR 中调用 Zephyr SPI API |
| CANopen | 延期 | 当前 M1-M4 不开发 CAN/CANopen；恢复范围后另开 M5 |

## 2. 硬件分析

依据仓库中的主板原理图、编码器板原理图及器件手册，已确认的资源如下。逐项证据、PDF 哈希和结论编号见 [hardware_reference.md](./hardware_reference.md)。

### 2.1 MCU

- STM32G431RBT6，Cortex-M4F，最高 170 MHz。
- 128 KiB Flash、32 KiB SRAM；包含 CORDIC/FMAC、2 个 12-bit ADC、2 个高级电机定时器、2 个 DMA 控制器（共 12 通道）、FDCAN1。
- 32 KiB SRAM 是硬约束。Zephyr、线程栈和遥测全部采用静态预算，链接 map 和栈水位必须进入 CI/HIL 证据；未来恢复 CANopen 时重新做 RAM 预算。
- 主板已有 16 MHz HSE、SWD、USB-UART 和基础模拟电源；没有发现 CAN 收发器、CAN 端子或隔离器。

### 2.2 PWM Timer 与 Gate Driver

| 相 | MCU | Timer | Gate Driver | 硬关断 |
|---|---|---|---|---|
| A | PC0 | TIM1_CH1 | EG2104 U14 | SD1 / PB0，低有效 |
| B | PC1 | TIM1_CH2 | EG2104 U15 | SD2 / PA1，低有效 |
| C | PC2 | TIM1_CH3 | EG2104 U16 | SD3 / PA2，低有效 |
| D（不用） | PC3 | TIM1_CH4 | EG2104 U17 | SD4 / PA0，永久保持低 |

- TIM1 使用中心对齐 PWM，CH1/2/3 驱动三相。D 桥臂是实际硬件，但项目策略要求 `SD4=0` 且 PC3 不输出功率 PWM；CH4 只能作为不出引脚的内部比较事件。
- EG2104 用单个 `IN` 生成互补 HO/LO，并内建死区；手册给出死区 50–300 ns、典型 100 ns。软件不能独立调节上下管波形，实际死区和传播延迟必须在 MOS 栅极实测。
- EG2104 的 SD 为低时 HO/LO 同时关闭；SD 为高且 IN 为低时低侧 MOS 导通。因此仅关闭 TIM1 输出不能等价于安全关断，安全路径必须把 SD 拉低。
- 当前原理图没有把过流/故障信号接到 TIM1 BKIN 或 SD 硬件门控。带电闭环前必须增加独立硬件关断路径；软件 ISR 只能记录故障，不能承担首要保护。
- 自举高侧驱动不允许长期 100% 高侧占空比；最终调制必须保留最小低侧导通/自举刷新窗口，该值由栅极波形验证决定。

### 2.3 ADC、DMA 与 Current Sense

| 信号 | MCU 输入 | ADC | 模拟链 |
|---|---|---|---|
| `IA` | PA6 / ADC2_IN3 | ADC2 | A 相 5 mΩ 低侧分流，COS722MR 增益 10，1.65 V 偏置 |
| `IC` | PA4 / ADC2_IN17 | ADC2 | C 相 5 mΩ 低侧分流，COS722MR 增益 10，1.65 V 偏置 |
| `V_BUS` | PC5 | ADC | 100 kΩ / 4.7 kΩ 分压并滤波 |
| `BEMF_A/B/C` | PC4 / PA7 / PA5 | ADC | 20 kΩ / 1 kΩ 分压并钳位，仅用于诊断/扩展 |
| `TEMP` | PB15 | ADC | NTC 分压 |

- 理想换算为 `I = (Vadc - 1.65 V) / (5 mΩ * 10)`，理想满量程约为 ±33 A；真实限值必须结合 ADC 裕量、运放输出摆幅、分流电阻功耗和温升重新确定。
- IA 与 IC 都在 ADC2，不能双 ADC 同时采样。设计采用同一 TIM1 事件启动固定序列 `[IA, IC]`，DMA 搬运两个 half-word；两通道采样时差必须测量并纳入相位误差预算。
- ADC2 在启动时完成自校准和静态零偏采集；运行时使用固定校准系数，不在 ISR 中滤波日志或访问配置存储。
- DMA 使用固定双缓冲/循环缓冲。传输完成触发 FOC ISR；DMA 错误立即请求关断并增加粘滞故障，不重试掩盖故障。
- TIM1 比较事件放在可测量的低侧采样窗口中。两分流在部分 SVPWM 扇区和极端占空比下不可观测，后续算法必须有最小采样窗口和不可观测策略，不能用陈旧样本伪装有效数据。

### 2.4 Encoder

- KTH7823 支持 16-bit SPI 绝对角、SSI、最高 4096 step/rev 的 ABZ 和 14-bit/910 Hz PWM；器件标称约 1 us 延时、±0.35° 非线性。
- 快速环采用编码器板 CN2 的 A/B，经独立 3.3 V 线束接到 PB4=TIM3_CH1、PB5=TIM3_CH2；Z 接可捕获 GPIO。主板与编码器板连接器针序不直接匹配，M2 前必须冻结线束。
- SPI1 使用 PB3/PB4/PB5/PB9，与 TIM3 的 A/B 复用冲突。首版不做运行中 SPI/ABZ 同时使用：停机时可用 SPI 配置和诊断，运行时切到 ABZ；如产品必须持续获得绝对角，需要修改硬件或分配另一组 SPI/GPIO。
- 主板接口标注 +5 V，编码器板标注 VCC3.3。尽管芯片手册支持 3.3 V/5 V，线束、输出电平和 MCU 容限未验证前不得直接连接；产品方案优先统一为 3.3 V。
- 编码器板是单端信号且器件资料标为消费级。长线、工业 EMC、磁铁偏心、温漂和断线诊断需要 HIL/环境验证，必要时改为差分编码器接口。

### 2.5 CAN / CANopen（延期）

- STM32G431 内含 FDCAN1，PA11/PA12 可承载 RX/TX，原理图上目前未连接。
- CANopen 不能直接连接 MCU 引脚。下一版硬件至少需要 CAN 收发器、ESD、可选 120 Ω 终端、电源去耦和工业连接器；是否隔离由系统接地和安全需求决定。
- 当前不进行 CAN 协议开发，缺少 CAN PHY 不阻止 M1-M4。恢复 CANopen 范围时再确认 PHY、对象范围和是否需要 CiA 402。

## 3. 软件分层

```text
Application
  device state / diagnostics / CLI / future CANopen object dictionary
       |
Control
  safety supervisor / mode arbitration / speed & position loops
       |
FOC Kernel
  pure C numeric transforms / regulators / modulation / limit handling
       |
BSP
  TIM1 PWM / ADC2+DMA / TIM3 encoder / SD & fault / DWT / board DT
       |
Hardware
  STM32G431 / EG2104 / MOS bridge / shunts / KTH7823 / CAN PHY
```

依赖只能向下。FOC Kernel 不得包含 Zephyr、CMSIS、STM32 HAL/LL 或板级头文件；它接收一个固定输入结构并产生固定输出结构。BSP 是唯一允许直接接触寄存器和引脚的层。Control 负责命令限幅、模式切换和慢环；Application 负责外部协议与操作接口。

计划目录仅作为编码阶段契约，不在本阶段创建：

```text
app/                    Zephyr Application
boards/                 out-of-tree STM32G431 board definition
bsp/                    motor_pwm, current_adc_dma, encoder, safety_io, cycle_counter
lib/foc/                RTOS-free FOC Kernel
control/                safety and slow control loops
application/            diagnostics; CANopen only when M5 resumes
tests/host/foc/          native CMake/CTest
tests/hil/               Twister/pytest hardware harness and evidence
```

## 4. ISR 与线程架构

STM32/Zephyr 的数值优先级越小越高。下表是目标配置；实际 IRQ 编号和 Zephyr 可用优先级由 board/Kconfig 生成后通过构建断言确认。

### 4.1 ISR

| 优先级 | ISR | 类型 | 允许工作 | 禁止工作 |
|---:|---|---|---|---|
| 硬件异步 | TIM1 break / 外部门控 | 无 CPU 路径 | 直接使 PWM 无效并拉低 SD | 依赖软件完成关断 |
| 0 | ADC2 DMA TC -> FOC | direct、zero-latency | 清标志；DWT 时间戳；读取固定 ADC 缓冲和 TIM3 CNT；调用纯 FOC Kernel；写 TIM1 CCR1/2/3 影子；更新固定遥测快照 | 任意 kernel/device API、日志、SPI、CAN、分配、等待、锁 |
| 1 | break/fault latch | direct/regular | 记录硬件已关断的原因、计数和时间；置粘滞故障 | 做控制运算或尝试延时关断 |
| 2 | DMA error / encoder Z | direct/regular | DMA 错误置故障；Z 捕获计数和方向一致性 | 日志、协议处理 |
| 3 | FDCAN RX/TX/error（未来） | Zephyr driver ISR | 恢复 CAN 范围后由驱动收发固定 CAN 缓冲 | 调用 FOC Kernel |
| 4 | system timer | Zephyr kernel | 调度与超时 | 电机控制计算 |
| 5 及更低 | USART1 / DMA1 Channel 3 / USB / LCD | Zephyr driver ISR | 非实时 I/O；USART1 TX DMA 仅搬运诊断日志 | 影响控制时序 |

FOC ISR 是唯一计划使用的零延迟 ISR。硬件保护必须在它之外先完成关断，因此即使 CPU 卡死或 FOC ISR 占用，功率级仍能进入安全态。FOC ISR 返回前必须检查本周期运行时间；超期计数为粘滞诊断，连续超期触发停机。

### 4.2 Threads

采用 Zephyr preemptive priority，数字越小优先级越高；不创建高优先级 cooperative 线程，避免无意占满 CPU。

| 线程 | 建议优先级 | 触发/周期 | 职责 | 截止期 |
|---|---:|---|---|---:|
| `safety_thread` | 0 | 故障事件 + 1 kHz 健康检查 | 故障分类、状态锁存、允许使能、看门狗健康判定 | 1 ms；不承担首次硬关断 |
| `control_thread` | 1 | 1 kHz | 速度环；可选位置环分频；形成下一周期命令快照 | 1 ms，WCET < 200 us |
| `canopen_thread`（未来） | 2 | 当前不创建 | 恢复 CAN 范围后再启用 | 待 M5 定义 |
| system workqueue | 4 | 事件 | 仅非关键 Zephyr 工作 | 不得承载控制周期 |
| `application_thread` | 5 | 10–100 Hz | 模式管理、参数提交、CLI | 10 ms |
| `diagnostic_thread` | 6 | 10 Hz 或更低 | 汇总遥测、日志、栈水位和统计 | 100 ms |

线程之间使用固定大小消息和快照。Control -> FOC ISR、FOC ISR -> telemetry 都采用单写者 sequence snapshot；平台适配层提供 32-bit 原子访问和内存屏障。FOC ISR 不获取 mutex/semaphore，也不向 Zephyr queue 写入。

M1 控制台固定为 USART1 921600 8N1，使用 deferred logging 与 asynchronous UART TX。日志处理线程优先级为 6，日志环形缓冲 1024 B、UART backend 缓冲 128 B，全部静态分配；缓冲满时允许丢弃旧诊断数据，不得阻塞实时控制。devicetree 当前保留 DMA selector `2`（DMA1 Channel 3）和 DMAMUX request `25`（USART1_TX），USART1/DMA IRQ 优先级为 5。M2 接入 ADC2 DMA 时先选择其他通道；若资源冲突，再调整诊断 UART DMA。无论是否使用 DMA，FOC ISR 都不得产生日志。

## 5. 实时性与可测量性

| 路径 | 目标 | 测量方式 | 失败动作 |
|---|---|---|---|
| PWM 周期 | 20 kHz，周期 50 us | 定时器输出 + 示波器 | 禁止使能 |
| ADC 触发到 DMA TC | 固定相位；抖动 < 0.5 us | TIM1 事件与调试 GPIO/逻辑分析仪 | 标记 BSP 不合格 |
| FOC ISR WCET | < 20 us；目标 < 周期 40% | DWT CYCCNT min/max/histogram | 超期锁存，连续超期停机 |
| FOC ISR 周期抖动 | peak-to-peak < 1 us | DWT 相邻入口时间 | 超限停机/降频评估 |
| PWM 更新 | 同一更新事件原子装载 CCR1/2/3 | 三通道示波器 | 禁止闭环 |
| 硬件故障到关断 | 目标 < 2 us，最终值由功率器件 SOA 决定 | 故障注入 + 栅极波形 | 未通过不得带电闭环 |
| 速度环 | 1 kHz，release jitter < 50 us | DWT entry/exit statistics | 停机或降级 |
| CANopen 同步命令（未来） | M5 恢复后定义 | CAN 时间戳 + cycle counter | EMCY/通信故障策略 |

测量数据至少包含当前值、最大值、超限次数和最后一次超限周期号。DWT 是产品内建证据；实验固件可在 SD4 始终为低时复用 D 相路径输出 ISR 测量脉冲。量产硬件应增加独立测试点，避免依赖复用引脚。

## 6. 启动、运行与故障流程

1. 复位期间 EG2104 内部下拉保持各 SD 低；BSP 首先显式把 SD1–SD4 配置为低。
2. 初始化时钟、DWT、TIM1（输出禁用）、ADC2/DMA、TIM3；确认 D 相保持关闭。CAN 当前不初始化。
3. ADC 自校准并在 PWM 关闭时采集 IA/IC 零偏；检查母线、温度和编码器有效性。
4. 启动线程，但安全状态机仍为 `NOT_READY`；只有所有检查和外部 enable 条件成立才置 `READY`。
5. 使能时先建立可采样的安全 PWM 状态，再同时释放 A/B/C 的 SD；禁止逐相随意使能。
6. 任意硬件故障先异步关闭门极，再由 ISR/`safety_thread` 锁存原因并发 EMCY。恢复必须经过显式 fault reset 和完整预检，不能自动重试。

## 7. CI 设计

本阶段只定义 CI 契约，确认后再创建 workflow 和源码骨架。

### 7.1 Host build：FOC 数学模块

- 独立 CMake/CTest，不设置 `ZEPHYR_BASE`，借此证明 `lib/foc` 无 RTOS 依赖。
- GCC，C11，`-Wall -Wextra -Werror`，Debug 下启用 ASan/UBSan。
- 命令契约：`cmake -S tests/host/foc -B build/host -DCMAKE_BUILD_TYPE=Debug`、`cmake --build build/host`、`ctest --test-dir build/host --output-on-failure`。
- 合并门：编译零告警、全部测试通过、无 sanitizer 报告。

### 7.2 ARM build：Zephyr firmware

- 本地构建使用 `/home/gtc/zephyrproject` 的 Zephyr commit `8dafb9a897da0f79a1a3724a108b21fc28719915` 和 Zephyr SDK 1.0.1；CI 对同一提交做基线检查。
- 自定义 board/devicetree 描述 STM32G431RBT6、时钟、引脚、Flash/RAM 和安全默认状态。
- 命令契约：`west build -p always -b <foc_motor_board> app`。
- 合并门：ARM 编译零告警、devicetree/Kconfig 通过、ELF/BIN/map 产出、Flash/RAM 和每个线程栈低于预算。
- CI 不烧录。HIL 使用独占 runner、板卡锁和显式安全预检，和普通 pull request build 分离。

## 8. 测试设计

### 8.1 PC / Host

算法实现获批后，每项使用同一份纯 C 源码和独立参考值：

| 模块 | 最小测试集 | 关键判据 |
|---|---|---|
| Clarke | 零向量、平衡三相基向量、正负满量程、随机向量 | 约定的幅值/功率归一化正确；第三相重构一致 |
| Park / inverse Park | 0、±π/2、π、跨 2π、随机 round-trip | 方向和符号约定固定；round-trip 在容差内 |
| PI | 零误差、阶跃、正负饱和、积分限幅/抗饱和、复位 | 输出有界、无 NaN/Inf、饱和恢复符合约定 |
| SVPWM | 六扇区中心与边界、零矢量、最大线性调制、过调制输入 | duty 有界、扇区连续、重构矢量在容差内、采样窗口满足下限 |

浮点容差按绝对误差与相对误差共同定义；不使用只复制被测公式的“镜像实现”作为唯一 oracle。随机测试固定 seed 并保存失败向量。

### 8.2 Hardware / HIL

所有功率测试先从无母线/低压限流开始，监视器先于激励启动，所有等待有 timeout。

| 对象 | 验证内容 | 证据与通过条件 |
|---|---|---|
| PWM | 频率、中心对齐、CH1/2/3 同步更新、D 相关闭、SD 关断、实际死区、自举刷新 | 栅极示波器截图与原始数据；无重叠导通；频率/死区在批准公差内 |
| ADC + DMA | 触发相位、序列 `[IA, IC]`、DMA 每周期一次、零偏、增益、噪声、饱和、通道时差、DMA 错误关断 | 已知电流/电压注入；误差和抖动在校准预算内；零丢周期 |
| Encoder | A/B 方向、counts/rev、Z 一圈一次、反转、最大边沿率、断线/非法跳变、SPI mode 3 停机读取 | 逻辑分析仪 + 角度基准；计数/方向一致、无丢边、故障可诊断 |

每次 HIL 记录固件 SHA、板卡序列号、探头/仪器、供电限制、接线、命令、原始日志、pass/fail/skip 数量和清理结果。重试不能覆盖首次失败。

## 9. 尚待确认的系统决策

- 电机额定/峰值电流、母线电压、极对数、最高机械转速和允许的 PWM 频率范围。
- CAN/CANopen 当前延期；恢复范围时再确认 CiA 402、隔离、连接器和终端策略。
- 编码器运行时只用 ABZ 是否可接受；如必须持续绝对角，需硬件改版。
- 硬件过流阈值、关断时间和复位策略。

## 10. 参考资料

- [主板原理图](./Schematic_浩盛单路电机开发板V2.0.pdf)
- [编码器板原理图](./Schematic_KTH7823编码器板V1.0.pdf)
- [STM32G431 数据手册](./STM32G431RBT6_规格书.PDF)
- [EG2104 数据手册](./栅极驱动芯片_EG2104_规格书.PDF)
- [KTH7823 数据手册](./KTH7823.pdf)
- [硬件证据与结论索引](./hardware_reference.md)
- [Zephyr supported releases](https://docs.zephyrproject.org/latest/releases/)
- [Zephyr interrupt management](https://docs.zephyrproject.org/latest/kernel/services/interrupts.html)
- [Zephyr testing with Twister](https://docs.zephyrproject.org/latest/develop/test/twister.html)
