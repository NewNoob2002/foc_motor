# Hardware Reference and Traceability

状态：M0 硬件阅读基线，2026-08-22。本文只记录硬件事实、推导和待验证项，不包含 FOC 算法实现。

## 1. 使用规则

- 后续设计、代码注释、测试报告引用本文的 `HW-*` 结论编号；若原始 PDF 更新，先更新文件哈希并重新核对受影响结论。
- `CONFIRMED` 表示文档交叉证据一致；`CONFIRMED_WITH_CONDITION` 表示硬件支持，但仍依赖线束、复用选择或实测；`POLICY` 表示项目约束而非电路天然限制；`OPEN` 表示缺少资料或硬件证据。
- 原理图网络名优先于接口丝印；MCU 复用功能以 STM32 数据手册为准；器件安全状态以器件手册真值表为准。
- PDF 页码均指阅读器页码；器件手册内部页码另行注明，避免封面导致偏移。

## 2. 已阅读文档清单

| 文档 ID | 仓库文件 | 页数/版本 | SHA-256 | 主要用途与限制 |
|---|---|---|---|---|
| `DOC-MAIN-SCH` | [主板原理图](./Schematic_浩盛单路电机开发板V2.0.pdf) | 1 页，REV1.0，图纸日期 2025-05-28 | `e3d94ad381ba5c3ac6b71007ef6e85fbde98f83957f7912b497e3c05d2b35cc8` | MCU、四桥臂、采样和接口网络；未提供 PCB、BOM、ERC 报告 |
| `DOC-ENC-SCH` | [编码器板原理图](./Schematic_KTH7823编码器板V1.0.pdf) | 1 页，REV1.0，图纸日期 2025-08-13 | `3e374bcd43ead46533bedef18c1e9db0f5a106ec55c1c43d98b5d0c0f250ba94` | KTH7823 型号、ABZ/SPI 两组接口和 3.3 V 供电 |
| `DOC-MCU-DS` | [STM32G431 数据手册](./STM32G431RBT6_规格书.PDF) | 198 页，DS12589 Rev 1，2019-05 | `3546211bc168e1c71440f62854d7faabe404d75ec7e4cfd34288892276bcc847` | MCU 能力、引脚复用和电气参数；版本较旧且 PDF 元数据显示为其他 G4 型号，M1 需补受控最新版、RM0440 和 errata |
| `DOC-GATE-DS` | [EG2104 数据手册](./栅极驱动芯片_EG2104_规格书.PDF) | 12 页，V1.1 | `1b4af9deaf026026a4c4b0e4b54b7e35d2226a051b9b501569fb8b6c6e88d920` | 输入、SD 真值表、传播延迟、死区、自举条件 |
| `DOC-ENC-DS` | [KTH7823 数据手册](./KTH7823.pdf) | 31 页 | `3ebc20e404700dc050ee6b10e3792a6a28038b5fa737ab688f0f41ad59e6fa52` | 角度性能、SPI、ABZ、PWM、订货型号 |

尚缺：STM32G431 最新数据手册、RM0440、对应 errata、COS722MR 手册、D0N70N06 MOSFET 手册、完整 BOM、PCB/layout、母线/散热/线束规范。缺失项对应 `R-06`、`R-12`、`R-16`，在取得资料前不得据理想计算冻结功率额定值。

## 3. 板级拓扑

主板由 STM32G431RBT6、四个 EG2104 半桥驱动级、A/C 两个低侧分流采样链和外部编码器接口组成。项目只使用 A/B/C 三个桥臂；D 桥臂在硬件上确实存在，但由项目策略永久禁止。

### 3.1 MCU

| 结论 ID | 状态 | 结论 | 证据 |
|---|---|---|---|
| `HW-MCU-001` | `CONFIRMED` | U3 是 STM32G431RBT6，LQFP64；最高 170 MHz，128 KiB Flash、32 KiB SRAM、Cortex-M4F | `DOC-MAIN-SCH` U3；`DOC-MCU-DS` PDF p1-p2、p196 ordering information |
| `HW-MCU-002` | `CONFIRMED` | 芯片有两个 12-bit ADC、TIM1/TIM8 高级电机定时器、TIM3 通用定时器、两个 DMA 控制器和 FDCAN1 | `DOC-MCU-DS` PDF p1-p2；§3.17、§3.18、§3.24、§3.26 |
| `HW-MCU-003` | `CONFIRMED` | NVIC 提供 16 个可编程优先级；TIM 事件可触发 ADC，比较器/ADC watchdog 可参与 TIM1/TIM8 break | `DOC-MCU-DS` PDF p25、p28 |

### 3.2 Console USART

- `HW-UART-001` — `CONFIRMED`：主板 `USART1_TX`/`USART1_RX` 分别连接 STM32G431 的 PB6/PB7，并经 U26 CH340N 接到板载 USB；Zephyr console 使用 USART1、921600 8N1。证据：`DOC-MAIN-SCH` U3/U26 与 USART1 网络；`DOC-MCU-DS` GPIO alternate-function table。
- `HW-UART-002` — `CONFIRMED`：USART1 日志 TX 使用 Zephyr deferred logging + asynchronous UART backend。devicetree 的 DMA selector `2` 在当前 STM32 DMA v2 驱动中映射到 DMA1 Channel 3，DMAMUX request `25` 对应 USART1_TX；USART1 和 DMA1 IRQ 均为普通优先级 5。该路径只服务诊断线程，FOC ISR 禁止调用日志或 UART API。
- 板载 U26 CH340 通过 Type-C 枚举为 `/dev/ttyUSB0`（`1a86:7523`，稳定路径 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`）。921600 8N1 下连续两次启动均完整收到 Zephyr banner、安全状态和 `seq=0..15`，共 2520 B，无乱码或丢失；运行态 BRR=`0x00b8`、DMAMUX C2CR=`0x19`。外接 `/dev/ttyACM1` 接排针时捕获为 0 B，属于外接线束问题，不影响板载控制台结论。
- 当前 DMA 分配是 M1 诊断基线。M2 接入 ADC2 DMA 时优先选择不冲突的 DMA selector；若硬件/驱动约束形成冲突，再调整 UART TX DMA，不改变 FOC ISR 禁止日志的边界。

### 3.3 PWM 与四桥臂

| 结论 ID | 相 | MCU 网络/引脚 | 外设复用 | Gate Driver | Shutdown |
|---|---|---|---|---|---|
| `HW-PWM-001` | A | `PWMA` / PC0 | TIM1_CH1 | U14 EG2104 | `SD1` / PB0，低有效 |
| `HW-PWM-002` | B | `PWMB` / PC1 | TIM1_CH2 | U15 EG2104 | `SD2` / PA1，低有效 |
| `HW-PWM-003` | C | `PWMC` / PC2 | TIM1_CH3 | U16 EG2104 | `SD3` / PA2，低有效 |
| `HW-PWM-004` | D | `PWMD` / PC3 | TIM1_CH4 | U17 EG2104 | `SD4` / PA0，低有效 |

证据链：`DOC-MAIN-SCH` 的 U3、U14-U17、`PWMA-D`、`SD1-4` 网络；`DOC-MCU-DS` PDF p52 Table 12（PC0/1/2/3 分别复用为 TIM1_CH1/2/3/4）。

- `HW-PWM-005` — `CONFIRMED`：硬件是四桥臂；A/B/C 对应 TIM1_CH1/2/3。
- `HW-PWM-006` — `POLICY`：D 相永久关闭不是原理图强制事实，而是本项目安全策略。复位和运行均保持 `SD4=0`，且不得把 PC3 配置为有效功率 PWM。
- `HW-PWM-007` — `CONFIRMED`：EG2104 在 `SD=0` 时 HO/LO 都关闭；`SD=1, IN=0` 时 HO 关闭而 LO 导通。因此“只停止 TIM1 或把 IN 拉低”不等价于安全关断，安全停机必须拉低 SD。证据：`DOC-GATE-DS` PDF p5 pin description、p11 truth table。
- `HW-PWM-008` — `CONFIRMED_WITH_CONDITION`：EG2104 标称内部死区 50-300 ns、典型 100 ns，传播延迟最高约 400 ns；最终最小脉宽、死区和自举刷新必须在真实栅极上测量。证据：`DOC-GATE-DS` PDF p8 electrical characteristics、p11 application notes。

## 4. ADC、DMA 与电流采样

| 结论 ID | 信号 | MCU 引脚 | ADC 通道 | 模拟链/用途 |
|---|---|---|---|---|
| `HW-ADC-001` | `IA` | PA6 | ADC2_IN3 | A 相 5 mΩ 低侧分流 R28，经 COS722MR，增益 10、1.65 V 偏置 |
| `HW-ADC-002` | `IC` | PA4 | ADC2_IN17 | C 相 5 mΩ 低侧分流 R29，经 COS722MR，增益 10、1.65 V 偏置 |
| `HW-ADC-003` | `BEMF_A` | PC4 | ADC2_IN5 | 相电压诊断/扩展 |
| `HW-ADC-004` | `BEMF_B` | PA7 | ADC2_IN4 | 相电压诊断/扩展 |
| `HW-ADC-005` | `BEMF_C` | PA5 | ADC2_IN13 | 相电压诊断/扩展 |
| `HW-ADC-006` | `V_BUS` | PC5 | ADC2_IN11 | 100 kΩ / 4.7 kΩ 分压和滤波 |

证据链：`DOC-MAIN-SCH` 的 R28/R29、U24/U25、IA/IC 网络和 U3 引脚；`DOC-MCU-DS` PDF p53 Table 12（PA6=ADC2_IN3、PA4=ADC2_IN17）。

- `HW-ADC-007` — `CONFIRMED`：IA/IC 均属于 ADC2，不能构成双 ADC 同时采样。首版必须使用单次 TIM1 触发的固定 rank 序列 `[IA, IC]`，由 DMA 搬运两个 half-word。
- `HW-ADC-008` — `CONFIRMED`：这是 A/C 两低侧分流；第三相需要由 `IA + IB + IC = 0` 重构。极端占空比和部分扇区存在不可观测窗口。
- `HW-ADC-009` — `CONFIRMED_WITH_CONDITION`：数据手册给出 ADC 可由定时器触发、支持 DMA 和启动校准；PA6/IN3 是 fast channel，PA4/IN17 是 slow channel。采样时间、rank 次序及两次采样时差必须在 M2 实测。证据：`DOC-MCU-DS` §3.18、§5.3.18。
- `HW-ADC-010` — `OPEN`：原理图标注理想关系 `Vout = I × 0.005 Ω × 10 + 1.65 V`，理论 0-3.3 V 对应约 ±33 A；缺少 COS722MR、分流器精度/功率和 PCB Kelvin 布局资料，不能把 ±33 A 当成安全额定值。
- `HW-DMA-001` — `CONFIRMED`：MCU 有两个 DMA 控制器、12 个通道，支持外设传输、循环模式、完成/半完成/错误中断。具体 ADC2 DMA request/channel 由 M1 的 Zephyr/STM32 描述和 RM0440 再确认。证据：`DOC-MCU-DS` §3.17。

## 5. 编码器

编码器板安装型号为 `KTH7823-X-N-QN16`，板上分别提供 CN1 SPI 和 CN2 ABZ；两组接口不是同一个连接器。

| 结论 ID | 状态 | 结论 | 证据 |
|---|---|---|---|
| `HW-ENC-001` | `CONFIRMED` | KTH7823 提供 16-bit 绝对角、ABZ、SPI/SSI/PWM；标称更新/延迟约 1 us、INL < ±0.35° | `DOC-ENC-DS` PDF p3-p5、p12 |
| `HW-ENC-002` | `CONFIRMED` | SPI 是 mode 3（CPOL=1、CPHA=1）、固定 16 bit，SCK 最高 10 MHz，响应流水重叠 | `DOC-ENC-DS` PDF p13-p18 |
| `HW-ENC-003` | `CONFIRMED` | ABZ 默认 4096 step/rev（1024 个 A/B 周期），可配置 4-4096 step/rev，A/B 最高 16 MHz，Z 每圈一次 | `DOC-ENC-DS` PDF p19-p21 |
| `HW-ENC-004` | `CONFIRMED` | PB4 可复用 TIM3_CH1/SPI1_MISO，PB5 可复用 TIM3_CH2/SPI1_MOSI；PB3 是 SPI1_SCK，PB9 可作 SPI1_NSS/GPIO CS | `DOC-MAIN-SCH` U3/U7；`DOC-MCU-DS` GPIO alternate-function tables |
| `HW-ENC-005` | `CONFIRMED_WITH_CONDITION` | 快速运行路径采用 CN2 A/B -> PB4/PB5 -> TIM3 encoder mode；Z 另接可捕获 GPIO。TIM3 是 16-bit，软件按每个控制周期读取有符号增量并扩展位置 | `DOC-ENC-SCH` CN2；`DOC-MCU-DS` §3.24.2 和引脚复用表 |
| `HW-ENC-006` | `POLICY` | SPI 仅用于电机停机时的配置/诊断，不进入快速环，也不在 FOC ISR 调用 SPI API | PB4/PB5 的 TIM3 与 SPI1 复用冲突；`DOC-ENC-SCH` CN1/CN2 分离 |
| `HW-ENC-007` | `OPEN` | 主板 U7 标 +5 V，编码器板标 VCC3.3，且两个连接器针序并不直接匹配；M2 前必须冻结 3.3 V 线束/调试夹具，禁止直接对插猜测 | `DOC-MAIN-SCH` U7；`DOC-ENC-SCH` CN1/CN2 |

选择 AB 作为快速路径意味着运行时得到的是增量位置，不是持续 SPI 绝对角。启动必须执行转子电角度对齐/归零，并用 Z 做索引或一致性检查；若产品要求上电即获得绝对机械角或运行中持续绝对诊断，需要增加独立 SPI 接线或改版。

## 6. 独立于 CPU 的硬件关断

`HW-SAFE-001` — `CONFIRMED`：当前原理图中 SD1-SD4 只由 MCU 普通 GPIO 驱动，未见外部过流比较器、驱动器故障输出、急停或锁存器直接门控所有 SD，也未见故障网络连接 TIM1_BKIN。

“独立于 CPU”指故障到门极关闭的电气链不需要 CPU 执行指令：

```text
过流/过压/驱动故障/急停
        -> 硬件比较器或故障逻辑（可锁存）
        -> 直接把 SD1/SD2/SD3 拉低
        -> EG2104 HO/LO 关闭
```

CPU 可以同时通过 TIM1 break 接收和记录故障，但不能是首次关断所必需的环节。否则 CPU 死机、时钟失效、中断被屏蔽、栈破坏或错误固件都可能让桥臂继续导通。STM32 内部 comparator/ADC watchdog 到 TIM1 break 可作为补充，但只停止 PWM `IN` 仍不能替代把 EG2104 `SD` 拉低，因为 `IN=0, SD=1` 会令低侧 MOS 导通。

- `HW-SAFE-002` — `POLICY`：该缺口不阻止 M1 工程构建，也不阻止所有 SD 保持低的 M2 无功率信号验证；它阻止进入 M4 带母线闭环，以及“工业级安全关断已验证”的声明。
- `HW-SAFE-003` — `OPEN`：建议下一硬件版以外部比较器/保护器件异步 wired-AND 拉低 A/B/C 的 SD，带故障锁存和人工复位；并行送 TIM1_BKIN 用于定时器关断和诊断。最终阈值与 `<2 us` 目标需按 MOSFET SOA、分流链带宽和实测确定。

## 7. CAN 与当前范围

- `HW-CAN-001` — `CONFIRMED`：STM32G431 有 FDCAN1，但主板未见 CAN PHY、保护和总线连接器。
- `HW-CAN-002` — `POLICY`：按当前决定，CAN/CANopen 不进入 M1-M4 范围；缺少 CAN PHY 记为延期事实，而不是当前阶段 Critical blocker。需要启动 CAN 开发时再恢复 M5 阶段门。

## 8. 已确认结论

| 结论 | 确认结果 | 限定条件 |
|---|---|---|
| TIM1_CH1/2/3 对应 A/B/C 三相；D 相永久关闭 | 确认 | D 关闭是项目策略：始终 `SD4=0`，PC3 不输出功率 PWM |
| IA/IC 均使用 ADC2，属于两低侧分流顺序采样 | 确认 | `[IA, IC]` 的 rank、采样时间和通道时差在 M2 实测 |
| 快速编码器采用 TIM3 AB；SPI 仅用于停机配置/诊断 | 有条件确认 | 需要独立 3.3 V 线束/夹具；PB4/PB5 运行时只配置为 TIM3，不能同时充当 SPI |
| CAN PHY 缺失当前不考虑 | 确认 | CAN/CANopen 延期到明确重新启动协议阶段 |
| 缺少独立于 CPU 的硬件关断路径 | 确认 | 不阻止 M1/M2 无功率验证；阻止 M4 带电闭环 |
| 基线使用当前本机 Zephyr 环境 | 确认 | M1 记录当前 checkout 的精确提交；升级环境时重新生成构建证据 |

## 9. 本机 Zephyr 环境快照

| 项目 | 实测值 | 结论 |
|---|---|---|
| 工作区 | `/home/gtc/zephyrproject` | west topdir 可识别 |
| 当前 Zephyr checkout | `VERSION=4.4.99`，`v4.4.0-12700-g8dafb9a897d`，commit `8dafb9a897da0f79a1a3724a108b21fc28719915`，branch `main` | 用户指定为当前 M1 基线；不是 stable release，升级前需显式确认 |
| west | 1.5.0，位于 `/home/gtc/zephyrproject/.venv` | 当前项目 shell 未自动激活该 venv |
| Zephyr SDK | 1.0.1 | 已安装于 `/home/gtc/zephyr-sdk-1.0.1` |
| ARM compiler | GCC 14.3.0 | `arm-zephyr-eabi-gcc (Zephyr SDK 1.0.1)` |

Context7 可用的最近版本化官方文档 ID 为 `/zephyrproject-rtos/zephyr/v4.4.0`。zero-latency IRQ 采用 direct ISR/`IRQ_ZERO_LATENCY`，位于 Cortex-M 的最高中断优先级且不得调用内核 API；M1 构建仍以本机精确提交和本地源码为准。
