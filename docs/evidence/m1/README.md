# M1 Evidence Index

记录日期：2026-08-22。以下文件是 `build/hil/` 原始日志的受控副本；固件产物本身由 CI artifact 保存。

最终远程 CI 为 [run 32583149447](https://github.com/NewNoob2002/foc_motor/actions/runs/32583149447)，认证 commit `7080602461b0a0d2339a4bf07e0e92a741a1055b`；六项 artifact 哈希见 [`m1_build_evidence.md`](../../m1_build_evidence.md)。

| 文件 | 内容 | SHA-256 |
|---|---|---|
| `flash_uart_dma_921600.log` | CMSIS-DAP 目标身份、写入和 verify | `c3fe2d6b10cd918ece217a470c10f52cc8c6cf8a76306a7c85c7fa0e499ec556` |
| `uart_dma_921600.log` | CH340 921600 8N1 两次启动与 DMA TX 序列 | `b3ebe1fb0bb62f44924775536bb0b50bb597a7ce4e0f435ef8ab352053063e83` |
| `openocd_uart_dma_registers.log` | USART1、DMA1、DMAMUX 和 SD GPIO 运行态寄存器 | `089ab15f6e37b91e3b7fe22a8694b62b1203e0f36c3e6425dca184db18ce38ef` |

本目录不包含 `preflash_uart_dma_921600.bin`。恢复镜像保留在本机 `build/hil/`，大小 131072 B，SHA-256 `056e105fc9eab905048fe5f798de926099e15421d9eff9114445f3aac322a21a`；避免把未知原固件二进制提交到源码仓库。

真实 SD/PWM 引脚波形尚未采集，按 `R-18` 保持阻塞；后续证据必须包含板卡身份、固件提交、探头接线、采样率、供电条件和原始波形。
