# Documentation

Project rules are in [`AGENTS.md`](../AGENTS.md) at the repository root.

| Folder | Contents |
|---|---|
| [`plan/`](./plan/) | [Development roadmap](./plan/development_roadmap.md) (M0–M6 stage gates), [risk list](./plan/risk_list.md) |
| [`design/`](./design/) | [Zephyr architecture](./design/zephyr_architecture.md): layers, ISR/thread plan, real-time budget, CI and test design |
| [`hardware/`](./hardware/) | [Hardware reference](./hardware/hardware_reference.md) with `HW-*` evidence IDs; `schematics/` and `datasheets/` hold the source PDFs (hashes in the reference) |
| [`study/`](./study/) | [FOC study guide](./study/foc_study_guide.md) (exercises), [FOC knowledge base](./study/foc_knowledge_base.md) (workflow, design rules, results) |
| [`evidence/`](./evidence/) | [M1 build evidence](./evidence/m1_build_evidence.md), [UART3 validation](./evidence/uart3_validation.md), and raw logs in `m1/` and `uart3/` |

Code: the Zephyr application is at the repository root (`CMakeLists.txt`, `prj.conf`, `src/`, `boards/`); the FOC kernel is in `lib/foc/`; host unit tests are in `tests/host/foc/`, with simulations and plot scripts in `tests/host/foc/sim/`.
