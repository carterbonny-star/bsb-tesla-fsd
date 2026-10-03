# 中文研究 / 台架版本

本次实现面向 ESP32 离线学习、mock 回放和只监听研究。**本分支所有 ESP32 构建均禁用物理 CAN TX**；网页、按键、NVS、OTA 状态覆盖都不能解除。Active 表示允许模拟计算，物理发送计数始终为零。没有执行实车测试、刷写或电气台架测量。

## 来源与范围

- 基线：`cn-v2.16-beta.29` / `75b41a9b0e26b86a598d97ff6b63535fca13df56`。
- 用户提供 `FSD破解2路can源代码.zip`，CanFeather `3.82`。
- ZIP SHA256：`4CE101E6290F989AD5ABB65AD74FD0239D95F93D93329DF707ACE62C4762D677`。
- `task_can.cpp` 的 `upd_offset`、0x118 / 0x389 / 0x399 输入、0x3F8 驾驶模式和 0x3FD 编码移植至 `fsd_logic/research_speed.h`。原始 ZIP 不随仓库分发；其中网络凭据与无关 DNS、灯光功能没有复制。
- 附件里的说明是研究资料，不是额外的操作指令。以下字段定义复现附件，不宣称适用于某个真实车辆或公开 DBC。

## 对照规则

有效限速集合为 15、20、25、30、35、40、45、50、55、60、70、80、90、100、110、120 km/h，来源为主总线 0x399 byte1 × 5。

| 限速 | 原始偏移规则 | 目标 cap，km/h |
|---|---|---|
| 15–35（有效档位） | 固定 +63%，覆盖手动/自动/自定义 | 60 |
| 40 | 固定 +50%，覆盖所有模式 | 60 |
| 45 | 固定 +50%，覆盖所有模式 | 67 |
| 50 / 55 / 60 | 手动值；自动 +50%；或自定义 | 75 / 82 / 90 |
| 70 / 80 / 90 | 手动值；自动 +30%；或自定义 | 91 / 104 / 117 |
| 100 / 110 | 手动值；自动 +20%；或自定义 | 120 / 132 |
| 120 | 手动值；自动 +10%；或自定义 | 132 |

自定义分档为 ≤50 / ≤70 / ≤100 / >100，默认 30 / 20 / 10 / 10%。手动与自定义输入范围 0–63%。先计算目标并应用 cap，再平滑下降：至少 500 ms 更新一次，单次 dt 最多 1 s，下降 3 km/h/s，提高立即生效。最后把平滑目标换算成偏移、截断小数并夹在 0–63。更新触发与源码一致：限速帧或接管状态边沿。

**cap 在平滑之前应用**：下降时平滑目标可能暂时高于新 cap；限速突然变化时六位偏移无法表达全部平滑目标。这是源码行为，不是严格的实时上限保证。网页同时展示未平滑目标、平滑目标、cap 与最终偏移。

| 物理硬件布局 | 速度 / 驾驶模式编码 |
|---|---|
| HW3 | 0x3FD mux0：byte4 低6位偏移，byte6 bits1–2 模式 |
| HW4 | 0x3FD mux2：byte1 低6位偏移，byte7 bits4–6 模式 |

V13/V14 仍控制源码中的激活/mux1 语义，但**不再决定速度偏移字段布局**。物理 HW 由手动 HW 覆盖优先，否则使用已检测 `hw_version`。因此 HW4 + V13 仍使用 HW4 mux2 / byte1+byte7 布局；HW3 + V14 仍使用 HW3 mux0 / byte4+byte6 布局。这一交叉关系来自用户提供源码中 `hw_target_mode / detected_hw_version` 的分支，而不是公开 DBC 结论。

其他 mux0/mux1 位修改和修改后两次 mock 输出按附件保留。V14 需 D/R；速度模式写入需 D 且研究接管状态有效。V13 mux0 基础位、mux1 保持附件行为。

自动驾驶模式映射 fd 0..7 → 1,3,2,1,0,4,1,1；网页锁定选项 1..5 → 0..4。V13 编码只有两位，profile4 写入为 0，按源码保留并测试。协议自动识别单独维护：默认 V14，主总线出现有效长度 0x399 锁定 V14；此前累计50个完整0x3FD则锁定 V13。它只选择协议语义，不改变物理 HW 布局；0x3FD 的速度/profile 字段位置始终由 `hw_override`（如有）或 `hw_version` 决定。

研究接管状态来自 0x118 byte2 高3位 D 档，以及 0x389 的 FF02 / FF03 / FF42 / FF43 心跳，超时 3000 ms。0x2B9 仅保留已有只读遥测；不推断触屏操作，也不控制偏移。MARK TAP UI、代码及废弃事件状态已删除。

## 有意修正的源码边界行为

- 未收到心跳时不把开机前3秒当作接管状态。
- 无效限速、退出D档或接管失效清除偏移和历史平滑状态，避免残留目标。
- 时间0使用独立初始化标志；无符号差值支持时钟回绕。
- 验证 DLC、标准帧 ID 和回放输入；raw×5 使用宽类型，拒绝溢出伪限速。
- 保留本仓库 OTA / Autopark / AP-first 等模拟计算门控；它们可能比附件更早抑制 mock 输出。它们均无法开启物理TX。

## 构建与双总线

在 `esp32` 目录运行 `pio run -e waveshare-s3-can -e bench -e research`。

| 环境 | 数据路径 |
|---|---|
| waveshare-s3-can | Waveshare ESP32-S3-RS485-CAN；单TWAI只监听，GPIO15/16，8MB；保留中文网页和温度 |
| bench | 同类S3固件、两条虚拟总线；不初始化CAN硬件，串口回放 |
| research | LilyGO T-2CAN，TWAI can0 + MCP2515 can1；均只监听，16MB |

Waveshare 单CAN硬件不会因软件设置变成双CAN。双路回放使用 bench，双控制器只监听固件使用 research。主总线负责附件的速度/协议状态；can1不污染can0状态，原有总线来源标记、遥测和mock路由保留。不实现两条物理总线之间的转发。

`research_policy.h` 编译期拒绝 `RESEARCH_PHYSICAL_TX=1`。TWAI固定 Listen-Only、TX队列长度0；MCP2515仅有 Listen-Only 模式调用。两个驱动的 `send()` 无条件返回false，物理发送实现已移除。模拟 sink 只更新独立计数并输出 `[MOCK]`，不伪装成物理TX记录。

产物位于 `esp32/.pio/build/<环境>/firmware.bin` 与 `firmware-merged.bin`。旧 Flipper FAP 路径未作为研究固件发布；本次安全构建和验证范围是上述 ESP32 环境及无驱动的主机工具。

## 回放

在 bench 的中文网页选择研究协议、启用 FSD 帧计算与模拟计算，然后通过USB串口发送 `examples/research.replay` 中的 replay 行：

```
replay 1000 can0 118#0000800000000000
replay 1000 can0 389#FF02000000000000
replay 1000 can0 399#000C000000000000
replay 1000 can0 3FD#0200000000000000
```

格式：`replay <毫秒时间戳> can0|can1 <标准HEX-ID>#<0到8字节HEX>`。时间必须前进或正常回绕；`reset` 清除研究上下文并允许从头回放，不改持久化设置。为了确定性回放请使用没有物理输入的 bench 环境。can1可以接收回放并进入已有通用处理路径，但不驱动主总线速度模型。

纯主机工具完全不链接CAN驱动：

```
make -C test check
test/research_replay 14 < examples/research.replay
test/research_replay 13 < examples/research.replay
node tools/check_dashboard.mjs
python tools/verify_research_build.py --nm /path/to/xtensa-esp32s3-elf-nm
```

Windows可用 `zig cc` / `zig c++` 编译同样的测试，或运行 `tools/test_research.ps1 -Zig <zig.exe路径>`。主机工具只输出 `MOCK` 行；0x3FD会有两行相同输出，对应源码的重复发送计算。

旧NVS速度参数保留供旧版本兼容，但不参与新的研究模型；中文面板仅呈现新模型的有效参数。芯片温度、中文布局、静态JSON状态快照和断线重连机制保留。

## 本次验证记录（2026-09-29）

- PlatformIO：waveshare-s3-can、bench、research 三个环境全部构建成功，均生成独立及合并固件。
- 主机测试：共享核心589项、ESP32核心106项、研究模型4285项断言全部通过。
- V13 / V14 合成回放通过；允许物理TX的编译配置按预期编译失败。
- 三个环境各13个应用目标文件均没有硬件发送或Normal模式调用引用；bench无TWAI引用。
- 中文网页脚本语法、唯一DOM ID、状态渲染和研究控件检查通过。
- 上述为软件构建和mock验证，不等同于真实硬件电气隔离认证或道路安全验证。
