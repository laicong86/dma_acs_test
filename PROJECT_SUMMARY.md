# 工程速览（新会话请先读这一页）

> 详细文档见同目录 `README.md`（含寄存器表、失败时序图、故障排查）；本文件只做"是什么/为什么/现在到哪了"的交接。

## 1. 这是什么

`dma_acs_test` 是一个 **Linux 用户态 C 单文件程序**，用 PCIe ACS Exerciser 的 DMA 控制器，在**被测设备 BAR** 与 **DMA 控制器内部内存**之间做数据搬运和闭环校验。

- 源码：`dma_acs_test/dma_acs_test.c`（单文件，约 3000 行）
- 构建：`dma_acs_test/Makefile`（`gcc -O2 -Wall -Wextra -std=gnu11`，无第三方依赖）
- 运行环境：Linux（目标是在 **emu / FPGA 仿真平台**上跑，需要 root 访问 `/sys/bus/pci/devices/*/resource*` 并 mmap BAR）

## 2. 解决什么问题

ACS Exerciser 的内部内存 CPU 看不到，所以单向 DMA 无法自证。程序固定走 **读-改-写** 闭环：

```
bdfdst BAR 的 init 区（CPU 按 pattern 填充）
        │  (1) DMA DIR=0 读
        ▼
bdfdma bar0 的 ACS Exerciser 内部内存
        │  (2) DMA DIR=1 写
        ▼
bdfdst BAR 的 test 区（CPU 回读，与 pattern 逐字节比对）
```

任何一步不一致 → 判定链路/设备出错，写错误码 + 打印，默认立即退出（非 0 退出码）。

`--chunk` 分块时，一轮拆成三段而不是每块各跑一遍闭环：**① CPU 整片准备**（填 init、污染 test、回读确认落盘）→ **② DMA 按块连续搬完整片** → **③ CPU 整片回读比对**。这样 CPU 访问不会夹在 DMA 事务中间，波形上是连续的一串。DMA 阶段仍保持"读一块、写一块"交替，因为内部内存起始偏移由 `--mem_off` 固定（默认 0），后一笔读会覆盖前一笔。

## 3. 关键设计（改代码前必读）

1. **目的 BAR 自管理布局（省空间版）**：bar 内维护"超级头 + 单向链表（占用链）"，自动分配每个实例的 init/test 区，记录谁占用、跑到第几轮。超级头/节点头都只有 `0x100`（超级头有效 0x18、节点头有效 0x80），数据区只按 **8 字节对齐**（不再页对齐）。**最多 72 个源头**，满了报 `E_FULL(14)`。不做回收，重置靠 `-c/--clear`（只清到链尾，不扫整条 bar）；`-L/--list` 看当前占用（显示 `N/72`）。头版本 `HDR_VERSION=4`，旧 bar 会要求先 `--clear`。
2. **`--dst_off` 定位，不设窗口长度**：被测设备只有 bar 内某一段可测时（例如 bar0 里划出的窗口），整段布局平移过去；`BASE` 之前一个字节都不碰，`BASE` 之后一直到 bar 末尾都可分配。**不做 `--dst_len` 这类限制**：可用空间只由 sysfs 的 bar 大小决定，多实例天然同一口径；之前那版让每个进程各报窗口尾，给得小的会把窗口外别人的节点判成链断、再分配一次，两个源头撞同一片地址，所以整个删掉了（头里的 `H_LIMIT` 也一并去掉）。
3. **错误标志在测试头里（trigger 抓点）**：超级头 `+0x08` 是 `H_ERRFLAG`，出错瞬间第一笔写就是它——`dst bar + --dst_off + 0x08` 一次 4 字节对齐的 32 位 store（trigger 就设这个地址，数据 != 0）。每个源头的节点里另有 `N_ERRFLAG` 只标记自己。**bar 上只有这一个错误字，没有第二处副本**；迭代号/地址/期望实际值/寄存器快照等全部写 `struct err_local`，并 `mmap` 到 `/tmp/dma_acs_err_<bdf>_bar<N>_<owner>.rec`（`--err_log` 可改），进程崩了也能 `--dump_err` 读回。`--no_alloc`（无测试头）没有打点位置，失败时 bar 上不写东西。
4. **失败时序三段式**（emu 抓波形的核心诉求）：`err_trip()` 只有 **1 条** 裸 store 把错误码写到头里 `err_flag`（地址在 `plan_layout` 里预计算成 `ctx.trip`，运行时只剩一次解引用写）→ `err_record_detail()` 只写本地 DDR（普通内存，无 PCIe 事务）→ 最后才 `report_fail()` printf。中间**不允许**任何打印/分配/屏障干扰。

    **头访问铁律**：字段全部 4 字节对齐、只做 32 位整字访问；64 位字段拆低/高两笔（不生成 8 字节事务）；`owner`/`src_bdf` 按 4 字节打包；`--clear` 清零也改成 32 位循环。总线上不会出现 8/16/8 位或非对齐的头访问。
5. **多源头并发**：多个 DMA 控制器（或多进程）可同时打同一块 bar，靠占用链隔离空间、靠 `--owner` 在日志里区分（PASS/FAIL 行格式 `名称(源bdf)`），错误记录文件也按 owner 分开。
6. **`--cpu` 模式**：不用 DMA 控制器，CPU 直读直写同一块 bar，用来构造"DMA 与 CPU 同时访问一个设备"的场景（真同址并发需两边都加 `--no_alloc`）。
7. **`--stress rd|wr|both`**：纯压力模式，只搬数据不校验，数据只初始化一次，用来压链路带宽。
8. **地址 pattern** `--addr_pat none|seq|rand`（+ `--addr_step`）：压力模式每轮从不同起点开始、绕一圈覆盖整区，解决"size 小 → 整轮一笔 → 一直打同一个地址"的问题。默认 `seq`（自动步长：区 >4K 按 4K，否则 8B）。
9. **emu 时序补偿**：`--init_delay_ms`（CPU 写完 → DMA 读之前）、`--check_delay_ms`（DMA 报完成 → CPU 回读之前）。另有初始化区**回读校验**（不符则重试，仍不符报 `E_INIT_CHECK=13`），用于区分"写丢了"和"写还没落盘"。
10. **事后取证**：`--dump_err` 读本地记录；`--peek auto` 重新 mmap 出错地址并与记录的期望/实际值对照；`--pause_on_err` 失败后 `SIGSTOP` 冻结进程保现场。
11. **数据模式** `--pat rand|inc|alt`：rand=splitmix64 随机流；inc=每轮整体 +1；alt=0x5A../0xA5.. 逐轮翻转（后两者不依赖随机种子，便于对着波形算期望值）。
12. **`--no_init` 跳过 CPU 初始化**：不填 init 区、不污染 test 区，只跑 DMA 读→DMA 写。用于"数据已由对端（另一张卡 / 别的进程）写好，本轮只验搬运"的场景。比对基准改为**起 DMA 前 CPU 读到的 init 实测内容**（整片 `gold` 缓冲），所以仍能判搬运是否无损；`--cpu` 模式下基准就是 `bounce`。与 `--stress` 互斥。
13. **`--selftest` 可选打真 bar**：不带 `-d` 时是老的纯内存自检；带 `-d` 时由 CPU 直接读写指定 bar 的指定段（`--st_bar` / `--st_off` / `-l`），方向由 `--st_op rw|rd|wr` 决定。`rd` 模式连 mmap 都是只读的，硬件侧不可能被改动。用来在跑闭环前先确认"那片 init 区里到底是什么"。

## 4. 错误码

| 码 | 名称 | 含义 |
|---|---|---|
| 1 | E_USAGE | 参数/对齐错误 |
| 2 | E_SYSFS | sysfs 读不到（权限？） |
| 3 | E_NO_BAR | BAR 不存在/未使能 |
| 4 | E_MMAP | mmap BAR 失败 |
| 5 | E_LAYOUT | 空间放不下 |
| 6 | E_HW_BUSY | 启动前 CTRL bit0 仍为 1 |
| 7 | E_HW_TIMEOUT | 轮询完成超时 |
| 8 | E_HW_STATUS | Status 寄存器非 0 |
| 9 | E_MISMATCH | 数据比对不一致 |
| 10 | E_NOMEM | pattern 缓冲分配失败 |
| 11 | E_INTR | 被信号中断 |
| 12 | E_LOCKED | bdf/bar 已被其他实例占用 |
| 13 | E_INIT_CHECK | 初始化区回读校验失败（写没落盘） |
| 14 | E_FULL | 占用链已满（最多 72 个源头） |

## 5. 常用命令

```bash
cd dma_acs_test && make

# 标准闭环：bdfdma 的 bar0 做 DMA，打 bdfdst 的 bar4，1 MiB × 无限轮
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 -l 1M -n 0

# 设备只在 bar0 的 0x100000 之后可测（off 之后到 bar 末尾都可分配）
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 --dst_bar 0 --dst_off 0x100000 -l 1M -n 0

# CPU 源头，与上面那个 DMA 进程并发打同一个 bar
sudo ./dma_acs_test -d 02:00.0 --dst_bar 0 --dst_off 0x100000 -l 1M -n 0 --cpu --owner cpu0

# 只读压力 + emu 时序补偿（默认 seq 地址 pattern，每轮换起点）
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 -l 1M -n 0 --stress rd --init_delay_ms 2 --check_delay_ms 5

# 小 size 压地址：每轮起点挪 64 B；随机地址用 --addr_pat rand
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 -l 4K -n 0 --stress both --addr_step 64
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 -l 256K -n 0 --stress wr --addr_pat rand

# 查看占用 / 清空 / 解析上次错误 / 事后取证
sudo ./dma_acs_test -d 02:00.0 -L
sudo ./dma_acs_test -d 02:00.0 -c
sudo ./dma_acs_test -d 02:00.0 --owner dma0 --dump_err     # 读本地记录文件
sudo ./dma_acs_test -d 02:00.0 --owner dma0 --peek auto    # 回读 bar 对照

# 跳过 CPU 初始化：数据由外部预置，只验"DMA 读 -> DMA 写"这段搬运
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 -l 1M -n 0 --no_init --check_delay_ms 5

# CPU 直接读写某个 bar 的某一段（不经过 DMA），先看清那片空间里是什么
sudo ./dma_acs_test -d 02:00.0 --selftest --st_bar 0 --st_off 0x1000 -l 4K --st_op rd
sudo ./dma_acs_test -d 02:00.0 --selftest --st_bar 0 --st_off 0x1000 -l 4K --st_op rw
```

## 6. 当前状态与待确认项

代码已实现并做过静态检查（括号/函数/格式串/宏定义顺序），**尚未在实机上验证**。以下几项等实机结果再迭代：

- [ ] 编译是否零告警（`-Wall -Wextra`）
- [ ] 之前出现的 `trap invalid opcode ... in libc.so.6`（SIGILL）是否消失（见 README 9.1）
- [ ] `--dst_off` 与占用链、`--clear`、`--list` 联动是否正确
- [ ] `--stress` 三方向 + 地址 pattern 实际行为与带宽读数
- [ ] 72 个源头的容量/上限提示是否符合预期
- [ ] emu 上 DMA 是否接受 8 字节对齐（非 4 KiB 页对齐）的源/目的地址
- [ ] `E_INIT_CHECK(13)` 在 emu 上是否被触发、频次如何
- [ ] 失败时序打点是否满足 trigger 抓波形需求（头里 err_flag 一条 32 位写；用 `--list` 打印的地址设 trigger）
- [ ] emu 上头的读写是否确实只出现 4 字节对齐的 32 位事务（无 8 字节、无字节/半字）
- [ ] `--clear` 只清到链尾（不扫整条 bar），清空后 `--list` 应为空链、`free_off` 回到 `off+0x100`
- [ ] `--no_init`：`gold` 基准是否与 DMA 实际读到的内容一致（若 emu 上 P2P 落盘延迟大，
      需调 `--init_delay_ms`；预期 mismatch 是"CPU 读到了、DMA 还没读到"）
- [ ] `--selftest --st_op rd|rw|wr`：指定 bar/off/size 的读写是否符合预期；
      `rd` 应一个字节都不写（映射只读）；`rw` 的"写反码必须读不回原值"这条能否判出硬连线区间

## 7. 约定

- 代码/注释/日志均为中文，风格保持现有单文件结构（不拆分模块）。
- 失败路径严禁插入 printf / malloc / 重屏障；bar 上只允许写头里那 4 字节 `err_flag`，详情写本地 DDR。
- 头的字段一律 4 字节对齐 + 32 位整字访问；新增字段时不许引入 8/16 位或跨字访问。
- 语义相近的旧参数名不做兼容保留（例如 `--check_delay_us` 已彻底删除改为 `--check_delay_ms`），避免混用。
- 文档改动需同步三处：`usage()` 帮助文本、`README.md` 参数表/注意事项、本文件的状态段。
- **每次改完代码必须打包快照**：`./pack.sh`，产物落在上一级目录，名字带日期时间到秒
  （`dma_acs_test_YYYYmmdd-HHMMSS.tar`）。只含源码/文档/构建脚本，排除 `.git`、编译产物、
  `debug.log`、`*.rec`。打完自动清理更早的包：默认只留最新 1 份，`./pack.sh 3` 可保留 3 份。
