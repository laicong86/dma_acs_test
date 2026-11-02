# dma_acs_test — PCIe ACS Exerciser DMA 用户态环路测试工具

针对 "DMA Control Register" 定义实现的一个 Linux 用户态测试程序：用 DMA 在 **目标设备 BAR（bdfdst）** 与 **DMA 控制器内部 ACS Exerciser 内存（bdfdma bar0）** 之间做读、写、以及读-改-写闭环校验。

```
  CPU 可见                                   CPU 不可见
┌───────────────────────────┐         ┌──────────────────────────┐
│ bdfdst  bar(--dst_bar)    │         │ bdfdma bar0              │
│                           │         │                          │
│  --dst_off  测试空间起点   │◄──(1)───┤  ACS Exerciser 内存      │
│  +0x000     超级头(0x100)  │  DMA读   │  mem_offset = 0          │
│  +0x008     err_flag(4B)   │         │        ▲        │        │
│  +0x100     节点头(0x100)  │         │        └────────┘        │
│  +0x200     init 区       │         │  0x08 CTRL  b0=START    │
│            (CPU 写 pattern)│         │             b4=DIR       │
│  +…         test 区        ├──(2)───►│  0x0C..0x1C 地址/长度/状态│
│            (CPU 读回比对)  │  DMA写   │                          │
│  bar 末尾                  │         │                          │
└───────────────────────────┘         └──────────────────────────┘
   (1) DMA 读：init  -> ACS 内部内存      DIR = 0
   (2) DMA 写：ACS 内部内存 -> test       DIR = 1

   --dst_off 之前的字节程序不读写；之后的都算测试空间（到 bar 末尾）。
```

## 1. 寄存器映射（bdfdma 的 bar0，`--src_bar` 默认 0）

| 偏移   | 名称                   | 本程序用法                                                       |
| ---- | -------------------- | ----------------------------------------------------------- |
| 0x08 | DMA Control Register | `b0` 写 1 启动，完成后硬件自动清零；`b4`=0 读(dst→内部内存)，`b4`=1 写(内部内存→dst) |
| 0x0C | DMA Offset Register  | 恒写 0（ACS 内存偏移固定 0）                                          |
| 0x10 | DMA Bus Address Low  | 目标 BAR 起始地址（bus 地址）低 32 位                                   |
| 0x14 | DMA Bus Address High | 高 32 位                                                      |
| 0x18 | DMA Length Register  | 传输字节数                                                       |
| 0x1C | DMA Status Register  | 完成后读一次，非 0 视为出错（`--ignore_status` 可关闭）                      |

启动时序：`MEM_OFF → ADDR_LO → ADDR_HI → LEN → CTRL(=START|DIR)`，全程带 `__sync_synchronize()` 屏障；随后 **轮询 CTRL bit0 直到自动清零**，超时由 `--timeout_ms` 控制（默认 1000ms）。

## 2. 为什么必须是"读-改-写"

ACS Exerciser 的内存不在 CPU 地址空间里，CPU 既不能填充它，也不能直接读它校验。所以单方向的读/写都无法自证：

- 写测试必须先把已知数据"塞进"DMA 内部内存 —— 只能靠一次 DMA 读；
- 读测试读进来的数据必须写到某个 CPU 可见的地方 —— 只能靠一次 DMA 写。

因此程序的默认（也是唯一）测试流程就是闭环：

1. CPU 用本轮唯一 pattern 填充 **初始化区**；
2. CPU 把 **测试区** 污染成 pattern 的**按位取反**；
3. DMA 读：初始化区 → ACS 内部内存；
4. DMA 写：ACS 内部内存 → 测试区；
5. CPU 回读测试区，逐 8 字节比对；首个不符的偏移 / 期望值 / 实际值全部记录下来。

每一步的 pattern 由 `--pat` 选择：

- **rand（默认）**：种子是 `seed ^ f(iter) ^ f(offset)`，每轮每块都不同——上一轮残留数据不可能"碰巧"通过；测试区没被真正写入时，内容仍是反码，必然比对失败；读写通路任何一侧丢字节/错位都能定位到具体偏移；
- **inc**：不用随机种子，本轮第 k 个 qword = `iter + k`，整轮数据随轮次整体 +1，相邻轮内容不同，同样能检出陈旧数据，失败时人工比对期望值最直观；
- **alt**：不用随机种子，偶数轮全 bar 是 `0x5A5A5A5A5A5A5A5A`、奇数轮是 `0xA5A5A5A5A5A5A5A5`，逐轮翻转，适合配合波形看固定特征值。

底层两个接口（需求 3）是独立可复用的：

```c
int dma_read (struct ctx *c, uint64_t bus, uint64_t len);  /* dst -> 内部内存  (DIR=0) */
int dma_write(struct ctx *c, uint64_t bus, uint64_t len);  /* 内部内存 -> dst  (DIR=1) */
```

## 3. 目的 bar 布局：测试头 + 占用链

目的 bar 不再靠人工划分地址，程序在 bar 内部维护"测试头 + 单向链表"，记录**哪段空间被哪个源头占用**。
目的 bar 空间紧张，所以头都很小、数据区只按 8 字节对齐：

| 偏移                            | 大小     | 用途                                        |
| ----------------------------- | ------ | ----------------------------------------- |
| `BASE + 0x000`                | 0x100  | 超级头（字段只用到前 0x18）                          |
| `BASE + 0x008`                | 4 字节   | **`err_flag`：出错瞬间第一笔写，trigger 抓这里**（见 4.1） |
| `BASE + 0x100`                | 0x100  | 节点 #0 头                                   |
| `BASE + 0x200`                | 2 × -l | 节点 #0 数据：init 区 + test 区                   |
| …                             | 0x100  | 节点 #1 头 + 数据 …                            |

`BASE = --dst_off`，默认 0。被测设备如果只在 bar 内某一段（例如 bar0 里划出来的一块）才是可测空间，用 `--dst_off` 把整段平移过去；**`BASE` 之后一直到 bar 末尾**就是可分配空间，没有"窗口长度"这类参数：

```bash
# 从 bar0 的 0x100000 起测，能放开多少就放多少（到 bar 末尾为止）
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 --dst_bar 0 \
    --dst_off 0x100000 -l 1M -n 0
```

- `--dst_off` 必须 4 KiB 对齐。`BASE` 之前的字节（设备自己的寄存器/保留区）程序**一个都不碰**；`--clear` 也只清占用链上分配过的那一段，不扫整条 bar。
- 可用空间只由 sysfs 的 bar 大小决定，所有实例算出来必然一致——多打一时**只需要 `--dst_off` 一致**，不存在"谁报的窗口尾才算数"的问题。之前确实有过 `--dst_len`，但它是每个进程各报一个值，给得小的那个会把窗口外别人已经占上的节点判成"链断了"，然后在同一段空间上再分配一次，两个源头打同一片地址，所以整个删掉了。
- **最多 72 个源头**（`NODE_MAX_COUNT`）：链满后再申请直接报 `E_FULL(14)`，不会无限切碎 bar。

超级头字段（0x18 字节，其余都能从链上推出来，不再单独存）：

| 偏移   | 类型  | 含义                                     |
| ---- | --- | -------------------------------------- |
| 0x00 | u32 | magic `'DMAH'`，**最后写**                  |
| 0x04 | u32 | 版本（当前 **4**，旧头会要求先 `--clear`）           |
| 0x08 | u32 | **`err_flag`**：0 = 正常，非 0 = 错误码；出错时第一笔写就是它 |
| 0x0C | u32 | 首节点偏移，0 = 空链                           |
| 0x10 | u32 | 节点数（上限 72）                             |
| 0x14 | u32 | 序号计数器                                  |

**头访问铁律**：所有字段偏移 4 字节对齐，全部用 32 位整字读写；64 位字段拆成"低字 + 高字"两笔（不生成 8 字节事务），`owner`/`src_bdf` 字符串也按 4 字节一个字打包。所以总线上看到的头访问永远是 4 字节对齐的单笔事务，不会出现 8/16/8 位或跨字的非对齐访问。

节点头字段（0x100 以内，不再含详细错误记录）：

| 偏移          | 字段                  | 说明                                   |
| ----------- | ------------------- | ------------------------------------ |
| 0x00        | magic               | `'DMAN'`，**最后写入**，非此值即节点无效           |
| 0x04        | state               | 1 = 占用中                              |
| 0x08        | err_flag            | 本源头错误码，0 = 正常（多源头时区分是谁挂的）           |
| 0x0C        | seq                 | 申请序号                                 |
| 0x10        | self                | 自身在 bar 内的偏移（自校验用，u64 拆两字）           |
| 0x18        | next_off            | 下一节点，0 = 链尾                          |
| 0x20        | data_off            | 数据起点（= init 区 = `off + 0x100`）       |
| 0x28        | test_off            | test 区偏移（`data_off + ALIGN(-l, 8B)`） |
| 0x30        | len                 | init 区长度（= 用户的 `-l`）                 |
| 0x38        | span                | 整节点占用（含节点头）                          |
| 0x40        | err_iter            | 失败时的迭代号（u64 拆两字）                     |
| 0x48        | done                | 已完成轮数（供 `--list` 观察进度）               |
| 0x50        | pid                 | 申请者进程号                               |
| 0x54        | tail magic          | `'DNEN'`（u32），节点完整性哨兵                |
| 0x58        | owner               | 源头名称（`--owner`，默认取 bdfdma，24 字节，6 个字） |
| 0x70        | src_bdf             | 源 bdf 字符串（16 字节，4 个字）                |

申请流程：

1. 读超级头（`BASE + 0x0`）；magic 不对就自动初始化（等价 `--clear`，但不清数据）；
2. 从链头走一遍，找到链尾，空闲起点 = `链尾.off + 链尾.span`（8 字节对齐）；空链时起点 = `BASE + 0x100`；
3. 需要 `0x100 + 2 × ALIGN(-l, 8B)`；放得下就把新节点头写进 bar、接上链尾；
4. 之后只用自己节点里的 init / test 两块区域，互不干扰。

**不做回收**：节点申请后一直保持占用（state=ACTIVE），要让空间重新可用就 `--clear`。所以空间不足时的提示是"用 `--list` 看占用，或 `--clear` 清空重来"。

`--clear` 的清空范围是 `[BASE + 0x100, 链尾)`——只清占用链上真正分配过的数据区，不扫整条 bar。这样 `--dst_off` 之后即使还有设备自己的功能区，也不会被顺手清掉。

数据区上限就是 bar 末尾（从 `--dst_off` 起算），不设人为窗口，也不为错误码让出末尾一页：4 MiB 的 bar4 上，一个 `-l 1M` 的节点占 `0x100 + 2 MiB`，可以放两个。

## 4. 错误记录：bar 上只打点，详情进本地 DDR

目的 bar 空间有限，失败时**不再往 bar 里写一整条记录**：

| 位置                                | 内容                                        |
| --------------------------------- | ----------------------------------------- |
| `dst bar + --dst_off + 0x08`（超级头的 `err_flag`） | **错误码**。这是 trigger 抓的唯一写事务（出错瞬间的第一笔写） |
| 本地 DDR（`--err_log` 指定的文件，默认 `/tmp/dma_acs_err_<bdf>_bar<N>_<owner>.rec`） | 迭代号、目标地址、期望/实际值、种子、寄存器快照等全部详情 |

bar 上**只有这一个错误字**，没有第二处副本：少一处写就少一条总线事务，trigger 之后的波形也干净。

`err_flag` 是**粘滞**的：一次失败后一直是那个错误码；程序下次启动时清 0，`--clear` 也会归零。这样事后 `--list` 还能看出"这个 bar 上出过错"；每个源头自己节点里的 `N_ERRFLAG` 只标记自己，多源头共用一块 bar 时用它区分是谁挂的。

`--no_alloc` 模式没有测试头，**也就没有打点位置**：该模式失败时 bar 上不写任何东西，现场只能看本地记录文件（banner 和 FAIL 行都会如实说明）。

本地记录用 `mmap(MAP_SHARED)` 映射到文件，所以进程退出、崩溃甚至被 kill 之后，仍然可以用 `--dump_err` 原样读回来；mmap 失败时退化成进程内缓冲（这次能打印，进程退出后丢失）。

`struct err_local` 字段（进程内存，按 C 结构体访问，不再按寄存器式偏移）：

| 字段                   | 含义                     |
| -------------------- | ---------------------- |
| `magic`              | `'EMRL'`，**最后写**，有效标志   |
| `code`               | 错误码（与 bar 上那个 4 字节一致）   |
| `iter` / `total`     | 失败迭代号 / 此前已通过轮数        |
| `bus` / `bar_off`    | 失败的目标 bus 地址 / bar 内偏移 |
| `len` / `bad_off`    | 本次传输长度 / 块内首个不符偏移      |
| `expect` / `actual`  | 期望值 / 实际值              |
| `seed` / `sys_errno` | pattern 种子 / 关联 errno   |
| `pid` / `dst_bar`    | 进程号 / 目的 bar 号         |
| `dst_off` / `init_off` / `test_off` / `dst_phys` | 本次布局，便于离线复算 |
| `snap[6]` / `status` / `flags` | 寄存器快照、STATUS 值、两者是否有效 |
| `owner` / `src_bdf` / `dst_bdf` | 源头与设备标识            |

错误码：

| 码  | 名称           | 说明                         |
| -- | ------------ | -------------------------- |
| 1  | E_USAGE      | 参数/对齐错误                    |
| 2  | E_SYSFS      | sysfs 读不到或打开失败（权限？）        |
| 3  | E_NO_BAR     | BAR 不存在/未使能/大小为 0          |
| 4  | E_MMAP       | mmap BAR 失败                |
| 5  | E_LAYOUT     | 空间放不下                     |
| 6  | E_HW_BUSY    | 启动前 CTRL bit0 仍为 1（上一次未结束） |
| 7  | E_HW_TIMEOUT | 轮询 CTRL bit0 清零超时          |
| 8  | E_HW_STATUS  | Status 寄存器非 0              |
| 9  | E_MISMATCH   | 数据比对不一致                    |
| 10 | E_NOMEM      | pattern 缓冲分配失败             |
| 11 | E_INTR       | 被信号中断                      |
| 12 | E_LOCKED     | bdf/bar 已被其他实例占用           |
| 13 | E_INIT_CHECK | 初始化区回读校验失败（CPU 写没落盘）      |
| 14 | E_FULL       | 占用链已满（最多 72 个源头）           |

失败后程序默认立即停止（`--no_stop_on_err` 可继续跑），进程退出码非 0。

### 4.1 失败时序：先打点，后打印（emu 抓波形用）

在 emu / 带逻辑分析仪的环境里，最怕的是"故障发生了，但程序先 printf 了半天才写错误码"——波形上故障点和写入点之间夹着一堆 CPU 侧动作，trigger 抓到的时刻和真实故障时刻对不上。所以失败路径被拆成严格的三段：

```
DMA 完成 / 回读比对不符
        │
        ├─(1) err_trip()           ← 立刻写，**只有 1 条 32 位 store**
        │      dst bar + dst_off + 0x08 = 错误码       ← 唯一触发点（超级头 err_flag）
        │      4 字节对齐、32 位整字、无 fence、无读、无格式化、无 printf
        │
        ├─(2) err_record_detail()  ← 详情写本地 DDR（普通内存，不产生 PCIe 事务），
        │                             bar 上不再写第二个字，仍不打印
        │
        └─(3) report_fail()        ← 记录落盘后才打印，并 fflush
```

**trigger 条件**（推荐这么设，抓到的第一个事务就是故障点）：

| 项    | 值                                                     |
| ---- | ----------------------------------------------------- |
| 地址   | `dst bar base + --dst_off + 0x08`（超级头 `err_flag`）       |
| 方向   | 写                                                     |
| 数据   | `!= 0`（值就是错误码 1~14）                                    |
| 触发位置 | 起始（first），单次触发（normal）                                |

`--list` 的头部行会直接把 `err_flag` 和 trigger 地址打出来，换设备/换 `--dst_off` 时照着抄即可。

要注意的几点：

- **没有前置打印**：`dma_single()` 里原先"Status 报错"的 `fprintf` 已去掉，改成把 STATUS 值存进 `ctx`，等 `report_fail()` 统一打印；`--verbose` 的寄存器快照也是**先打点、后读**，不会出现在触发点之前。
- **只有一笔 4 字节写**：`err_trip()` 直接对算好的地址做一次 32 位 store（无内存屏障、无判断、无计算）。bar 上没有错误码副本、详情走本地 DDR，所以一次失败在 bar 上**只多这一笔事务**，触发点前后不会夹带别的错误相关写。
- **头的所有访问都是 4 字节对齐的 32 位整字**：64 位字段拆成低/高两笔，`owner`/`src_bdf` 按 4 字节打包，`--clear` 的清零也改成 32 位循环（原来是 8 字节 store）。emu 上不会出现 8 字节事务、半字/字节事务或跨字边界的非对齐访问。
- **本地记录 magic 最后写**：`err_record_detail` 先清 magic → 填字段 → 置 magic。只有在"打点完成、详情还没写完"的几微秒窗口里，读者可能看到新码配旧详情，属于正常现象。
- **比对是"首个不符即返回"**：`buf_compare()` 一旦发现不同立刻返回，不会把整块扫完再报，因此触发点紧贴故障数据。
- **默认失败即停**（`--no_stop_on_err` 除外），后续不再有 DMA 流量污染波形尾部。

事后取证：

```bash
sudo ./dma_acs_test -d 02:00.0 --owner dma0 --dump_err   # 读本地记录，打印全部字段
sudo ./dma_acs_test -d 02:00.0 --owner dma0 --peek auto  # 重新读 bar 上出错地址，与记录对照
```


## 5. 编译与使用

```bash
make                  # 生成 dma_acs_test
sudo ./dma_acs_test -s 0000:01:00.0 -d 0000:02:00.0 -l 1M -n 0 --dst_bar 4 --dbg_times 100
```

参数：

```
-s, --src_bdf <bdf>     DMA 控制器设备（bdfdma），用其 bar0（`--cpu` 时不需要）
-d, --dst_bdf <bdf>     被测设备（bdfdst）
-l, --size <size>       每个区（初始化区/测试区）的字节数；数据区只按 8 字节对齐，
                        不再有 4 KiB 页填充
-n, --times <n>         循环次数，0 = 无限（默认 1）
    --dbg_times <n>     每 n 次打印一次成功状态，0 = 只打印错误与汇总（默认 100）
    --dst_bar <n>       被测 bar 号（默认 4）
    --dst_off <off>     目的 bar 内测试空间的起始偏移（默认 0，必须 4 KiB 对齐）。
                        设备只在 bar 内某一段才可测时用它；测试头、占用链、
                        init/test 区整体从 off 开始排布，off 之前的字节不读写；
                        off 之后一直到 bar 末尾都可分配（没有长度参数）
    --src_bar <n>       DMA 控制寄存器 bar（默认 0）
    --err_log <path>    本地错误记录文件（默认 /tmp/dma_acs_err_<bdf>_bar<N>_<owner>.rec）。
                        mmap 到文件，进程退出后仍可用 --dump_err 读回
    --mem_off <off>     ACS 内存偏移（默认 0，按要求固定）
    --chunk <bytes>     单次 DMA 最大长度，0 = 整区一次传（大长度受硬件上限限制时用）。
                        分块后一轮拆成三段：① CPU 准备整片 ② DMA 按块连续搬完整片
                        ③ CPU 回读整片比对，CPU 访问不再插在 DMA 事务中间
    --timeout_ms <ms>   单次 DMA 超时（默认 1000）
    --check_delay_ms <ms>
                        DMA 报完成后、CPU 回读校验前的等待毫秒数（默认 0）。
                        emu 上 P2P 写可能比 CPU 慢：状态位已清零但写数据尚未
                        真正落到 BAR，马上回读会拿到旧值造成假性 mismatch。
    --init_delay_ms <ms>
                        CPU 写完初始化数据后、DMA 开始读之前的等待毫秒数（默认 0）。
                        同时是初始化区回读校验的重试间隔（未配置则 1ms）：
                        CPU 写完 init 区后先回读确认数据真正落盘，不符就等待
                        重试（至多 1000 次），仍不符报 E_INIT_CHECK。
    --pat <rand|inc|alt>
                        数据模式（默认 rand）:
                        rand = splitmix64 随机种子流（受 --seed 影响）；
                        inc  = 每轮整体 +1，本轮第 k 个 qword = iter + k；
                        alt  = 0x5A5A... 与 0xA5A5... 逐轮翻转。
                        inc/alt 不使用随机种子，失败时手工比对期望值更直观。
    --dst_phys <addr>   覆盖 sysfs 得来的目标地址（IOMMU/非 1:1 映射时用）
    --seed <n>          pattern 种子（默认 0x5AA5A55A，仅 rand 模式使用）
    --ignore_status     不检查 Status 寄存器
    --no_stop_on_err    出错后继续（默认首个错误即退出）
    --pause_on_err      失败打印后 SIGSTOP 进程（mmap 与 BAR 现场保持），
                        检查/抓波形完 kill -CONT <pid> 恢复
-v, --verbose           启动/结束时 dump 一次控制寄存器
    --probe             只打印两个设备的 BAR 资源表
    --dump_err          读回本地 DDR 的错误记录文件并解析（失败后单独用，不需要碰 bar）
    --peek <off|auto>   事后取证：重新 mmap 目的 bar，hex dump 指定偏移附近的内容后退出；
                        off 是 bar 内绝对偏移（含 --dst_off）；
                        auto 表示出错偏移从本地记录里取，并自动与期望/实际值对照
    --peek_len <n>      --peek 的回读长度（默认 64 字节）
    --selftest          只跑 CPU 侧数据通路自检（不碰硬件），并打印 CPU 特性

目的 bar 测试头 / 占用链：
-c, --clear             启动时清空目的 bar（测试头 + 链上分配过的数据区，到链尾为止），然后正常测试
    --clear_only        只清空后退出（不需要 -s / -l）
-L, --list              只打印目的 bar 的占用链后退出（不需要 -s / -l）
    --owner <name>      本实例在链上和日志里的名称（默认取 bdfdma）；
                        PASS/FAIL 行与结束汇总显示为 name(源bdf)
    --no_alloc          不用测试头分配，退回固定布局 init@dst_off / test@dst_off+ALIGN(size)

测试方式：
    --cpu               不用 DMA 控制器，CPU 直接读写目的 bar（不需要 -s），见第 7 节
    --stress <rd|wr|both>
                        纯压力模式：只按方向搬数据，不做任何校验，数据只在启动时
                        初始化一次，之后每轮不再改写（去掉 CPU 侧开销，压链路本身）。
                        rd   = 只读：init 区 -> DMA 内部内存
                        wr   = 只写：DMA 内部内存 -> test 区（内部内存先由一次 DMA 读填入）
                        both = 每轮先读一遍再写一遍
                        与 --cpu 合用可压 CPU 直读直写通路，见 5.1 节
    --addr_pat <none|seq|rand>
                        压力模式的地址 pattern（默认 seq）：每轮从哪个地址开始。
                        none = 固定从区首开始（老行为；size 小时一直打同一地址）
                        seq  = 每轮起点前进 --addr_step，到区尾回绕
                        rand = 起点由 seed/iter 伪随机决定（可复现）
                        每轮仍覆盖整个区，只是起点不同，见 5.1 节
    --addr_step <bytes> seq 的步长（默认 0 = 自动：区 > 4 KiB 按 4 KiB，否则按 8 B）
```




典型用法：

```bash
# 首次使用 / 想重置目的 bar：清空后再跑
sudo ./dma_acs_test -d 02:00.0 --clear_only --dst_bar 4
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 -l 1M -n 0 --dst_bar 4 --clear

# 看当前有哪些源头在占这个 bar
sudo ./dma_acs_test -d 02:00.0 --dst_bar 4 --list

# 被测设备只在 bar0 的 0x100000 之后可测（第 3 节）
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 --dst_bar 0 \
    --dst_off 0x100000 -l 1M -n 0 --list
```

`--list` 输出示例（带 `--dst_off`）：

```
目的 bar 0000:02:00.0 bar0  (phys=0x00000000fcc00000, size=0x4000000)
  测试空间     : bar+0x100000 起（--dst_off），数据区 0x100100 ~ 0x400000
  测试头: ver=4 节点数=2/72 err_flag=0x00000000
  错误标志位: bar+0x100008（trigger 地址，--clear 归零）
  序号  节点偏移   数据偏移   init长度  状态    源头                      pid      完成轮数  错误
  ----  ---------  ---------  --------  ------  ------------------------  -------  ---------  ----
     1  0x100100   0x100200    1048576  占用    0000:01:00.0              12345       10000  -0
     2  0x240200   0x240300     262144  占用    0000:03:00.0              12346        2500  -0
  共 2 个节点（上限 72）；空闲起点 0x300300，剩余 0xfed00 字节
```

其它常用组合：

```bash
# 硬件单次传输有上限时（例：单次最多 64 KiB）
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 -l 1M -n 100 --chunk 64K

# 每笔只搬 8 B、地址递增：-l 定区大小，--chunk 8 定单笔长度
# 一轮分三段：CPU 先填满整片 init（并污染 test）-> DMA 连续 512 读 + 512 写
# -> CPU 再整片回读比对。DMA 阶段中间没有 CPU 访问，波形上是连续一串
# 一个实例占 0x100(超级头) + 0x100(节点头) + 2×size，数据落在 bar+off+0x200
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 --dst_bar 0 \
    --dst_off 0x200000 -l 4K --chunk 8 -n 1

# 设备在这一段只有 4 KiB 可用：单区取 1K（0x200 + 2×0x400 = 0xA00 放得下），
# 128 个 8 B 地址（128 读 + 128 写 = 256 笔），跑 8 轮 = 2048 笔。
# --clear 也只清这 0xA00，不会碰后面
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 --dst_bar 0 \
    --dst_off 0x200000 -l 1K --chunk 8 -n 8

# DMA 侧看到的是 IOVA 而不是 CPU 物理地址
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 -l 64K -n 10 --dst_phys 0x12340000

# 上次测试失败过？单独读回本地 DDR 里的错误记录（不需要碰 bar）
sudo ./dma_acs_test -d 02:00.0 --owner dma0 --dump_err

# 失败退出后事后取证：从本地记录取出错地址，回读 bar 现场并与期望值对照（见 9.4 节）
sudo ./dma_acs_test -d 02:00.0 --owner dma0 --peek auto

# 也可以手动指定偏移和长度
sudo ./dma_acs_test -d 02:00.0 --peek 0x1FF800 --peek_len 128

# 不用 DMA 控制器，纯 CPU 打这个 bar（构造 DMA + CPU 并发，见第 7 节）
sudo ./dma_acs_test -d 02:00.0 --dst_bar 4 -l 256K -n 0 --cpu --owner cpu-A

# 纯压力：只读 / 只写 / 读+写，不做校验，数据只初始化一次（见 5.1 节）
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 --dst_bar 4 -l 1M -n 0 --stress rd
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 --dst_bar 4 -l 1M -n 0 --stress wr --dbg_times 1000
```

输出示例：

```
[       1] PASS iter=0 owner=0000:01:00.0            size=1 MiB     1.482 ms   1349.83 MiB/s  avg   1349.83 MiB/s  (pass=1 fail=0)
[     101] PASS iter=100 owner=0000:01:00.0          size=1 MiB     1.377 ms   1452.90 MiB/s  avg   1443.21 MiB/s  (pass=101 fail=0)
...
==================== 测试结束 ====================
  激励源头    : 0000:01:00.0
  迭代总数    : 10000
  通过 / 失败 : 10000 / 0
  单区大小    : 1 MiB (init@0x100200, test@0x200200, block=1048576)
  运行时间    : 13.921 s
  平均带宽    : 1436.62 MiB/s (读+写，累计 20971520000 字节)
=================================================
```

> PASS / FAIL 行都带 `owner=`，格式是 `--owner 名称(源 bdf)`；没给 `--owner` 时只显示源 bdf，`--cpu` 实例显示 `cpu`。多个实例往同一个终端/日志里打时，靠这个字段区分"谁发的激励"。

> 带宽是"整轮"吞吐（读 + 写 + CPU 初始化/污染/比对），不是单纯的 DMA 峰值。

### 5.1 纯压力模式（`--stress`）

闭环测试每轮都要 CPU 填 init 区、污染 test 区、再回读比对，CPU 侧开销和 DMA 本身相当，压不出链路的极限。`--stress` 把 CPU 侧完全摘掉：

- **数据只初始化一次**：启动时把 init / test 区各填一遍（内容仍由 `--pat` / `--seed` 决定），之后每轮都不再改写；
- **不做任何校验**：每轮只按方向把整块数据搬一遍，所以不会出现 `E_MISMATCH`；
- 失败只可能来自 DMA 硬件/总线（`E_HW_TIMEOUT` / `E_HW_STATUS`），仍然是"先打点、后补详情、最后打印"（见 4.1 节）。

| 参数 | 每轮动作 | 带宽口径 |
|---|---|---|
| `--stress rd` | DMA 读：init 区 → ACS 内部内存 | `-l` × 1 |
| `--stress wr` | DMA 写：ACS 内部内存 → test 区 | `-l` × 1 |
| `--stress both` | 先读一遍，再写一遍 | `-l` × 2 |

写方向要求 ACS 内部内存里先有数据：准备阶段会多做一次 DMA 读，把 init 区搬进内部内存（偏移固定 0），之后每轮反复写出去，**内部内存内容全程不变**，所以波形上看到的写数据是同一份固定值。

#### 地址 pattern（`--addr_pat` / `--addr_step`）

固定起点时，每轮打的是同一批地址——`-l` 小（比如 4 KiB）时整轮就一笔传输，反复打同一个地址，地址线/通道根本翻不动。`--addr_pat` 让**每轮的起始地址**变起来：

| 值 | 起始地址 | 说明 |
|---|---|---|
| `seq`（默认） | `(iter × step) % size`，8 字节对齐 | 每轮前移，到区尾回绕；`--addr_step` 默认 0 = 自动（区 > 4 KiB 按 4 KiB，否则按 8 B） |
| `rand` | 由 `seed`/`iter` 经 splitmix64 决定 | 可复现，适合随机地址压力 |
| `none` | 恒为 0 | 老行为，只在需要固定地址复现时用 |

每轮仍然**覆盖整个区**（从起点顺序搬到区尾，再回绕到区首搬完剩余部分），所以带宽口径不变，只是起始地址不同；`size` 只有 4 KiB 而 `--chunk` 也是 4 KiB 时，起点会在 0/8/16/… 之间轮转，不会再一直打同一笔。

```bash
# 1 MiB 区，每轮起点 +4 KiB（默认 seq 自动步长），到区尾回绕
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 -l 1M -n 0 --stress rd

# 小 size + 细步长：每轮起点只挪 64 B，把 qword/地址线压得更碎
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 -l 4K -n 0 --stress both --addr_step 64

# 随机地址（同样的 seed 可复现）
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 -l 256K -n 0 --stress wr --addr_pat rand --seed 0x1234
```

```bash
# 只读压力（最常用来量链路读带宽上限）
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 --dst_bar 4 -l 1M -n 0 --stress rd

# 只写压力，每 1000 轮打一次带宽
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 --dst_bar 4 -l 1M -n 0 --stress wr --dbg_times 1000

# 读+写混合，再叠一个 CPU 源头造并发（见第 7 节）
sudo ./dma_acs_test -d 02:00.0 --dst_bar 4 -l 256K -n 0 --cpu --stress wr --owner cpu-A
```

启动时会多打印三行，用来确认模式、数据只初始化了一次、地址怎么变：

```
  测试模式     : 纯压力 只读（不校验，数据只初始化一次）
  数据模式     : 随机 rand（splitmix64 种子流）  —— 只用于启动时初始化
  地址模式     : 顺序（每轮起点前移，到区尾回绕）
                 步长 4 KiB（自动）
压力数据已初始化（仅此一次）: init@0x200 / test@0x200200 各 1048576 字节
```

注意：

- 空间仍按"init + test"两份申请（占用链节点结构不变）。只读模式其实用不到 test 区，但为了地址布局与闭环测试一致、方便和并发实例对照，不做裁剪。
- `--check_delay_ms` 在压力模式下无意义（没有回读）；`--init_delay_ms` 只在启动时"初始化回读重试"里用到。
- `--chunk` 照常生效，分块时每块单独一次 DMA，口径与闭环测试一致。
- 压力模式不会写 `init_cur` 现场（没有 mismatch），FAIL 行只给错误码、寄存器快照和本地记录文件路径（详情用 `--dump_err` 看）。
- 地址 pattern 只改"每轮从哪开始"，不改总字节数；`none` 可复现老行为。`--addr_step` 过大会在区里跳着打（每轮仍覆盖整区，只是起点稀疏）。
- 汇总行的带宽口径会跟着方向变：`只读` / `只写` 按 `-l` × 1 计，`读+写`（含闭环）按 × 2 计。

## 6. 多任务并行（多个 DMA 控制器同时跑）

**支持**，而且现在多个源头可以共用**同一个**目的 bar——各自的空间由 bar 内的占用链自动分配，不用人工划分地址。

```bash
# 三个不同的 DMA 控制器，全部打同一个目的 bar4
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 --dst_bar 4 -l 512K -n 0 --dbg_times 1000 > run_a.log 2>&1 &
sudo ./dma_acs_test -s 03:00.0 -d 02:00.0 --dst_bar 4 -l 512K -n 0 --dbg_times 1000 > run_b.log 2>&1 &
sudo ./dma_acs_test -s 05:00.0 -d 02:00.0 --dst_bar 4 -l 512K -n 0 --owner "ch5-ch0" > run_c.log 2>&1 &

# 随时看谁占了哪段
sudo ./dma_acs_test -d 02:00.0 --dst_bar 4 --list

# 收尾：统一停掉，再清空
pkill -INT dma_acs_test
sudo ./dma_acs_test -d 02:00.0 --dst_bar 4 --clear_only
```

并行时的隔离与保护：

| 资源 | 冲突处理 | 机制 |
|------|---------|------|
| DMA 控制寄存器（bdfdma bar） | bdfdma 相同 → **直接拒绝** | 对 `resource<N>` 加独占 `flock`，第二个实例启动即报 `E_LOCKED`，杜绝两个进程交叉编程同一组寄存器 |
| 目的数据 BAR | 允许共享 | 每个实例启动时走一遍占用链，把节点接在链尾之后的空闲空间；分配临界区用 `/tmp` 锁文件串行化，避免并发启动抢同一段 |
| 错误记录 | 每源头一份 | trigger 打在一块 bar 共用的 `err_flag`（`dst bar + dst_off + 0x08`），每源头在自己节点里有 `N_ERRFLAG`；详情写本地 DDR 的 `<bdf>_bar<N>_<owner>.rec`，用 `--dump_err --owner <名>` 查看 |
| CPU 侧映射 | 互不影响 | 各自独立 mmap |
| CPU 模式实例（`--cpu`） | 可与任意多实例并存 | 不打开 bdfdma，不占控制器锁；同样通过占用链拿自己的空间 |

规则：**一个 DMA 控制器只能给一个实例**，目的 bar 可以随便共用。

多打一时每个进程需要自己声明的量只有 `--dst_off`（必须一致）；可用空间长度不用谁报，一律算到 bar 末尾（sysfs 的 bar 大小），所有实例天然同一口径，不会出现"把别人窗口外的节点当成链断了、然后在这段上再分配一次"的撞车。

多个实例的输出混在同一个终端/日志里时，看每行的 `owner=` 就知道是哪个源头发的激励（`--owner` 名称 + 源 bdf，见第 5 节输出示例）。

注意 `--clear` 是全局复位，会把所有源头的节点一起清掉，**只能在确认没有其他实例在跑时使用**（程序不做存活检测，也不回收节点）。

带宽注意：多个实例共享同一条 PCIe 链路，单实例打满链路时再加实例不会增加总吞吐，只会分摊。

## 7. CPU 模式（`--cpu`）：构造 DMA 与 CPU 并发访问

加了 `--cpu` 就不碰 DMA 控制器，由 CPU 直接 mmap 读写目的 bar，跑**同样结构**的读-改-写闭环。用途是构造"一边 DMA 在打、一边 CPU 在读写在打同一个设备"的场景，观察 PCIe 链路争用、设备内部资源冲突、以及一致性问题。

CPU 模式不需要 `-s`：

```bash
# 终端 1：DMA 源头，正常跑
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 --dst_bar 4 -l 512K -n 0 --owner dma-A

# 终端 2：CPU 源头，打同一个设备的同一个 bar
sudo ./dma_acs_test -d 02:00.0 --dst_bar 4 -l 256K -n 0 --dbg_times 1000 --cpu --owner cpu-A

# 看两个源头各自占了哪段
sudo ./dma_acs_test -d 02:00.0 --dst_bar 4 --list
```

闭环对照（`bounce` 是本进程的一块堆内存，顶替 DMA 模式里 CPU 看不见的 ACS 内部内存）：

| 步骤 | DMA 模式 | CPU 模式 |
|---|---|---|
| 1 | CPU 把 init 区填成 pattern | 同左 |
| 2 | CPU 把 test 区污染成 `~pattern` | 同左 |
| 3 | DMA 读：init → ACS 内部内存 | CPU 读：init → `bounce` |
| 4 | — | `bounce` 与 pattern 比对（读被并发流量改坏会立刻暴露） |
| 5 | DMA 写：ACS 内部内存 → test | CPU 写：`bounce` → test |
| 6 | CPU 回读 test 区比对 | 同左 |

两种用法：

- **默认（走占用链）**：两个源头各自拿一段独立空间，互不重叠。测的是"同一设备、同一 bar、不同地址"的并发流量。
- **两边都加 `--no_alloc`**：都落到固定布局 `init@dst_off / test@dst_off+ALIGN(size)`，即**完全同址**。这时双方的 pattern 会互相踩，必然快速 mismatch —— 这正是"看谁先改坏"的用法，配合 `--no_stop_on_err` 与波形触发点观察。

注意：

- CPU 模式的带宽就是本进程对 BAR 的读写吞吐，同样受 `--chunk` 控制，口径与 DMA 模式一致（读 + 写 = `2 × -l`）。
- 并发场景下 mismatch 是**预期结果**，不是 bug；用 `--no_stop_on_err` 可以持续跑并统计失败率。
- CPU 模式同样会写错误标志、同样有触发点（见 4.1）；只有叠加 `--no_alloc` 时没有头、没有触发点。`--list` 里 `源 bdf` 会显示 `cpu`。

## 8. 注意事项

1. **需要 root**（或对 `/sys/bus/pci/devices/<bdf>/resource<N>` 有读写权限）。BAR 通过 `O_SYNC` 方式 mmap，保证非 cache 语义、寄存器写不被重排。
2. **地址语义**：默认拿 sysfs 里 BAR 的起始地址当 DMA bus 地址（无 IOMMU、1:1 映射时成立）。若存在 IOMMU / 地址转换，请用 `--dst_phys` 显式指定设备侧可见地址。
3. **状态寄存器**：程序把 Status 非 0 判为失败。若你的 Status 里含"有效/忙"等非错误位，加 `--ignore_status`，仅靠数据比对判定。
4. **`-l 2M` 与 4 MiB bar4**：见第 3 节，单个节点需要 `0x100 + 4 MiB`，略超 bar 容量，需减小 `-l` 或换更大的 bar。
5. **多实例互斥**：同一个 bdfdma 被两个进程同时打开时，第二个实例会因 `flock` 失败直接退出（`E_LOCKED`）；目的 bar 允许多个源头共享，空间由占用链分配。
6. **`--clear` 是全局动作**：会清掉该 bar 上所有源头的节点，清空范围只到链尾（`[BASE+0x100, 链尾)`），不扫整条 bar。程序不做存活检测/节点回收（按需求），所以只应在没有其他实例运行时使用。
7. **`--no_alloc` 与测试头不共存**：固定布局从 `dst_off` 开始，会覆盖超级头。只在老脚本兼容时使用；`--cpu` 想和 DMA 实例**同址**并发时，两边都加 `--no_alloc`（见第 7 节）。
8. **`--dst_off` 是"整段平移"**：超级头、占用链、init/test、`--clear` 的清空范围全部跟着走，`dst_off` 之前的字节一律不碰。同一块 bar 上多实例**必须用相同的 `--dst_off`**（否则各管各的段）。`--dst_off` 必须 4 KiB 对齐，`dst_off` 之后一直到 bar 末尾都是可分配空间（没有长度参数）。
9. **头只做 4 字节对齐的 32 位访问**：字段偏移全部 4 字节对齐；64 位字段拆两笔 32 位写；`owner`/`src_bdf` 按 4 字节打包；`--clear` 清零也走 32 位循环。emu 上不会看到 8 字节事务、字节/半字事务或跨字边界的非对齐访问——用 trigger 抓波形时地址是稳定可预期的。
10. **`err_flag` 是粘滞的**：它在超级头里、由同一块 bar 上所有源头共用，一次失败后保持那个错误码，直到下次启动或 `--clear`。它同时是 trigger 抓点（`dst bar + dst_off + 0x08`）；要看"是谁挂的"看各节点自己的 `N_ERRFLAG`（`--list` 里有）。
11. **CPU 缓存一致性**：在 x86 + `O_SYNC` 映射下无需额外维护；若在某些平台（如部分 ARM）出现"必然 mismatch"，请确认该 BAR 映射是否为 Device 属性，必要时在比对前做一次 `dc civac` 类的 cache 维护。
12. **中断/退出**：`Ctrl-C` 会跑完当前这一轮后退出，并打印统计与累计结果，不会打断正在进行的 DMA。多实例时用 `pkill -INT dma_acs_test` 给所有实例发 SIGINT。
13. 测试区数据是随机 pattern，且每轮不同；比对失败时 `bar 偏移 / 期望 / 实际` 三个值足够定位是数据错、错位还是根本没写。
14. **`--stress` 不校验数据**：它只量吞吐、只报总线/控制器错误。压力模式跑出 `PASS` 不代表数据没被改坏——要判数据正确性请用默认闭环模式（或跑完再用 `--peek` 抽查）。

## 9. 故障排查

### 9.1 `trap invalid opcode ... in libc.so.6`（SIGILL）

现象：

```
[ 3204.369590] traps: dma_acs_test[1755637] trap invalid opcode ip:7f06f92164b sp:... error:0 in libc.so.6[...]
```

这是 **CPU 非法指令**（不是段错误，不是我们的逻辑错），发生点在 libc 内部，绝大多数情况是系统 glibc 的向量化 `memcpy/memset` 用到了当前 vCPU 不支持的指令：

- 系统按 **x86-64-v3** 基线构建（AVX2 起步；RHEL/CentOS 10、新版 Ubuntu/SUSE 等），
- 而 VMware/Hyper-V 的 vCPU 型号/掩码没有暴露 `avx2`（老硬件、老硬件版本、或迁移到低代 CPU 的主机后）。

**第一步，确认崩溃点：**

```bash
# 取 ip 相对 libc 的偏移（用 dmesg 里的 ip 减 libc 的映射起始地址，或直接：）
addr2line -f -e /usr/lib/x86_64-linux-gnu/libc.so.6 0x14c64b
objdump -d --start-address=0x14c630 --stop-address=0x14c670 \
        /usr/lib/x86_64-linux-gnu/libc.so.6

# 看 CPU 到底有没有 avx2
grep -m1 '^flags' /proc/cpuinfo | tr ' ' '\n' | grep -E '^avx|^sse4_2|^erms'
```

**第二步，判断是环境还是程序**（最快的判别方法）：

```bash
./dma_acs_test --selftest          # 只跑 CPU 侧数据通路，不碰硬件
```


```

- `--selftest` 就崩 → 纯环境问题（连 malloc/memcpy 都用不了），与本程序无关；
- `--selftest` PASS 但实机测试崩 → 崩溃点在硬件访问路径，用上面的 objdump 看具体指令。

**第三步，处理：**

| 措施 | 说明 |
|------|------|
| 改虚拟机 CPU 型号 | VMware：关机 → 编辑设置 → CPU → 勾选"虚拟化 Intel VT-x/EPT"并把 CPU 型号设为 Host 或更新的代次；确认硬件版本足够新。这是根治办法 |
| 换系统基线 | 用 x86-64-v2 基线（如 CentOS 9 / Ubuntu 22.04）的镜像，老 CPU 也能跑 |
| 降低 glibc 的 hwcaps | `GLIBC_TUNABLES=glibc.cpu.hwcaps=-AVX2,-AVX512F ./dma_acs_test ...`，仅当 glibc 仍保留标量实现时有效 |
| 本程序的规避 | 数据搬运已改为自带的 8 字节标量循环，**不再调用 libc 的 memcpy/memset**，所以只要 libc 的 printf/malloc 本身能用，本程序就能跑 |

> 本程序从 v3 起（就是这次修复）：`pattern_fill / copy_words / buf_compare` 全部用 8 字节 load/store 实现，既规避上述问题，也让设备 BAR 的访问严格保持 8 字节粒度、行为可预测。

### 9.2 `-l 128` 却显示传输 4 KiB

v2 及更早版本的 bug：内部把 `-l` 先按 4 KiB 对齐算成"区跨度"，然后用这个跨度做 DMA 长度。v3 已修正——**`-l` 就是实际 DMA 字节数**；v4 起对齐粒度进一步降到 8 字节，只用于决定 test 区起始偏移（避免与初始化区重叠）。现在 `-l 128` 会打印 `单次 DMA: 128 B`。

### 9.3 带宽列一直是 0

已修复。原因有两个，都会让 `MiB/s` 显示成 `0.0`：

1. 输出用的是 `%9.1f MiB/s`，只保留一位小数。emu 上 4 KiB 一轮要几十到几百毫秒，真实带宽只有零点零几 MiB/s，四舍五入就变成 `0.0`；
2. 单次耗时取两次 `clock_gettime` 的差值，分辨率不够时 `t1 == t0`，带宽直接被算成 0。

现在的输出改成：

```
[    1000] PASS iter=999 owner=dma-A(0000:01:00.0)   size=4K    182.451 ms     43.87 KiB/s  avg     44.02 KiB/s  (pass=1000 fail=0)
[   10000] PASS iter=9999 owner=dma-A(0000:01:00.0)  size=1M      1.204 ms   1664.05 MiB/s  avg   1650.11 MiB/s  (pass=10000 fail=0)
```

- 低于 1 MiB/s 自动切到 `KiB/s`，并保留两位小数；
- 每行多一个 `avg`，是按"累计字节 / 累计时间"算的滑动平均，单次计时被时钟分辨率吃掉时用它兜底；
- 结束处的 `平均带宽` 用同一套格式化，并额外打印累计字节数。

### 9.4 假性 mismatch（emu 上 DMA 报完成但回读到旧值）

现象：emu 上跑，每轮（或固定几轮后）在**回读比对**处报 `E_MISMATCH`，期望值是本轮 pattern、实际值是上一轮 pattern 或污染反码；波形上 DMA 写事务其实已经发完。

原因：emu 上 P2P 事务走 fabric 可能比 CPU 慢——DMA 引擎已经置完成（CTRL bit0 自清零、Status 正常），但写数据真正落到被测 BAR 还差几十微秒，CPU 马上回读就读到旧值。

**变体症状（inc 模式实测）**：期望值是本轮的 `iter`，但**初始化区和测试区事后读出来都是上一轮的值**（例如期望 3、两区都是 2）。这说明问题不只发生在"DMA 写→CPU 回读"一侧：CPU 把 3 写进 init 区之后，**DMA 的读比 CPU 的 MMIO 写先到**——DMA 读到旧值 2、原样写进测试区，CPU 拿 pattern(3) 比对必然失败。

**"失败后手动读 init 区还是 2，写丢了吗？"** 三种可能，用下面两个手段区分：

1. **看失败瞬间的 init 区值**（本版本会在 FAIL 行里直接打印）：
   - `init 区当前值 == 期望` → CPU 写当时已经落盘，丢的是 DMA 读路径的可见性（读侧拿到旧值）；
   - `init 区当前值 != 期望` → 失败那一刻 CPU 的写确实还没落到 BAR——配合回读校验重试 1000 次仍不符，基本可以认定**写卡死/丢失在 emu fabric 里**，需要抓波形看这笔 MemWr 有没有到达 exerciser、exerciser 有没有处理。
2. **`--pause_on_err` 保住现场**：失败后进程 SIGSTOP、mmap 不解除，BAR 状态静止，从容抓波形/手动读，避免"事后读"与"失败时"之间被其它流量或时间差污染。
3. **排除读错位置**：占用链分配的 init/test 偏移取决于当次运行链上空闲位置。确认手动读的偏移与 FAIL 行打印的 `bar 偏移`（减去 test 区到 init 区的差 `ALIGN(-l,8B)`）一致；对不上就是位置漂移，不是写丢失。

另外注意一个对照点：bar 上的错误标志和本地记录（`--dump_err` 能读到）都是 CPU 写的。若错误码每次都在、而 init 区的 pattern 写时有时无，说明 CPU→BAR 写通路本身没问题，丢失与时序/冲突相关——emu 的 posted write 队列或内部存储读写端口冲突是重点怀疑对象（真实硬件不会出现这种现象）。

处理：CPU 写完 init 区后**强制回读校验**（本版本已默认开启）——回读不符就按 `--init_delay_ms`（未配置则 1ms）等待重试，至多 1000 次，仍不符报 `E_INIT_CHECK`。回读一致证明 CPU 视角已落盘，再配合 `--init_delay_ms` 盲等给 DMA 路径留可见性余量、`--check_delay_ms` 给写落盘留余量：

```bash
sudo ./dma_acs_test -s 01:00.0 -d 02:00.0 --dst_bar 4 -l 1M -n 0 --check_delay_ms 1 --init_delay_ms 1
```

一轮按三段跑：**① CPU 准备整片 → ② DMA 搬完整片 → ③ CPU 回读校验整片**，所以两个延迟都只在阶段边界插入一次（`--chunk` 切块也不再每块各等一次）：`--init_delay_ms` 在阶段 1 回读确认之后、第一笔 DMA 之前；`--check_delay_ms` 在整片 DMA 写完成之后、阶段 3 比对之前。两者都设为 0 则不加额外等待。

**事后取证确认**：程序失败退出后，BAR 内容不会消失，重新 mmap 读出来即可确认是否"延迟落盘"：

```bash
sudo ./dma_acs_test -d 02:00.0 --peek auto
```

`auto` 会从本地记录里取出错的 bar 偏移与期望/实际值，hex dump 现场并给出结论（记录文件按 owner 区分，所以要带 `--owner`）：

- `当前值 == 期望值` → 写数据最终落盘了，失败时读到旧值就是延迟所致（假性 mismatch），调大 `--check_delay_ms` 即可；
- `当前值 == 当时实际值` → 数据至今没变，不是单纯延迟，需要继续查（地址错、ACS 丢弃写等）；
- 其它值 → 该地址可能被其它流量改写过，注意此结论仅在没有其它实例还在跑时可信。

也可以手动指定偏移与长度：`--peek 0x1FF800 --peek_len 128`。

### 9.5 其它常见错误

| 输出 | 原因 |
|------|------|
| `E_LOCKED(bdf/bar已被其他实例占用)` | 同一 bdfdma 或同一 BAR 已有一个实例在跑，见第 6 节 |
| `E_NO_BAR(BAR不存在/未使能)` | BAR 未使能，先 `lspci -vv -s <bdf>` 或 `--probe` 看资源表 |
| `E_SYSFS(sysfs访问失败)` | 非 root，或该 resource 被其他驱动占用 |
| `E_HW_TIMEOUT` | CTRL bit0 一直不清零：DMA 地址错误（如 IOMMU 场景地址不对）、目标无人响应、或长度超过引擎上限（用 `--chunk`） |
| `E_HW_STATUS(状态寄存器报错)` | 引擎报错；若 Status 含非错误位，用 `--ignore_status` |
| `E_MISMATCH(数据校验不一致)` | 测试区回读与 pattern 不符；emu 上多为写数据尚未落到 BAR，见 9.4 |
| `E_INIT_CHECK(初始化区回读校验失败)` | CPU 写完 init 区后回读重试 1000 次仍不符：CPU 的 MMIO 写迟迟没有落到 BAR，emu 时序问题，见 9.4 |
