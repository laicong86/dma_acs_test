/*
 * dma_acs_test.c -- PCIe ACS Exerciser DMA 用户态环路测试工具
 *
 * 设计依据（DMA 控制寄存器，bdfdma 的 bar0 基址）：
 *   0x08  DMA Control Register        b0: START(写1启动DMA, 完成后硬件自动清零)
 *                                     b4: DIR(0=读: dst -> DMA内部内存, 1=写: DMA内部内存 -> dst)
 *                                       (其余 bit 为 PASID/地址类型等属性，本程序只用 b0/b4)
 *   0x0C  DMA Offset Register         ACS Exerciser 内存中用于 DMA 数据的偏移量（本程序固定 0）
 *   0x10  DMA Bus Address Low         目标系统地址低 32 位
 *   0x14  DMA Bus Address High        目标系统地址高 32 位
 *   0x18  DMA Length Register         传输长度（字节）
 *   0x1C  DMA Status Register         状态寄存器
 *
 * 为什么需要"读-改-写"闭环：
 *   DMA 侧的 ACS Exerciser 内存（bar0 内的数据缓冲）在 CPU 地址空间不可见，
 *   因此无法用 CPU 直接填充/读取它。唯一可控的数据通道是目标设备的 BAR：
 *     (1) CPU 把已知 pattern 写进 dst 的“初始化区”
 *     (2) DMA 读：把初始化区搬进 DMA 内部内存
 *     (3) DMA 写：把 DMA 内部内存搬到 dst 的“测试区”
 *     (4) CPU 回读测试区，与 pattern 逐字节比对
 *   一次闭环同时验证了读通路与写通路，且数据不可伪造（test 区在 DMA 写之前被污染成反码）。
 *
 * dst bar 布局（BASE = --dst_off，默认 0；--size 指每个区的字节数）：
 *   BASE + 0x000      超级头（0x100，只用到前 0x18）
 *                     +0x08 = err_flag ← 出错时第一笔写，trigger 抓这里
 *   BASE + 0x100      节点#0 头（0x100），紧随其后是该节点的初始化区 + 测试区
 *   ...               多个源头按链继续往后排（最多 72 个，数据区 8 字节对齐）
 *
 * BASE 之前的字节（被测设备自己的寄存器/保留区）程序一个都不碰。
 *
 * 头里的所有字段都 4 字节对齐、只做 32 位整字访问（64 位字段拆两个字写，
 * 字符串按字打包），总线上不会出现非 4 字节或非对齐的头访问。
 *
 * 校验失败时第一件事是把错误码写进头里的 err_flag（一次 32 位 store，
 * 无 fence/读/打印）；迭代号、目标地址、期望/实际值、寄存器快照等详情写到
 * 本地 DDR 的记录文件，不占 bar 空间，也不在 trigger 之前产生总线事务。
 *
 * CPU 模式（--cpu）：不碰 DMA 控制器，CPU 直接 mmap 读写 dst bar 跑同样的闭环，
 * 用来和另一个跑 DMA 的进程一起构造“DMA 与 CPU 同时访问同一个设备”的场景。
 *
 * 编译：make            用法：./dma_acs_test -s 0000:01:00.0 -d 0000:02:00.0 -l 1M -n 0 --dst_bar 4
 *                       CPU 模式：./dma_acs_test -d 0000:02:00.0 -l 256K -n 0 --dst_bar 4 --cpu
 *                       只测 bar0 内某一段：./dma_acs_test -s 0000:01:00.0 -d 0000:02:00.0 \
 *                                            -l 1M --dst_bar 0 --dst_off 0x100000
 * 需要 root（或对该 resource 有读写权限）。
 */

#define _GNU_SOURCE

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

/* ============================== 寄存器定义 ============================== */

#define DMA_REG_CTRL        0x08u   /* DMA Control Register   */
#define DMA_REG_MEM_OFF     0x0Cu   /* DMA Offset Register    */
#define DMA_REG_ADDR_LO     0x10u   /* DMA Bus Addr Low       */
#define DMA_REG_ADDR_HI     0x14u   /* DMA Bus Addr High      */
#define DMA_REG_LEN         0x18u   /* DMA Length Register    */
#define DMA_REG_STATUS      0x1Cu   /* DMA Status Register    */

#define DMA_CTRL_START      (1u << 0)   /* 写 1 启动，完成后硬件自动清零 */
#define DMA_CTRL_DIR_WRITE  (1u << 4)   /* 0 = 读(dst->mem)，1 = 写(mem->dst) */

/* ACS Exerciser 内存偏移固定为 0（需求 3） */
#define DMA_MEM_OFFSET      0u

/* 数据区/地址的对齐粒度：qword 访问，只要求 8 字节对齐 */
#define DATA_ALIGN          0x8ull

/* ============================== 错误打点 ==============================
 *
 * 打点只有一个位置：**目的 bar 超级头里的 err_flag**（H_ERRFLAG）。
 *   dst bar + --dst_off + 0x08  ← 出错瞬间的唯一一笔写，trigger 设这里
 *
 * 不在别的 bar 上再留"错误码副本"——多一处写就多一条总线事务，既费 bar 空间，
 * 又让 trigger 之后出现无谓的流量。迭代号、目标地址、期望值/实际值、寄存器
 * 快照等详细信息全部写到**本地 DDR**（见 struct err_local），普通内存写不产生
 * 任何 PCIe 事务，波形最干净。
 */

/* ============================== 错误码 ============================== */

enum err_code {
    E_OK = 0,
    E_USAGE = 1,        /* 参数错误               */
    E_SYSFS = 2,        /* 读取/打开 sysfs 失败   */
    E_NO_BAR = 3,        /* BAR 不存在/未使能      */
    E_MMAP = 4,        /* mmap 失败              */
    E_LAYOUT = 5,        /* 空间布局放不下         */
    E_HW_BUSY = 6,        /* 启动前 START 位仍为 1  */
    E_HW_TIMEOUT = 7,    /* 等待完成超时           */
    E_HW_STATUS = 8,    /* 状态寄存器报错         */
    E_MISMATCH = 9,    /* 数据比对不一致         */
    E_NOMEM = 10,       /* 内存分配失败           */
    E_INTR = 11,       /* 被信号中断             */
    E_LOCKED = 12,      /* 该 bdf/bar 已被其他实例占用 */
    E_INIT_CHECK = 13,  /* 初始化区回读校验失败（CPU 写久落不到 BAR） */
    E_FULL = 14,       /* 占用链已满（最多 72 个源头） */
};

static const char *err_name(int code)
{
    switch (code) {
    case E_OK:          return "OK";
    case E_USAGE:       return "E_USAGE(参数错误)";
    case E_SYSFS:       return "E_SYSFS(sysfs访问失败)";
    case E_NO_BAR:      return "E_NO_BAR(BAR不存在/未使能)";
    case E_MMAP:        return "E_MMAP(mmap失败)";
    case E_LAYOUT:      return "E_LAYOUT(bar空间布局不满足)";
    case E_HW_BUSY:     return "E_HW_BUSY(启动前START位未清零)";
    case E_HW_TIMEOUT:  return "E_HW_TIMEOUT(等待DMA完成超时)";
    case E_HW_STATUS:   return "E_HW_STATUS(状态寄存器报错)";
    case E_MISMATCH:    return "E_MISMATCH(数据校验不一致)";
    case E_NOMEM:       return "E_NOMEM(内存分配失败)";
    case E_INTR:        return "E_INTR(被信号中断)";
    case E_LOCKED:      return "E_LOCKED(bdf/bar已被其他实例占用)";
    case E_INIT_CHECK:  return "E_INIT_CHECK(初始化区回读校验失败)";
    case E_FULL:        return "E_FULL(占用链已满，最多72个源头)";
    default:            return "E_UNKNOWN";
    }
}

/* ============================== 数据结构 ============================== */

/*
 * 本地 DDR 里的一条错误记录（不占目的 bar 的空间）。
 *
 * 启动时把它 mmap 到一个文件（默认 /tmp/dma_acs_err_<bdf>_bar<N>_<owner>.rec），
 * 所以进程退出/崩溃后还能用 --dump_err 原样读回来；mmap 失败时退化成进程内
 * 缓冲，只保证本次运行能打印。
 */
#define ERL_MAGIC   0x4C524D45u     /* 'EMRL'，最后写入，充当 valid 标志 */
#define ERL_SIZE    0x1000ull       /* 一条记录固定占一页（预留扩展） */

struct err_local {
    uint32_t magic;         /* 0x00 valid 标志，最后写 */
    uint32_t code;          /* 0x04 错误码 */
    uint64_t iter;          /* 0x08 失败迭代号 */
    uint64_t total;         /* 0x10 之前已通过轮数 */
    uint64_t bus;           /* 0x18 失败的目标 bus 地址 */
    uint64_t bar_off;       /* 0x20 该地址在 bar 内的偏移 */
    uint64_t len;           /* 0x28 本次传输长度 */
    uint64_t bad_off;       /* 0x30 块内首个不符偏移 */
    uint64_t expect;        /* 0x38 期望值 */
    uint64_t actual;        /* 0x40 实际值 */
    uint64_t ts;            /* 0x48 单调时钟 ns */
    uint32_t seed;          /* 0x50 本轮 pattern 种子 */
    uint32_t sys_errno;     /* 0x54 关联 errno */
    uint32_t pid;           /* 0x58 进程号 */
    uint32_t dst_bar;       /* 0x5C 目的 bar 号 */
    uint64_t dst_off;       /* 0x60 --dst_off */
    uint64_t init_off;      /* 0x68 init 区 bar 内偏移 */
    uint64_t test_off;      /* 0x70 test 区 bar 内偏移 */
    uint64_t dst_phys;      /* 0x78 目的 bar 物理地址 */
    uint32_t snap[6];       /* 0x80 CTRL/OFF/LO/HI/LEN/STATUS */
    uint32_t status;        /* 0x98 E_HW_STATUS 的 STATUS 值 */
    uint32_t flags;         /* 0x9C bit0=snap 有效 bit1=status 有效 */
    char     owner[64];     /* 0xA0 源头名称 */
    char     src_bdf[32];   /* 0xE0 源 bdf */
    char     dst_bdf[32];   /* 0x100 目的 bdf */
};

#define ERL_F_SNAP    (1u << 0)
#define ERL_F_STATUS  (1u << 1)

struct cfg {
    char     src_bdf[64];   /* -s  bdfdma  */
    char     dst_bdf[64];   /* -d  bdfdst  */
    int      src_bar;       /* --src_bar  默认 0（DMA 控制寄存器） */
    int      dst_bar;       /* --dst_bar  被测 bar */
    uint64_t dst_off;       /* --dst_off  目的 bar 内测试空间的起始偏移（4 KiB 对齐） */
    char     err_log[256];  /* --err_log 本地错误记录文件路径，空 = 按 bdf/bar/owner 自动生成 */
    uint64_t mem_off;       /* --mem_off  ACS 内存偏移，默认 0 */
    uint64_t size;          /* -l  每个区(init/test)的字节数 */
    uint64_t times;          /* -n  循环次数，0 = 无限 */
    uint64_t dbg_times;     /* --dbg_times 每 N 次打印一次成功状态，0 = 不打印 */
    uint64_t chunk;         /* --chunk 单次 DMA 最大长度，0 = 整块一次传 */
    uint64_t seed;          /* --seed   pattern 种子 */
    uint64_t dst_phys;      /* --dst_phys 覆盖从 sysfs 读到的 bus 地址，0 = 用 sysfs */
    uint32_t timeout_ms;    /* --timeout_ms 单次 DMA 超时 */
    uint32_t check_delay_ms; /* --check_delay_ms DMA 完成后、回读校验前的延迟 */
    uint32_t init_delay_ms;  /* --init_delay_ms  CPU 初始化数据后、DMA 读前的延迟 */
    int      pat_mode;       /* --pat 0=rand(默认) 1=每轮+1 2=5A5A/A5A5 翻转 */
    int      stop_on_err;   /* 失败即退出（默认 1） */
    int      pause_on_err;  /* --pause_on_err 失败打印后 SIGSTOP，保住现场供检查 */
    int      check_status;  /* 检查 Status 寄存器非 0 视为失败（默认 1） */
    int      verbose;       /* 启动时 dump 寄存器 */
    int      probe;         /* 只打印 BAR 信息后退出 */
    int      dump_err;      /* 只读取并解析本地错误记录后退出 */
    int      peek;          /* --peek 退出后回读目的 bar 指定偏移（事后取证） */
    int      peek_auto;     /* --peek auto：出错偏移从错误记录里取 */
    uint64_t peek_off;      /* --peek 指定的 bar 偏移 */
    uint64_t peek_len;      /* --peek_len 回读长度，默认 64 字节 */
    int      selftest;      /* --selftest CPU 侧自检；带 -d 时改为直接读写 bar */
    int      no_init;       /* --no_init 跳过 CPU 填/污染 init 区，拿 bar 实测值当基准 */
    int      st_op;         /* --st_op 自检方向：0=读写 1=只读 2=只写 */
    int      st_bar;        /* --st_bar 自检用的 bar 号，-1 = 跟随 --dst_bar */
    uint64_t st_off;        /* --st_off 自检区间在 bar 内的起始偏移 */
    int      clear;         /* 启动时清空目的 bar（测试头 + 链表 + 数据区） */
    int      clear_only;    /* 只清空后退出 */
    int      list;          /* 只打印占用链后退出 */
    int      no_alloc;      /* 关闭测试头分配，用固定 init/test 布局 */
    int      cpu_mode;      /* --cpu 不用 DMA 控制器，CPU 直接读写目的 bar */
    int      stress;        /* --stress 纯压力方向：0=关 1=只读 2=只写 3=读+写 */
    int      addr_pat;      /* --addr_pat 压力模式地址 pattern：0=固定 1=顺序 2=随机 */
    uint64_t addr_step;     /* --addr_step 顺序模式的地址步长，0 = 自动 */
    char     owner[64];     /* 源头名称，默认取 bdfdma */
};

struct bar {
    char      bdf[64];
    int       num;
    uint64_t  phys;     /* 资源起始地址（作为 DMA bus address 使用） */
    uint64_t  size;
    uint8_t  *va;
    int       fd;       /* 保持打开以持有 flock（多实例互斥） */
    int       owner;    /* 1 = 本结构负责 munmap/close */
};

struct ctx {
    struct cfg cfg;
    struct bar src;      /* bdfdma  bar(src_bar)：DMA 控制寄存器 */
    struct bar dst;      /* bdfdst  bar(dst_bar)：测试数据区 + 错误打点 */
    uint64_t   region;    /* 单区对齐跨度（固定布局时使用）  */
    uint64_t   init_off;  /* 初始化区在 dst bar 内偏移  */
    uint64_t   test_off;  /* 测试区在 dst bar 内偏移    */
    uint64_t   node_off;  /* 本实例的节点头偏移 */
    int        has_node;  /* 是否已成功申请到节点 */

    /* 失败打点地址：指向超级头的 err_flag（--no_alloc 没有头，为 NULL）。
     * 只在这里存指针，err_trip 里就是一条 4 字节对齐的 32 位 store，
     * 不判断、不计算。 */
    volatile uint32_t *trip;
    uint64_t   block;     /* 单次 DMA 实际长度          */
    uint8_t   *pat;       /* pattern 缓冲（同时作为黄金比对源） */
    uint8_t   *gold;      /* --no_init 用：init 区实测内容快照（整片 size 大小） */
    uint8_t   *bounce;    /* --cpu 模式的"中间内存"，模拟 ACS 内部内存 */

    /* 失败现场：抓在打点之后，供事后打印；打点之前一律不碰 */
    uint32_t   snap[6];         /* CTRL/MEM_OFF/ADDR_LO/ADDR_HI/LEN/STATUS */
    int        snap_valid;
    uint32_t   last_status;     /* E_HW_STATUS 时的 STATUS 值 */
    int        last_status_valid;
    uint64_t   init_cur;        /* mismatch 时 init 区同位置 qword（打点后抓） */
    int        init_cur_valid;

    /* 本地 DDR 错误记录：el 指向映射文件，映射失败时退回 el_buf */
    struct err_local *el;
    struct err_local  el_buf;
    void      *el_map;          /* mmap 出来的页，NULL = 用的是 el_buf */
    char       el_path[256];    /* 实际使用的记录文件路径 */
};

struct fail_info {
    int       code;
    uint64_t  bus;
    uint64_t  bar_off;
    uint64_t  len;
    uint64_t  bad_off;
    uint64_t  expect;
    uint64_t  actual;
    uint32_t  seed;
    int       sys_errno;
};

/* ============================== 工具函数 ============================== */

static volatile sig_atomic_t g_stop;

static void on_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void sleep_us(long us)
{
    struct timespec ts;
    ts.tv_sec = us / 1000000;
    ts.tv_nsec = (us % 1000000) * 1000L;
    nanosleep(&ts, NULL);
}

/*
 * 毫秒级延迟：直接 nanosleep。emu 上毫秒级等待不需要自旋补精度，
 * nanosleep 只会多睡不会少睡，满足"等够再动"的语义。
 */
static void delay_ms(uint64_t ms)
{
    if (ms == 0)
        return;
    sleep_us((long)(ms * 1000ull));
}

static uint64_t align_up(uint64_t v, uint64_t a)
{
    return (v + a - 1) & ~(a - 1);
}

static uint64_t splitmix64(uint64_t *s)
{
    uint64_t z = (*s += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static void fmt_size(char *buf, size_t n, uint64_t v)
{
    if (v >= (1ull << 30) && (v % (1ull << 30)) == 0)
        snprintf(buf, n, "%" PRIu64 " GiB", v >> 30);
    else if (v >= (1ull << 20) && (v % (1ull << 20)) == 0)
        snprintf(buf, n, "%" PRIu64 " MiB", v >> 20);
    else if (v >= (1ull << 10) && (v % (1ull << 10)) == 0)
        snprintf(buf, n, "%" PRIu64 " KiB", v >> 10);
    else
        snprintf(buf, n, "%" PRIu64 " B", v);
}

/*
 * 带宽格式化。
 *
 * 之前的写法是 printf("%9.1f MiB/s")，在 emu 上会"永远是 0"：
 *   - 4 KiB 传输跑几十毫秒时，真实带宽只有零点几 MiB/s，一位小数直接四舍五入成 0.0；
 *   - 单次耗时用 clock_gettime 差值得出，分辨率不够时 t1 == t0，带宽被算成 0。
 * 这里改成：小于 1 MiB/s 自动降到 KiB/s 显示，并且保留两位小数。
 */
static void fmt_rate(char *buf, size_t n, double mibps)
{
    if (!(mibps > 0.0)) {
        snprintf(buf, n, "--");
        return;
    }
    if (mibps >= 1.0)
        snprintf(buf, n, "%.2f MiB/s", mibps);
    else
        snprintf(buf, n, "%.2f KiB/s", mibps * 1024.0);
}

/* 支持 1024 / 0x1000 / 4K / 2M / 1G 写法 */
static int parse_size(const char *s, uint64_t *out)
{
    char *end = NULL;
    uint64_t v;

    if (!s || !*s)
        return -1;

    errno = 0;
    v = strtoull(s, &end, 0);
    if (errno != 0 || end == s)
        return -1;

    while (*end == ' ' || *end == '\t')
        end++;

    switch (toupper((unsigned char)*end)) {
    case '\0':  break;
    case 'B':   end++; break;
    case 'K':   v <<= 10; end++; if (toupper((unsigned char)*end) == 'B') end++; break;
    case 'M':   v <<= 20; end++; if (toupper((unsigned char)*end) == 'B') end++; break;
    case 'G':   v <<= 30; end++; if (toupper((unsigned char)*end) == 'B') end++; break;
    default:    return -1;
    }

    if (*end != '\0')
        return -1;

    *out = v;
    return 0;
}

/* ============================== sysfs / BAR 访问 ============================== */

/* 允许 01:00.0 / 0000:01:00.0 两种写法，统一成小写 4 段形式 */
static void normalize_bdf(const char *in, char *out, size_t n)
{
    int         colons = 0;
    const char *p;
    char       *q;

    for (p = in; *p; p++) {
        if (*p == ':')
            colons++;
    }

    if (colons == 1)
        snprintf(out, n, "0000:%s", in);
    else
        snprintf(out, n, "%s", in);

    for (q = out; *q; q++)
        *q = (char)tolower((unsigned char)*q);
}

/* 从 /sys/bus/pci/devices/<bdf>/resource 解析出第 bar 段的起始地址与长度 */
static int pci_bar_info(const char *bdf, int bar, uint64_t *start, uint64_t *size)
{
    char      path[PATH_MAX];
    char      line[256];
    FILE     *fp;
    int       idx = 0;
    int       ret = -1;

    snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/resource", bdf);
    fp = fopen(path, "r");
    if (!fp)
        return -1;

    while (fgets(line, sizeof(line), fp)) {
        uint64_t s = 0, e = 0, fl = 0;

        if (sscanf(line, "%" SCNx64 " %" SCNx64 " %" SCNx64, &s, &e, &fl) != 3)
            continue;

        if (idx == bar) {
            *start = s;
            *size = (s == 0 && e == 0) ? 0 : (e - s + 1);
            ret = 0;
            break;
        }
        idx++;
    }

    fclose(fp);
    return ret;
}

static void print_bar_table(const char *bdf)
{
    char  path[PATH_MAX];
    char  line[256];
    FILE *fp;
    int   idx = 0;

    snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/resource", bdf);
    fp = fopen(path, "r");
    if (!fp) {
        printf("  [%s] 打开 resource 失败: %s\n", bdf, strerror(errno));
        return;
    }

    printf("  [%s] BAR 资源表：\n", bdf);
    while (fgets(line, sizeof(line), fp)) {
        uint64_t s = 0, e = 0, fl = 0;

        if (sscanf(line, "%" SCNx64 " %" SCNx64 " %" SCNx64, &s, &e, &fl) != 3)
            continue;

        if (s == 0 && e == 0)
            printf("    bar%d : <unused>\n", idx);
        else
            printf("    bar%d : [0x%016" PRIx64 " - 0x%016" PRIx64 "] size=0x%" PRIx64
                   " (%" PRIu64 " B) flags=0x%" PRIx64 "\n",
                   idx, s, e, e - s + 1, e - s + 1, fl);
        idx++;
    }

    fclose(fp);
}

/*
 * 打开并映射一个 BAR。
 * 控制寄存器 BAR 与数据 BAR 均使用 O_SYNC 映射，得到非 cache 的属性，
 * 保证：(a) 寄存器写不会被合并/重排；(b) DMA 写完后 CPU 回读能直接看到设备内存内容。
 *
 * lock_mode: 0=不加锁（目的 bar 允许多个源头共享，靠占用链协调）
 *            1=共享锁  2=独占锁（DMA 控制寄存器必须独占）
 */
static int open_bar(struct bar *b, const char *bdf, int num, int need_write, int lock_mode)
{
    char  path[PATH_MAX];
    int   fd;
    void *va;

    memset(b, 0, sizeof(*b));
    b->fd = -1;
    snprintf(b->bdf, sizeof(b->bdf), "%s", bdf);
    b->num = num;

    if (pci_bar_info(bdf, num, &b->phys, &b->size) != 0) {
        fprintf(stderr, "读取 %s 的 bar%d 信息失败: %s\n", bdf, num, strerror(errno));
        return -E_SYSFS;
    }

    if (b->size == 0) {
        fprintf(stderr, "%s 的 bar%d 未使能或不存在\n", bdf, num);
        return -E_NO_BAR;
    }

    snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/resource%d", bdf, num);
    fd = open(path, need_write ? (O_RDWR | O_SYNC) : (O_RDONLY | O_SYNC));
    if (fd < 0) {
        fprintf(stderr, "打开 %s 失败: %s (需要 root 权限?)\n", path, strerror(errno));
        return -E_SYSFS;
    }

    va = mmap(NULL, (size_t)b->size, PROT_READ | (need_write ? PROT_WRITE : 0),
              MAP_SHARED, fd, 0);
    if (va == MAP_FAILED) {
        fprintf(stderr, "mmap %s (%" PRIu64 " B) 失败: %s\n", path, b->size, strerror(errno));
        close(fd);
        return -E_MMAP;
    }

    if (lock_mode != 0 &&
        flock(fd, (lock_mode == 2 ? LOCK_EX : LOCK_SH) | LOCK_NB) != 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            fprintf(stderr,
                    "%s 的 bar%d 已被另一个实例%s占用。\n",
                    bdf, num, lock_mode == 2 ? "独占" : "独占锁定");
            munmap(va, (size_t)b->size);
            close(fd);
            memset(b, 0, sizeof(*b));
            b->fd = -1;
            return -E_LOCKED;
        }
        fprintf(stderr, "警告: flock %s 失败(%s)，跳过实例互斥检查\n", path, strerror(errno));
    }

    b->fd = fd;
    b->va = (uint8_t *)va;
    b->owner = 1;
    return E_OK;
}

static void close_bar(struct bar *b)
{
    if (b->owner) {
        if (b->va)
            munmap(b->va, (size_t)b->size);
        if (b->fd >= 0)
            close(b->fd);   /* 关闭 fd 同时释放 flock */
    }
    b->va = NULL;
    b->fd = -1;
    b->owner = 0;
}

/* ============================== MMIO 访问 ============================== */
static inline uint32_t reg_rd32(void *base, uint32_t off)
{
    uint32_t v = *(volatile uint32_t *)((uint8_t *)base + off);
    __sync_synchronize();
    return v;
}

static inline void reg_wr32(void *base, uint32_t off, uint32_t v)
{
    __sync_synchronize();
    *(volatile uint32_t *)((uint8_t *)base + off) = v;
    __sync_synchronize();
}

/* 数据区的 64 位读（--peek / 现场快照用）：同样是两笔 32 位访问 */
static inline uint64_t mmio_rd64(void *base, uint32_t off)
{
    uint64_t lo = reg_rd32(base, off);
    uint64_t hi = reg_rd32(base, off + 4);

    return lo | (hi << 32);
}

/* ============================== DMA 读写接口（需求 3） ============================== */

/*
 * 启动一次 DMA 并等待完成。
 * dir_write = 0 : 读  —— 把目标地址 bus 的内容搬进 ACS Exerciser 内存 (mem_off 起)
 * dir_write = 1 : 写  —— 把 ACS Exerciser 内存 (mem_off 起) 的内容搬到目标地址 bus
 * 完成判据：CTRL bit0 由硬件自动清零；随后检查 STATUS 是否报错。
 */
static int dma_single(struct ctx *c, int dir_write, uint64_t bus, uint32_t len)
{
    uint32_t ctrl;
    uint64_t t0, deadline;

    ctrl = reg_rd32(c->src.va, DMA_REG_CTRL);
    if (ctrl & DMA_CTRL_START) {
        /* 上一次还没结束，先给它一点时间；仍不结束则报错 */
        t0 = now_ns();
        deadline = t0 + (uint64_t)c->cfg.timeout_ms * 1000000ull;
        while ((reg_rd32(c->src.va, DMA_REG_CTRL) & DMA_CTRL_START) && now_ns() < deadline)
            sleep_us(1);
        if (reg_rd32(c->src.va, DMA_REG_CTRL) & DMA_CTRL_START)
            return -E_HW_BUSY;
    }

    /* 编程：偏移 -> 目标地址(低/高) -> 长度 -> 方向+启动 */
    reg_wr32(c->src.va, DMA_REG_MEM_OFF, (uint32_t)c->cfg.mem_off);
    reg_wr32(c->src.va, DMA_REG_ADDR_LO, (uint32_t)(bus & 0xFFFFFFFFull));
    reg_wr32(c->src.va, DMA_REG_ADDR_HI, (uint32_t)((bus >> 32) & 0xFFFFFFFFull));
    reg_wr32(c->src.va, DMA_REG_LEN, len);

    ctrl = DMA_CTRL_START | (dir_write ? DMA_CTRL_DIR_WRITE : 0u);
    reg_wr32(c->src.va, DMA_REG_CTRL, ctrl);

    /* 轮询完成：bit0 自动清零 */
    t0 = now_ns();
    deadline = t0 + (uint64_t)c->cfg.timeout_ms * 1000000ull;

    while (reg_rd32(c->src.va, DMA_REG_CTRL) & DMA_CTRL_START) {
        if (now_ns() >= deadline)
            return -E_HW_TIMEOUT;
        sleep_us(1);
    }

    if (c->cfg.check_status) {
        uint32_t st = reg_rd32(c->src.va, DMA_REG_STATUS);
        if (st != 0) {
            /* 只记下来，不打印：调用方会先打点（写错误码），之后才输出 */
            c->last_status = st;
            c->last_status_valid = 1;
            return -E_HW_STATUS;
        }
    }

    return E_OK;
}

/* 带分块的长传输封装：chunk=0 时一次传完 */
static int dma_run(struct ctx *c, int dir_write, uint64_t bus, uint64_t len)
{
    uint64_t done = 0;

    if (len == 0)
        return -E_USAGE;

    while (done < len) {
        uint64_t n = len - done;
        int      rc;

        if (c->block && n > c->block)
            n = c->block;

        if (n > 0xFFFFFFFFull)
            n = 0xFFFFFFFFull;

        rc = dma_single(c, dir_write, bus + done, (uint32_t)n);
        if (rc != E_OK)
            return rc;

        done += n;
    }

    return E_OK;
}

/* 读测试接口：dst -> DMA 内部内存 */
static int dma_read(struct ctx *c, uint64_t bus, uint64_t len)
{
    return dma_run(c, 0, bus, len);
}

/* 写测试接口：DMA 内部内存 -> dst（内容即上一次 dma_read 读进来的数据） */
static int dma_write(struct ctx *c, uint64_t bus, uint64_t len)
{
    return dma_run(c, 1, bus, len);
}

/* ============================== pattern 生成与比对 ============================== */
/*
 * 这里所有数据搬运都自己做，不用 libc 的 memcpy/memset：
 *  1) 部分虚拟机(VMware/Hyper-V)的 vCPU 特性掩码与系统 glibc 基线不一致时
 *     （典型：系统按 x86-64-v3 构建，而 vCPU 不暴露 AVX2），glibc 的大块
 *     memcpy 会执行非法指令直接 SIGILL（trap invalid opcode in libc.so.6）。
 *     自带的 8 字节循环在任何 x86-64 上都能跑，不受影响；
 *  2) 对设备 BAR 只发 8 字节 store，行为比向量化写更可预测。
 */

/* 以 8 字节为粒度拷贝；invert=1 时写入按位取反的内容（用于污染） */
static void copy_words(uint8_t *dst, const uint8_t *src, uint64_t len, int invert)
{
    uint64_t i = 0;

    for (; i + 8 <= len; i += 8) {
        uint64_t v;

        memcpy(&v, src + i, 8);              /* 定长 8B，编译器内联为单条 load */
        if (invert)
            v = ~v;
        *(volatile uint64_t *)(dst + i) = v; /* 单条 store，避免被向量化 */
    }

    for (; i < len; i++)
        dst[i] = invert ? (uint8_t)~src[i] : src[i];
}

/* 数据模式：--pat rand(默认) / inc(每轮+1) / alt(5A5A 与 A5A5 逐轮翻转) */
#define PAT_RAND 0
#define PAT_INC  1
#define PAT_ALT  2

/* --stress 纯压力模式的方向。压力模式不做任何比对：
 *   数据只在启动时初始化一次，之后每轮只按方向搬一遍，用来压 PCIe 链路/fabric。
 *   RD  = DMA 读：初始化区 -> ACS 内部内存（数据进控制器，不回写 BAR）
 *   WR  = DMA 写：ACS 内部内存 -> 测试区（内容固定，不做校验）
 *   BOTH= 每轮先读一遍再写一遍，读+写混合流量 */
#define STRESS_OFF  0
#define STRESS_RD   1
#define STRESS_WR   2
#define STRESS_BOTH 3

/*
 * --addr_pat：压力模式下"每轮从哪个地址开始"的规律。
 *
 *   none = 固定从区首开始（老行为；size 小时会一直打同一个地址，压不到地址线）
 *   seq  = 每轮起点前进 --addr_step 字节，走到区尾回绕
 *   rand = 起点伪随机（由 seed/iter 决定，可复现）
 *
 * 每轮仍然覆盖整个区，只是**起始地址**不同，所以地址/通道的翻转位置跟着变；
 * 小 size（比如 4 KiB）时也能每轮换着起点打，而不是永远打同一笔。
 */
#define ADDR_FIX   0
#define ADDR_SEQ   1
#define ADDR_RAND  2

/* --selftest 打到 bar 时的方向（纯内存自检时不使用） */
#define SELFTEST_RW 0
#define SELFTEST_RD 1
#define SELFTEST_WR 2

/* 顺序模式的步长：0 = 自动（大区按页滑、小区按 qword 滑） */
static uint64_t stress_addr_step(const struct ctx *c)
{
    uint64_t step = c->cfg.addr_step;

    if (step == 0)
        step = (c->cfg.size > 4096ull) ? 4096ull : DATA_ALIGN;

    step &= ~(DATA_ALIGN - 1);
    if (step == 0)
        step = DATA_ALIGN;
    return step;
}

/* 本轮起始偏移，保证 8 字节对齐且落在 [0, size) 内 */
static uint64_t stress_base(const struct ctx *c, uint64_t iter)
{
    uint64_t size = c->cfg.size;
    uint64_t base;

    if (c->cfg.addr_pat == ADDR_FIX || size == 0)
        return 0;

    if (c->cfg.addr_pat == ADDR_RAND) {
        uint64_t s = c->cfg.seed ^ (iter * 0x9E3779B97F4A7C15ull);
        uint64_t slots = size / DATA_ALIGN;

        if (slots == 0)
            slots = 1;
        base = (splitmix64(&s) % slots) * DATA_ALIGN;
    } else {
        base = (iter * stress_addr_step(c)) % size;
    }

    base &= ~(DATA_ALIGN - 1);
    return (base < size) ? base : 0;
}

static void pattern_fill(uint8_t *buf, uint64_t len, uint64_t seed,
                         int mode, uint64_t iter)
{
    uint64_t s = seed;
    uint64_t i = 0;

    if (mode == PAT_INC) {
        /* 每轮整体 +1：本轮第 k 个 qword = iter + k，相邻轮错开 1，能检出陈旧数据 */
        for (; i + 8 <= len; i += 8) {
            uint64_t v = iter + (i >> 3);

            memcpy(buf + i, &v, 8);
        }
    } else if (mode == PAT_ALT) {
        /* 5A5A... 与 A5A5... 逐轮翻转 */
        for (; i + 8 <= len; i += 8) {
            uint64_t v = (iter & 1) ? 0xA5A5A5A5A5A5A5A5ull
                                    : 0x5A5A5A5A5A5A5A5Aull;

            memcpy(buf + i, &v, 8);
        }
    } else {
        if (s == 0)
            s = 0x0BADCAFEull;

        for (; i + 8 <= len; i += 8) {
            uint64_t v = splitmix64(&s);

            memcpy(buf + i, &v, 8);
        }
    }

    if (i < len) {
        uint64_t v;
        uint8_t *p = (uint8_t *)&v;
        uint64_t k;

        if (mode == PAT_INC)
            v = iter + (i >> 3);
        else if (mode == PAT_ALT)
            v = (iter & 1) ? 0xA5A5A5A5A5A5A5A5ull
                           : 0x5A5A5A5A5A5A5A5Aull;
        else
            v = splitmix64(&s);

        for (k = 0; i + k < len; k++)
            buf[i + k] = p[k];
    }
}

/*
 * 逐 8 字节比对，返回 0 表示一致；不一致时回填首个不符位置与期望/实际值。
 */
static int buf_compare(const uint8_t *got, const uint8_t *exp, uint64_t len,
                       uint64_t *bad_off, uint64_t *g, uint64_t *e)
{
    uint64_t i = 0;

    __sync_synchronize();

    for (; i + 8 <= len; i += 8) {
        uint64_t va = *(volatile uint64_t *)(got + i);
        uint64_t vb;

        memcpy(&vb, exp + i, 8);
        if (va != vb) {
            *bad_off = i;
            *g = va;
            *e = vb;
            return -1;
        }
    }

    if (i < len) {
        uint64_t va = 0, vb = 0;
        uint64_t k;

        for (k = 0; i + k < len; k++) {
            ((uint8_t *)&va)[k] = got[i + k];
            ((uint8_t *)&vb)[k] = exp[i + k];
        }
        if (va != vb) {
            *bad_off = i;
            *g = va;
            *e = vb;
            return -1;
        }
    }

    return 0;
}

/* ============================== 错误记录 ============================== */

/* ---------- 本地 DDR 错误记录（不占目的 bar 空间） ---------- */

/* bdf / owner 里可能带 ':' '.' '/'，换成文件名安全的字符 */
static void path_sanitize(const char *in, char *out, size_t n)
{
    size_t i = 0;

    if (!in)
        in = "";
    for (; in[i] != '\0' && i + 1 < n; i++) {
        char ch = in[i];

        out[i] = (ch == ':' || ch == '.' || ch == '/' || ch == ' ' || ch == '\t')
               ? '_' : ch;
    }
    out[i] = '\0';
    if (i == 0)
        snprintf(out, n, "src");
}

/* 记录文件路径：--err_log 优先；否则按 bdf/bar/owner 生成，多源头天然分开 */
static void err_local_path(const struct ctx *c, char *buf, size_t n)
{
    char bdf[64], own[80];
    const char *o;

    if (c->cfg.err_log[0]) {
        snprintf(buf, n, "%s", c->cfg.err_log);
        return;
    }

    o = c->cfg.owner[0] ? c->cfg.owner : (c->src.bdf[0] ? c->src.bdf : "src");
    path_sanitize(c->dst.bdf[0] ? c->dst.bdf : c->cfg.dst_bdf, bdf, sizeof(bdf));
    path_sanitize(o, own, sizeof(own));
    snprintf(buf, n, "/tmp/dma_acs_err_%s_bar%d_%s.rec", bdf, c->cfg.dst_bar, own);
}

/* 建立记录区：mmap 一个本地文件，进程退出/崩溃后仍可用 --dump_err 读回 */
static int err_local_init(struct ctx *c)
{
    char  path[256];
    void *p;
    int   fd;

    err_local_path(c, path, sizeof(path));
    snprintf(c->el_path, sizeof(c->el_path), "%s", path);

    fd = open(path, O_CREAT | O_RDWR, 0666);
    if (fd >= 0) {
        if (ftruncate(fd, (off_t)ERL_SIZE) == 0) {
            p = mmap(NULL, (size_t)ERL_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            if (p != MAP_FAILED) {
                close(fd);
                c->el = p;
                c->el_map = p;
                memset(c->el, 0, sizeof(*c->el));   /* 清掉上次运行的残留 */
                return E_OK;
            }
        }
        close(fd);
    }

    /* 退化成进程内缓冲：这次能打印，进程退出后记录就没了 */
    c->el = &c->el_buf;
    memset(c->el, 0, sizeof(*c->el));
    fprintf(stderr, "提示: 本地错误记录 %s 不可用（%s），记录只留在进程内存\n",
            path, strerror(errno));
    return E_OK;
}

static void err_local_close(struct ctx *c)
{
    if (c->el_map) {
        munmap(c->el_map, (size_t)ERL_SIZE);
        c->el_map = NULL;
    }
    c->el = NULL;
}

/* 从记录文件读回一条（--dump_err / --peek auto 用，不需要碰 bar） */
static int err_local_read(const struct ctx *c, struct err_local *out)
{
    char path[256];
    int  fd;
    int  rc;

    err_local_path(c, path, sizeof(path));
    fd = open(path, O_RDONLY);
    if (fd < 0)
        return -1;

    rc = (read(fd, out, sizeof(*out)) == (ssize_t)sizeof(*out)) ? 0 : -1;
    close(fd);
    return rc;
}

/* 读回并解析本地错误记录（--dump_err） */
static int dump_err_record(struct ctx *c)
{
    struct err_local r;
    char             path[256];

    err_local_path(c, path, sizeof(path));
    printf("本地错误记录: %s\n", path);

    if (err_local_read(c, &r) != 0) {
        printf("  读不到记录文件（还没失败过，或路径/--owner 不对）\n");
        return E_OK;
    }
    if (r.magic != ERL_MAGIC) {
        printf("  magic=0x%08x (!= 0x%08x)，没有有效错误记录（上次测试未失败或已重跑）\n",
               r.magic, ERL_MAGIC);
        return E_OK;
    }

    printf("  错误码      : %u (%s)\n", r.code, err_name((int)r.code));
    printf("  源头        : %s (%s -> %s bar%d)\n",
           r.owner, r.src_bdf, r.dst_bdf, r.dst_bar);
    printf("  pid         : %u\n", r.pid);
    printf("  失败迭代号  : %" PRIu64 "（此前已通过 %" PRIu64 " 轮）\n", r.iter, r.total);
    printf("  DMA 目标地址: 0x%016" PRIx64 "\n", r.bus);
    printf("  bar 内偏移  : 0x%" PRIx64 "\n", r.bar_off);
    printf("  传输长度    : 0x%" PRIx64 "\n", r.len);
    printf("  首个不符偏移: 0x%" PRIx64 "\n", r.bad_off);
    printf("  期望值      : 0x%016" PRIx64 "\n", r.expect);
    printf("  实际值      : 0x%016" PRIx64 "\n", r.actual);
    printf("  pattern 种子: 0x%08x\n", r.seed);
    printf("  errno       : %u\n", r.sys_errno);
    printf("  时间戳(ns)  : %" PRIu64 "\n", r.ts);
    if (r.flags & ERL_F_STATUS)
        printf("  DMA STATUS  : 0x%08x\n", r.status);
    if (r.flags & ERL_F_SNAP)
        printf("  寄存器快照  : CTRL=0x%08x OFF=0x%08x LO=0x%08x"
               " HI=0x%08x LEN=0x%08x ST=0x%08x\n",
               r.snap[0], r.snap[1], r.snap[2], r.snap[3], r.snap[4], r.snap[5]);

    return E_OK;
}

/* 以 8 字节粒度读 bar，按 16 字节/行 hex dump（--peek 事后取证用） */
static void hexdump_bar(struct bar *b, uint64_t off, uint64_t len)
{
    uint64_t start = off & ~0x7ull;
    uint64_t end;
    uint64_t a;

    if (len == 0)
        len = 64;
    end = start + ((len + 7) & ~0x7ull);
    if (end > b->size)
        end = b->size & ~0x7ull;

    __sync_synchronize();
    for (a = start; a < end; a += 8) {
        if ((a - start) % 16 == 0)
            printf("  0x%06" PRIx64 ":", a);
        printf(" %016" PRIx64, mmio_rd64(b->va, (uint32_t)a));
        if ((a - start) % 16 == 8)
            printf("\n");
    }
    if ((end - start) % 16 != 0)
        printf("\n");
}

/* 清除本地记录的 valid 标志与头里的错误标志，避免把上次的残留当本次错误 */
static void err_clear(struct ctx *c)
{
    if (c->el) {
        c->el->magic = 0;
        c->el->code = 0;
        __sync_synchronize();
    }
    /* 头里的 err_flag 是粘滞值，新一轮测试开始前归零；写 0 不会满足
     * trigger 的 "data != 0" 条件，不会误触发 */
    if (c->trip)
        *c->trip = 0u;
}

/*
 * ========================= 失败打点（trigger 抓点） =========================
 *
 * 故障判定成立的那一刻只做这一件事：把错误码写进**目的 bar 超级头里的
 * err_flag**（H_ERRFLAG）。
 *
 * 为什么放在头里：
 *   - 头本来就在目的 bar 上、本来就有测试头，不需要额外的"错误记录 bar"；
 *   - 头的位置是固定的（bar + --dst_off + 0x08），trigger 可以直接按地址设；
 *   - 一次 4 字节对齐的 32 位 store，波形上就是干干净净的一笔写事务，
 *     中间没有读、没有格式化、没有 printf、没有 fence。
 *
 * trigger 条件建议：
 *     地址   = dst bar base + --dst_off + 0x08
 *     方向   = 写
 *     数据   = != 0（值就是错误码 1~14）
 *     位置   = first / normal（单次）
 *
 * 详细字段不写 bar（bar 空间紧张），全部写本地 DDR，见 err_record_detail。
 * bar 上除了头里这一个错误标志，不再有第二处拷贝。
 *
 * err_trip 本身定义在下面"目的 bar 布局"之后：它只解引用 ctx->trip，
 * 而 trip 的地址已经在 plan_layout 里算好（指向超级头的 err_flag），
 * 所以失败路径上没有任何计算。
 */

/* 打点之后补全详情：写入本地 DDR（普通内存，不产生任何 PCIe 事务） */
static void err_record_detail(struct ctx *c, uint64_t iter, uint64_t total,
                              const struct fail_info *fi)
{
    struct err_local *e = c->el;
    const char       *o;

    if (!e)
        return;

    e->magic = 0;               /* 先失效，避免读到半成品 */
    __sync_synchronize();

    e->code      = (uint32_t)fi->code;
    e->iter      = iter;
    e->total     = total;
    e->bus       = fi->bus;
    e->bar_off   = fi->bar_off;
    e->len       = fi->len;
    e->bad_off   = fi->bad_off;
    e->expect    = fi->expect;
    e->actual    = fi->actual;
    e->ts        = now_ns();
    e->seed      = fi->seed;
    e->sys_errno = (uint32_t)fi->sys_errno;
    e->pid       = (uint32_t)getpid();
    e->dst_bar   = (uint32_t)c->cfg.dst_bar;
    e->dst_off   = c->cfg.dst_off;
    e->init_off  = c->init_off;
    e->test_off  = c->test_off;
    e->dst_phys  = c->dst.phys;
    e->status    = c->last_status;
    e->flags     = (c->snap_valid ? ERL_F_SNAP : 0u) |
                   (c->last_status_valid ? ERL_F_STATUS : 0u);
    memcpy(e->snap, c->snap, sizeof(e->snap));

    o = c->cfg.owner[0] ? c->cfg.owner : (c->src.bdf[0] ? c->src.bdf : "src");
    snprintf(e->owner, sizeof(e->owner), "%s", o);
    snprintf(e->src_bdf, sizeof(e->src_bdf), "%s", c->src.bdf);
    snprintf(e->dst_bdf, sizeof(e->dst_bdf), "%s", c->dst.bdf);

    __sync_synchronize();
    e->magic = ERL_MAGIC;
    __sync_synchronize();
}

/* ============================== 自检（不碰硬件） ============================== */

/* 打印 /proc/cpuinfo 里与本问题相关的特性：glibc 若按 x86-64-v3 构建而 vCPU
 * 不暴露 avx2，用户态程序会出现 "illegal instruction in libc.so.6" */
static void print_cpu_flags(void)
{
    static const char *want[] = { "avx ", "avx2 ", "avx512f", "sse4_2", "erms", "bmi2" };
    FILE *fp;
    char  line[8192];
    size_t k;

    fp = fopen("/proc/cpuinfo", "r");
    if (!fp) {
        printf("  (无法读取 /proc/cpuinfo)\n");
        return;
    }

    while (fgets(line, sizeof(line), fp)) {
        if (strncmp(line, "flags", 5) != 0)
            continue;

        printf("  CPU 特性:");
        for (k = 0; k < sizeof(want) / sizeof(want[0]); k++) {
            if (strstr(line, want[k]))
                printf(" %s", want[k]);
            else
                printf(" -%s", want[k]);
        }
        printf("\n");

        if (!strstr(line, "avx2"))
            printf("  注意: 该 CPU 未暴露 avx2。若系统 glibc 以 x86-64-v3 为基线，\n"
                   "        大块 memcpy/memset 会产生 SIGILL；请检查虚拟机 CPU 型号/掩码。\n");
        break;
    }

    fclose(fp);
}

/*
 * ============================== CPU 自检（可指定 bar/off/size） ==============================
 *
 * --selftest 原本只跑纯内存自检（pattern -> 内存 -> 比对），用来把"环境问题"
 * 和"程序问题"分开。这里扩展成"也能直接打 bar"：
 *
 *   ./dma_acs_test --selftest                          # 老行为：纯内存
 *   ./dma_acs_test -d 02:00.0 --selftest --st_bar 0 \
 *                  --st_off 0x1000 -l 4K               # 打 dst bar0 的 0x1000~0x2000
 *
 * st_op 决定方向：
 *   rw（默认） 写 pattern -> 回读比对 -> 再写反码 -> 回读确认确实变了
 *   rd         只读：dump 内容、算累加和，不改动任何字节（安全的"看一眼"）
 *   wr         只写：pattern / 反码交替写，不做回读比对
 *
 * 与 --no_init 的配合见 do_one_iter：--no_init 让闭环测试阶段 1 不碰 init 区，
 * 数据由对端（另一张卡的 DMA 或别的进程）预先写好；本函数就是用来确认
 * "我想让 DMA 读的那片 init 区，CPU 能看到什么"——读出来的内容也就是
 * --no_init 下阶段 3 的比对基准来源。
 */
static int run_selftest_bar(struct ctx *c)
{
    struct bar *b = &c->dst;
    uint64_t    off = c->cfg.st_off;
    uint64_t    n = c->cfg.size;
    uint64_t    bad_off = 0, got = 0, exp = 0;
    uint64_t    i;
    uint64_t    loops;
    int         rc = E_OK;

    if (n == 0)
        n = 4096;

    /* -n 0 在闭环测试里表示无限循环；自检没有退出判据，按 1 轮处理 */
    loops = c->cfg.times ? c->cfg.times : 1;
    if (loops > 1000000ull) {
        fprintf(stderr, "--selftest 打 bar 的 -n 上限 1000000（当前 %" PRIu64 "）\n", loops);
        return -E_USAGE;
    }

    if (off & 0x7ull) {
        fprintf(stderr, "--st_off 必须 8 字节对齐: 0x%" PRIx64 "\n", off);
        return -E_USAGE;
    }
    /*
     * --st_off 和 --dst_off 是两套坐标：前者只描述"这次自检读哪一段"，
     * 后者描述"测试头/占用链从哪开始"。同时给了容易看混，明确提示一次。
     */
    if (off && c->cfg.dst_off && off != c->cfg.dst_off)
        printf("提示: --st_off 0x%" PRIx64 " 只用于本次自检，与 --dst_off 0x%" PRIx64
               "（测试头起点）无关\n", off, c->cfg.dst_off);
    if (off >= b->size || n > b->size - off) {
        fprintf(stderr,
                "--st_off 0x%" PRIx64 " + -l %" PRIu64 " 越界：bar%d 只有 0x%" PRIx64 " 字节\n",
                off, n, b->num, b->size);
        return -E_LAYOUT;
    }
    /* 与 --dst_off 同一口径：bar 内偏移最终按 32 位传给 mmio 访问 */
    if (off + n > 0xFFFFFFFFull) {
        fprintf(stderr, "--st_off + -l 超出 32 位寻址范围（bar 内偏移按 32 位）\n");
        return -E_LAYOUT;
    }

    c->pat = malloc((size_t)n);
    if (!c->pat) {
        fprintf(stderr, "分配 %" PRIu64 " 字节 pattern 缓冲失败\n", n);
        return -E_NOMEM;
    }

    printf("================ CPU 直读直写 BAR 自检 ================\n");
    print_cpu_flags();
    printf("  目标        : %s bar%d  phys=0x%016" PRIx64 "\n",
           b->bdf, b->num, b->phys);
    printf("  区间        : bar+0x%" PRIx64 " ~ bar+0x%" PRIx64 "（%s）\n",
           off, off + n, c->cfg.st_op == SELFTEST_RD ? "只读" :
                          c->cfg.st_op == SELFTEST_WR ? "只写" : "写+回读");
    printf("  长度/轮次   : %" PRIu64 " B × %" PRIu64 " 轮\n", n, loops);
    printf("  数据模式    : %s\n",
           c->cfg.pat_mode == PAT_INC ? "递增 inc" :
           c->cfg.pat_mode == PAT_ALT ? "翻转 alt" : "随机 rand");
    printf("------------------------------------------------------\n");

    if (c->cfg.st_op == SELFTEST_RD) {
        /* 只读：dump 头 64 字节 + 整片累加和（不改动 BAR 任何字节） */
        uint64_t sum = 0;

        hexdump_bar(b, off, n < 64 ? n : 64);
        for (i = 0; i + 8 <= n; i += 8)
            sum ^= mmio_rd64(b->va, (uint32_t)(off + i));

        printf("  累加和(XOR) : 0x%016" PRIx64 "\n", sum);
        printf("  READ PASS: 已读出 bar+0x%" PRIx64 " 起 %" PRIu64 " 字节，未写入任何字节\n",
               off, n);
        free(c->pat);
        c->pat = NULL;
        printf("======================================================\n");
        return E_OK;
    }

    for (i = 0; i < (uint64_t)loops && !g_stop; i++) {
        uint64_t seed = c->cfg.seed ^ (i * 0x9E3779B97F4A7C15ull);

        /* 1) 写 pattern */
        pattern_fill(c->pat, n, seed, c->cfg.pat_mode, i);
        copy_words(b->va + off, c->pat, n, 0);
        __sync_synchronize();

        if (c->cfg.st_op == SELFTEST_WR) {
            /* 只写：不比对，每轮换 pattern 直接覆盖 */
            if (c->cfg.verbose)
                printf("  [%s] 第 %" PRIu64 " 轮已写入 %" PRIu64 " 字节\n",
                       c->cfg.owner[0] ? c->cfg.owner : b->bdf, i + 1, n);
            continue;
        }

        /* 2) 回读比对：刚写的东西必须能原样读回 */
        if (buf_compare(b->va + off, c->pat, n, &bad_off, &got, &exp) != 0) {
            printf("  FAIL: 写入后回读不一致 @bar+0x%" PRIx64
                   " got=0x%016" PRIx64 " exp=0x%016" PRIx64 "\n",
                   off + bad_off, got, exp);
            printf("  说明: 该地址写后读不回原值——设备未实现可写，或写被吞/被改写\n");
            rc = -E_MISMATCH;
            break;
        }

        /* 3) 写反码再回读：确认这一片确实可写（不是只回显旧值） */
        copy_words(b->va + off, c->pat, n, 1);
        __sync_synchronize();
        if (buf_compare(b->va + off, c->pat, n, &bad_off, &got, &exp) == 0) {
            printf("  FAIL: 写入反码后回读仍等于原 pattern @bar+0x%" PRIx64 "\n",
                   off + bad_off);
            printf("  说明: 该区间不可写或被硬连线，写入被静默丢弃\n");
            rc = -E_MISMATCH;
            break;
        }
    }

    if (rc == E_OK)
        printf("  PASS: %" PRIu64 " 轮读写回环全部一致（bar+0x%" PRIx64 "，%s）\n",
               i < (uint64_t)loops ? i + 1 : (uint64_t)loops, off,
               c->cfg.st_op == SELFTEST_WR ? "只写" : "写+回读");

    free(c->pat);
    c->pat = NULL;
    printf("======================================================\n");
    return rc;
}

/* 纯内存自检：验证 pattern 生成 -> 写入 -> 比对 这条 CPU 侧数据通路 */
static int run_selftest(struct ctx *c)
{
    uint64_t n = c->cfg.size ? c->cfg.size : 4096;
    uint64_t bad_off = 0, got = 0, exp = 0;
    uint8_t *pat;
    uint8_t *buf;
    int      rc = E_OK;

    pat = malloc((size_t)n);
    buf = malloc((size_t)n);
    if (!pat || !buf) {
        free(pat);
        free(buf);
        return E_NOMEM;
    }

    printf("==================== CPU 侧数据通路自检 ====================\n");
    print_cpu_flags();
    printf("  测试长度    : %" PRIu64 " B，循环 %" PRIu64 " 次\n",
           n, c->cfg.times ? c->cfg.times : 100);

    {
        uint64_t it;
        uint64_t loops = c->cfg.times ? c->cfg.times : 100;

        for (it = 0; it < loops && !g_stop; it++) {
            uint64_t seed = c->cfg.seed ^ (it * 0x9E3779B97F4A7C15ull);

            pattern_fill(pat, n, seed, c->cfg.pat_mode, it);
            copy_words(buf, pat, n, 0);
            if (buf_compare(buf, pat, n, &bad_off, &got, &exp) != 0) {
                printf("  FAIL: 正常拷贝后比对不一致 @0x%" PRIx64
                       " got=0x%016" PRIx64 " exp=0x%016" PRIx64 "\n", bad_off, got, exp);
                rc = E_MISMATCH;
                break;
            }

            copy_words(buf, pat, n, 1);
            if (buf_compare(buf, pat, n, &bad_off, &got, &exp) == 0) {
                printf("  FAIL: 污染(取反)后比对仍然一致，校验机制失效\n");
                rc = E_MISMATCH;
                break;
            }
        }

        if (rc == E_OK)
            printf("  PASS: %" PRIu64 " 轮 pattern 生成/写入/比对 全部正常\n", it);
    }

    printf("  提示: 若此处 PASS 但实机测试仍 SIGILL，崩溃点在硬件访问路径，\n"
           "        可用 objdump 定位故障指令（见 README 第 8 节）\n");
    printf("============================================================\n");

    free(pat);
    free(buf);
    return rc;
}

/* ============================== 目的 bar 测试头 + 占用链 ============================== */
/*
 * 目的 bar 由"测试头"统一管理。默认从 bar 偏移 0 开始；如果被测设备只在
 * bar 内某一段（比如 bar0 里划出来的一块）才是可测空间，用 --dst_off 指定
 * 起点，下面所有布局整体平移，bar 起点到 dst_off 之间的内容一个字节都不碰。
 *
 *   dst_off + 0x000   超级头（0x100，只用到前 0x18），+0x08 = err_flag
 *   dst_off + 0x100   节点#0 头（0x100）
 *   dst_off + 0x200   节点#0 数据：init 区 + test 区（8 字节对齐）
 *   ...               节点#1 头 + 节点#1 数据 ...
 *
 * 错误打点就在超级头的 err_flag 里，bar 上不再有第二处错误码。
 *
 * 每个节点头记录"谁在占这段空间、占了多少、跑得怎么样"，节点按申请顺序串成
 * 单向链表。新实例启动时走一遍链，把新节点接在链尾之后的空闲空间上——这样多个
 * DMA 源头可以共用同一个目的 bar，不需要人工划分地址。
 *
 * 目的 bar 空间紧张，所以：
 *   - 超级头只留 magic/ver/err_flag/head/节点数/序号 6 个字段（0x18 字节），
 *     数据区上限就是 bar 末尾，不再单独存；
 *   - 节点头只留分配与统计必需字段（0x80 有效，整块占 0x100）；
 *   - 数据区只要求 8 字节对齐（qword 访问），不再按页对齐；
 *   - 错误详情写本地 DDR，bar 上只有 err_flag 这一个字。
 *
 * 最多 72 个源头（节点），超出就报 E_FULL。
 * 不做回收：节点一旦申请就一直占用（state=ACTIVE），要让空间重新可用就 --clear。
 */
#define HDR_MAGIC        0x48414D44u    /* 'DMAH' */
#define NODE_MAGIC       0x4E414D44u    /* 'DMAN' */
#define NODE_TAIL_MAGIC  0x444E454Eu    /* 'DNEN' 节点完整性哨兵 */
#define HDR_VERSION      4u             /* 布局/字段变更后 +1，旧头必须 --clear */

#define BAR_HDR_SIZE     0x100ull      /* 超级头 0x100（字段只用到 0x18） */
#define NODE_HDR_SIZE    0x100ull      /* 节点头 0x100 */
#define NODE_MAX_COUNT   72            /* 最多 72 个源头 */
#define NODE_MAX_CHAIN   (NODE_MAX_COUNT + 8)  /* 走链保护，顺带能发现超限节点 */
#define BAR_ALIGN        0x1000ull     /* --dst_off 必须按这个粒度对齐 */

/*
 * 头的访问铁律（emu 上抓波形的硬要求）：
 *   1) 所有字段偏移都 4 字节对齐，超级头基址 = --dst_off（4 KiB 对齐）；
 *   2) 只用 32 位整字读写（reg_rd32 / reg_wr32），不出现 8/16/8 位访问；
 *   3) 64 位字段拆成"低字 + 高字"两次 32 位访问（hdr_wr64 / hdr_rd64），
 *      不生成 8 字节事务，也不会跨字边界；
 *   4) 头里的字符串（owner / src_bdf）按 4 字节一个字打包，同样是整字访问。
 * 这样总线上看到的头访问永远是干净的单周期 4 字节事务，trigger 不会因为
 * 拆包/非对齐访问而抓到多笔或对不上地址。
 */

/* 超级头字段偏移（全部 u32，4 字节对齐） */
#define H_MAGIC     0x00u   /* u32 magic，最后写 */
#define H_VER       0x04u   /* u32 版本 */
#define H_ERRFLAG   0x08u   /* u32 错误标志：0 = 正常，非 0 = 错误码。
                             *     ★ 失败瞬间第一笔写就是它（trigger 抓点） */
#define H_HEAD      0x0Cu   /* u32 首节点偏移，0 = 空链 */
#define H_NODES     0x10u   /* u32 当前节点数（上限 72） */
#define H_SEQ       0x14u   /* u32 序号计数器 */
/* 0x18 之后空闲。数据区上限不再存头里：它就是"bar 末尾"（--dst_off 起算），
 * 从 sysfs 的 bar 大小直接可得，多实例天然一致，没有第二个口径。 */

/* 节点字段偏移（0x100 以内；错误详情不放这里，见本地 DDR） */
#define N_MAGIC    0x00u    /* u32 'DMAN'，最后写 */
#define N_STATE    0x04u    /* u32 1 = 占用中 */
#define N_ERRFLAG  0x08u    /* u32 本源头错误标志（错误码，0 = 正常） */
#define N_SEQ      0x0Cu    /* u32 序号 */
#define N_SELF     0x10u    /* u64 自身偏移（两个 32 位字） */
#define N_NEXT     0x18u    /* u64 下一节点，0 = 链尾 */
#define N_DATA     0x20u    /* u64 数据区起点（= init 区） */
#define N_TEST     0x28u    /* u64 test 区偏移 */
#define N_LEN      0x30u    /* u64 用户 -l */
#define N_SPAN     0x38u    /* u64 整节点占用（含节点头） */
#define N_ERRITER  0x40u    /* u64 失败时的迭代号 */
#define N_DONE     0x48u    /* u64 已完成轮数 */
#define N_PID      0x50u    /* u32 申请者 pid */
#define N_TAIL     0x54u    /* u32 完整性哨兵 'DNEN' */
#define N_OWNER    0x58u    /* char[24] = 6 个 32 位字 */
#define N_SRCBDF   0x70u    /* char[16] = 4 个 32 位字 */
/* 0x80 之后空闲，整块 0x100 都是节点头 */

#define NODE_OWNER_MAX  24
#define NODE_BDF_MAX    16
#define NODE_OWNER_WORDS (NODE_OWNER_MAX / 4)   /* 6 个字，按字读写 */
#define NODE_BDF_WORDS   (NODE_BDF_MAX / 4)     /* 4 个字，按字读写 */

struct node_info {
    int      valid;
    uint32_t state;
    uint32_t seq;
    uint32_t pid;
    uint64_t off;
    uint64_t span;
    uint64_t data_off;
    uint64_t len;
    uint64_t test_off;
    uint64_t next_off;
    uint32_t err_code;      /* 对应头里的 N_ERRFLAG */
    uint64_t err_iter;
    uint64_t done;
    char     owner[NODE_OWNER_MAX];
    char     src_bdf[NODE_BDF_MAX];
};

/* 测试空间的起点：--dst_off 指定的 bar 内偏移 */
static inline uint64_t hdr_base(const struct ctx *c)
{
    return c->cfg.dst_off;
}

/* 头里 err_flag 的绝对 bar 内偏移，也是 trigger 要设的地址 */
static inline uint32_t hdr_errflag_off(const struct ctx *c)
{
    return (uint32_t)(hdr_base(c) + H_ERRFLAG);
}

/*
 * 失败瞬间的唯一动作：把错误码写到 c->trip 指向的那个字。
 * 一次 32 位整字 store、地址 4 字节对齐、无 fence、无读、无打印。
 * 正常情况下 trip = dst bar + --dst_off + 0x08（超级头的 err_flag）。
 *
 * 这里故意不加内存屏障：
 *   - 屏障会在总线上插入额外同步开销，还会把 store 往后推，让 trigger 抓到的
 *     时刻偏离故障点；
 *   - BAR 是 O_SYNC(UC) 映射，x86 对同一 UC 区的 store 是强序的，
 *     "读 CTRL 判定完成 → 写 err_flag" 这种 读→写 顺序本就不会被重排，
 *     不需要额外屏障来保证先后正确。
 */
static inline void err_trip(struct ctx *c, int code)
{
    if (__builtin_expect(c->trip != NULL, 1))
        *c->trip = (uint32_t)code;
}

/* 超级头之后的第一个空闲字节，也是第一个节点可能的落点 */
static inline uint64_t data_start(const struct ctx *c)
{
    return c->cfg.dst_off + BAR_HDR_SIZE;
}

/*
 * 数据区上限：不做人为限制——--dst_off 之后一直到 bar 末尾都可分配。
 *
 * 这个值只由 sysfs 的 bar 大小决定，所有实例算出来必然一样。早先那版有个
 * --dst_len 让每个进程各报一个窗口尾，多打一时反而各算各的：给得小的那个会把
 * 别人窗口外已经占上的节点判成“链断了”（node_read 的 span 校验、chain_walk 的
 * free_off 都跟着变小），然后在同一段空间上再分配一次，两个源头打同一片地址。
 * 所以那个参数和头里的 H_LIMIT 一起删掉了，这里只留一个物理上界。
 */
static uint64_t data_limit(const struct ctx *c)
{
    uint64_t limit = c->dst.size;

    if (limit > 0xFFFFFFFFull)   /* bar 内偏移按 32 位寻址，留出回绕余量 */
        limit = 0xFFFFFFFFull;

    return limit;
}

/*
 * 头的 64 位字段一律拆成"低字 + 高字"两笔 32 位访问：
 * 不生成 8 字节事务（emu 上 64 位 MMIO 可能被拆包或直接不支持），
 * 也不会出现跨字边界的非对齐访问。
 */
static inline void hdr_wr64(void *va, uint32_t off, uint64_t v)
{
    reg_wr32(va, off, (uint32_t)(v & 0xFFFFFFFFull));
    reg_wr32(va, off + 4u, (uint32_t)((v >> 32) & 0xFFFFFFFFull));
}

static inline uint64_t hdr_rd64(void *va, uint32_t off)
{
    uint64_t lo = (uint64_t)reg_rd32(va, off);
    uint64_t hi = (uint64_t)reg_rd32(va, off + 4u);

    return lo | (hi << 32);
}

/* 头里的字符串按 4 字节一个字打包写：每笔都是 4 字节对齐的 32 位 store */
static void hdr_wr_str(struct bar *b, uint32_t off, const char *s, uint32_t words)
{
    uint32_t n = 0, i, k;
    uint32_t maxn = words * 4u - 1u;   /* 末尾必须留一个 '\0' */

    if (s) {
        while (n < maxn && s[n] != '\0')
            n++;
    }

    if (n > maxn)
        n = maxn;

    for (i = 0; i < words; i++) {
        uint32_t w = 0;

        for (k = 0; k < 4u; k++) {
            uint32_t idx = i * 4u + k;

            if (idx < n)
                w |= (uint32_t)(uint8_t)s[idx] << (8u * k);
        }
        reg_wr32(b->va, off + i * 4u, w);
    }
}

/* 对应读取：同样只发 4 字节对齐的 32 位 load */
static void hdr_rd_str(struct bar *b, uint32_t off, char *out, uint32_t cap)
{
    uint32_t i, k;
    int      end = 0;

    memset(out, 0, cap);

    for (i = 0; i + 4u <= cap && !end; i += 4u) {
        uint32_t w = reg_rd32(b->va, off + i);

        for (k = 0; k < 4u && i + k < cap - 1u; k++) {
            char ch = (char)((w >> (8u * k)) & 0xFFu);

            out[i + k] = ch;
            if (ch == '\0') {
                end = 1;
                break;
            }
        }
    }
    out[cap - 1] = '\0';
}

/* 分配临界区锁：不在 bar 内存里做原子操作，用 /tmp 锁文件，跨进程串行化 */
static int alloc_lock(const struct bar *dst)
{
    char name[80];
    char path[192];
    int  i, fd;

    snprintf(name, sizeof(name), "%s", dst->bdf);
    for (i = 0; name[i] != '\0'; i++) {
        if (name[i] == ':' || name[i] == '.')
            name[i] = '_';
    }
    snprintf(path, sizeof(path), "/tmp/.dma_acs_alloc_%s_bar%d.lock", name, dst->num);

    fd = open(path, O_CREAT | O_RDWR, 0666);
    if (fd < 0)
        return -1;

    if (flock(fd, LOCK_EX) != 0) {
        close(fd);
        return -1;
    }

    return fd;
}

static void alloc_unlock(int fd)
{
    if (fd < 0)
        return;
    flock(fd, LOCK_UN);
    close(fd);
}

/* 走链（定义在后面），hdr_reset 要借它算出"清到哪儿为止" */
static void chain_walk(struct ctx *c, struct node_info *tail, uint64_t *free_off);

/* 初始化/清空测试头本体，不加锁（调用者必须已持有分配锁）。
 * wipe=1 时把占用链上分配过的数据区清零（节点 magic 随之失效）。
 * 只清到链尾，不扫整条 bar：没有窗口参数兜底以后，按 bar 末尾清会把设备
 * 同一块 bar 里的其它功能区一起清掉，而没分配过的字节本来也不需要清。 */
static int hdr_reset(struct ctx *c, int wipe)
{
    struct bar      *b = &c->dst;
    struct node_info tail;
    uint32_t         hb = (uint32_t)hdr_base(c);
    uint64_t         start = data_start(c);
    uint64_t         limit = data_limit(c);
    uint64_t         off, end = start, t0 = now_ns();

    if (limit <= start) {
        fprintf(stderr,
                "目的 bar 从 0x%" PRIx64 " 起的可用空间只到 0x%" PRIx64
                "，放不下测试头（需要 0x%" PRIx64 "）\n",
                hdr_base(c), limit, BAR_HDR_SIZE);
        return -E_LAYOUT;
    }

    /* 只有头有效时链才可信；新 bar 上的随机内容是垃圾，不去走它 */
    if (wipe && reg_rd32(b->va, hb + H_MAGIC) == HDR_MAGIC) {
        chain_walk(c, &tail, &end);
        if (end > limit)
            end = limit;
        end &= ~3ull;                 /* 只清到 4 字节边界，不产生非整字访问 */
    }

    /* 先失效 magic，避免别人看到半成品头 */
    reg_wr32(b->va, hb + H_MAGIC, 0);
    __sync_synchronize();

    if (wipe && end > start) {
        char sz[32];

        fmt_size(sz, sizeof(sz), end - start);
        printf("清空目的 bar 数据区 [0x%" PRIx64 ", 0x%" PRIx64 ") 共 %s ...\n",
               start, end, sz);

        /* 一律 32 位整字写：emu 上 8 字节 store 可能被拆包或直接不支持 */
        for (off = start; off < end; off += 4)
            *(volatile uint32_t *)(b->va + off) = 0;

        __sync_synchronize();
        printf("清空完成，用时 %.1f ms\n", (double)(now_ns() - t0) / 1e6);
    } else if (wipe) {
        printf("占用链为空，没有数据区需要清，只重置测试头\n");
    }

    reg_wr32(b->va, hb + H_HEAD, 0);
    reg_wr32(b->va, hb + H_NODES, 0);
    reg_wr32(b->va, hb + H_SEQ, 0);
    reg_wr32(b->va, hb + H_ERRFLAG, 0);      /* 清空时连同错误标志一起归零 */
    reg_wr32(b->va, hb + H_VER, HDR_VERSION);

    __sync_synchronize();
    reg_wr32(b->va, hb + H_MAGIC, HDR_MAGIC);
    __sync_synchronize();

    return E_OK;
}

/* 加锁版本，供 --clear / --clear_only 使用 */
static int hdr_init(struct ctx *c, int wipe)
{
    int lk, rc;

    lk = alloc_lock(&c->dst);
    if (lk < 0)
        fprintf(stderr, "警告: 无法创建分配锁，清空期间请确保无其他实例在运行\n");

    rc = hdr_reset(c, wipe);
    alloc_unlock(lk);
    return rc;
}

/* 读一个节点并自校验；返回 0 表示节点完整可信 */
static int node_read(struct ctx *c, uint64_t off, struct node_info *n)
{
    struct bar *b = &c->dst;
    uint64_t    limit = data_limit(c);
    uint32_t    o;

    memset(n, 0, sizeof(*n));

    if (off < data_start(c) || (off & (DATA_ALIGN - 1)) != 0 || off + NODE_HDR_SIZE > limit)
        return -1;

    o = (uint32_t)off;

    if (reg_rd32(b->va, o + N_MAGIC) != NODE_MAGIC)
        return -1;
    if (reg_rd32(b->va, o + N_TAIL) != NODE_TAIL_MAGIC)
        return -1;

    n->off      = off;
    n->state    = reg_rd32(b->va, o + N_STATE);
    n->seq      = reg_rd32(b->va, o + N_SEQ);
    n->pid      = reg_rd32(b->va, o + N_PID);
    n->span     = hdr_rd64(b->va, o + N_SPAN);
    n->data_off = hdr_rd64(b->va, o + N_DATA);
    n->len      = hdr_rd64(b->va, o + N_LEN);
    n->test_off = hdr_rd64(b->va, o + N_TEST);
    n->next_off = hdr_rd64(b->va, o + N_NEXT);
    n->err_code = reg_rd32(b->va, o + N_ERRFLAG);
    n->err_iter = hdr_rd64(b->va, o + N_ERRITER);
    n->done     = hdr_rd64(b->va, o + N_DONE);

    hdr_rd_str(b, o + N_OWNER, n->owner, sizeof(n->owner));
    hdr_rd_str(b, o + N_SRCBDF, n->src_bdf, sizeof(n->src_bdf));

    /* 自校验：自身偏移、数据偏移、跨度都要自洽 */
    if (hdr_rd64(b->va, o + N_SELF) != off)
        return -1;
    if (n->data_off != off + NODE_HDR_SIZE)
        return -1;
    if (n->span < NODE_HDR_SIZE || off + n->span > limit)
        return -1;
    if (n->test_off < n->data_off || n->test_off + n->len > off + n->span)
        return -1;

    n->valid = 1;
    return 0;
}

/* 写节点：magic 最后写，保证别人只看到完整节点 */
static void node_write(struct ctx *c, const struct node_info *n)
{
    struct bar *b = &c->dst;
    uint32_t    o = (uint32_t)n->off;

    reg_wr32(b->va, o + N_MAGIC, 0);
    __sync_synchronize();

    reg_wr32(b->va, o + N_STATE, n->state);
    hdr_wr64(b->va, o + N_SELF, n->off);
    hdr_wr64(b->va, o + N_SPAN, n->span);
    hdr_wr64(b->va, o + N_DATA, n->data_off);
    hdr_wr64(b->va, o + N_LEN, n->len);
    hdr_wr64(b->va, o + N_TEST, n->test_off);
    hdr_wr64(b->va, o + N_NEXT, n->next_off);
    reg_wr32(b->va, o + N_SEQ, n->seq);
    reg_wr32(b->va, o + N_PID, n->pid);
    reg_wr32(b->va, o + N_ERRFLAG, n->err_code);
    hdr_wr64(b->va, o + N_ERRITER, n->err_iter);
    hdr_wr64(b->va, o + N_DONE, n->done);

    hdr_wr_str(b, o + N_OWNER, n->owner, NODE_OWNER_WORDS);
    hdr_wr_str(b, o + N_SRCBDF, n->src_bdf, NODE_BDF_WORDS);

    reg_wr32(b->va, o + N_TAIL, NODE_TAIL_MAGIC);
    __sync_synchronize();
    reg_wr32(b->va, o + N_MAGIC, NODE_MAGIC);
    __sync_synchronize();
}

/* 走链：返回末节点（可能 invalid=0）与空闲起点 */
static void chain_walk(struct ctx *c, struct node_info *tail, uint64_t *free_off)
{
    uint64_t        off = (uint64_t)reg_rd32(c->dst.va, (uint32_t)hdr_base(c) + H_HEAD);
    uint64_t        limit = data_limit(c);
    int             guard = 0;
    struct node_info cur;

    memset(tail, 0, sizeof(*tail));

    while (off != 0 && guard++ < NODE_MAX_CHAIN) {
        if (node_read(c, off, &cur) != 0) {
            fprintf(stderr, "警告: 占用链在 0x%" PRIx64 " 处断裂，按链尾处理\n", off);
            break;
        }
        *tail = cur;
        off = cur.next_off;
    }

    if (tail->valid) {
        *free_off = align_up(tail->off + tail->span, DATA_ALIGN);
    } else {
        *free_off = data_start(c);
    }

    if (*free_off > limit)
        *free_off = limit;
}

/* 申请一个节点：走链找到链尾，把新节点接上去 */
static int alloc_node(struct ctx *c, struct node_info *out)
{
    struct bar      *b = &c->dst;
    struct node_info tail;
    uint32_t         hb = (uint32_t)hdr_base(c);
    uint64_t         limit = data_limit(c);
    uint64_t         free_off = 0, need, init_span;
    int              lk, rc = E_OK;

    /* 锁要罩住"检查 magic -> 初始化 -> 走链 -> 追加节点"整段，
     * 否则两个实例同时在一张新 bar 上启动会各自初始化、互相覆盖 */
    lk = alloc_lock(b);
    if (lk < 0)
        fprintf(stderr, "警告: 无法加分配锁，并发启动可能抢到同一段空间\n");

    if (reg_rd32(b->va, hb + H_MAGIC) != HDR_MAGIC) {
        printf("目的 bar +0x%" PRIx64 " 无有效测试头(0x%08x)，"
               "自动初始化（等价 --clear 但不清数据）\n",
               hdr_base(c), reg_rd32(b->va, hb + H_MAGIC));
        rc = hdr_reset(c, 0);
        if (rc != E_OK)
            goto out;
    } else if (reg_rd32(b->va, hb + H_VER) != HDR_VERSION) {
        fprintf(stderr, "测试头版本 %u 与本程序 %u 不匹配，请先 --clear\n",
                reg_rd32(b->va, hb + H_VER), HDR_VERSION);
        rc = -E_LAYOUT;
        goto out;
    }

    chain_walk(c, &tail, &free_off);

    init_span = align_up(c->cfg.size, DATA_ALIGN);
    need = NODE_HDR_SIZE + init_span * 2;

    /* 源头上限 72：链再长也不收，避免某个 bar 被无限切碎 */
    if (reg_rd32(b->va, hb + H_NODES) >= NODE_MAX_COUNT) {
        fprintf(stderr,
                "占用链已满: 该 bar 上已有 %u 个源头，上限 %d 个\n",
                reg_rd32(b->va, hb + H_NODES), NODE_MAX_COUNT);
        fprintf(stderr, "处理: 用 --list 看当前占用，确认哪些源头已经跑完，"
                        "再 --clear 后重新拉起需要的源头\n");
        rc = -E_FULL;
        goto out;
    }

    if (free_off + need > limit) {
        char n1[32], n2[32];

        fmt_size(n1, sizeof(n1), need);
        fmt_size(n2, sizeof(n2), (limit > free_off) ? (limit - free_off) : 0);
        fprintf(stderr,
                "目的 bar 剩余空间不足: 需要 %s (0x%" PRIx64 ")，"
                "空闲起点 0x%" PRIx64 " 到上限 0x%" PRIx64 " 只剩 %s\n",
                n1, need, free_off, limit, n2);
        fprintf(stderr, "处理: 用 --list 看当前占用，或 --clear 清空后重来；"
                        "bar 从 --dst_off 到末尾放不下时，减小 -l 或换更大的 bar\n");
        rc = -E_LAYOUT;
        goto out;
    }

    memset(out, 0, sizeof(*out));
    out->valid    = 1;
    out->state    = 1;
    out->off      = free_off;
    out->span     = need;
    out->data_off = free_off + NODE_HDR_SIZE;
    out->len      = c->cfg.size;
    out->test_off = out->data_off + init_span;
    out->next_off = 0;
    out->seq      = reg_rd32(b->va, hb + H_SEQ) + 1u;
    out->pid      = (uint32_t)getpid();
    out->err_code = 0;
    snprintf(out->owner, sizeof(out->owner), "%s",
             c->cfg.owner[0] ? c->cfg.owner : c->src.bdf);
    snprintf(out->src_bdf, sizeof(out->src_bdf), "%s", c->src.bdf);

    node_write(c, out);

    /* 链接：先接链尾，再更新头，顺序保证任何时刻链都是可走的 */
    if (tail.valid)
        hdr_wr64(b->va, (uint32_t)tail.off + N_NEXT, out->off);
    else
        reg_wr32(b->va, hb + H_HEAD, (uint32_t)out->off);

    reg_wr32(b->va, hb + H_NODES, reg_rd32(b->va, hb + H_NODES) + 1u);
    reg_wr32(b->va, hb + H_SEQ, out->seq);
    __sync_synchronize();

out:
    alloc_unlock(lk);
    return rc;
}

/* 打印占用链（--list） */
static int list_nodes(struct ctx *c)
{
    struct bar      *b = &c->dst;
    struct node_info n;
    uint32_t         hb = (uint32_t)hdr_base(c);
    uint64_t         off;
    uint64_t         free_off = data_start(c);
    uint64_t         limit = data_limit(c);
    uint64_t         err_node = 0;
    uint32_t         magic = reg_rd32(b->va, hb + H_MAGIC);
    int              guard = 0;
    int              idx = 0;

    printf("目的 bar %s bar%d  (phys=0x%016" PRIx64 ", size=0x%" PRIx64 ")\n",
           b->bdf, b->num, b->phys, b->size);
    if (c->cfg.dst_off)
        printf("  测试空间     : bar+0x%" PRIx64 " 起（--dst_off），"
               "数据区 0x%" PRIx64 " ~ 0x%" PRIx64 "\n",
               c->cfg.dst_off, data_start(c), limit);
    else
        printf("  测试空间     : bar+0x0 起（--dst_off 未指定）\n");

    if (magic != HDR_MAGIC) {
        printf("  bar+0x%" PRIx64 " 处无有效测试头 (magic=0x%08x)，该段空间尚未被管理\n",
               hdr_base(c), magic);
        return E_OK;
    }

    printf("  测试头: ver=%u 节点数=%u/%d err_flag=0x%08x%s\n",
           reg_rd32(b->va, hb + H_VER),
           reg_rd32(b->va, hb + H_NODES), NODE_MAX_COUNT,
           reg_rd32(b->va, hb + H_ERRFLAG),
           reg_rd32(b->va, hb + H_ERRFLAG) ? " (该 bar 上有源头报过错)" : "");
    printf("  错误标志位: bar+0x%" PRIx64 "（trigger 地址，--clear 归零）\n",
           hdr_base(c) + H_ERRFLAG);

    off = (uint64_t)reg_rd32(b->va, hb + H_HEAD);
    if (off == 0) {
        printf("  占用链为空\n");
        return E_OK;
    }

    printf("  序号  节点偏移   数据偏移   init长度  状态    源头                      pid      完成轮数  错误\n");
    printf("  ----  ---------  ---------  --------  ------  ------------------------  -------  ---------  ----\n");

    while (off != 0 && guard++ < NODE_MAX_CHAIN) {
        if (node_read(c, off, &n) != 0) {
            printf("  !! 节点 0x%" PRIx64 " 损坏，链在此中断\n", off);
            break;
        }

        printf("  %4u  0x%07" PRIx64 "  0x%07" PRIx64 "  %8" PRIu64 "  %-6s  %-24s  %-7u  %9" PRIu64 "  %s%u\n",
               ++idx, n.off, n.data_off, n.len,
               n.state == 1 ? "占用" : "空闲",
               n.owner, n.pid, n.done,
               n.err_code ? "!" : "-", n.err_code);

        free_off = align_up(n.off + n.span, DATA_ALIGN);
        if (n.err_code)
            err_node = n.off;
        off = n.next_off;
    }

    printf("  共 %d 个节点（上限 %d）；空闲起点 0x%" PRIx64 "，剩余 0x%" PRIx64 " 字节\n",
           idx, NODE_MAX_COUNT, free_off, (limit > free_off) ? (limit - free_off) : 0);

    /* 详细错误在本地 DDR 的记录文件里，这里只提示去哪儿看 */
    if (err_node != 0) {
        char tpl[160];

        path_sanitize(c->dst.bdf, tpl, sizeof(tpl));
        printf("\n  节点 0x%" PRIx64 " 报过错；详细现场在各源头自己的本地记录里：\n"
               "    /tmp/dma_acs_err_%s_bar%d_<owner>.rec\n"
               "    用该节点的源头名配合 --owner <名> --dump_err 查看\n",
               err_node, tpl, c->cfg.dst_bar);
    }

    return E_OK;
}

/* 失败时把错误码写进自己节点，便于 --list 一眼看出哪个源头挂了 */
static void node_mark_error(struct ctx *c, int code, uint64_t iter)
{
    if (!c->has_node)
        return;
    reg_wr32(c->dst.va, (uint32_t)c->node_off + N_ERRFLAG, (uint32_t)code);
    hdr_wr64(c->dst.va, (uint32_t)c->node_off + N_ERRITER, iter);
    __sync_synchronize();
}

/* 进度回填，供 --list 观察 */
static void node_mark_progress(struct ctx *c, uint64_t done)
{
    if (!c->has_node)
        return;
    hdr_wr64(c->dst.va, (uint32_t)c->node_off + N_DONE, done);
}

/* ============================== 布局规划 ============================== */

/*
 * --dst_off 自身合法性检查。--list / --clear 不走 plan_layout，但同样要在
 * 正确的偏移上操作，所以单独抽出来，三处（列链、清空、跑测试）都先过一遍。
 */
static int check_dst_off(struct ctx *c)
{
    /* 必须页对齐：超级头、节点头、init/test 区都按 4 KiB 摆放，
     * 起点不齐会让后面所有块都跟着错位 */
    if (c->cfg.dst_off & (BAR_ALIGN - 1)) {
        fprintf(stderr, "--dst_off 必须按 4 KiB 对齐: 0x%" PRIx64
                "（向下最近的合法值 0x%" PRIx64 "）\n",
                c->cfg.dst_off,
                c->cfg.dst_off & ~(BAR_ALIGN - 1));
        return -E_USAGE;
    }
    if (c->cfg.dst_off >= c->dst.size) {
        fprintf(stderr, "--dst_off 0x%" PRIx64 " 超出 bar%d 大小 0x%" PRIx64 "\n",
                c->cfg.dst_off, c->dst.num, c->dst.size);
        return -E_LAYOUT;
    }
    /* 所有 bar 内偏移最终以 32 位传给 mmio 访问，留一点余量防止回绕 */
    if (c->cfg.dst_off > 0xFFFFFFFFull - BAR_HDR_SIZE) {
        fprintf(stderr, "--dst_off 过大: 0x%" PRIx64 "（bar 内偏移按 32 位寻址）\n",
                c->cfg.dst_off);
        return -E_LAYOUT;
    }

    return E_OK;
}

static int plan_layout(struct ctx *c)
{
    uint64_t usable;
    int      drc = check_dst_off(c);

    if (drc != E_OK)
        return drc;

    /* 数据区上限 = bar 末尾（不设人为窗口） */
    usable = data_limit(c);

    c->region = align_up(c->cfg.size, 0x1000);

    /* 默认由测试头分配（init_off/test_off 由 alloc_node 给出）；
     * --no_alloc 时退回固定布局：init@dst_off、test@dst_off+ALIGN(size) */
    if (c->cfg.no_alloc) {
        c->init_off = c->cfg.dst_off;
        c->test_off = c->cfg.dst_off + c->region;

        if (c->test_off + c->region > usable) {
            char a[32], b[32], d[32];
            uint64_t room = (usable > c->cfg.dst_off) ? (usable - c->cfg.dst_off) : 0;
            uint64_t max_size = (room / 2) & ~0xFFFull;

            fmt_size(a, sizeof(a), c->cfg.size);
            fmt_size(b, sizeof(b), room);
            fmt_size(d, sizeof(d), max_size);
            fprintf(stderr,
                    "空间不足: 需要 2 x %s（对齐后 2 x 0x%" PRIx64 "），"
                    "bar%d 从 0x%" PRIx64 " 到上限 0x%" PRIx64 " 只有 0x%" PRIx64
                    " (%s)，单区上限 %s (0x%" PRIx64 ")\n",
                    a, c->region, c->dst.num, c->cfg.dst_off, usable, room, b, d, max_size);
            return -E_LAYOUT;
        }
    }

    /* 单次 DMA 长度按 -l 走（不是对齐后的跨度），--chunk 只是把它再切小。
     * 切块后整轮按"CPU 整片准备 -> DMA 整片搬运 -> CPU 整片校验"分三段跑，
     * 不再每块穿插一次 CPU 访问（见 do_one_iter） */
    c->block = c->cfg.chunk ? c->cfg.chunk : c->cfg.size;
    if (c->block > c->cfg.size)
        c->block = c->cfg.size;

    /* 让每个分块的起始偏移保持 8 字节对齐，8 字节 store 才不会跨边界 */
    if (c->cfg.chunk && (c->block & 0x7ull)) {
        uint64_t b = c->block & ~0x7ull;

        if (b == 0)
            b = 8;
        printf("提示: --chunk %" PRIu64 " 未按 8 字节对齐，按 %" PRIu64 " 使用\n",
               c->cfg.chunk, b);
        c->block = b;
    }
    if (c->block > 0xFFFFFFFFull) {
        fprintf(stderr, "--chunk 过大（DMA Length 是 32 位）\n");
        return -E_USAGE;
    }

    /*
     * 打点地址在这里一次算好：dst bar + --dst_off + H_ERRFLAG（超级头 err_flag）。
     * --no_alloc 没有超级头，也就没有打点位置（trip = NULL），
     * 此时失败现场只能看本地记录文件。
     * 之后 err_trip 只做一次解引用写，不再有任何计算/分支。
     */
    c->trip = NULL;
    if (!c->cfg.no_alloc && c->dst.va)
        c->trip = (volatile uint32_t *)((uint8_t *)c->dst.va + hdr_errflag_off(c));

    return E_OK;
}

/* ============================== 单次迭代（需求 4） ============================== */

/* 抓一份控制器寄存器快照（6 次读，也是总线事务，只在 --verbose 时抓，
 * 且一定在 err_trip 打点之后调用，避免污染 trigger 之前的波形） */
static void snap_regs(struct ctx *c)
{
    if (!c->src.va)
        return;

    c->snap[0] = reg_rd32(c->src.va, DMA_REG_CTRL);
    c->snap[1] = reg_rd32(c->src.va, DMA_REG_MEM_OFF);
    c->snap[2] = reg_rd32(c->src.va, DMA_REG_ADDR_LO);
    c->snap[3] = reg_rd32(c->src.va, DMA_REG_ADDR_HI);
    c->snap[4] = reg_rd32(c->src.va, DMA_REG_LEN);
    c->snap[5] = reg_rd32(c->src.va, DMA_REG_STATUS);
    c->snap_valid = 1;
}

/*
 * CPU 模式的单次读-改-写闭环（--cpu）。
 *
 * 不用 DMA 控制器，全部由 CPU 通过 mmap 出来的 BAR 直读直写，用来和另一个
 * 还在跑 DMA 的进程一起，构造"DMA 与 CPU 同时访问同一个设备"的场景。
 *
 * 闭环结构和 DMA 模式完全对应，只是把"ACS 内部内存"换成本进程的 bounce 缓冲：
 *   1) init 区 <- pattern           （CPU 写）
 *   2) test 区 <- ~pattern          （CPU 写，污染，保证"没写进去"能检出）
 *   3) init 区 -> bounce            （CPU 读，对应 DMA 读方向）
 *   4) bounce 与 pattern 比对        （读到被并发流量改坏的内容，这里就会暴露）
 *   5) bounce -> test 区            （CPU 写，对应 DMA 写方向）
 *   6) test 区回读比对
 * 失败时同样第一时间 err_trip()，之后才允许做别的事。
 */
static int cpu_one_iter(struct ctx *c, uint64_t iter, struct fail_info *fi)
{
    uint64_t off = 0;

    memset(fi, 0, sizeof(*fi));
    c->snap_valid = 0;
    c->last_status_valid = 0;

    while (off < c->cfg.size) {
        uint64_t n = c->cfg.size - off;
        uint8_t *init = c->dst.va + c->init_off + off;
        uint8_t *test = c->dst.va + c->test_off + off;
        uint64_t seed;

        if (n > c->block)
            n = c->block;

        seed = c->cfg.seed
             ^ (iter * 0x9E3779B97F4A7C15ull)
             ^ (off * 0xD1B54A32D192ED03ull);

        if (c->cfg.no_init) {
            /*
             * --no_init：不填 init、不污染 test，只做 读 init -> 写 test 的搬运。
             * 基准换成"读到的 init 实际内容"（bounce 本身），所以这里改比对
             * test 是否等于 bounce，而不是等于预设 pattern。
             */
            copy_words(c->bounce, init, n, 0);
            __sync_synchronize();
            copy_words(test, c->bounce, n, 0);
            __sync_synchronize();

            if (buf_compare(test, c->bounce, n,
                            &fi->bad_off, &fi->actual, &fi->expect) != 0) {
                err_trip(c, E_MISMATCH);
                fi->code = E_MISMATCH;
                fi->bus = c->dst.phys + c->test_off + off + fi->bad_off;
                fi->bar_off = c->test_off + off + fi->bad_off;
                fi->len = n;
                fi->seed = (uint32_t)seed;
                return -E_MISMATCH;
            }

            off += n;
            continue;
        }

        /* 1) 初始化区：CPU 写已知 pattern */
        pattern_fill(c->pat, n, seed, c->cfg.pat_mode, iter);
        __sync_synchronize();
        copy_words(init, c->pat, n, 0);

        /* 2) 测试区：污染成 pattern 反码 */
        copy_words(test, c->pat, n, 1);
        __sync_synchronize();

        /* 3) CPU 读：init -> bounce */
        copy_words(c->bounce, init, n, 0);
        __sync_synchronize();

        /* 4) 读回来的必须还是 pattern（并发写破坏 init 区会在这里暴露） */
        if (buf_compare(c->bounce, c->pat, n,
                        &fi->bad_off, &fi->actual, &fi->expect) != 0) {
            err_trip(c, E_MISMATCH);
            fi->code = E_MISMATCH;
            fi->bus = c->dst.phys + c->init_off + off + fi->bad_off;
            fi->bar_off = c->init_off + off + fi->bad_off;
            fi->len = n;
            fi->seed = (uint32_t)seed;
            return -E_MISMATCH;
        }

        /* 5) CPU 写：bounce -> test */
        copy_words(test, c->bounce, n, 0);
        __sync_synchronize();

        /* 6) 回读校验 */
        if (buf_compare(test, c->bounce, n,
                        &fi->bad_off, &fi->actual, &fi->expect) != 0) {
            err_trip(c, E_MISMATCH);
            fi->code = E_MISMATCH;
            fi->bus = c->dst.phys + c->test_off + off + fi->bad_off;
            fi->bar_off = c->test_off + off + fi->bad_off;
            fi->len = n;
            fi->seed = (uint32_t)seed;
            return -E_MISMATCH;
        }

        off += n;
    }

    return E_OK;
}

/*
 * ============================== 纯压力模式 ==============================
 *
 * 闭环测试每轮都要"CPU 填 init、污染 test、回读比对"，CPU 侧开销和 DMA 相当，
 * 压不出链路本身的极限。压力模式把 CPU 侧彻底摘掉：
 *
 *   - 数据只在启动时初始化一次（stress_prepare），之后每一轮都不再改写；
 *   - 每一轮只按指定方向把整块数据搬一遍，不做任何比对；
 *   - 失败只可能是 DMA 硬件/总线错误（超时、Status 非 0），不会出现 mismatch。
 *
 * 写方向要求 ACS 内部内存里先有数据，所以准备阶段会多做一次 DMA 读，
 * 把 init 区搬进内部内存（偏移固定 0），后续每轮反复把它写出去。
 */

/* 压力模式的数据准备：整个测试只做这一次。 */
static int stress_prepare(struct ctx *c, struct fail_info *fi)
{
    uint64_t seed = c->cfg.seed ^ 0x9E3779B97F4A7C15ull;
    uint64_t off  = 0;

    memset(fi, 0, sizeof(*fi));

    /* 1) 一次填满 init 与 test 区（test 区内容对压力模式无意义，
     *    填同一份只是为了失败现场看起来一致） */
    while (off < c->cfg.size) {
        uint64_t n = c->cfg.size - off;

        if (n > c->block)
            n = c->block;

        pattern_fill(c->pat, n, seed ^ (off * 0xD1B54A32D192ED03ull),
                     c->cfg.pat_mode, 0);
        __sync_synchronize();
        copy_words(c->dst.va + c->init_off + off, c->pat, n, 0);
        copy_words(c->dst.va + c->test_off + off, c->pat, n, 0);
        __sync_synchronize();
        off += n;
    }

    /* 2) init 区回读校验：判定逻辑与闭环测试一致，同样报 E_INIT_CHECK。
     *    压力模式不校验数据，但"初始化都没落盘"必须在这里挡住，
     *    否则后面压的是空气。 */
    off = 0;
    while (off < c->cfg.size) {
        uint64_t n = c->cfg.size - off;
        int      tries = 0;

        if (n > c->block)
            n = c->block;

        pattern_fill(c->pat, n, seed ^ (off * 0xD1B54A32D192ED03ull),
                     c->cfg.pat_mode, 0);

        while (buf_compare(c->dst.va + c->init_off + off, c->pat, n,
                           &fi->bad_off, &fi->actual, &fi->expect) != 0) {
            if (++tries >= 1000 || g_stop) {
                err_trip(c, E_INIT_CHECK);
                fi->code = E_INIT_CHECK;
                fi->bus = c->dst.phys + c->init_off + off + fi->bad_off;
                fi->bar_off = c->init_off + off + fi->bad_off;
                fi->len = n;
                fi->seed = (uint32_t)seed;
                return -E_INIT_CHECK;
            }
            delay_ms(c->cfg.init_delay_ms ? c->cfg.init_delay_ms : 1);
        }
        off += n;
    }

    /* 3) 写方向：先把 init 区搬进 ACS 内部内存，之后每轮反复写出去 */
    if (!c->cfg.cpu_mode &&
        (c->cfg.stress == STRESS_WR || c->cfg.stress == STRESS_BOTH)) {
        int rc = dma_read(c, c->dst.phys + c->init_off, c->cfg.size);

        if (rc != E_OK) {
            err_trip(c, -rc);
            fi->code = -rc;
            fi->bus = c->dst.phys + c->init_off;
            fi->bar_off = c->init_off;
            fi->len = c->cfg.size;
            fi->seed = (uint32_t)seed;
            return rc;
        }
    }

    /* 保证 pat 里留有一份完整的分块内容（CPU 写方向直接拿它当数据源） */
    pattern_fill(c->pat, c->block, seed, c->cfg.pat_mode, 0);

    printf("压力数据已初始化（仅此一次）: init@0x%" PRIx64 " / test@0x%" PRIx64
           " 各 %" PRIu64 " 字节\n", c->init_off, c->test_off, c->cfg.size);
    fflush(stdout);
    return E_OK;
}

/* 压力模式的一轮：只搬数据，不比对、不打印、不改写 init/test 区。
 *
 * 本轮从 stress_base() 给出的起点开始，顺序搬到区尾后回绕到区首，直到整区
 * 都搬过一遍。所以总字节数不变，但**起始地址每轮不同**——size 只有 4 KiB 时
 * 也不会永远打同一笔地址。 */
static int stress_one_iter(struct ctx *c, uint64_t iter, struct fail_info *fi)
{
    int      do_rd = (c->cfg.stress & STRESS_RD) != 0;
    int      do_wr = (c->cfg.stress & STRESS_WR) != 0;
    uint64_t size  = c->cfg.size;
    uint64_t base  = stress_base(c, iter);
    uint64_t done  = 0;

    memset(fi, 0, sizeof(*fi));
    c->snap_valid = 0;
    c->last_status_valid = 0;
    c->init_cur_valid = 0;

    while (done < size) {
        uint64_t off = base + done;
        uint64_t n, remain;
        int      rc;

        if (off >= size)
            off -= size;                 /* 走到区尾后回绕到区首 */

        remain = size - done;
        n = size - off;
        if (n > remain)
            n = remain;
        if (n > c->block)
            n = c->block;

        if (c->cfg.cpu_mode) {
            /* CPU 压力：同一方向直接打 BAR，一个方向一次 8 字节 store/load */
            uint8_t *init = c->dst.va + c->init_off + off;
            uint8_t *test = c->dst.va + c->test_off + off;

            if (do_rd) {
                copy_words(c->pat, init, n, 0);
                __sync_synchronize();
            }
            if (do_wr) {
                copy_words(test, c->pat, n, 0);
                __sync_synchronize();
            }
            done += n;
            continue;
        }

        if (do_rd) {
            rc = dma_read(c, c->dst.phys + c->init_off + off, n);
            if (rc != E_OK) {
                err_trip(c, -rc);       /* 打点永远在最前，之后才允许做别的 */
                if (c->cfg.verbose)
                    snap_regs(c);
                fi->code = -rc;
                fi->bus = c->dst.phys + c->init_off + off;
                fi->bar_off = c->init_off + off;
                fi->len = n;
                fi->seed = (uint32_t)c->cfg.seed;
                return rc;
            }
        }
        if (do_wr) {
            rc = dma_write(c, c->dst.phys + c->test_off + off, n);
            if (rc != E_OK) {
                err_trip(c, -rc);
                if (c->cfg.verbose)
                    snap_regs(c);
                fi->code = -rc;
                fi->bus = c->dst.phys + c->test_off + off;
                fi->bar_off = c->test_off + off;
                fi->len = n;
                fi->seed = (uint32_t)c->cfg.seed;
                return rc;
            }
        }
        done += n;
    }

    return E_OK;
}

/* 每块、每轮的种子都不同 -> 每轮数据唯一，能检出陈旧数据 */
static inline uint64_t iter_seed(const struct ctx *c, uint64_t iter, uint64_t off)
{
    return c->cfg.seed
         ^ (iter * 0x9E3779B97F4A7C15ull)
         ^ (off * 0xD1B54A32D192ED03ull);
}

/*
 * 一次读-改-写闭环，分三个阶段。CPU 侧不再插进 DMA 数据流中间：
 *
 *   阶段 1 (CPU) : 整个 init 区填本轮 pattern，整个 test 区污染成反码；
 *                  再回读 init 区，确认 CPU 写已落到 BAR（emu 上必需）。
 *   阶段 2 (DMA) : 按 --chunk 逐块 DMA 读 init 区 -> 内部内存，再写 -> test 区。
 *                  这一段只有 DMA，中间不插任何 CPU 访问，波形上是连续的一串。
 *   阶段 3 (CPU) : 回读整个 test 区，与 pattern 比对。
 *
 * 分块时"先整片准备、再整片搬运、最后整片校验"，而不是原来那样每块各跑一遍
 * 完整闭环（CPU 写 -> DMA -> CPU 读 反复穿插），DMA 事务在时间上才连得上。
 *
 * 为什么 DMA 阶段仍是"读一块、写一块"交替：ACS 内部内存的起始偏移由
 * --mem_off 固定（默认 0），后一笔 DMA 读会覆盖前一笔的内容，必须读完立刻
 * 写出。除非内部内存足够大、并让 mem_off 逐块递增。
 *
 * --chunk 未指定（或 >= -l）时三个阶段各只有一块，行为与逐块闭环完全一致。
 */
static int do_one_iter(struct ctx *c, uint64_t iter, struct fail_info *fi)
{
    uint64_t off;

    /* 纯压力模式：只搬数据，不校验、不初始化（数据在 run_test 前初始化一次） */
    if (c->cfg.stress)
        return stress_one_iter(c, iter, fi);

    if (c->cfg.cpu_mode)
        return cpu_one_iter(c, iter, fi);

    memset(fi, 0, sizeof(*fi));
    c->snap_valid = 0;
    c->last_status_valid = 0;
    c->init_cur_valid = 0;

    /* ---------- 阶段 1：CPU 把整片数据准备好 ---------- */
    /*
     * --no_init：不填 init 区、也不污染 test 区。数据由外部预先写好
     * （典型场景：对端卡的 DMA 刚写进来，或别的进程已经铺好），本轮只做
     * "DMA 读 -> DMA 写 -> 对拍"。此时没有预设 pattern 可比，所以基准改成
     * "CPU 在起 DMA 之前读到的 init 区实际内容"（存进 gold）。
     *
     * 注意 gold 必须在阶段 2 之前抓：DMA 写会把 test 区覆盖，但 init 区
     * 本身不动，所以 stage1 抓一次即可，整轮有效。
     */
    if (c->cfg.no_init) {
        for (off = 0; off < c->cfg.size; ) {
            uint64_t n = c->cfg.size - off;

            if (n > c->block)
                n = c->block;

            copy_words(c->gold + off, c->dst.va + c->init_off + off, n, 0);
            off += n;
        }
        __sync_synchronize();

        /*
         * 抓完基准后仍等一次：让"刚落盘的 init 内容"对 DMA 路径也可见。
         * CPU 能读到不代表 DMA 能读到（emu 上两条路径的可见性可以不同），
         * 所以 --init_delay_ms 在这里依然是唯一的补偿手段。
         */
        if (c->cfg.init_delay_ms)
            delay_ms(c->cfg.init_delay_ms);
    } else {
        for (off = 0; off < c->cfg.size; ) {
            uint64_t n = c->cfg.size - off;
            uint64_t seed;

            if (n > c->block)
                n = c->block;

            seed = iter_seed(c, iter, off);

            /* 1) 初始化区：CPU 写入已知 pattern */
            pattern_fill(c->pat, n, seed, c->cfg.pat_mode, iter);
            __sync_synchronize();
            copy_words(c->dst.va + c->init_off + off, c->pat, n, 0);

            /* 2) 测试区：污染成 pattern 的反码（保证"没真正写进去"一定会被检出） */
            copy_words(c->dst.va + c->test_off + off, c->pat, n, 1);
            __sync_synchronize();

            off += n;
        }

        /*
         * 阶段 1.5：回读 init 区，确认 CPU 写的数据真正落到 BAR 之后才起 DMA。
         * emu 上 CPU 与 DMA 读写时间不一致，CPU 的 MMIO 写可能比 DMA 的读还慢——
         * 实测 inc 模式下 DMA 读到的是上一轮的值，init/test 两个区都停在 iter-1。
         * CPU 读自己的写要穿过 fabric（非 posted），回读一致即可证明"写已落"；
         * 不一致就按 init_delay_ms（未配置则 1ms）等待重试，最多 1000 次。
         * 注意回读只保证 CPU 视角落盘，DMA 路径的可见性仍可能滞后，所以
         * init_delay_ms 的盲等仍有意义。
         */
        for (off = 0; off < c->cfg.size; ) {
            uint64_t n = c->cfg.size - off;
            uint64_t seed;
            int      tries = 0;

            if (n > c->block)
                n = c->block;

            seed = iter_seed(c, iter, off);
            pattern_fill(c->pat, n, seed, c->cfg.pat_mode, iter);

            while (buf_compare(c->dst.va + c->init_off + off, c->pat, n,
                               &fi->bad_off, &fi->actual, &fi->expect) != 0) {
                if (++tries >= 1000 || g_stop) {
                    err_trip(c, E_INIT_CHECK);  /* 打点永远在最前 */
                    if (c->cfg.verbose)
                        snap_regs(c);
                    fi->code = E_INIT_CHECK;
                    fi->bus = c->dst.phys + c->init_off + off + fi->bad_off;
                    fi->bar_off = c->init_off + off + fi->bad_off;
                    fi->len = n;
                    fi->seed = (uint32_t)seed;
                    return -E_INIT_CHECK;
                }
                delay_ms(c->cfg.init_delay_ms ? c->cfg.init_delay_ms : 1);
            }

            off += n;
        }

        /* 整片准备好后等一次，让 CPU 侧的写在 DMA 之前全部落地 */
        if (c->cfg.init_delay_ms)
            delay_ms(c->cfg.init_delay_ms);
    }

    /* ---------- 阶段 2：DMA 连续跑完整片 ---------- */
    for (off = 0; off < c->cfg.size; ) {
        uint64_t n = c->cfg.size - off;
        uint64_t seed;
        int      rc;

        if (n > c->block)
            n = c->block;

        seed = iter_seed(c, iter, off);

        /* 3) DMA 读：初始化区 -> DMA 内部内存 */
        rc = dma_read(c, c->dst.phys + c->init_off + off, n);
        if (rc != E_OK) {
            err_trip(c, -rc);           /* 第一时间打点，之后才允许做别的 */
            if (c->cfg.verbose)
                snap_regs(c);
            fi->code = -rc;
            fi->bus = c->dst.phys + c->init_off + off;
            fi->bar_off = c->init_off + off;
            fi->len = n;
            fi->seed = (uint32_t)seed;
            return rc;
        }

        /* 4) DMA 写：DMA 内部内存 -> 测试区 */
        rc = dma_write(c, c->dst.phys + c->test_off + off, n);
        if (rc != E_OK) {
            err_trip(c, -rc);
            if (c->cfg.verbose)
                snap_regs(c);
            fi->code = -rc;
            fi->bus = c->dst.phys + c->test_off + off;
            fi->bar_off = c->test_off + off;
            fi->len = n;
            fi->seed = (uint32_t)seed;
            return rc;
        }

        off += n;
    }

    /* ---------- 阶段 3：CPU 回读整片 test 区 ---------- */
    /* emu 上 P2P 写可能比 CPU 慢：DMA 状态位已报完成，但写数据穿过 fabric
     * 落到 BAR 有延迟，马上回读会拿到旧值造成假性 mismatch。整片等一次。 */
    if (c->cfg.check_delay_ms)
        delay_ms(c->cfg.check_delay_ms);

    for (off = 0; off < c->cfg.size; ) {
        uint64_t    n = c->cfg.size - off;
        uint64_t    seed;
        const uint8_t *exp;

        if (n > c->block)
            n = c->block;

        seed = iter_seed(c, iter, off);

        /*
         * 比对基准：
         *   正常       = 本轮 pattern（CPU 亲手写的，DMA 原样搬回来才对）
         *   --no_init = 阶段 1 抓的 init 区实测快照（gold）
         */
        if (c->cfg.no_init) {
            exp = c->gold + off;
        } else {
            pattern_fill(c->pat, n, seed, c->cfg.pat_mode, iter);
            exp = c->pat;
        }

        if (buf_compare(c->dst.va + c->test_off + off, exp, n,
                        &fi->bad_off, &fi->actual, &fi->expect) != 0) {
            err_trip(c, E_MISMATCH);    /* 校验一失败就写错误码，不做任何打印 */
            if (c->cfg.verbose)
                snap_regs(c);
            /* 打点之后抓 init 区同位置的值：区分"CPU 写没落到 BAR"
             * （init != 期望）和"CPU 写已落、DMA 读侧拿到旧值"（init == 期望） */
            c->init_cur = mmio_rd64(c->dst.va,
                                    (uint32_t)(c->init_off + off +
                                               (fi->bad_off & ~0x7ull)));
            c->init_cur_valid = 1;
            fi->code = E_MISMATCH;
            fi->bus = c->dst.phys + c->test_off + off + fi->bad_off;
            fi->bar_off = c->test_off + off + fi->bad_off;
            fi->len = n;
            fi->seed = (uint32_t)seed;
            return -E_MISMATCH;
        }

        off += n;
    }

    return E_OK;
}

/* ============================== 主循环 ============================== */

static void dump_regs(struct ctx *c, const char *tag)
{
    if (!c->src.va)
        return;

    printf("  [%s] CTRL=0x%08x OFF=0x%08x LO=0x%08x HI=0x%08x LEN=0x%08x ST=0x%08x\n",
           tag,
           reg_rd32(c->src.va, DMA_REG_CTRL),
           reg_rd32(c->src.va, DMA_REG_MEM_OFF),
           reg_rd32(c->src.va, DMA_REG_ADDR_LO),
           reg_rd32(c->src.va, DMA_REG_ADDR_HI),
           reg_rd32(c->src.va, DMA_REG_LEN),
           reg_rd32(c->src.va, DMA_REG_STATUS));
}

/*
 * 日志里标识"谁发的激励"。
 *   --owner 指定了名称 -> "名称(源bdf)"；只有 bdf -> bdf；CPU 模式 -> "cpu"。
 * 多实例共用一个终端/日志文件时，PASS/FAIL 行靠它区分是哪个源头。
 */
static void fmt_owner(const struct ctx *c, char *buf, size_t n)
{
    const char *own = c->cfg.owner[0] ? c->cfg.owner : NULL;
    const char *src = c->src.bdf[0] ? c->src.bdf : NULL;

    if (own && src)
        snprintf(buf, n, "%s(%s)", own, src);
    else if (own)
        snprintf(buf, n, "%s", own);
    else if (src)
        snprintf(buf, n, "%s", src);
    else
        snprintf(buf, n, "-");
}

/*
 * 打点地址落在目的 bar 的哪个偏移（只用于打印）。
 * 正常情况下就是超级头里的 err_flag；--no_alloc 没有头，返回 NULL。
 */
static void fmt_trip_addr(const struct ctx *c, const struct bar **pb, uint64_t *poff)
{
    const uint8_t *p = (const uint8_t *)c->trip;

    *pb = NULL;
    *poff = 0;

    if (!p)
        return;

    if (c->dst.va && p >= c->dst.va && p < c->dst.va + c->dst.size) {
        *pb = &c->dst;
        *poff = (uint64_t)(p - c->dst.va);
    }
}

/*
 * 失败信息的统一输出。
 * 调用约定：错误码已由 err_trip 打进 bar，完整详情已由 err_record_detail 落盘，
 * 之后才调用本函数。所以这里做多少格式化、打印、刷新都不影响 trigger 的抓点。
 */
static void report_fail(struct ctx *c, uint64_t iter, uint64_t pass, uint64_t fail_cnt,
                        const struct fail_info *fi)
{
    char own[96];

    fmt_owner(c, own, sizeof(own));

    printf("[%8" PRIu64 "] FAIL iter=%" PRIu64 " owner=%s code=%s code_num=%d"
           " (pass=%" PRIu64 " fail=%" PRIu64 ")\n",
           iter + 1, iter, own, err_name(fi->code), fi->code, pass, fail_cnt);

    if (fi->code == E_MISMATCH)
        printf("           首个不符: bar 偏移 0x%" PRIx64 " (bus 0x%016" PRIx64 ")"
               "  期望 0x%016" PRIx64 "  实际 0x%016" PRIx64
               "  长度 0x%" PRIx64 "  种子 0x%08x\n",
               fi->bar_off, fi->bus, fi->expect, fi->actual, fi->len, fi->seed);

    if (fi->code == E_INIT_CHECK)
        printf("           初始化区回读不符: bar 偏移 0x%" PRIx64 " (bus 0x%016" PRIx64 ")"
               "  期望 0x%016" PRIx64 "  读回 0x%016" PRIx64
               "  重试 %u ms x1000 次仍未落盘\n",
               fi->bar_off, fi->bus, fi->expect, fi->actual,
               c->cfg.init_delay_ms ? c->cfg.init_delay_ms : 1);

    if (c->init_cur_valid)
        printf("           init 区当前值: 0x%016" PRIx64 "%s\n",
               c->init_cur,
               c->init_cur == fi->expect
                   ? "  (== 期望，CPU 写已落盘，DMA 读侧拿到了旧值)"
                   : "  (!= 期望，失败时 CPU 写尚未落到 BAR)");

    {
        const struct bar *tb;
        uint64_t         toff;

        fmt_trip_addr(c, &tb, &toff);
        if (tb)
            printf("           打点(trigger): %s bar%d + 0x%" PRIx64 " = %d"
                   "（超级头 err_flag，4 字节整字写）\n",
                   tb->bdf, tb->num, toff, fi->code);
        else
            printf("           打点(trigger): 不可用（%s）\n",
                   c->cfg.no_alloc ? "--no_alloc 无测试头" : "目的 bar 未映射");
    }

    printf("           详情已存本地: %s（--dump_err 查看）\n", c->el_path);

    if (c->last_status_valid)
        printf("           DMA STATUS : 0x%08x\n", c->last_status);

    if (c->snap_valid)
        printf("           寄存器快照 : CTRL=0x%08x OFF=0x%08x LO=0x%08x"
               " HI=0x%08x LEN=0x%08x ST=0x%08x\n",
               c->snap[0], c->snap[1], c->snap[2], c->snap[3], c->snap[4], c->snap[5]);
}

static int run_test(struct ctx *c)
{
    uint64_t iter = 0;
    uint64_t pass = 0;
    uint64_t fail = 0;
    uint64_t bytes_ok = 0;              /* 累计成功搬运的字节 */
    uint64_t bytes_iter;                /* 每轮搬运的字节数（方向不同而不同） */
    const char *flow;                   /* 带宽口径，打印用 */
    uint64_t t_start = now_ns();
    uint64_t t_end;
    char     sz[32];
    int      rc_last = E_OK;

    fmt_size(sz, sizeof(sz), c->cfg.size);

    /* 压力模式只搬一个方向就只算一份字节；闭环测试和 both 都算读+写两份 */
    bytes_iter = c->cfg.size * 2ull;
    flow = "读+写";
    if (c->cfg.stress == STRESS_RD) {
        bytes_iter = c->cfg.size;
        flow = "只读";
    } else if (c->cfg.stress == STRESS_WR) {
        bytes_iter = c->cfg.size;
        flow = "只写";
    }

    /* 压力模式的数据只在启动时初始化一次；失败同样走"打点 -> 详情 -> 打印" */
    if (c->cfg.stress) {
        struct fail_info pfi;
        int              prc = stress_prepare(c, &pfi);

        if (prc != E_OK) {
            err_record_detail(c, 0, 0, &pfi);
            node_mark_error(c, pfi.code, 0);
            report_fail(c, 0, 0, 1, &pfi);
            fflush(stdout);
            node_mark_progress(c, 0);
            return -prc;
        }
        t_start = now_ns();             /* 准备耗时不计入带宽 */
    }

    while (!g_stop) {
        struct fail_info fi;
        uint64_t         t0, t1;
        int              rc;

        if (c->cfg.times != 0 && iter >= c->cfg.times)
            break;

        t0 = now_ns();
        rc = do_one_iter(c, iter, &fi);
        t1 = now_ns();

        if (rc != E_OK) {
            fail++;
            rc_last = -rc;

            /* 顺序就是全部重点：
             *   1) 错误码已在检测点（err_trip）写进 bar —— 波形上已经抓到；
             *   2) 这里补全详情字段，仍然不发任何打印；
             *   3) 记录全部落盘后，才输出人类可读的信息。
             */
            err_record_detail(c, iter, pass, &fi);
            node_mark_error(c, fi.code, iter);

            report_fail(c, iter, pass, fail, &fi);
            fflush(stdout);

            if (c->cfg.pause_on_err) {
                printf("  --pause_on_err: 进程已 SIGSTOP（pid=%d），"
                       "mmap 与 BAR 现场保持不变。\n"
                       "  检查/抓波形完成后: kill -CONT %d 恢复（随后按 stop_on_err 正常收尾）。\n",
                       (int)getpid(), (int)getpid());
                fflush(stdout);
                raise(SIGSTOP);
                printf("  --pause_on_err: 已恢复。\n");
            }

            if (c->cfg.stop_on_err)
                break;
        } else {
            pass++;
            rc_last = E_OK;
            bytes_ok += bytes_iter;

            if (c->cfg.dbg_times != 0 &&
                (iter % c->cfg.dbg_times == 0 || iter + 1 == c->cfg.times)) {
                uint64_t dt = t1 - t0;
                uint64_t dt_all = t1 - t_start;
                double   ms = (double)dt / 1e6;
                double   mib_iter;
                double   mib_all;
                char     r_iter[32], r_all[32], own[96];

                /* 单次耗时可能被时钟分辨率吃掉（emu 上常见 t1 == t0），
                 * 这时用累计平均兜底，而不是把带宽算成 0 */
                if (dt > 0)
                    mib_iter = ((double)bytes_iter / (1024.0 * 1024.0))
                             / ((double)dt / 1e9);
                else
                    mib_iter = 0.0;

                mib_all = dt_all > 0
                        ? ((double)bytes_ok / (1024.0 * 1024.0)) / ((double)dt_all / 1e9)
                        : 0.0;

                if (!(mib_iter > 0.0))
                    mib_iter = mib_all;

                fmt_rate(r_iter, sizeof(r_iter), mib_iter);
                fmt_rate(r_all, sizeof(r_all), mib_all);
                fmt_owner(c, own, sizeof(own));   /* 谁发的激励：owner(源bdf) */

                printf("[%8" PRIu64 "] PASS iter=%" PRIu64 " owner=%-24s size=%s"
                       "  %9.3f ms  %12s  avg %12s  (pass=%" PRIu64 " fail=%" PRIu64 ")\n",
                       iter + 1, iter, own, sz, ms, r_iter, r_all, pass, fail);
                node_mark_progress(c, pass);
                fflush(stdout);
            }
        }

        iter++;
    }

    t_end = now_ns();
    node_mark_progress(c, pass);

    printf("\n==================== 测试结束 ====================\n");
    {
        char own[96];

        fmt_owner(c, own, sizeof(own));
        printf("  激励源头    : %s\n", own);
    }
    printf("  迭代总数    : %" PRIu64 "\n", iter);
    printf("  通过 / 失败 : %" PRIu64 " / %" PRIu64 "\n", pass, fail);
    printf("  单区大小    : %s (init@0x%" PRIx64 ", test@0x%" PRIx64 ", block=%" PRIu64 ")\n",
           sz, c->init_off, c->test_off, c->block);
    if (c->cfg.stress)
        printf("  测试模式    : 纯压力 %s（不校验，数据只初始化一次）\n", flow);
    if (t_end > t_start) {
        double sec = (double)(t_end - t_start) / 1e9;
        double mibps = sec > 0.0
                     ? ((double)bytes_ok / (1024.0 * 1024.0)) / sec
                     : 0.0;
        char   rate[32];

        fmt_rate(rate, sizeof(rate), mibps);
        printf("  运行时间    : %.3f s\n", sec);
        printf("  平均带宽    : %s (%s，累计 %" PRIu64 " 字节)\n", rate, flow, bytes_ok);
    }
    if (g_stop)
        printf("  备注        : 收到中断信号，提前退出\n");
    printf("=================================================\n");

    if (fail == 0)
        return E_OK;
    return rc_last;
}

/* ============================== 参数解析 ============================== */

static void usage(const char *prog)
{
    printf(
        "用法: %s -s <bdfdma> -d <bdfdst> -l <test_size> [-n <test_times>] [选项]\n"
        "      %s -d <bdfdst> -l <test_size> --cpu        # CPU 直读直写，不用 DMA 控制器\n"
        "      %s -s <bdfdma> -d <bdfdst> -l <size> -n 0 --stress rd   # 只读压力，不校验\n"
        "\n"
        "必选/基础参数:\n"
        "  -s, --src_bdf <bdf>      DMA 控制器所在设备 (bdfdma)，使用其 bar0 作为控制寄存器基址\n"
        "                           （--cpu 模式下可省略）\n"
        "  -d, --dst_bdf <bdf>      被测目标设备 (bdfdst)，DMA 数据落盘到它的 --dst_bar\n"
        "  -l, --size <size>        每个区(初始化区/测试区)的字节数，支持 1M/0x100000/1048576\n"
        "  -n, --times <n>          循环次数，0 = 无限循环（默认 1）\n"
        "\n"
        "测试方式:\n"
        "      --cpu                不用 DMA 控制器，由 CPU 直接读写目的 bar 做同样的闭环测试。\n"
        "                           用于构造“DMA 与 CPU 同时访问同一个设备”的场景：\n"
        "                           另一个进程照常跑 DMA 模式，本进程 --cpu 打同一个 bar；\n"
        "                           要打到完全相同的地址（真同址并发）时两边都加 --no_alloc。\n"
        "      --stress <rd|wr|both>\n"
        "                           纯压力模式：只按方向搬数据，不做任何校验，数据只在启动时\n"
        "                           初始化一次，之后每轮不再改写（去掉 CPU 侧开销，压链路本身）。\n"
        "                           rd   = 只读：初始化区 -> DMA 内部内存\n"
        "                           wr   = 只写：DMA 内部内存 -> 测试区（内部内存先由一次 DMA 读填入）\n"
        "                           both = 每轮先读一遍再写一遍\n"
        "                           与 --cpu 合用时可压 CPU 直读直写通路；失败仍按错误码打点。\n"
        "      --addr_pat <none|seq|rand>\n"
        "                           压力模式的地址 pattern（默认 seq）：每轮的起始地址怎么变。\n"
        "                           none = 固定从区首开始（老行为；size 小时一直打同一地址）\n"
        "                           seq  = 每轮起点前进 --addr_step，到区尾回绕\n"
        "                           rand = 起点由 seed/iter 伪随机决定（可复现）\n"
        "                           每轮仍覆盖整个区，只是起点不同；+ 后面仍接整区长度。\n"
        "      --addr_step <bytes>  seq 的步长（默认 0 = 自动：区 > 4 KiB 按 4 KiB，否则按 8 B）\n"
        "\n"
        "配置参数:\n"
        "      --dbg_times <n>      每测试 n 次打印一次成功状态，0 = 不打印（默认 100）\n"
        "      --dst_bar <n>        被测 bdf 的 bar number（默认 4）\n"
        "      --dst_off <off>      目的 bar 内测试空间的起始偏移（默认 0，必须 4 KiB 对齐）。\n"
        "                           被测设备只在 bar 内某一段才是可测空间时用它，\n"
        "                           测试头/占用链/init/test 区整体从 off 开始排布，\n"
        "                           off 之前的内容程序不会读写；off 之后一直到 bar 末尾\n"
        "                           都可分配（不再有长度参数，多实例天然同一口径）。\n"
        "      --src_bar <n>        DMA 控制寄存器所在 bar（默认 0）\n"
        "      --mem_off <off>      ACS Exerciser 内存偏移（默认 0，需求约定固定 0）\n"
        "\n"
        "错误打点（唯一一处写，trigger 就设这里）:\n"
        "      失败瞬间只写 dst bar + --dst_off + 0x08（超级头 err_flag，\n"
        "      4 字节对齐的一次 32 位 store），没有第二处拷贝、没有 fence/打印。\n"
        "      迭代号/地址/期望实际值/寄存器快照等详情写本地 DDR，不占 bar 空间。\n"
        "      --err_log <path>     本地错误记录文件（默认 /tmp/dma_acs_err_<bdf>_bar<N>_<owner>.rec）\n"
        "\n"
        "目的 bar 测试头 / 占用链:\n"
        "  -c, --clear              启动时清空目的 bar（测试头 + 链上分配过的数据区，到链尾为止），\n"
        "                           然后正常测试\n"
        "      --clear_only         只清空后退出（不测试）\n"
        "  -L, --list               只打印目的 bar 的占用链后退出\n"
        "      --owner <name>       本实例在链上/日志里的名称（默认取 bdfdma）；\n"
        "                           PASS/FAIL 行显示为 name(源bdf)，多实例混在一个终端时靠它区分\n"
        "      --no_alloc           不用测试头分配，退回固定布局 init@dst_off / test@dst_off+ALIGN(size)\n"
        "\n"
        "其他:\n"
        "      --chunk <bytes>      单次 DMA 最大长度，0 = 整区一次传输（默认 0）\n"
        "                           分块后一轮拆成三段跑，CPU 不再插进 DMA 数据流中间：\n"
        "                           ① CPU 准备整片（填 init、污染 test、回读确认落盘）\n"
        "                           ② DMA 按块连续搬完整片（每块读完立刻写出，内部内存\n"
        "                              偏移固定 --mem_off，不立刻写会被下一笔覆盖）\n"
        "                           ③ CPU 回读整片 test 区比对\n"
        "      --timeout_ms <ms>    单次 DMA 等待完成超时（默认 1000）\n"
        "      --check_delay_ms <ms> DMA 报完成后、CPU 回读校验前的等待毫秒数（默认 0）\n"
        "                           emu 上 P2P 写比 CPU 慢时用它避免假性 mismatch\n"
        "      --init_delay_ms <ms>  CPU 写完初始化数据后、DMA 开始读之前的等待毫秒数（默认 0）\n"
        "                           同时作为初始化区回读校验的重试间隔（未配置则 1ms）\n"
        "      --pat <rand|inc|alt>  数据模式（默认 rand）:\n"
        "                           rand = splitmix64 随机种子流；inc = 每轮整体 +1（qword 递增）；\n"
        "                           alt  = 0x5A5A... 与 0xA5A5... 逐轮翻转。inc/alt 不用随机种子\n"
        "      --dst_phys <addr>    覆盖从 sysfs 读到的目标地址（有 IOMMU/非 1:1 映射时用）\n"
        "      --seed <n>           pattern 种子（默认 0x5AA5A55A）\n"
        "      --ignore_status      不检查 DMA Status 寄存器\n"
        "      --no_stop_on_err     失败后继续跑（默认首个错误即退出）\n"
        "      --pause_on_err       失败打印后 SIGSTOP 进程保住现场，检查完 kill -CONT 恢复\n"
        "  -v, --verbose            启动时 dump 一次控制寄存器\n"
        "      --probe              只打印两个设备的 BAR 资源表后退出\n"
        "      --dump_err           读回本地 DDR 的错误记录文件并解析后退出\n"
        "      --peek <off|auto>    事后取证：重新 mmap 目的 bar，hex dump 出错地址附近内容后退出\n"
        "                           off 是 bar 内绝对偏移（含 --dst_off）；\n"
        "                           auto = 从本地记录里取 bar 偏移，并自动与记录的期望/实际值对照\n"
        "      --peek_len <n>       --peek 的回读长度（默认 64 字节）\n"
        "      --selftest           只跑 CPU 侧数据通路自检，并打印 CPU 特性。\n"
        "                           不带 -d 时是纯内存自检（不碰硬件）；\n"
        "                           带 -d 时改为 CPU 直接读写该设备的 bar，位置由\n"
        "                           --st_bar / --st_off / -l 指定，方向由 --st_op 指定。\n"
        "      --st_op <rw|rd|wr>   --selftest 打 bar 时的方向（默认 rw）:\n"
        "                           rw = 写 pattern -> 回读比对 -> 写反码确认确实可写\n"
        "                           rd = 只读：dump 内容后退出，一个字节都不写\n"
        "                           wr = 只写：按 pattern 反复覆盖，不回读比对\n"
        "      --st_bar <n>         --selftest 用的 bar 号（默认跟随 --dst_bar）\n"
        "      --st_off <off>       --selftest 区间的 bar 内起始偏移（默认 0，须 8 字节对齐）\n"
        "      --no_init            闭环测试跳过\"CPU 填 init 区 + 污染 test 区\"这一步。\n"
        "                           init 区内容由外部预置（典型：对端卡的 DMA 刚写进来，\n"
        "                           或另一个进程已经铺好），本轮只做 DMA 读 -> DMA 写。\n"
        "                           此时没有预设 pattern，比对基准改成\"起 DMA 之前 CPU\n"
        "                           读到的 init 区实际内容\"，仍能验证搬运是否无损。\n"
        "                           与 --stress 互斥（压力模式本来就不改写数据）。\n"
        "  -h, --help               显示本帮助\n"
        "\n"
        "说明:\n"
        "  1) 程序默认执行“读-改-写”闭环测试：初始化区 -> DMA 内部内存 -> 测试区 -> CPU 回读比对。\n"
        "     --stress 改为纯压力：每个方向只搬数据、不比对，数据只初始化一次；\n"
        "     此时失败只可能来自 DMA 硬件/总线（超时、Status 非 0），不会出现 mismatch。\n"
        "  2) 目的 bar 由“测试头 + 占用链”管理：从 --dst_off 起算，+0x0 是超级头(0x100)，\n"
        "     +0x100 起每个源头的节点（头 0x100 + init/test 数据）按申请顺序串成链，\n"
        "     新实例自动接在链尾之后的空闲空间，最多 72 个源头（满了报 E_FULL）。\n"
        "     数据区只保证 8 字节对齐，不再按页对齐，省 bar 空间；\n"
        "     不做回收，空间不够时用 --clear 整体复位。\n"
        "     同一块 bar 上多实例必须用相同的 --dst_off，否则各自管各自的段，互相看不见。\n"
        "     可用空间统一算到 bar 末尾，没有“窗口长度”这类要各进程自己声明的量，\n"
        "     所以多打一时不会出现各自算出不同边界、把别人的节点判成链外的情况。\n"
        "  3) --clear 只清占用链上分配过的数据区（到链尾为止）+ 测试头，不扫整条 bar，\n"
        "     因此不会碰到 --dst_off 之后设备自己的其它功能区。\n"
        "     --no_alloc 才回到固定布局（init@dst_off, test@dst_off+ALIGN(size)），不与测试头共存。\n"
        "     该模式没有超级头，也就没有 err_flag，失败不打点，现场只看本地记录文件。\n"
        "  4) 支持多实例并行：不同 bdfdma 各起一个进程，目的 bar 可以相同（靠链分配）；\n"
        "     同一个 bdfdma 会被 flock 拦下，不会出现两个进程交叉编程同一组寄存器。\n"
        "     --cpu 实例不占控制器，可以和任意多个 DMA 实例同时打同一个 bar。\n"
        "  5) 失败时序（emu 抓波形）：判定失败后第一时间只写一个地方——\n"
        "     目的 bar 超级头的 err_flag（dst bar + --dst_off + 0x08），\n"
        "     一次 4 字节对齐的 32 位 store，无 fence/无读/无打印。\n"
        "     trigger 建议设为“写 dst bar + dst_off + 0x08 且数据非 0”，抓到的第一笔\n"
        "     写事务就是故障点。之后才把详情写本地 DDR（普通内存）、最后打印，\n"
        "     bar 上不会再有第二笔错误相关的写。\n"
        "     err_flag 是“该 bar 最近一次失败”的粘滞值，下次启动或 --clear 时归零；\n"
        "     每个源头自己节点里的 N_ERRFLAG 只标记自己，--list 可直接看。\n"
        "  6) 头里的字段全部 4 字节对齐、只做 32 位整字访问（64 位字段拆成两个字、\n"
        "     字符串按 4 字节打包），不产生 8/16/8 位或非对齐的头访问。\n"
        "  7) 需要 root 或对应 resource 节点读写权限。\n"
        "  8) --no_init 与 --selftest 的分工：前者让闭环测试不写 init 区（数据由外部预置），\n"
        "     后者让 CPU 单独直接读写某个 bar 的某一段、不涉及 DMA。\n"
        "     想先确认\"DMA 会读的那片 init 区里到底是什么\"，先用 --selftest --st_op rd 看一眼，\n"
        "     再用 --no_init 跑闭环；两者用同一组 --st_bar/--st_off/-l 对齐坐标。\n",
        prog, prog, prog);
}

int main(int argc, char **argv)
{
    struct ctx c;
    int        opt;
    int        idx = 0;
    int        rc;
    char       src_bdf[64] = "";
    char       dst_bdf[64] = "";
    struct sigaction sa;

    memset(&c, 0, sizeof(c));

    /* ---- 默认值 ---- */
    c.cfg.src_bar = 0;
    c.cfg.dst_bar = 4;
    c.cfg.dst_off = 0;             /* 目的 bar 内测试空间起点，0 = 从 bar 开头 */
    c.cfg.mem_off = DMA_MEM_OFFSET;
    c.cfg.times = 1;
    c.cfg.dbg_times = 100;
    c.cfg.chunk = 0;
    c.cfg.seed = 0x5AA5A55Aull;
    c.cfg.timeout_ms = 1000;
    c.cfg.check_delay_ms = 0;
    c.cfg.init_delay_ms = 0;
    c.cfg.pat_mode = PAT_RAND;
    c.cfg.stress = STRESS_OFF;
    c.cfg.addr_pat = ADDR_SEQ;     /* 压力模式默认每轮换起点，避免小 size 一直同地址 */
    c.cfg.addr_step = 0;           /* 0 = 自动（4 KiB 页滑 / 小区按 qword 滑） */
    c.cfg.stop_on_err = 1;
    c.cfg.check_status = 1;
    c.cfg.st_op = SELFTEST_RW;
    c.cfg.st_bar = -1;             /* -1 = 跟随 --dst_bar */
    c.cfg.st_off = 0;

    static const struct option long_opts[] = {
        { "src_bdf",        required_argument, NULL, 's' },
        { "dst_bdf",        required_argument, NULL, 'd' },
        { "size",           required_argument, NULL, 'l' },
        { "times",          required_argument, NULL, 'n' },
        { "dbg_times",      required_argument, NULL, 'D' },
        { "dbg-times",      required_argument, NULL, 'D' },
        { "dst_bar",        required_argument, NULL, 'B' },
        { "dst-bar",        required_argument, NULL, 'B' },
        { "dst_off",        required_argument, NULL, 'o' },
        { "dst-off",        required_argument, NULL, 'o' },
        { "src_bar",        required_argument, NULL, 'S' },
        { "src-bar",        required_argument, NULL, 'S' },
        { "mem_off",        required_argument, NULL, 'M' },
        { "mem-off",        required_argument, NULL, 'M' },
        { "err_log",        required_argument, NULL, 'V' },
        { "err-log",        required_argument, NULL, 'V' },
        { "chunk",          required_argument, NULL, 'C' },
        { "timeout_ms",     required_argument, NULL, 'T' },
        { "timeout-ms",     required_argument, NULL, 'T' },
        { "check_delay_ms", required_argument, NULL, 'U' },
        { "check-delay-ms", required_argument, NULL, 'U' },
        { "init_delay_ms",  required_argument, NULL, 'i' },
        { "init-delay-ms",  required_argument, NULL, 'i' },
        { "pat",            required_argument, NULL, 'j' },
        { "pattern",        required_argument, NULL, 'j' },
        { "dst_phys",       required_argument, NULL, 'P' },
        { "dst-phys",       required_argument, NULL, 'P' },
        { "seed",           required_argument, NULL, 'R' },
        { "ignore_status",  no_argument,       NULL, 'I' },
        { "ignore-status",  no_argument,       NULL, 'I' },
        { "no_stop_on_err", no_argument,       NULL, 'K' },
        { "no-stop-on-err", no_argument,       NULL, 'K' },
        { "pause_on_err",   no_argument,       NULL, 'k' },
        { "pause-on-err",   no_argument,       NULL, 'k' },
        { "verbose",        no_argument,       NULL, 'v' },
        { "probe",          no_argument,       NULL, 'Q' },
        { "dump_err",       no_argument,       NULL, 'Z' },
        { "dump-err",       no_argument,       NULL, 'Z' },
        { "peek",           required_argument, NULL, 'p' },
        { "peek_len",       required_argument, NULL, 'g' },
        { "peek-len",       required_argument, NULL, 'g' },
        { "selftest",       no_argument,       NULL, 'X' },
        { "self-test",      no_argument,       NULL, 'X' },
        { "no_init",        no_argument,       NULL, 'q' },
        { "no-init",        no_argument,       NULL, 'q' },
        { "st_op",          required_argument, NULL, 'y' },
        { "st-op",          required_argument, NULL, 'y' },
        { "st_bar",         required_argument, NULL, 'b' },
        { "st-bar",         required_argument, NULL, 'b' },
        { "st_off",         required_argument, NULL, 'f' },
        { "st-off",         required_argument, NULL, 'f' },
        { "clear",          no_argument,       NULL, 'c' },
        { "clear_only",     no_argument,       NULL, 'L' },
        { "clear-only",     no_argument,       NULL, 'L' },
        { "list",           no_argument,       NULL, 'N' },
        { "owner",          required_argument, NULL, 'W' },
        { "no_alloc",       no_argument,       NULL, 'A' },
        { "no-alloc",       no_argument,       NULL, 'A' },
        { "cpu",            no_argument,       NULL, 'Y' },
        { "stress",         required_argument, NULL, 'F' },
        { "addr_pat",       required_argument, NULL, 'G' },
        { "addr-pat",       required_argument, NULL, 'G' },
        { "addr_step",      required_argument, NULL, 'J' },
        { "addr-step",      required_argument, NULL, 'J' },
        { "help",           no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 }
    };

    while ((opt = getopt_long(argc, argv, "s:d:l:n:hvcL", long_opts, &idx)) != -1) {
        switch (opt) {
        case 's': snprintf(c.cfg.src_bdf, sizeof(c.cfg.src_bdf), "%s", optarg); break;
        case 'd': snprintf(c.cfg.dst_bdf, sizeof(c.cfg.dst_bdf), "%s", optarg); break;
        case 'l':
            if (parse_size(optarg, &c.cfg.size) != 0 || c.cfg.size == 0) {
                fprintf(stderr, "-l 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            break;
        case 'n':
            if (parse_size(optarg, &c.cfg.times) != 0) {
                fprintf(stderr, "-n 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            break;
        case 'D':
            if (parse_size(optarg, &c.cfg.dbg_times) != 0) {
                fprintf(stderr, "--dbg_times 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            break;
        case 'B': c.cfg.dst_bar = atoi(optarg); break;
        case 'o':
            if (parse_size(optarg, &c.cfg.dst_off) != 0) {
                fprintf(stderr, "--dst_off 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            break;
        case 'S': c.cfg.src_bar = atoi(optarg); break;
        case 'M':
            if (parse_size(optarg, &c.cfg.mem_off) != 0) {
                fprintf(stderr, "--mem_off 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            break;
        case 'V':
            snprintf(c.cfg.err_log, sizeof(c.cfg.err_log), "%s", optarg);
            break;
        case 'C':
            if (parse_size(optarg, &c.cfg.chunk) != 0) {
                fprintf(stderr, "--chunk 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            break;
        case 'T': {
            uint64_t v;

            if (parse_size(optarg, &v) != 0 || v > 0xFFFFFFFFull) {
                fprintf(stderr, "--timeout_ms 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            c.cfg.timeout_ms = (uint32_t)v;
            break;
        }
        case 'U': {
            uint64_t v;

            if (parse_size(optarg, &v) != 0 || v > 3600ull * 1000ull) {
                fprintf(stderr, "--check_delay_ms 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            c.cfg.check_delay_ms = (uint32_t)v;
            break;
        }
        case 'i': {
            uint64_t v;

            if (parse_size(optarg, &v) != 0 || v > 3600ull * 1000ull) {
                fprintf(stderr, "--init_delay_ms 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            c.cfg.init_delay_ms = (uint32_t)v;
            break;
        }
        case 'j':
            if (strcmp(optarg, "rand") == 0 || strcmp(optarg, "random") == 0)
                c.cfg.pat_mode = PAT_RAND;
            else if (strcmp(optarg, "inc") == 0)
                c.cfg.pat_mode = PAT_INC;
            else if (strcmp(optarg, "alt") == 0)
                c.cfg.pat_mode = PAT_ALT;
            else {
                fprintf(stderr, "--pat 参数非法: %s（rand / inc / alt）\n", optarg);
                return E_USAGE;
            }
            break;
        case 'P':
            if (parse_size(optarg, &c.cfg.dst_phys) != 0) {
                fprintf(stderr, "--dst_phys 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            break;
        case 'R':
            if (parse_size(optarg, &c.cfg.seed) != 0) {
                fprintf(stderr, "--seed 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            break;
        case 'I': c.cfg.check_status = 0; break;
        case 'K': c.cfg.stop_on_err = 0; break;
        case 'k': c.cfg.pause_on_err = 1; break;
        case 'v': c.cfg.verbose = 1; break;
        case 'Q': c.cfg.probe = 1; break;
        case 'Z': c.cfg.dump_err = 1; break;
        case 'p':
            c.cfg.peek = 1;
            if (strcmp(optarg, "auto") == 0 || strcmp(optarg, "err") == 0) {
                c.cfg.peek_auto = 1;
            } else if (parse_size(optarg, &c.cfg.peek_off) != 0) {
                fprintf(stderr, "--peek 参数非法: %s（offset 或 auto）\n", optarg);
                return E_USAGE;
            }
            break;
        case 'g':
            if (parse_size(optarg, &c.cfg.peek_len) != 0) {
                fprintf(stderr, "--peek_len 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            break;
        case 'X': c.cfg.selftest = 1; break;
        case 'q': c.cfg.no_init = 1; break;
        case 'y':
            if (strcmp(optarg, "rw") == 0 || strcmp(optarg, "both") == 0)
                c.cfg.st_op = SELFTEST_RW;
            else if (strcmp(optarg, "rd") == 0 || strcmp(optarg, "read") == 0)
                c.cfg.st_op = SELFTEST_RD;
            else if (strcmp(optarg, "wr") == 0 || strcmp(optarg, "write") == 0)
                c.cfg.st_op = SELFTEST_WR;
            else {
                fprintf(stderr, "--st_op 参数非法: %s（rw / rd / wr）\n", optarg);
                return E_USAGE;
            }
            break;
        case 'b':
            c.cfg.st_bar = atoi(optarg);
            if (c.cfg.st_bar < 0) {
                fprintf(stderr, "--st_bar 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            break;
        case 'f':
            if (parse_size(optarg, &c.cfg.st_off) != 0) {
                fprintf(stderr, "--st_off 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            break;
        case 'c': c.cfg.clear = 1; break;
        case 'L': c.cfg.clear_only = 1; break;
        case 'N': c.cfg.list = 1; break;
        case 'A': c.cfg.no_alloc = 1; break;
        case 'Y': c.cfg.cpu_mode = 1; break;
        case 'F':
            if (strcmp(optarg, "rd") == 0 || strcmp(optarg, "read") == 0)
                c.cfg.stress = STRESS_RD;
            else if (strcmp(optarg, "wr") == 0 || strcmp(optarg, "write") == 0)
                c.cfg.stress = STRESS_WR;
            else if (strcmp(optarg, "both") == 0 || strcmp(optarg, "rw") == 0)
                c.cfg.stress = STRESS_BOTH;
            else {
                fprintf(stderr, "--stress 参数非法: %s（rd / wr / both）\n", optarg);
                return E_USAGE;
            }
            break;
        case 'W':
            snprintf(c.cfg.owner, sizeof(c.cfg.owner), "%s", optarg);
            break;
        case 'G':
            if (strcmp(optarg, "none") == 0 || strcmp(optarg, "fix") == 0)
                c.cfg.addr_pat = ADDR_FIX;
            else if (strcmp(optarg, "seq") == 0)
                c.cfg.addr_pat = ADDR_SEQ;
            else if (strcmp(optarg, "rand") == 0)
                c.cfg.addr_pat = ADDR_RAND;
            else {
                fprintf(stderr, "--addr_pat 参数非法: %s（none / seq / rand）\n", optarg);
                return E_USAGE;
            }
            break;
        case 'J':
            if (parse_size(optarg, &c.cfg.addr_step) != 0) {
                fprintf(stderr, "--addr_step 参数非法: %s\n", optarg);
                return E_USAGE;
            }
            break;
        case 'h': usage(argv[0]); return E_OK;
        default:  usage(argv[0]); return E_USAGE;
        }
    }

    if (c.cfg.probe) {
        if (c.cfg.src_bdf[0]) {
            normalize_bdf(c.cfg.src_bdf, src_bdf, sizeof(src_bdf));
            print_bar_table(src_bdf);
        }
        if (c.cfg.dst_bdf[0]) {
            normalize_bdf(c.cfg.dst_bdf, dst_bdf, sizeof(dst_bdf));
            print_bar_table(dst_bdf);
        }
        return E_OK;
    }

    /*
     * 自检：带 -d 就打真 bar（--st_bar/--st_off/-l 指定位置），
     * 不带 -d 走老行为（纯内存，不碰硬件）。
     */
    if (c.cfg.selftest) {
        if (!c.cfg.dst_bdf[0])
            return run_selftest(&c);

        normalize_bdf(c.cfg.dst_bdf, dst_bdf, sizeof(dst_bdf));
        if (c.cfg.st_bar >= 0)
            c.cfg.dst_bar = c.cfg.st_bar;

        /* 只读模式不开写权限：映射只读，硬件侧就不可能被改动 */
        rc = open_bar(&c.dst, dst_bdf, c.cfg.dst_bar,
                      c.cfg.st_op != SELFTEST_RD, 0);
        if (rc != E_OK)
            return -rc;

        rc = run_selftest_bar(&c);
        close_bar(&c.dst);
        return rc == E_OK ? 0 : 1;
    }

    /* 只读回本地 DDR 里的错误记录（失败后单独使用，不需要碰 bar） */
    if (c.cfg.dump_err) {
        if (!c.cfg.dst_bdf[0] && !c.cfg.err_log[0]) {
            fprintf(stderr, "--dump_err 需要 -d <bdfdst>（用于定位记录文件）或 --err_log <path>\n");
            return E_USAGE;
        }
        if (c.cfg.dst_bdf[0]) {
            normalize_bdf(c.cfg.dst_bdf, dst_bdf, sizeof(dst_bdf));
            snprintf(c.dst.bdf, sizeof(c.dst.bdf), "%s", dst_bdf);
            c.dst.num = c.cfg.dst_bar;
        }

        rc = dump_err_record(&c);
        return rc == E_OK ? 0 : 1;
    }

    /* ---- --peek：程序报错退出后，重新 mmap 目的 bar 把出错地址读出来对照 ---- */
    if (c.cfg.peek) {
        struct err_local rec;
        int         have_rec = 0;
        uint64_t    off, len, cur;

        if (!c.cfg.dst_bdf[0]) {
            fprintf(stderr, "--peek 需要配合 -d <bdfdst>\n");
            return E_USAGE;
        }
        normalize_bdf(c.cfg.dst_bdf, dst_bdf, sizeof(dst_bdf));
        rc = open_bar(&c.dst, dst_bdf, c.cfg.dst_bar, 0, 0);
        if (rc != E_OK)
            return -rc;

        /* 详细现场在本地记录文件里，不需要再碰 err bar */
        if (err_local_read(&c, &rec) == 0 && rec.magic == ERL_MAGIC)
            have_rec = 1;

        if (c.cfg.peek_auto) {
            if (!have_rec) {
                fprintf(stderr,
                        "--peek auto：本地记录里没有有效记录（magic 不符），无法定位出错地址\n");
                close_bar(&c.dst);
                return 1;
            }
            off = rec.bar_off;
        } else {
            off = c.cfg.peek_off;
        }

        len = c.cfg.peek_len ? c.cfg.peek_len : 64;
        if (off & 0x7ull) {
            printf("提示: 起始偏移 0x%" PRIx64 " 已向下对齐到 8 字节\n", off);
            off &= ~0x7ull;
        }
        if (off >= c.dst.size) {
            fprintf(stderr, "--peek 偏移 0x%" PRIx64 " 超出 %s bar%d 大小 0x%" PRIx64 "\n",
                    off, c.dst.bdf, c.dst.num, c.dst.size);
            close_bar(&c.dst);
            return 1;
        }
        if (off + len > c.dst.size)
            len = c.dst.size - off;

        printf("====== BAR 回读: %s bar%d + 0x%" PRIx64 "  (%" PRIu64 " 字节) ======\n",
               c.dst.bdf, c.dst.num, off, len);
        hexdump_bar(&c.dst, off, len);

        if (have_rec) {
            cur = mmio_rd64(c.dst.va, (uint32_t)(rec.bar_off & ~0x7ull));

            printf("------ 与本地错误记录对照 ------\n");
            printf("  错误码       : %u (%s)\n", rec.code, err_name((int)rec.code));
            printf("  出错 bar 偏移: 0x%" PRIx64 " (block 内首个不符 0x%" PRIx64 ")\n",
                   rec.bar_off, rec.bad_off);
            printf("  记录的期望值 : 0x%016" PRIx64 "\n", rec.expect);
            printf("  记录的实际值 : 0x%016" PRIx64 "\n", rec.actual);
            printf("  当前读取值   : 0x%016" PRIx64 "\n", cur);
            if (cur == rec.expect)
                printf("  结论         : 当前值 == 期望值 → 写数据已落盘，"
                       "失败时读到旧值是延迟所致（假性 mismatch）\n");
            else if (cur == rec.actual)
                printf("  结论         : 当前值 == 当时实际值 → 数据至今未变，"
                       "不是单纯延迟，需要继续查\n");
            else
                printf("  结论         : 当前值既非期望也非当时实际值，"
                       "该地址可能已被其它流量改写\n");
            printf("  注意         : 若还有其它实例在同一 bar 上跑，事后读到的可能不是当时的现场\n");
        } else {
            printf("（本地记录无效，仅原始 dump；可与测试报告里打印的期望/实际值对照）\n");
        }

        close_bar(&c.dst);
        return 0;
    }

    if (!c.cfg.dst_bdf[0]) {
        fprintf(stderr, "必须指定 -d <bdfdst>\n\n");
        usage(argv[0]);
        return E_USAGE;
    }
    /* --list / --clear_only 只跟目的 bar 打交道，不需要 -s 和 -l */
    if (!c.cfg.clear_only && !c.cfg.list) {
        /* --cpu 模式不碰 DMA 控制器，-s 不是必选项 */
        if (!c.cfg.src_bdf[0] && !c.cfg.cpu_mode) {
            fprintf(stderr, "必须指定 -s <bdfdma>（或加 --cpu 走 CPU 直读直写）\n\n");
            usage(argv[0]);
            return E_USAGE;
        }
        if (c.cfg.size == 0) {
            fprintf(stderr, "必须指定 -l <test_size>\n\n");
            usage(argv[0]);
            return E_USAGE;
        }
    }
    if (c.cfg.dst_bar < 0 || c.cfg.src_bar < 0) {
        fprintf(stderr, "bar number 非法\n");
        return E_USAGE;
    }
    if (c.cfg.size > 0xFFFFFFFFull) {
        fprintf(stderr, "-l 不能超过 4 GiB（DMA Length 是 32 位）\n");
        return E_USAGE;
    }

    /* --no_init 与纯压力互斥：压力模式本来就不填 init 区，语义重复容易误解 */
    if (c.cfg.no_init && c.cfg.stress) {
        fprintf(stderr, "--no_init 不能与 --stress 同时使用"
                        "（压力模式本来就不改写数据）\n");
        return E_USAGE;
    }

    if (c.cfg.cpu_mode) {
        /* CPU 模式不需要 DMA 控制器：把 src 描述成一个虚拟源头，
         * 这样日志、占用链里的"源头"字段仍然有内容，也不用到处判空 */
        snprintf(c.src.bdf, sizeof(c.src.bdf), "cpu");
        c.src.num = -1;
        c.src.phys = 0;
        c.src.size = 0;
        c.src.va = NULL;
        if (c.cfg.src_bdf[0])
            printf("提示: --cpu 已开启，忽略 -s %s\n", c.cfg.src_bdf);
        c.cfg.src_bdf[0] = '\0';
    } else {
        normalize_bdf(c.cfg.src_bdf, src_bdf, sizeof(src_bdf));
    }
    normalize_bdf(c.cfg.dst_bdf, dst_bdf, sizeof(dst_bdf));

    /* ---- 打开目的 bar（允许与其它源头共享，靠占用链协调） ---- */
    rc = open_bar(&c.dst, dst_bdf, c.cfg.dst_bar, 1, 0);
    if (rc != E_OK)
        return -rc;

    if (c.cfg.dst_phys)
        c.dst.phys = c.cfg.dst_phys;

    /* --dst_off 先校验：--list / --clear 也要落在同一段空间上 */
    rc = check_dst_off(&c);
    if (rc != E_OK) {
        close_bar(&c.dst);
        return -rc;
    }

    /* ---- 测试头的清空 / 列举（这两个动作不需要 DMA 控制器） ---- */
    if (c.cfg.clear || c.cfg.clear_only) {
        rc = hdr_init(&c, 1);
        if (rc != E_OK) {
            close_bar(&c.dst);
            return -rc;
        }
        if (c.cfg.clear_only) {
            printf("已清空目的 bar 的占用链与测试头（从 0x%" PRIx64 " 起），退出。\n",
                   c.cfg.dst_off);
            close_bar(&c.dst);
            return 0;
        }
    }

    if (c.cfg.list) {
        list_nodes(&c);
        close_bar(&c.dst);
        return 0;
    }

    /* ---- 本地 DDR 错误记录（mmap 到文件，进程退出后仍可 --dump_err） ---- */
    err_local_init(&c);

    /* ---- 打开 DMA 控制寄存器 bar（必须独占，同一个控制器不能给两个进程）
     *      --cpu 模式跳过：本进程根本不碰控制器 ---- */
    if (!c.cfg.cpu_mode) {
        rc = open_bar(&c.src, src_bdf, c.cfg.src_bar, 1, 2);
        if (rc != E_OK) {
            close_bar(&c.dst);
            return -rc;
        }
    }

    /* ---- 布局规划 ---- */
    rc = plan_layout(&c);
    if (rc != E_OK) {
        close_bar(&c.dst);
        close_bar(&c.src);
        return -rc;
    }

    /* ---- 在目的 bar 的占用链上申请一段空间（默认行为） ---- */
    if (!c.cfg.no_alloc) {
        struct node_info ni;

        rc = alloc_node(&c, &ni);
        if (rc != E_OK) {
            close_bar(&c.dst);
            close_bar(&c.src);
            return -rc;
        }

        c.init_off = ni.data_off;
        c.test_off = ni.test_off;
        c.node_off = ni.off;
        c.has_node = 1;

        printf("已申请节点 #%u：节点头@0x%" PRIx64 "，数据@0x%" PRIx64
               "（init 0x%" PRIx64 " / test 0x%" PRIx64 "，%s）\n",
               ni.seq, ni.off, ni.data_off, ni.data_off, ni.test_off,
               c.cfg.owner[0] ? c.cfg.owner : c.src.bdf);
    } else {
        printf("提示: --no_alloc 已关闭测试头分配，使用固定布局（不与占用链共存）\n");
    }

    c.pat = malloc((size_t)c.block);
    if (!c.pat) {
        fprintf(stderr, "分配 %" PRIu64 " 字节 pattern 缓冲失败\n", c.block);
        close_bar(&c.dst);
        close_bar(&c.src);
        return E_NOMEM;
    }

    /*
     * --no_init 需要一个整片大小的快照缓冲当比对基准（跨分块，不能按 block 分）。
     * --cpu 模式不用 gold：它的基准就是 bounce（读回来的中间副本），
     * 分配了也用不上，省一片内存。
     */
    if (c.cfg.no_init && !c.cfg.cpu_mode) {
        c.gold = malloc((size_t)c.cfg.size);
        if (!c.gold) {
            fprintf(stderr, "分配 %" PRIu64 " 字节 init 快照缓冲失败\n", c.cfg.size);
            free(c.pat);
            close_bar(&c.dst);
            close_bar(&c.src);
            return E_NOMEM;
        }
    }

    /* CPU 模式需要一个"中间内存"顶替 ACS 内部内存，完成 读->中间->写 的闭环 */
    if (c.cfg.cpu_mode) {
        c.bounce = malloc((size_t)c.block);
        if (!c.bounce) {
            fprintf(stderr, "分配 %" PRIu64 " 字节中间缓冲失败\n", c.block);
            free(c.pat);
            close_bar(&c.dst);
            close_bar(&c.src);
            return E_NOMEM;
        }
    }

    /* ---- 打印配置 ---- */
    {
        char sz[32], bar_sz[32], blk[32];

        fmt_size(sz, sizeof(sz), c.cfg.size);
        fmt_size(bar_sz, sizeof(bar_sz), c.dst.size);
        fmt_size(blk, sizeof(blk), c.block);

        printf("================= DMA ACS Exerciser 测试 =================\n");
        if (c.cfg.cpu_mode)
            printf("  访问方式     : CPU 直读直写（--cpu，不使用 DMA 控制器）\n");
        else
            printf("  DMA 控制器   : %s bar%d  phys=0x%016" PRIx64 "  (CTRL@0x08)\n",
                   c.src.bdf, c.src.num, c.src.phys);
        printf("  目标设备     : %s bar%d  phys=0x%016" PRIx64 "  size=%s\n",
               c.dst.bdf, c.dst.num, c.dst.phys, bar_sz);
        {
            char     win[32];
            uint64_t lim = data_limit(&c);
            uint64_t wlen = (lim > c.cfg.dst_off) ? (lim - c.cfg.dst_off) : 0;

            fmt_size(win, sizeof(win), wlen);
            printf("  测试空间     : bar+0x%" PRIx64 " ~ bar+0x%" PRIx64 " (%s)  [%s]\n",
                   c.cfg.dst_off, lim, win,
                   c.cfg.no_alloc ? "固定布局，无测试头" : "占用链从 off 起往后分配，多实例共用");
            printf("                 此范围之外的字节程序不读写\n");
        }
        {
            const struct bar *tb;
            uint64_t         toff;

            fmt_trip_addr(&c, &tb, &toff);
            if (tb)
                printf("  打点(trigger): %s bar%d + 0x%" PRIx64
                       "（超级头 err_flag，唯一一处 4 字节整字写）\n",
                       tb->bdf, tb->num, toff);
            else
                printf("  打点(trigger): 不可用（%s）\n",
                       c.cfg.no_alloc ? "--no_alloc 无测试头，不打点" : "目的 bar 未映射");
        }
        printf("  错误详情     : 本地 DDR  %s\n", c.el_path);
        printf("  单区大小     : %s   初始化区@bar+0x%" PRIx64
               "  测试区@bar+0x%" PRIx64 "\n", sz, c.init_off, c.test_off);
        if (c.cfg.no_init)
            printf("  初始化       : 已跳过（--no_init，init 区内容由外部预置，"
                   "以其实测值作比对基准）\n");
        else
            printf("  初始化       : CPU 填 pattern + 污染 test 区 + 回读确认落盘\n");
        printf("  单次传输     : %s%s\n", blk,
               c.cfg.chunk ? " (分块)" : " (整区一次传输)");
        printf("  循环次数     : %" PRIu64 "%s\n", c.cfg.times,
               c.cfg.times == 0 ? " (无限)" : "");
        printf("  打印间隔     : 每 %" PRIu64 " 次\n", c.cfg.dbg_times);
        if (c.cfg.cpu_mode)
            printf("  超时         : 不适用（无 DMA 等待）\n");
        else
            printf("  超时         : %u ms    Status 检查: %s\n",
                   c.cfg.timeout_ms, c.cfg.check_status ? "开启" : "关闭");
        printf("  测试模式     : %s\n",
               c.cfg.stress == STRESS_RD ? "纯压力 只读（不校验，数据只初始化一次）" :
               c.cfg.stress == STRESS_WR ? "纯压力 只写（不校验，数据只初始化一次）" :
               c.cfg.stress == STRESS_BOTH ? "纯压力 读+写（不校验，数据只初始化一次）" :
               c.cfg.no_init ? "读-改-写闭环（--no_init：跳过 CPU 初始化，以 init 实测值对拍）" :
                               "读-改-写闭环（逐轮 pattern + CPU 回读校验）");
        printf("  数据模式     : %s%s\n",
               c.cfg.pat_mode == PAT_INC ? "递增 inc（每轮 +1，qword 逐个递增）" :
               c.cfg.pat_mode == PAT_ALT ? "翻转 alt（5A5A/A5A5 逐轮交替）" :
                                           "随机 rand（splitmix64 种子流）",
               c.cfg.stress ? "  —— 只用于启动时初始化" : "");
        if (c.cfg.stress) {
            char st[32];

            fmt_size(st, sizeof(st), stress_addr_step(&c));
            printf("  地址模式     : %s\n",
                   c.cfg.addr_pat == ADDR_FIX ? "固定（每轮都从区首开始）" :
                   c.cfg.addr_pat == ADDR_RAND ? "随机（起点由 seed/iter 决定）" :
                                                 "顺序（每轮起点前移，到区尾回绕）");
            if (c.cfg.addr_pat == ADDR_SEQ)
                printf("                 步长 %s（%s）\n", st,
                       c.cfg.addr_step ? "显式 --addr_step" : "自动");
        }
        if (c.cfg.cpu_mode) {
            printf("  回读延迟     : 不适用（CPU 模式无 DMA 等待）\n");
        } else if (c.cfg.stress) {
            printf("  初始化延迟   : %u ms（仅压力数据初始化回读重试用）\n",
                   c.cfg.init_delay_ms);
            printf("  回读延迟     : 不适用（压力模式不校验数据）\n");
        } else {
            printf("  初始化延迟   : %u ms%s\n", c.cfg.init_delay_ms,
                   c.cfg.init_delay_ms ? "" : " (未启用，CPU 写完立即起 DMA)");
            if (c.cfg.no_init)
                printf("  初始化回读   : 已跳过（--no_init：只抓 init 实测值作基准，不回读重试）\n");
            else
                printf("  初始化回读   : 开启（CPU 写后回读校验，不符按初始化延迟重试至多 1000 次）\n");
            printf("  回读延迟     : %u ms%s\n", c.cfg.check_delay_ms,
                   c.cfg.check_delay_ms ? "" : " (未启用，DMA 完成后立即回读)");
        }
        printf("  目的 bar 管理: %s\n",
               c.cfg.no_alloc ? "固定布局（未使用测试头，无法与其他源头共存）"
                              : "测试头 + 占用链（--list 查看占用，--clear 复位）");
        printf("==========================================================\n");
    }

    if (c.cfg.verbose)
        dump_regs(&c, "before");

    /* ---- 清错误记录 + 装信号 ---- */
    err_clear(&c);

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_sigint;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    rc = run_test(&c);

    if (c.cfg.verbose)
        dump_regs(&c, "after");

    free(c.bounce);
    free(c.gold);
    free(c.pat);
    err_local_close(&c);
    close_bar(&c.dst);
    close_bar(&c.src);

    return rc == E_OK ? 0 : 1;
}
