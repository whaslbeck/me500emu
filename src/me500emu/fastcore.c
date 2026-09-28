/* fastcore - the hot glue of the ME-500 emulator in C, Unicorn stays the CPU.
 *
 * What lives here: the slice
 * loop with its tick scheduler, the 8259 (ports 0x10/0x11), the interrupt
 * dispatch through the IVT (software INT from the hook, hardware IRQ from the
 * loop), and the IN/OUT and MMIO trampolines that hand every other port and
 * device access to Python with the PC already computed. The device models
 * themselves stay in Python, so the measurement scripts keep their hooks and
 * attributes.
 *
 * Semantics are a line-for-line transcription of Machine.run(), Cpu._dispatch_ivt
 * and devices.Pic; the acceptance is a strobe-identical run against the Python
 * loop (the Python loop has since been retired).
 *
 * No linking against libunicorn: Python passes the addresses of the six
 * entry points it already has loaded, so exactly one copy of the library
 * serves both sides.
 */
#include <stdint.h>
#include <string.h>
#include <unicorn/unicorn.h>

typedef struct {
    /* accounting and scheduler */
    uint64_t instr;
    uint64_t tick_interval, motion_tick_interval, service_tick_interval, uart_byte_interval;
    int32_t abort;
    int32_t pad0;
    int64_t resume_at;                 /* -1: none; set by BRKXA/RETXA (Python) and by dispatch */
    /* pending IRQ vectors, in request order */
    int32_t pending[16];
    int32_t npending;
    /* 8259 */
    uint32_t pic_isr, pic_imr, pic_read_isr, pic_icw_left, pic_eois, pic_blocked;
    /* UART mirrors (the data path stays in Python) */
    uint32_t uart_rx_len, uart_ctrl, uart_honour_dtr;
    /* results and counters */
    int32_t err;
    uint64_t err_pc;
    uint64_t dispatched, dropped, slices;
    uint32_t log_n, log_cap;           /* dispatch log: intno, seg, off */
    uint32_t dropped_n;
    uint32_t dropped_log[64];
    /* Phase jitter: 0 = off. Otherwise a due interrupt lands after 1..cap instead of exactly
     * cap instructions, and every tick period scatters by +-1/8. The core delivers interrupts
     * only at slice boundaries - without jitter always at the same places in the program,
     * and a race that hits the machine stays invisible here. */
    uint64_t jitter, jrng;
    /* 8251 overrun: a new byte becomes due while the previous one is still unread in the
     * receive register (IRQ1 requested but not serviced - e.g. a probe inside the ISR). The
     * real 8251 then loses a byte; here it is only counted, the gate requires 0. */
    uint64_t uart_overruns;
    uint64_t uart_pending_since;       /* instruction count since which the byte has been waiting unread */
    uint64_t uart_overrun_after;       /* wait in instructions after which the 8251 loses (~1 byte time real, 0 = default 8000) */
    uint64_t last_mtick;               /* instruction count of the last IRQ0 delivery (8253 counter 0 model, port 0x14) */
    uint64_t last_tick;                /* instruction count of the last IRQ3 delivery (8253 counter 1 model, port 0x15) */
    uint64_t pic_lowest;               /* 8259: IRQ with the lowest priority (7 = default; OCW2 0xC0|n sets n) */
    /* 8251 receive register: bytes arrive at a fixed byte rate, read or not.
     * uart_delivered: an unread byte sits in the register (RxRDY); uart_oe: overrun status (bit 4 of
     * port 0x19, until the error reset ER); uart_drop: lost bytes that Python skips when reading */
    uint32_t uart_delivered, uart_oe, uart_drop, uart_pad;
    /* Deadlines: kept across fc_run calls - previously local variables, reset to "now" on every harness
     * run(): a byte, a tick and a service tick right at the start of every call (probe gate S8:
     * byte 18 instructions after the previous one -> overrun). 0 = not set yet */
    uint64_t next_tick, next_mtick, next_stick, next_byte, next_txirq;
    /* IF watch: if an IRQ is pending and IF = 0, the code hook checks each instruction for IF = 1 and ends the
     * slice there - the real processor delivers at every instruction boundary; with 64-instruction slices the core
     * missed the 1..5 instruction long IF=1 gaps of the LCD renderer (pushf/cli/popf per character) and delivered
     * IRQ1 up to 1000 instructions late (test bench at 19200: one lost byte per circle, a model artefact). */
    uint64_t watch_if, watch_count, watch_last;
    /* Control switch: uart_deep = 1 models an 8251 with an infinitely deep receive buffer - a byte is
     * delivered only once the previous one has been read, so it is never lost, only the stream is delayed.
     * Only for control runs (polling off at 19200), never for the gates. */
    uint64_t uart_deep;
    /* 8259A (workshop run on the machine): in edge-triggered mode the request must stay asserted until the
     * first INTA. If the tick poll has already read the byte (RxRDY = 0), the real chip delivers the default
     * vector IR7 (INT 27h, no in-service bit; an `iret` in the factory ROM) instead of IRQ1 - the receive ISR
     * does NOT run after the poll. Previously the model delivered IRQ1 latched (stashed ENQ, rx_uart path): on
     * the machine the ENQ stayed stashed until the next byte arrived. pic_default7 counts these. */
    uint64_t pic_default7;
    /* Executor instruction costs (2026-09-23, calibrated against the machine's tick trace): the chain tick 5be6
     * takes 677 us on the machine for 959 instructions in the model, the plateau tick 252 us for 411 - 0.6..0.7 us
     * per instruction instead of the flat 0.5 us (2000 instructions per ms). Every instruction in the executor
     * range 0x84300..0x86300 (Z track, idle, plateau, chain, handover) therefore counts 1.4: two extra units per
     * five instructions. */
    int64_t cost_acc, cost_extra;       /* cost_acc = eighth-unit accumulator of the class costs (signed), cost_extra = folded net units */
    /* 8251 transmit side: holding register (-1 empty) and the instruction count from which the shifter is free -
     * maintained by devices.Uart (Python) on every port access; the slice loop raises IRQ2 (TxRDY) only when
     * the holding register is empty or empties on the next access. Previously IRQ2 came at a fixed byte rate,
     * and the factory ISR dropped TxEN at the end of the queue while the last byte was still in the holding register. */
    int64_t uart_tx_hold;
    uint64_t uart_tx_free_at;
    /* Counting modes in the C core (previously only in the Python core, which has thereby been retired):
     * trace_mode 0 = fast (instruction budget), 1 = count (every hook firing is one instruction, including every
     * rep iteration - like the Python counting hook; plus watch counters), 2 = full (plus last_pc and
     * the ring of the last 24 addresses). hcount counts firings in count/full. */
    uint64_t trace_mode, hcount, last_pc;
    uint64_t recent[24];
    uint64_t recent_pos, recent_n;
} fc_state;

static inline uint64_t jrand(fc_state *s)
{
    uint64_t x = s->jrng ? s->jrng : 0x9E3779B97F4A7C15ull;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    s->jrng = x;
    return x;
}

/* interval with jitter: interval +- interval/8 */
static inline uint64_t jint(fc_state *s, uint64_t interval)
{
    if (!s->jitter || interval < 16)
        return interval;
    uint64_t span = interval / 4;
    return interval - span / 2 + (jrand(s) % (span + 1));
}

/* ---- stage 2: motion window, counter and axis physics -------------------- */
typedef struct {
    int64_t pos, pending, travel;
    double steps_per_mm;
    int32_t margin, home_at_low, both_ends, has_band;
    int64_t band_lo, band_hi;
    uint32_t clipped, hit_low, hit_high, max_command, follow_error_max, follow_trips, follow_limit;
    uint32_t pad;
} fc_axis;

typedef struct {
    uint8_t regs[256];
    int64_t pos[3];
    uint64_t strobes;
    uint32_t handshakes, sync_ptr, sync_posts, sync_acks;
    uint32_t counter_value, counter_latched;
    int32_t motion_py, consume_py, physics, instr_per_step;
    int64_t instr_credit;
    uint64_t steps_consumed;
    fc_axis axes[3];
    uint32_t path_n, path_cap;
    uint32_t other_n, other_cap;
    uint64_t sync_lo, sync_hi;
} fc_motion;

typedef struct {
    uc_engine *uc;
    fc_state *st;
    fc_motion *mo;
    int64_t *path;                     /* 3 words per strobe */
    uint64_t *other;                   /* 2 words per entry: off|value<<16, pc */
    int motion_region;
    void (*consume_cb)(int64_t dx, int64_t dy, int64_t dz);
    /* unicorn entry points, handed over by Python */
    uc_err (*emu_start)(uc_engine *, uint64_t, uint64_t, uint64_t, size_t);
    uc_err (*emu_stop)(uc_engine *);
    uc_err (*reg_read)(uc_engine *, int, void *);
    uc_err (*reg_write)(uc_engine *, int, const void *);
    uc_err (*mem_read)(uc_engine *, uint64_t, void *, uint64_t);
    uc_err (*mem_write)(uc_engine *, uint64_t, const void *, uint64_t);
    uc_err (*hook_add)(uc_engine *, uc_hook *, int, void *, void *, uint64_t, uint64_t, ...);
    uc_err (*mmio_map)(uc_engine *, uint64_t, uint64_t, uc_cb_mmio_read_t, void *,
                       uc_cb_mmio_write_t, void *);
    /* register ids */
    int r_cs, r_ip, r_ss, r_sp, r_eflags;
    /* Python callbacks */
    uint32_t (*port_in)(uint32_t port, int size, uint64_t pc);
    void (*port_out)(uint32_t port, int size, uint32_t value, uint64_t pc);
    uint64_t (*mmio_read)(int region, uint64_t off, unsigned size, uint64_t pc);
    void (*mmio_write)(int region, uint64_t off, unsigned size, uint64_t value, uint64_t pc);
    void (*intr_fallback)(uint32_t intno, uint64_t pc);
    void (*between)(uint64_t delta);
    /* dispatch log */
    uint16_t *log;                     /* 3 words per entry */
} fc_core;

static fc_core C;

static inline uint32_t rd(int reg)
{
    uint32_t v = 0;
    C.reg_read(C.uc, reg, &v);
    return v;
}

static inline void wr(int reg, uint32_t v)
{
    C.reg_write(C.uc, reg, &v);
}

static inline uint64_t pc_now(void)
{
    return (uint64_t)rd(C.r_cs) * 16 + rd(C.r_ip);
}

/* ---- 8259 ---------------------------------------------------------- */
static inline int pic_prio(int irq)        /* 0 = highest priority; default IR0 (lowest 7) */
{
    return (irq - (int)C.st->pic_lowest - 1) & 7;
}

static int pic_acceptable(int vec)
{
    int irq = vec - 0x20;
    if (irq < 0 || irq > 7)
        return 1;
    if (C.st->pic_imr & (1u << irq))
        return 0;
    for (int j = 0; j < 8; j++)            /* nothing of equal or higher priority in service */
        if ((C.st->pic_isr & (1u << j)) && pic_prio(j) <= pic_prio(irq))
            return 0;
    return 1;
}

static void pic_eoi_nonspecific(fc_state *s)
{
    int best = -1;
    for (int j = 0; j < 8; j++)
        if ((s->pic_isr & (1u << j)) && (best < 0 || pic_prio(j) < pic_prio(best)))
            best = j;
    if (best >= 0)
        s->pic_isr &= ~(1u << best);
    s->pic_eois++;
}

static void pic_write(uint32_t port, uint32_t value)
{
    fc_state *s = C.st;
    uint32_t v = value & 0xFF;
    if (port == 0x10) {
        if (v & 0x10) {                          /* ICW1 */
            s->pic_icw_left = 1 + ((v & 1) ? 1 : 0) + ((v & 2) ? 0 : 1);
            s->pic_isr = 0;
            s->pic_read_isr = 0;
            s->pic_lowest = 7;
        } else if (v & 0x08) {                   /* OCW3 */
            if (v & 0x02)
                s->pic_read_isr = (v & 1) ? 1 : 0;
        } else {                                 /* OCW2 */
            uint32_t kind = v & 0xE0;
            if (kind == 0x20) {                  /* non-specific EOI */
                pic_eoi_nonspecific(s);
            } else if (kind == 0x60) {           /* specific EOI */
                s->pic_isr &= ~(1u << (v & 7));
                s->pic_eois++;
            } else if (kind == 0xC0) {           /* Set Priority: v&7 becomes lowest (~O C0: IR1 before IR0) */
                s->pic_lowest = v & 7;
            } else if (kind == 0xA0) {           /* Rotate on non-specific EOI */
                int best = -1;
                for (int j = 0; j < 8; j++)
                    if ((s->pic_isr & (1u << j)) && (best < 0 || pic_prio(j) < pic_prio(best)))
                        best = j;
                pic_eoi_nonspecific(s);
                if (best >= 0) s->pic_lowest = best;
            } else if (kind == 0xE0) {           /* Rotate on specific EOI */
                s->pic_isr &= ~(1u << (v & 7));
                s->pic_eois++;
                s->pic_lowest = v & 7;
            }
        }
    } else {
        if (s->pic_icw_left)
            s->pic_icw_left--;
        else
            s->pic_imr = v;
    }
}

static uint32_t pic_read(uint32_t port)
{
    fc_state *s = C.st;
    if (port == 0x10) {
        if (s->pic_read_isr)
            return s->pic_isr;
        uint32_t irr = 0;
        for (int i = 0; i < s->npending; i++) {
            int vec = s->pending[i];
            if (vec >= 0x20 && vec <= 0x27)
                irr |= 1u << (vec - 0x20);
        }
        return irr;
    }
    return s->pic_imr;
}

/* ---- interrupt dispatch through the IVT ----------------------------- */
/* returns 1 if dispatched, 0 if the vector is null */
static int dispatch(uint32_t intno, int from_hook)
{
    fc_state *s = C.st;
    uint8_t v[4];
    C.mem_read(C.uc, intno * 4, v, 4);
    uint32_t off = v[0] | (v[1] << 8), seg = v[2] | (v[3] << 8);
    if (seg == 0 && off == 0) {
        s->dropped++;
        if (s->dropped_n < 64)
            s->dropped_log[s->dropped_n++] = intno;
        return 0;
    }
    uint32_t sp = rd(C.r_sp), ss = rd(C.r_ss), fl = rd(C.r_eflags) & 0xFFFF;
    uint32_t cs = rd(C.r_cs), ip = rd(C.r_ip);
    sp = (sp - 6) & 0xFFFF;
    uint8_t frame[6] = { ip & 0xFF, ip >> 8, cs & 0xFF, cs >> 8, fl & 0xFF, fl >> 8 };
    C.mem_write(C.uc, (uint64_t)ss * 16 + sp, frame, 6);
    wr(C.r_sp, sp);
    /* like the CPU: IF and TF are cleared on entry (IF used to stay set - every handler ran
     * interruptible, although the motion tick never issues `sti` and the others only do so after their prologue) */
    wr(C.r_eflags, rd(C.r_eflags) & ~0x300u);
    wr(C.r_cs, seg);
    wr(C.r_ip, off);
    s->dispatched++;
    if (s->log_n < s->log_cap) {
        uint16_t *e = C.log + 3 * s->log_n++;
        e[0] = intno; e[1] = seg; e[2] = off;
    }
    if (!from_hook)
        s->resume_at = (int64_t)seg * 16 + off;
    return 1;
}

/* ---- unicorn hooks --------------------------------------------------- */
/* Instruction costs per class (2026-09-25, replaces the flat executor 1.4 calibration): the cost module builds, per
 * instruction start in the image, a delta in SIXTEENTH UNITS against the base of 1 unit (0.5 us); the hook fires
 * per instruction (for rep, per iteration at the same address) and sums up; folding happens at the end of the slice. */
/* Watch counters (count/full): sorted addresses, one counter per address; Python mirrors m.watch into it
 * (fc_watch_set) and fetches the counters back after the run (fc_watch_get, resets them to 0). */
#define WATCH_MAX 4096
static uint64_t watch_addr[WATCH_MAX];
static uint64_t watch_cnt[WATCH_MAX];
static int watch_n = 0;

void fc_watch_set(const uint64_t *addrs, int n)
{
    if (n > WATCH_MAX) n = WATCH_MAX;
    for (int i = 0; i < n; i++) { watch_addr[i] = addrs[i]; watch_cnt[i] = 0; }
    watch_n = n;
}

void fc_watch_get(uint64_t *out)
{
    for (int i = 0; i < watch_n; i++) { out[i] = watch_cnt[i]; watch_cnt[i] = 0; }
}

static inline int watch_find(uint64_t addr)
{
    int lo = 0, hi = watch_n - 1;
    while (lo <= hi) {
        int mid = (lo + hi) >> 1;
        if (watch_addr[mid] == addr) return mid;
        if (watch_addr[mid] < addr) lo = mid + 1; else hi = mid - 1;
    }
    return -1;
}

static int8_t *cost_tab = NULL;
static uint64_t cost_tab_len = 0;
static int64_t cost_peer8 = 0;

void fc_costs(void *tab, uint64_t len, int64_t peer8)
{
    cost_tab = (int8_t *)tab;
    cost_tab_len = len;
    cost_peer8 = peer8;
}

static void hook_code(uc_engine *uc, uint64_t addr, uint32_t size, void *ud)
{
    fc_state *s = C.st;
    if (cost_tab && addr - 0x80000ull < cost_tab_len)
        s->cost_acc += cost_tab[addr - 0x80000ull];
    if (s->trace_mode) {
        s->hcount++;
        if (watch_n) { int w = watch_find(addr); if (w >= 0) watch_cnt[w]++; }
        if (s->trace_mode >= 2) {
            s->last_pc = addr;
            s->recent[s->recent_pos] = addr;
            s->recent_pos = (s->recent_pos + 1) % 24;
            if (s->recent_n < 24) s->recent_n++;
        }
    }
    if (!s->watch_if) return;
    if (addr != s->watch_last) s->watch_count++;    /* rep iterations fire the hook several times at the same address:
    s->watch_last = addr;                           * count only once, like unicorn's count (and the Python single step) */
    uint32_t fl = 0;
    C.reg_read(uc, C.r_eflags, &fl);
    if (fl & 0x200) {
        s->watch_if = 2; C.emu_stop(uc);
        if (cost_tab && addr - 0x80000ull < cost_tab_len)
            s->cost_acc -= cost_tab[addr - 0x80000ull];   /* the instruction has not run yet (the next slice counts it) */
        if (s->trace_mode) {                            /* likewise in the counting modes: undo */
            s->hcount--;
            if (watch_n) { int w = watch_find(addr); if (w >= 0) watch_cnt[w]--; }
            if (s->trace_mode >= 2) {
                s->recent_pos = (s->recent_pos + 23) % 24;
                if (s->recent_n) s->recent_n--;
            }
        }
    }
}

static void hook_intr(uc_engine *uc, uint32_t intno, void *ud)
{
    (void)uc; (void)ud;
    uint8_t v[4];
    C.mem_read(C.uc, intno * 4, v, 4);
    if ((v[0] | v[1] | v[2] | v[3]) == 0 && intno >= 0x40 && intno <= 0x47) {
        C.intr_fallback(intno, pc_now());
        return;
    }
    dispatch(intno, 1);
}

static uint32_t hook_in(uc_engine *uc, uint32_t port, int size, void *ud)
{
    (void)uc; (void)ud;
    if (port == 0x10 || port == 0x11)
        return pic_read(port);
    return C.port_in(port, size, pc_now());
}

static void hook_out(uc_engine *uc, uint32_t port, int size, uint32_t value, void *ud)
{
    (void)uc; (void)ud;
    if (port == 0x10 || port == 0x11) {
        pic_write(port, value);
        return;
    }
    if (port == 0x19)
        C.st->uart_ctrl = value & 0xFF;      /* mirror before Python sees it */
    C.port_out(port, size, value, pc_now());
}

/* ---- the motion window (transcription of devices.MotionRegs) -------------- */
static inline int32_t mo_word(int off)
{
    int32_t v = C.mo->regs[off] | (C.mo->regs[off + 1] << 8);
    return v & 0x8000 ? v - 0x10000 : v;
}

static void axis_owe(fc_axis *a, int64_t steps)
{
    a->pending += steps;
    uint64_t m = steps < 0 ? -steps : steps;
    if (m > a->max_command)
        a->max_command = (uint32_t)m;
}

static int64_t axis_advance(fc_axis *a, int64_t max_steps)
{
    int64_t ap = a->pending < 0 ? -a->pending : a->pending;
    int64_t e = ap - (int64_t)a->max_command;
    if (e < 0) e = 0;
    if (e > (int64_t)a->follow_error_max) a->follow_error_max = (uint32_t)e;
    if (e >= (int64_t)a->follow_limit) a->follow_trips++;
    if (!a->pending || max_steps <= 0)
        return 0;
    int64_t step = a->pending;
    if (step > max_steps) step = max_steps;
    if (step < -max_steps) step = -max_steps;
    int64_t target = a->pos + step;
    if (target < 0) target = 0;
    else if (target > a->travel) target = a->travel;
    int64_t moved = target - a->pos;
    a->pos = target;
    if (moved != step || target == 0 || target == a->travel) {
        a->pending = 0;
        if (moved != step) a->clipped++;
    } else {
        a->pending -= moved;
    }
    return moved;
}

void fc_consume(int64_t dx, int64_t dy, int64_t dz)
{
    fc_motion *m = C.mo;
    m->steps_consumed++;
    if (!m->physics)
        return;
    axis_owe(&m->axes[0], dx);
    axis_owe(&m->axes[1], dy);
    axis_owe(&m->axes[2], dz);
}

void fc_advance(int64_t instr_delta)
{
    fc_motion *m = C.mo;
    if (!m->physics || instr_delta <= 0)
        return;
    m->instr_credit += instr_delta;
    int64_t steps = m->instr_credit / m->instr_per_step;
    if (steps <= 0)
        return;
    m->instr_credit -= steps * m->instr_per_step;
    for (int i = 0; i < 3; i++)
        axis_advance(&m->axes[i], steps);
}

int64_t fc_axis_advance(int i, int64_t max_steps)
{
    return axis_advance(&C.mo->axes[i], max_steps);
}

void fc_axis_owe(int i, int64_t steps)
{
    axis_owe(&C.mo->axes[i], steps);
}

static void maybe_handshake(void)
{
    fc_motion *m = C.mo;
    uint8_t n = m->regs[0];
    if (n == 0 || n == 0xFF || m->regs[0xFF] != n)
        return;
    for (int i = 1; i <= n; i++)
        m->regs[i] ^= 0xFF;
    m->regs[0] = 0xFF;
    m->regs[0xFF] = 0xFF;
    m->handshakes++;
}

uint64_t fc_motion_read(uint64_t off, unsigned size, uint64_t pc)
{
    fc_motion *m = C.mo;
    C.st->cost_acc += cost_peer8;            /* sub-CPU window: surcharge per access */
    if (off < 0x100 && pc >= m->sync_lo && pc <= m->sync_hi
        && off == m->sync_ptr && m->regs[off] != 0xAA) {
        m->regs[off] = 0xAA;
        m->sync_posts++;
    }
    if (size == 1)
        return m->regs[off];
    return (uint64_t)(mo_word((int)off) & 0xFFFF);
}

void fc_motion_write(uint64_t off, unsigned size, uint64_t value, uint64_t pc)
{
    fc_motion *m = C.mo;
    C.st->cost_acc += cost_peer8;            /* sub-CPU window: surcharge per access */
    if (size == 1) {
        m->regs[off] = value & 0xFF;
    } else {
        m->regs[off] = value & 0xFF;
        m->regs[off + 1] = (value >> 8) & 0xFF;
    }
    if (off < 0x100 && size == 1 && off == m->sync_ptr && (value & 0xFF) == 0x55) {
        m->regs[off] = 0;
        m->sync_ptr = (m->sync_ptr + 1) & 0xFF;
        m->sync_acks++;
    }
    if (off == 0x98 && (value & 0xFF) == 1) {
        int64_t d[3], total = 0;
        static const int base[3] = { 0x80, 0x88, 0x90 };
        for (int ax = 0; ax < 3; ax++) {
            d[ax] = mo_word(base[ax]) + mo_word(base[ax] + 2) + mo_word(base[ax] + 4);
            m->pos[ax] += d[ax];
            total += d[ax];
        }
        m->strobes++;
        if (m->path_n < m->path_cap) {
            int64_t *e = C.path + 3 * m->path_n++;
            e[0] = m->pos[0]; e[1] = m->pos[1]; e[2] = m->pos[2];
        }
        m->counter_value = (uint32_t)((m->counter_value + total) & 0xFFFF);
        if (m->consume_py)
            C.consume_cb(d[0], d[1], d[2]);
        else
            fc_consume(d[0], d[1], d[2]);
        return;
    }
    if (off == 0x00 || off == 0xFF)
        maybe_handshake();
    if (!((off >= 0x80 && off < 0x86) || (off >= 0x88 && off < 0x8e) || (off >= 0x90 && off < 0x96))) {
        if (m->other_n < m->other_cap) {
            uint64_t *e = C.other + 2 * m->other_n++;
            e[0] = (off & 0xFFFF) | ((value & 0xFFFF) << 16);
            e[1] = pc;
        }
    }
}

static uint64_t hook_mmio_read(uc_engine *uc, uint64_t off, unsigned size, void *ud)
{
    (void)uc;
    int region = (int)(intptr_t)ud;
    if (region == C.motion_region && C.mo && !C.mo->motion_py && off < 0x100)
        return fc_motion_read(off, size, pc_now());
    return C.mmio_read(region, off, size, pc_now());
}

static void hook_mmio_write(uc_engine *uc, uint64_t off, unsigned size, uint64_t value, void *ud)
{
    (void)uc;
    int region = (int)(intptr_t)ud;
    if (region == C.motion_region && C.mo && !C.mo->motion_py && off < 0x100) {
        fc_motion_write(off, size, value, pc_now());
        return;
    }
    C.mmio_write(region, off, size, value, pc_now());
}

/* ---- API ------------------------------------------------------------- */
int fc_init(uc_engine *uc, fc_state *st, void **fns, int *regs, void **cbs,
            uint16_t *log, uint32_t log_cap)
{
    memset(&C, 0, sizeof C);
    /* The core is a process singleton: a new machine must not inherit the previous one's cost table (a pointer
     * into a buffer the old Python object owned) or its watch list. */
    cost_tab = NULL; cost_tab_len = 0; cost_peer8 = 0;
    watch_n = 0;
    C.uc = uc;
    C.st = st;
    C.emu_start = (void *)fns[0];
    C.emu_stop = (void *)fns[1];
    C.reg_read = (void *)fns[2];
    C.reg_write = (void *)fns[3];
    C.mem_read = (void *)fns[4];
    C.mem_write = (void *)fns[5];
    C.hook_add = (void *)fns[6];
    C.mmio_map = (void *)fns[7];
    C.r_cs = regs[0]; C.r_ip = regs[1]; C.r_ss = regs[2]; C.r_sp = regs[3]; C.r_eflags = regs[4];
    C.port_in = (void *)cbs[0];
    C.port_out = (void *)cbs[1];
    C.mmio_read = (void *)cbs[2];
    C.mmio_write = (void *)cbs[3];
    C.intr_fallback = (void *)cbs[4];
    C.between = (void *)cbs[5];
    C.log = log;
    st->log_cap = log_cap;
    st->resume_at = -1;
    return 0;
}

int fc_init_motion(fc_motion *mo, int64_t *path, uint32_t path_cap, uint64_t *other, uint32_t other_cap,
                   int motion_region, void *consume_cb)
{
    C.mo = mo;
    C.path = path;
    C.other = other;
    C.motion_region = motion_region;
    C.consume_cb = consume_cb;
    mo->path_cap = path_cap;
    mo->other_cap = other_cap;
    return 0;
}

int fc_install_hooks(int hook_intr_type, int hook_insn_type, int insn_in, int insn_out)
{
    uc_hook h;
    uc_err e;
    e = C.hook_add(C.uc, &h, hook_intr_type, (void *)hook_intr, NULL, 1, 0);
    if (e) return e;
    e = C.hook_add(C.uc, &h, hook_insn_type, (void *)hook_in, NULL, 1, 0, insn_in);
    if (e) return e;
    e = C.hook_add(C.uc, &h, hook_insn_type, (void *)hook_out, NULL, 1, 0, insn_out);
    if (e) return e;
    e = C.hook_add(C.uc, &h, UC_HOOK_CODE, (void *)hook_code, NULL, 1, 0);
    return e;
}

int fc_mmio_map(uint64_t base, uint64_t size, int region)
{
    return C.mmio_map(C.uc, base, size, hook_mmio_read, (void *)(intptr_t)region,
                      hook_mmio_write, (void *)(intptr_t)region);
}

/* hardware interrupt from Python (rare: service stubs); returns 1 if dispatched */
int fc_dispatch(uint32_t intno)
{
    return dispatch(intno, 0);
}

static int pending_has(fc_state *s, int vec)
{
    for (int i = 0; i < s->npending; i++)
        if (s->pending[i] == vec)
            return 1;
    return 0;
}

static void pending_push(fc_state *s, int vec)
{
    if (s->npending < 16)
        s->pending[s->npending++] = vec;
}

static void pending_remove_at(fc_state *s, int i)
{
    for (; i + 1 < s->npending; i++)
        s->pending[i] = s->pending[i + 1];
    s->npending--;
}

/* The slice loop - Machine.run() transcribed. `at` is the physical start address. */
uint64_t fc_run(uint64_t at, uint64_t max_instr, uint64_t slice_size)
{
    fc_state *s = C.st;
    if (!s->next_tick) s->next_tick = s->instr + s->tick_interval;
    if (!s->next_mtick) s->next_mtick = s->instr + s->motion_tick_interval;
    if (!s->next_stick) s->next_stick = s->instr + s->service_tick_interval;
    if (!s->next_byte) s->next_byte = s->instr;
    uint64_t next_tick = s->next_tick, next_mtick = s->next_mtick, next_stick = s->next_stick;
    uint64_t next_txirq = s->next_txirq, next_byte = s->next_byte;
    uint64_t last_advance = s->instr;
    s->err = 0;
    while (s->instr < max_instr) {
        if (s->abort)
            break;
        s->resume_at = -1;
        uint64_t budget = slice_size;
        if (max_instr - s->instr < budget)
            budget = max_instr - s->instr;
        if (s->npending) {
            uint64_t cap = (rd(C.r_eflags) & 0x200) ? 1 : 64;   /* IF = 1 and IRQ pending: deliver at once (previously up to 512 instructions late) */
            if (s->jitter)
                cap = 1 + (jrand(s) % cap);
            if (cap < budget) budget = cap;
        } else {
            uint64_t due = 0; int have = 0;
            if (s->motion_tick_interval) { due = next_mtick; have = 1; }
            if (s->tick_interval && (!have || next_tick < due)) { due = next_tick; have = 1; }
            if (s->service_tick_interval && (!have || next_stick < due)) { due = next_stick; have = 1; }
            /* the next byte also ends the slice - otherwise bytes arrived only on the tick grid
             * (2000 instructions): "19200 baud" delivered 990 bytes/s, half the rate (circle test, 512) */
            if (s->uart_byte_interval && s->uart_rx_len && (!have || next_byte < due)) { due = next_byte; have = 1; }
            if (have) {
                uint64_t left = due > s->instr ? due - s->instr : 1;
                left = left * 5 / 7 + 1;             /* the executor's cost units can reach the deadline earlier */
                if (left < budget) budget = left;
            }
        }
        s->watch_if = (s->npending && !(rd(C.r_eflags) & 0x200)) ? 1 : 0;
        s->watch_count = 0; s->watch_last = ~0ull;
        uint64_t h0 = s->hcount;
        uc_err e = C.emu_start(C.uc, at, 0x100000, 0, (size_t)budget);
        if (s->trace_mode)
            s->instr += s->hcount - h0;                            /* count/full: exact per hook firing */
        else if (s->watch_if == 2)
            s->instr += s->watch_count ? s->watch_count - 1 : 0;   /* the instruction where the watch stopped has not run yet */
        else if (s->watch_if && s->resume_at >= 0)
            s->instr += s->watch_count;                            /* software INT under watch: counted, like the Python core */
        else
            s->instr += budget;
        {   /* fold the class costs (signed, sixteenths), like the Python core */
            int64_t whole = s->cost_acc / 16;
            if (whole) { s->instr += whole; s->cost_extra += whole; s->cost_acc -= whole * 16; }
        }
        s->watch_if = 0;
        s->slices++;
        if (e) {
            s->err = e;
            s->err_pc = pc_now();
            break;
        }
        if (s->resume_at >= 0)
            at = (uint64_t)s->resume_at;
        else
            at = pc_now();
        if (C.mo && C.mo->physics)
            fc_advance((int64_t)(s->instr - last_advance));
        else
            C.between(s->instr - last_advance);
        last_advance = s->instr;
        int dtr = (s->uart_ctrl == 0) || (s->uart_ctrl & 0x02);
        if (s->uart_rx_len > s->uart_drop && (dtr || !s->uart_honour_dtr)) {
            if (!s->uart_byte_interval) {
                if (!s->uart_delivered) {          /* without a byte rate: byte by byte, as soon as the previous one is read */
                    s->uart_delivered = 1; s->uart_pending_since = s->instr;
                    if (!pending_has(s, 0x21)) pending_push(s, 0x21);
                }
            } else if (s->instr >= next_byte && s->uart_deep && s->uart_delivered) {
                /* control switch: the register is occupied, the byte waits in the (fictitious) buffer */
            } else if (s->instr >= next_byte) {
                /* the 8251 keeps receiving at a fixed byte rate (previously the stream waited until the
                 * byte was fetched - "19200" delivered only 990 bytes/s and never lost a byte). If the
                 * previous byte is still unread in the register, it is lost: OE, uart_drop (Python
                 * skips it on the next read of port 0x18), counter uart_overruns */
                if (s->uart_delivered) { s->uart_overruns++; s->uart_oe = 1; s->uart_drop++; }
                s->uart_delivered = 1; s->uart_pending_since = s->instr;
                if (!pending_has(s, 0x21)) pending_push(s, 0x21);
                next_byte = s->instr + jint(s, s->uart_byte_interval);
            }
        }
        if (s->motion_tick_interval && s->instr >= next_mtick && !pending_has(s, 0x20))
            pending_push(s, 0x20);
        if ((s->uart_ctrl & 1) && s->instr >= next_txirq && !pending_has(s, 0x22)
            && (s->uart_tx_hold < 0 || s->instr >= s->uart_tx_free_at))   /* TxRDY pin: holding register empty */
            pending_push(s, 0x22);
        if (s->service_tick_interval && s->instr >= next_stick && !pending_has(s, 0x24))
            pending_push(s, 0x24);
        if (s->tick_interval && s->instr >= next_tick && !pending_has(s, 0x23))
            pending_push(s, 0x23);
        if (s->npending && (rd(C.r_eflags) & 0x200)) {
            int idx = -1;                        /* the 8259 takes the highest-priority acceptable one */
            for (int i = 0; i < s->npending; i++)
                if (pic_acceptable(s->pending[i])) {
                    int irq = s->pending[i] - 0x20;
                    if (irq < 0 || irq > 7) { idx = i; break; }
                    if (idx < 0 || pic_prio(irq) < pic_prio(s->pending[idx] - 0x20)) idx = i;
                }
            if (idx < 0) {
                s->pic_blocked++;
                continue;
            }
            int vec = s->pending[idx];
            pending_remove_at(s, idx);
            if (vec == 0x21 && !s->uart_delivered) {   /* request gone before INTA: default vector IR7 */
                vec = 0x27; s->pic_default7++;
            } else if (vec >= 0x20 && vec <= 0x27)
                s->pic_isr |= 1u << (vec - 0x20);
            if (vec == 0x23) { next_tick = s->instr + jint(s, s->tick_interval); s->last_tick = s->instr; }
            else if (vec == 0x20) { next_mtick = s->instr + jint(s, s->motion_tick_interval); s->last_mtick = s->instr; }
            else if (vec == 0x24) next_stick = s->instr + jint(s, s->service_tick_interval);
            else if (vec == 0x22) {
                uint64_t g = s->uart_byte_interval > 200 ? s->uart_byte_interval : 200;
                next_txirq = s->instr + g;
            }
            if (dispatch(vec, 0))
                at = (uint64_t)s->resume_at;
        }
    }
    s->next_tick = next_tick; s->next_mtick = next_mtick; s->next_stick = next_stick;
    s->next_txirq = next_txirq; s->next_byte = next_byte;
    return s->instr;
}
