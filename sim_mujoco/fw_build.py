# -*- coding: utf-8 -*-
"""
Bien dich NGUYEN cac khoi USER CODE trong main.c (PTD, PD, PV, 0, 2, 3, 4) thanh thu vien
(.so / .dll) chay tren PC, voi HAL gia trong firmware/hal_stub.h.
-> Mo phong goi dung: HAL_TIM_PeriodElapsedCallback (TIM6), HAL_GPIO_EXTI_Callback (encoder),
   khoi khoi dong trong main() va vong while(1). Khong chep tay PID nao ca.

Can gcc (Linux: co san; Windows: cai MSYS2/MinGW-w64 roi them gcc vao PATH).
"""
import ctypes
import hashlib
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MAIN_C = os.path.normpath(os.path.join(HERE, '..', 'testNewBrain', 'Core', 'Src', 'main.c'))
DEFAULT_MAIN_H = os.path.normpath(os.path.join(HERE, '..', 'testNewBrain', 'Core', 'Inc', 'main.h'))
BUILD = os.path.join(HERE, 'build')


def find_main_c():
    p = os.environ.get('MAIN_C', DEFAULT_MAIN_C)
    h = os.environ.get('MAIN_H', os.path.join(os.path.dirname(os.path.dirname(p)), 'Inc', 'main.h'))
    return p, h


def block(src, name):
    m = re.search(r'/\* USER CODE BEGIN %s \*/(.*?)/\* USER CODE END %s \*/' % (re.escape(name), re.escape(name)),
                  src, re.S)
    if not m:
        raise RuntimeError('khong tim thay khoi USER CODE %s trong main.c' % name)
    return m.group(1)


def read_defines(main_c):
    """Doc cac #define so trong main.c -> dict (bo hau to f/U, tinh bieu thuc don gian)."""
    src = open(main_c, encoding='utf-8', errors='replace').read()
    out = {}
    for m in re.finditer(r'^\s*#define\s+(\w+)\s+(.+?)\s*(?:/\*.*)?$', src, re.M):
        name, expr = m.group(1), m.group(2).strip()
        e = re.sub(r'(?<=[0-9.])[fFuU]+\b', '', expr)
        e = re.sub(r'\(\s*(?:uint\d+_t|int\d+_t|float)\s*\)', '', e)
        try:
            out[name] = eval(e, {'__builtins__': {}}, dict(out))
        except Exception:
            pass
    return out


def read_cubemx(main_c):
    """Chu ky TIM6 va tan so PWM TIM1/TIM8 tu ham MX_TIMx_Init."""
    src = open(main_c, encoding='utf-8', errors='replace').read()

    def get(h, field):
        m = re.search(r'%s\.Init\.%s\s*=\s*(\d+)' % (h, field), src)
        return int(m.group(1)) if m else None
    r = {}
    for h in ('htim1', 'htim6', 'htim8'):
        r[h] = (get(h, 'Prescaler'), get(h, 'Period'))
    # clock: APB2 timer 168 MHz (TIM1, TIM8), APB1 timer 84 MHz (TIM6) - theo SystemClock_Config
    r['tick_s'] = (r['htim6'][0] + 1) * (r['htim6'][1] + 1) / 84e6
    r['pwm1_hz'] = 168e6 / ((r['htim1'][0] + 1) * (r['htim1'][1] + 1))
    r['pwm2_hz'] = 168e6 / ((r['htim8'][0] + 1) * (r['htim8'][1] + 1))
    return r


def generate(main_c, main_h):
    src = open(main_c, encoding='utf-8', errors='replace').read()
    hdr = open(main_h, encoding='utf-8', errors='replace').read()
    pins = '\n'.join(re.findall(r'^#define \w+_(?:Pin|GPIO_Port) .*$', hdr, re.M))
    cm = read_cubemx(main_c)
    a1, a6, a8 = cm['htim1'][1], cm['htim6'][1], cm['htim8'][1]
    blk3 = block(src, '3')
    pv = block(src, 'PV')
    gnames = sorted(set(re.findall(r'^\s*(?:volatile\s+)?\w+\s+(g_\w+)\s*(?:=|;|\[)', pv, re.M)))
    lookup = '\n'.join('    if (strcmp(n, "%s") == 0) return (void *)&%s;' % (g, g) for g in gnames)
    blk2 = block(src, '2')
    code = f"""/* TU SINH boi fw_build.py tu {os.path.basename(main_c)} - KHONG SUA TAY */
#include "hal_stub.h"
#include <math.h>
{pins}

/* ===== USER CODE PTD ===== */
{block(src, 'PTD')}
/* ===== USER CODE PD ===== */
{block(src, 'PD')}
/* ===== bien HAL (thay cho phan CubeMX) ===== */
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1 = {{TIM1, {a1}U, 0U}};
TIM_HandleTypeDef htim6 = {{TIM6, {a6}U, 0U}};
TIM_HandleTypeDef htim8 = {{TIM8, {a8}U, 0U}};
UART_HandleTypeDef huart1 = {{HAL_UART_STATE_READY}};
DMA_HandleTypeDef hdma_usart1_tx;

/* ===== USER CODE PV ===== */
{block(src, 'PV')}
/* ===== USER CODE 0 ===== */
{block(src, '0')}
/* ===== USER CODE 4 ===== */
{block(src, '4')}

/* =================== giao dien cho mo phong =================== */
GPIO_TypeDef sim_gpio[5];
DWT_Type sim_dwt;
CoreDebug_Type sim_coredebug;
uint32_t SystemCoreClock = 168000000U;
TIM_TypeDef sim_tim[3] = {{{{1U}}, {{6U}}, {{8U}}}};
volatile uint8_t sim_i2c_busy = 0U;
SIM_API uint8_t sim_mpu_regs[128];
static uint32_t sim_ms = 0U;
static void (*sim_delay_cb)(uint32_t) = 0;

uint32_t HAL_GetTick(void) {{ return sim_ms; }}
void HAL_Delay(uint32_t ms) {{ if (sim_delay_cb) sim_delay_cb(ms); }}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t dev, uint16_t reg, uint16_t sz,
                                   uint8_t *buf, uint16_t n, uint32_t timeout)
{{ (void)dev; (void)sz; (void)timeout; h->ErrorCode = 0U;
   memcpy(buf, &sim_mpu_regs[reg & 0x7FU], n); return HAL_OK; }}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t dev, uint16_t reg, uint16_t sz,
                                    uint8_t *buf, uint16_t n, uint32_t timeout)
{{ (void)h; (void)dev; (void)sz; (void)timeout; memcpy(&sim_mpu_regs[reg & 0x7FU], buf, n); return HAL_OK; }}
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t dev, uint32_t trials, uint32_t timeout)
{{ (void)h; (void)dev; (void)trials; (void)timeout; return HAL_OK; }}

SIM_API void sim_set_delay_cb(void (*cb)(uint32_t)) {{ sim_delay_cb = cb; }}
SIM_API void sim_set_time(uint32_t ms, uint32_t cyc) {{ sim_ms = ms; sim_dwt.CYCCNT = cyc; }}
SIM_API void sim_set_input(int port, uint16_t pin, int level)
{{ if (level) sim_gpio[port].IDR |= pin; else sim_gpio[port].IDR &= ~(uint32_t)pin; }}
SIM_API uint32_t sim_get_odr(int port) {{ return sim_gpio[port].ODR; }}
SIM_API void sim_get_pwm(uint32_t *ccr1, uint32_t *arr1, uint32_t *ccr8, uint32_t *arr8)
{{ *ccr1 = htim1.CCR1; *arr1 = htim1.ARR; *ccr8 = htim8.CCR1; *arr8 = htim8.ARR; }}
SIM_API void sim_exti(uint16_t pin) {{ HAL_GPIO_EXTI_Callback(pin); }}
SIM_API void sim_tim6_irq(void) {{ HAL_TIM_PeriodElapsedCallback(&htim6); }}
SIM_API void sim_init(void) {{ sim_mpu_regs[0x75] = 0x68U;
{blk2}
}}
/* khoi USER CODE 3 nam trong while(1) cua main() va chua dau '}}' dong vong lap -> boc bang do {{ ... }} while (0) */
SIM_API void sim_main_loop(void) {{ do {{
{blk3}
while (0); }}
/* dia chi bien toan cuc g_* (DLL Windows khong tu export bien) */
SIM_API void *sim_addr(const char *n) {{
{lookup}
    return 0;
}}
/* PID dung rieng de kiem tra / ve dap ung */
SIM_API float sim_pid_update(PID_t *p, float e, float ff, float ts) {{ return PID_Update(p, e, ff, ts); }}
SIM_API float sim_pid_update_rate(PID_t *p, float e, float ff, float ts, float r) {{ return PID_UpdateRate(p, e, ff, ts, r); }}
SIM_API float sim_kalman_update(Kalman_t *k, float g, float a, float dt) {{ return Kalman_Update(k, g, a, dt); }}
SIM_API void sim_kalman_init(Kalman_t *k, float a0, float b0, float r) {{ Kalman_Init(k, a0, b0, r); }}
"""
    return code


def compiler():
    """gcc neu co; neu khong thi ziglang (pip install ziglang - khong can cai MinGW)."""
    gcc = shutil.which('gcc')
    if gcc:
        return [gcc]
    try:
        import ziglang  # noqa: F401
        return [sys.executable, '-m', 'ziglang', 'cc']
    except ImportError:
        raise RuntimeError('khong co trinh bien dich C: chay "pip install ziglang" (hoac cai gcc)')


def _key(main_c, main_h, target):
    h = hashlib.sha1()
    for p in (main_c, main_h, os.path.join(HERE, 'firmware', 'hal_stub.h'), os.path.abspath(__file__)):
        with open(p, 'rb') as f:
            h.update(f.read().replace(b'\r\n', b'\n'))
    h.update(target.encode())
    return h.hexdigest()


def build(main_c=None, main_h=None, quiet=False, target=None):
    """Sinh + bien dich, tra ve duong dan thu vien.
    Chi bien dich lai khi NOI DUNG main.c / main.h doi (so ma bam) -> neu chua sua main.c
    thi dung luon thu vien dung san trong build/ (khong can trinh bien dich)."""
    if main_c is None:
        main_c, main_h = find_main_c()
    if not os.path.isfile(main_c):
        raise FileNotFoundError(main_c)
    os.makedirs(BUILD, exist_ok=True)
    win = sys.platform.startswith('win') if target is None else target.startswith('win')
    tgt = 'win-x64' if win else 'linux-x64'
    lib = os.path.join(BUILD, 'fw_sim' + ('.dll' if win else '.so'))
    csrc = os.path.join(BUILD, 'fw_sim.c')
    stamp = lib + '.stamp'
    key = _key(main_c, main_h, tgt)
    if os.path.isfile(lib) and os.path.isfile(stamp) and open(stamp).read().strip() == key:
        return lib
    cc = compiler() if target is None else [sys.executable, '-m', 'ziglang', 'cc', '-target', 'x86_64-windows-gnu']
    with open(csrc, 'w', encoding='utf-8', newline='\n') as f:
        f.write(generate(main_c, main_h))
    # -ffp-contract=off: khong gop nhan-cong (giong lam tron float tung phep)
    cmd = cc + ['-O2', '-shared', '-w', '-ffp-contract=off', '-fno-fast-math',
                '-I', os.path.join(HERE, 'firmware'), csrc, '-o', lib]
    if not win:
        cmd += ['-fPIC', '-lm']
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError('bien dich loi:\n' + r.stderr[-3000:])
    with open(stamp, 'w') as f:
        f.write(key)
    if not quiet:
        print('built', lib, 'from', main_c)
    return lib


# ---------------- ctypes ----------------
class PID_t(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in
                ('kp', 'ki', 'kd', 'kff', 'd_alpha', 'kaw', 'out_min', 'out_max', 'e_prev', 'uI', 'uD', 'out')]


class Kalman_t(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in
                ('q_angle', 'q_bias', 'r_meas', 'angle', 'bias', 'rate', 'p00', 'p01', 'p10', 'p11', 'innov', 'k0')]


DELAY_CB = ctypes.CFUNCTYPE(None, ctypes.c_uint32)


class Firmware:
    """Mot ban 'STM32' chay code that. Moi Sim nen dung 1 ban rieng (nap thu vien rieng)."""
    _count = 0

    def __init__(self, main_c=None, main_h=None):
        lib_path = build(main_c, main_h, quiet=True)
        # nap ban sao rieng de bien toan cuc (static) khong dung chung giua cac Sim
        Firmware._count += 1
        tmp = os.path.join(BUILD, f'inst_{os.getpid()}_{Firmware._count}' + os.path.splitext(lib_path)[1])
        shutil.copyfile(lib_path, tmp)
        self.lib = ctypes.CDLL(tmp)
        self._tmp = tmp
        L = self.lib
        L.sim_set_time.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
        L.sim_set_input.argtypes = [ctypes.c_int, ctypes.c_uint16, ctypes.c_int]
        L.sim_get_odr.argtypes = [ctypes.c_int]
        L.sim_get_odr.restype = ctypes.c_uint32
        L.sim_exti.argtypes = [ctypes.c_uint16]
        L.sim_get_pwm.argtypes = [ctypes.POINTER(ctypes.c_uint32)] * 4
        L.sim_addr.argtypes = [ctypes.c_char_p]
        L.sim_addr.restype = ctypes.c_void_p
        self.regs = (ctypes.c_uint8 * 128).in_dll(L, 'sim_mpu_regs')
        self._cb = None

    def __del__(self):
        try:
            h = self.lib._handle
            del self.lib
            if sys.platform.startswith('win'):
                ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(h))
            os.remove(self._tmp)
        except Exception:
            pass

    def var(self, name, ctype=ctypes.c_float):
        a = self.lib.sim_addr(name.encode())
        if not a:
            raise KeyError(name)
        return ctypes.cast(a, ctypes.POINTER(ctype)).contents

    def get(self, name, ctype=ctypes.c_float):
        return self.var(name, ctype).value

    def set(self, name, value, ctype=ctypes.c_float):
        self.var(name, ctype).value = value

    def pid(self, name):
        return self.var(name, PID_t)

    def set_delay_cb(self, fn):
        self._cb = DELAY_CB(fn)
        self.lib.sim_set_delay_cb(self._cb)

    def pwm(self):
        v = [ctypes.c_uint32() for _ in range(4)]
        self.lib.sim_get_pwm(*[ctypes.byref(x) for x in v])
        return [x.value for x in v]


if __name__ == '__main__':
    mc, mh = find_main_c()
    if '--cross-win' in sys.argv:          # dung san DLL Windows (tu Linux, can ziglang)
        print(build(mc, mh, target='win'))
    else:
        print(build(mc, mh))
    print(read_cubemx(mc))
