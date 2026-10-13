# sim_v2.py - chạy THỬ node2.py + master2.py trên máy tính (CPython), không cần board.
# Giả lập: thời gian ảo, chân GPIO, motor + công tắc home, encoder AS5600, RTC memory,
# ESP-NOW (có thể rớt gói). Chạy:   python3 firmware/v2/sim_v2.py
# Kết quả: PASS/FAIL từng kịch bản + số gói mỗi bên đã gửi.

import os
import sys
import types
import random
import struct
import io
import contextlib
import time as _time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.dirname(HERE)]

# ======================================================================
# Thời gian ảo (MicroPython: time.ticks_ms / ticks_us ...)
# ======================================================================
CLOCK = [0]          # micro giây


def ticks_us():
    return CLOCK[0]


def ticks_ms():
    return CLOCK[0] // 1000


def ticks_diff(a, b):
    return a - b


def ticks_add(a, b):
    return a + b


def sleep_us(us):
    CLOCK[0] += int(us)


def sleep_ms(ms):
    CLOCK[0] += int(ms) * 1000


for _n, _f in dict(ticks_us=ticks_us, ticks_ms=ticks_ms, ticks_diff=ticks_diff,
                   ticks_add=ticks_add, sleep_us=sleep_us, sleep_ms=sleep_ms).items():
    setattr(_time, _n, _f)

# ======================================================================
# Phần cứng giả của từng board
# ======================================================================
CTX = [None]         # board đang chạy code


class Board:
    def __init__(self, name, mac, cause=1):
        self.name = name
        self.mac = mac
        self.cause = cause
        self.pins = {}
        self.rtc = b''
        self.inbox = []
        self.sent = {}
        self.motor = None


class Motor:
    """Motor + hộp số + công tắc home + nam châm encoder. Đơn vị: vi bước (1/8)."""

    def __init__(self, board, pos, encdir, offset):
        self.b = board
        self.pos = pos            # vị trí THẬT (vi bước), công tắc nhấn khi pos <= 0
        self.encdir = encdir
        self.offset = offset
        self.skip = 0             # số xung STEP sắp tới sẽ bị "mất bước"
        self.homedir = 0

    def pin(self, n):
        p = self.b.pins.get(n)
        return p.v if p else 0

    def step(self):
        if self.pin(2) == 1:      # EN cao = nhả motor
            return
        if self.skip > 0:
            self.skip -= 1
            return
        toward_home = self.pin(8) == self.homedir
        self.pos += -1 if toward_home else 1

    def switch(self):
        return 0 if self.pos <= 0 else 1

    def raw(self):
        return int(round(self.pos * 4096 / 1600 * self.encdir + self.offset)) % 4096


class Pin:
    IN, OUT, OPEN_DRAIN, PULL_UP, PULL_DOWN = 1, 2, 3, 4, 5

    def __init__(self, n, mode=None, pull=None, value=None):
        self.b = CTX[0]
        self.n = n
        self.v = 0 if value is None else value
        self.b.pins[n] = self

    def value(self, v=None):
        if v is None:
            if self.n == 3 and self.b.motor:
                return self.b.motor.switch()
            return self.v
        self.on() if v else self.off()

    def on(self):
        if self.n == 7 and self.v == 0 and self.b.motor:
            self.b.motor.step()
        self.v = 1

    def off(self):
        self.v = 0


class ADC:
    ATTN_11DB = 3

    def __init__(self, pin, atten=None):
        self.b = CTX[0]

    def read_uv(self):
        return int(12000 * 5 / 105 * 1000)


class RTC:
    def __init__(self):
        self.b = CTX[0]

    def memory(self, data=None):
        if data is None:
            return self.b.rtc
        self.b.rtc = bytes(data)


class I2C:
    def __init__(self, *a, **k):
        self.b = CTX[0]

    def readfrom_mem(self, addr, reg, n):
        if reg == 0x0B:
            return bytes([0x20])
        raw = self.b.motor.raw()
        return bytes([raw >> 8, raw & 0xFF])[:n]

    def readfrom_mem_into(self, addr, reg, buf):
        d = self.readfrom_mem(addr, reg, len(buf))
        buf[:len(d)] = d


machine = types.ModuleType('machine')
machine.Pin, machine.ADC, machine.RTC, machine.I2C = Pin, ADC, RTC, I2C
machine.WDT = lambda **k: types.SimpleNamespace(feed=lambda: None)
machine.PWRON_RESET, machine.HARD_RESET, machine.WDT_RESET = 1, 2, 3
machine.DEEPSLEEP_RESET, machine.SOFT_RESET = 4, 5
machine.reset_cause = lambda: CTX[0].cause
sys.modules['machine'] = machine

LOSS = [0.0]         # tỉ lệ rớt gói ESP-NOW
BOARDS = []


class ESPNow:
    def __init__(self):
        self.b = CTX[0]

    def config(self, **k):
        pass

    def active(self, on):
        pass

    def add_peer(self, mac):
        pass

    def any(self):
        return bool(self.b.inbox)

    def irecv(self, t=0):
        return self.b.inbox.pop(0) if self.b.inbox else (None, None)

    def send(self, peer, msg, sync=True):
        typ = msg[2]
        self.b.sent[typ] = self.b.sent.get(typ, 0) + 1
        for o in BOARDS:
            if o is self.b or not o.alive:
                continue
            if peer != b'\xff' * 6 and peer != o.mac:
                continue
            if random.random() < LOSS[0]:
                continue
            o.inbox.append((self.b.mac, bytes(msg)))


espnow = types.ModuleType('espnow')
espnow.ESPNow = ESPNow
sys.modules['espnow'] = espnow


class WLAN:
    PM_NONE = 0

    def __init__(self, *a):
        self.b = CTX[0]

    def active(self, *a):
        return True

    def disconnect(self):
        pass

    def config(self, *a, **k):
        if a and a[0] == 'mac':
            return self.b.mac


network = types.ModuleType('network')
network.WLAN, network.STA_IF = WLAN, 0
sys.modules['network'] = network


class NeoPixel:
    def __init__(self, pin, n):
        self.buf = [(0, 0, 0)] * n

    def __setitem__(self, i, v):
        self.buf[i] = v

    def write(self):
        pass


sys.modules['neopixel'] = types.SimpleNamespace(NeoPixel=NeoPixel)
sys.modules['esp32'] = types.SimpleNamespace(
    RMT=types.SimpleNamespace(bitstream_channel=lambda *a: None))

import delta_proto as P0          # noqa: E402
import proto2 as P                # noqa: E402
import node2                      # noqa: E402
import master2                    # noqa: E402

MAC = {i: P.mac_bytes(m) for m, i in P0.NODE_MACS.items()}

# Mỗi board giả có config2.json riêng (không ghi file ra máy tính)
node2.load_cfg = lambda: dict(node2.CFG_DEFAULT, **getattr(CTX[0], 'cfg', {}))
node2.save_cfg = lambda cfg: setattr(CTX[0], 'cfg', dict(cfg))


# ======================================================================
# Hệ thống giả lập
# ======================================================================
class Sim:
    def __init__(self, seed=1, loss=0.0):
        random.seed(seed)
        LOSS[0] = loss
        CLOCK[0] = 0
        BOARDS.clear()
        self.out = io.StringIO()
        self.mb = Board('master', P.mac_bytes(P.MASTER_MAC))
        self.mb.alive = True
        BOARDS.append(self.mb)
        self.nb = {}
        self.nodes = {}
        for i in (1, 2, 3):
            b = Board('n%d' % i, MAC[i])
            b.alive = True
            b.motor = Motor(b, pos=random.randint(300, 1200), encdir=random.choice((1, -1)),
                            offset=random.randint(0, 4095))
            BOARDS.append(b)
            self.nb[i] = b
            self.boot_node(i, cause=1)
        CTX[0] = self.mb
        with contextlib.redirect_stdout(self.out):
            self.m = master2.Master()

    def boot_node(self, i, cause, keep_rtc=True):
        b = self.nb[i]
        b.cause = cause
        b.inbox = []
        b.pins = {}
        if not keep_rtc:
            b.rtc = b''
        CTX[0] = b
        with contextlib.redirect_stdout(self.out):
            led = node2.StatusLed(1)
            n = node2.Node(i, led)
        n.ax.home_dir = 0
        b.motor.homedir = n.ax.home_dir
        self.nodes[i] = n

    def run(self, seconds, tick_us=50):
        end = CLOCK[0] + int(seconds * 1e6)
        last_m = -1
        with contextlib.redirect_stdout(self.out):
            while CLOCK[0] < end:
                CLOCK[0] += tick_us
                for i, n in self.nodes.items():
                    if not self.nb[i].alive:
                        continue
                    CTX[0] = self.nb[i]
                    n.ax.update()
                    n.service()
                if CLOCK[0] // 1000 != last_m:
                    last_m = CLOCK[0] // 1000
                    CTX[0] = self.mb
                    self.m.poll_rx()
                    self.m.service()

    def cmd(self, line):
        CTX[0] = self.mb
        with contextlib.redirect_stdout(self.out):
            self.m.handle_line(line)

    def log(self):
        return self.out.getvalue()

    def node_tx(self):
        return sum(sum(b.sent.values()) for b in self.nb.values())

    def true_deg(self, i):
        return self.nb[i].motor.pos / node2.STEPS_PER_DEG


RESULTS = []


def check(name, ok, info=''):
    RESULTS.append(ok)
    print('{} {}{}'.format('PASS' if ok else 'FAIL', name, (' - ' + info) if info else ''))


def scenario_basic(loss):
    s = Sim(seed=3, loss=loss)
    s.cmd('set all min 0.5')
    s.run(25)                                   # tự home lúc khởi động
    log = s.log()
    check('[loss {:.0%}] tự home lúc đầu'.format(loss), 'OK đã home lại xong tất cả' in log)
    check('[loss {:.0%}] dò được chiều encoder'.format(loss),
          all(n.cfg['encdir'] == s.nb[i].motor.encdir for i, n in s.nodes.items()),
          str([(n.cfg['encdir'], s.nb[i].motor.encdir) for i, n in s.nodes.items()]))
    s.m.neutral = [12.857] * 3
    s.cmd('clear')
    tx0 = s.node_tx()
    mtx0 = sum(s.mb.sent.values())
    random.seed(7)
    for k in range(40):                         # 40 lệnh nhìn trong 4 s
        s.cmd('look {} {} 0 100'.format(random.uniform(0, 360), random.uniform(0, 8)))
        s.run(0.1)
    s.run(1.0)
    tx = s.node_tx() - tx0
    mtx = sum(s.mb.sent.values()) - mtx0
    check('[loss {:.0%}] node gần như im lặng khi chạy'.format(loss), tx <= 9,
          'node gửi {} gói / 5 s (chủ yếu trả lời kiểm tra định kỳ), master gửi {}'.format(tx, mtx))
    tgt = [n.target / 1000 for n in s.m.nodes]
    err = max(abs(s.true_deg(i) - tgt[i - 1]) for i in (1, 2, 3))
    check('[loss {:.0%}] tới đúng đích cuối'.format(loss), err < 0.1, 'sai lệch {:.3f} độ'.format(err))
    return s


def scenario_lost_steps():
    s = Sim(seed=5)
    s.run(25)
    s.m.neutral = [12.857] * 3
    s.cmd('clear')
    s.nb[2].motor.skip = 60                     # n2 mất 60 vi bước (~1 độ) ở lệnh tới
    mark = len(s.log())
    s.cmd('move 5 5 5 600')
    s.run(2.0)
    e = abs(s.true_deg(2) - 5.0)
    check('mất 60 vi bước -> encoder tự bù', e < 0.1, 'n2 thật {:.3f} độ, đích 5.000'.format(s.true_deg(2)))
    check('không FAULT vì mất bước nhỏ', 'FAULT' not in s.log()[mark:])
    # bị đẩy lệch khi đứng yên
    s.nb[3].motor.pos += 50
    s.run(1.0)
    check('bị đẩy lệch khi đứng yên -> tự về lại', abs(s.true_deg(3) - 5.0) < 0.1,
          'n3 thật {:.3f} độ'.format(s.true_deg(3)))
    return s


def scenario_reset():
    s = Sim(seed=9)
    s.run(25)
    s.m.neutral = [12.857] * 3
    s.cmd('clear')
    s.cmd('move 8 8 8 500')
    s.run(1.5)
    mark = len(s.log())
    s.nb[1].motor.pos += 7                      # lúc reset, trục hơi trôi
    s.boot_node(1, cause=2, keep_rtc=True)      # HARD_RESET: RTC memory còn
    s.run(3.0)
    after = s.log()[mark:]
    check('HARD_RESET -> khôi phục vị trí, không home lại',
          'khôi phục vị trí' in after and 'HOME LẠI' not in after and s.nodes[1].homed)
    check('vị trí sau khôi phục đúng', abs(s.true_deg(1) - 8.0) < 0.1,
          'n1 thật {:.3f} độ'.format(s.true_deg(1)))
    mark = len(s.log())
    s.boot_node(3, cause=1, keep_rtc=False)     # mất điện hẳn: RTC mất -> phải home
    s.run(30)
    after = s.log()[mark:]
    check('mất điện -> home lại tất cả', 'HOME LẠI TẤT CẢ' in after and 'OK đã home lại xong' in after)
    return s


def scenario_jam():
    s = Sim(seed=13)
    s.run(25)
    s.cmd('clear')
    mark = len(s.log())
    s.nb[1].motor.skip = 10 ** 9                # n1 kẹt cứng: không xung nào làm trục quay
    s.cmd('move 10 10 10 800')
    s.run(4.0)
    out = s.log()[mark:]
    check('motor kẹt cứng -> báo STALL và dừng tất cả', 'ERR n1 STALL' in out and 'FAULT' in out)


def scenario_status():
    s = Sim(seed=11)
    s.run(25)
    mark = len(s.log())
    s.cmd('status')
    s.run(0.5)
    out = s.log()[mark:]
    check("'status' in đủ 3 dòng STAT ON", sum(1 for l in out.splitlines()
                                              if l.startswith('STAT n') and ' ON ' in l) == 3)
    s.nb[2].alive = False                       # n2 mất điện, không tự báo được
    s.run(12)
    check('node chết được phát hiện nhờ kiểm tra định kỳ', 'ERR n2 OFFLINE' in s.log())


if __name__ == '__main__':
    t0 = _time.time()
    scenario_basic(0.0)
    scenario_basic(0.2)
    scenario_lost_steps()
    scenario_reset()
    scenario_jam()
    scenario_status()
    print('\n{} / {} PASS  ({:.0f} s)'.format(sum(RESULTS), len(RESULTS), _time.time() - t0))
    if '-v' in sys.argv:
        print(Sim.__doc__ or '')
    sys.exit(0 if all(RESULTS) else 1)
