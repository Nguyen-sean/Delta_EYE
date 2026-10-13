# sim_v3.py - chạy THỬ node3.py + master3.py (2 mắt, 6 node) trên máy tính, không cần board.
# Giả lập: thời gian ảo, chân GPIO, motor + công tắc home, encoder AS5600, RTC memory,
# ESP-NOW (có thể rớt gói). Chạy:   python3 firmware/v3/sim_v3.py
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

import proto3 as P                # noqa: E402
import node3                      # noqa: E402
import master3                    # noqa: E402
import spm_ik as K                # noqa: E402

MAC = {i: P.mac_bytes(m) for m, i in P.NODE_MACS.items()}

# Mỗi board giả có config3.json riêng; master ghi eyes.txt / neutral3.txt vào thư mục tạm
node3.load_cfg = lambda: dict(node3.CFG_DEFAULT, **getattr(CTX[0], 'cfg', {}))
node3.save_cfg = lambda cfg: setattr(CTX[0], 'cfg', dict(cfg))
import tempfile                   # noqa: E402
os.chdir(tempfile.mkdtemp(prefix='sim_v3_'))


class Sim:
    def __init__(self, seed=1, loss=0.0, dead=()):
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
        for i in range(1, 7):
            b = Board('n%d' % i, MAC[i])
            b.alive = i not in dead
            b.motor = Motor(b, pos=random.randint(300, 1200), encdir=random.choice((1, -1)),
                            offset=random.randint(0, 4095))
            BOARDS.append(b)
            self.nb[i] = b
            self.boot_node(i, cause=1)
        CTX[0] = self.mb
        with contextlib.redirect_stdout(self.out):
            self.m = master3.Master()

    def boot_node(self, i, cause, keep_rtc=True):
        b = self.nb[i]
        b.cause = cause
        b.inbox = []
        b.pins = {}
        if not keep_rtc:
            b.rtc = b''
        CTX[0] = b
        with contextlib.redirect_stdout(self.out):
            led = node3.StatusLed(1)
            n = node3.Node(i, led)
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
        return self.nb[i].motor.pos / node3.STEPS_PER_DEG


RESULTS = []


def check(name, ok, info=''):
    RESULTS.append(ok)
    print('{} {}{}'.format('PASS' if ok else 'FAIL', name, (' - ' + info) if info else ''))


NEUTRAL = {1: [12.857, 12.857, 12.857], 2: [12.0, 13.5, 12.5]}   # 2 mắt calib khác nhau


def booted(eyes='12', dead=(), seed=3, loss=0.0):
    if os.path.exists('neutral3.txt'):
        os.remove('neutral3.txt')
    with open('eyes.txt', 'w') as f:
        f.write(eyes)
    s = Sim(seed=seed, loss=loss, dead=dead)
    s.run(25)
    s.m.neutral = {1: list(NEUTRAL[1]), 2: list(NEUTRAL[2])}
    s.cmd('clear')
    return s


def expect(s, eye, az, tilt):
    """Góc 3 node của 1 mắt tính độc lập (để so với vị trí thật của motor giả)."""
    a, t = s.m.eye_gaze(eye, az, tilt)
    th = K.ik(a, t, 0)
    return [NEUTRAL[eye][k] + K.wrap180(th[k] - s.m.th0[k]) for k in range(3)]


def err_eye(s, eye, az, tilt):
    exp = expect(s, eye, az, tilt)
    return max(abs(s.true_deg(nid) - exp[k]) for k, nid in enumerate(P.EYES[eye]))


def scenario_both(loss):
    s = booted('12', loss=loss)
    log = s.log()
    check('[loss {:.0%}] 2 mắt: tự home cả 6 node'.format(loss),
          'OK đã home lại xong tất cả' in log and all(n.homed for n in s.nodes.values()))
    tx0 = s.node_tx()
    for k in range(30):
        s.cmd('look {} {} 0 100'.format((k * 47) % 360, 2 + k % 6))
        s.run(0.1)
    s.cmd('look 30 6 0 300')
    s.run(1.5)
    e1, e2 = err_eye(s, 1, 30, 6), err_eye(s, 2, 30, 6)
    check('[loss {:.0%}] 2 mắt tới đúng hướng (calib riêng)'.format(loss), max(e1, e2) < 0.1,
          'sai lệch mắt 1 {:.3f}, mắt 2 {:.3f} độ'.format(e1, e2))
    check('[loss {:.0%}] node gần như im lặng'.format(loss), s.node_tx() - tx0 <= 14,
          '6 node gửi {} gói / 4.5 s'.format(s.node_tx() - tx0))
    same = [round(s.true_deg(i), 2) for i in (1, 2, 3)] != [round(s.true_deg(i), 2) for i in (4, 5, 6)]
    check('calib khác nhau -> góc motor 2 mắt khác nhau', same)


def scenario_verg():
    s = booted('12')
    s.cmd('verg 6')
    s.cmd('look 0 0 0 400')
    s.run(1.5)
    e1, e2 = err_eye(s, 1, 0, 0), err_eye(s, 2, 0, 0)
    g1, g2 = s.m.eye_gaze(1, 0, 0), s.m.eye_gaze(2, 0, 0)
    check('quy tụ 6 độ: mỗi mắt quay vào 3 độ, ngược chiều nhau',
          abs(g1[1] - 3) < 1e-6 and abs(g2[1] - 3) < 1e-6 and abs(abs(g1[0] - g2[0]) - 180) < 1e-6
          and max(e1, e2) < 0.1, 'mắt 1 {} / mắt 2 {}'.format(g1, g2))
    s.cmd('dist 0')
    check("'dist 0' tắt quy tụ", s.m.verg == 0)
    s.cmd('dist 620')
    check("'dist 620' -> quy tụ ~6 độ (IPD 65)", abs(s.m.verg - 6.0) < 0.05, '{:.2f}'.format(s.m.verg))


def scenario_one_eye(eye):
    other = 2 if eye == 1 else 1
    s = booted(str(eye), dead=P.EYES[other])         # mắt kia KHÔNG cắm điện
    log = s.log()
    check('chỉ bật mắt {} (mắt {} không cắm điện): home, không FAULT'.format(eye, other),
          'OK đã home lại xong tất cả' in log and 'FAULT' not in log)
    s.cmd('look 120 5 0 300')
    s.run(1.5)
    check('chỉ bật mắt {}: look chạy đúng'.format(eye), err_eye(s, eye, 120, 5) < 0.1)
    mark = len(s.log())
    s.cmd('status')
    s.run(0.5)
    out = s.log()[mark:]
    stat = [l for l in out.splitlines() if l.startswith('STAT n')]
    check("chỉ bật mắt {}: 'status' in 3 dòng của mắt đó".format(eye),
          len(stat) == 3 and all(' ON ' in l for l in stat)
          and all('STAT n{}'.format(i) in out for i in P.EYES[eye]))
    s.run(12)
    check('chỉ bật mắt {}: mắt tắt không bị báo OFFLINE'.format(eye), 'OFFLINE' not in s.log())


def scenario_switch_eyes():
    s = booted('1')                                   # cả 6 node có điện, chỉ bật mắt 1
    pos2 = [s.true_deg(i) for i in (4, 5, 6)]
    s.cmd('look 0 5 0 300')
    s.run(1.5)
    check('mắt tắt không nhận lệnh chạy', [s.true_deg(i) for i in (4, 5, 6)] == pos2)
    mark = len(s.log())
    p1 = [s.true_deg(i) for i in (1, 2, 3)]
    s.cmd('eyes 12')
    s.run(25)
    out = s.log()[mark:]
    check("'eyes 12' -> chỉ home mắt 2 vừa bật, mắt 1 giữ nguyên",
          'mắt 2 chưa biết vị trí' in out and out.count('Homing...') == 3
          and 'OK đã home lại xong' in out and [s.true_deg(i) for i in (1, 2, 3)] == p1)


def scenario_reset_eye2():
    s = booted('12')
    s.cmd('look 200 6 0 400')
    s.run(1.5)
    mark = len(s.log())
    s.boot_node(5, cause=2, keep_rtc=True)
    s.run(3)
    out = s.log()[mark:]
    check('node mắt 2 HARD_RESET -> khôi phục, không home lại',
          'khôi phục vị trí' in out and 'HOME LẠI' not in out and err_eye(s, 2, 200, 6) < 0.1)
    mark = len(s.log())
    pos1 = [s.true_deg(i) for i in (1, 2, 3)]
    s.boot_node(4, cause=1, keep_rtc=False)            # node mắt 2 mất điện hẳn
    s.run(30)
    out = s.log()[mark:]
    check('node mắt 2 mất điện -> chỉ home lại mắt 2',
          'HOME LẠI MẮT 2' in out and 'OK đã home lại xong' in out and 'MẮT 1' not in out,
          'mắt 1 trước/sau: {} / {}'.format([round(x, 2) for x in pos1],
                                            [round(s.true_deg(i), 2) for i in (1, 2, 3)]))


def scenario_center():
    s = booted('12')
    s.cmd('set all max 30')
    s.run(0.5)
    s.cmd('center')
    s.run(3)
    ok_all = all(abs(s.true_deg(i) - 15.25) < 0.05 for i in range(1, 7))
    s.cmd('center 2 1500')
    s.cmd('center e1')
    check("'center' đưa cả 6 node về giữa (0.5+30)/2 = 15.25", ok_all,
          str([round(s.true_deg(i), 2) for i in range(1, 7)]))
    s.cmd('set 4 max 20')
    s.run(0.5)
    s.cmd('center e2 1000')
    s.run(2)
    check("'center e2' theo hành trình riêng từng node (n4: (0.5+20)/2)",
          abs(s.true_deg(4) - 10.25) < 0.05 and abs(s.true_deg(5) - 15.25) < 0.05,
          str([round(s.true_deg(i), 2) for i in (4, 5, 6)]))


if __name__ == '__main__':
    import time as _t
    t0 = _t.time()
    scenario_both(0.0)
    scenario_both(0.2)
    scenario_verg()
    scenario_one_eye(1)
    scenario_one_eye(2)
    scenario_switch_eyes()
    scenario_reset_eye2()
    scenario_center()
    print('\n{} / {} PASS  ({:.0f} s)'.format(sum(RESULTS), len(RESULTS), _t.time() - t0))
    sys.exit(0 if all(RESULTS) else 1)
