# motor_node.py - code cho TẤT CẢ board động cơ (giống hệt nhau).
# Nạp lên mỗi board với tên main.py, kèm theo delta_proto.py.
#
# ID của board (1, 2, 3) được xác định bằng địa chỉ MAC, tra trong bảng NODE_MACS
# của delta_proto.py -> không cần cấu hình riêng từng board.
#
# Chuyển động chạy không chặn (non-blocking): vòng lặp chính phát từng xung STEP
# theo lịch thời gian, xen giữa là xử lý ESP-NOW -> lệnh STOP luôn có tác dụng
# ngay cả khi động cơ đang chạy, và watchdog luôn được "cho ăn".

from machine import Pin, ADC, RTC, WDT, reset_cause
from time import sleep_ms, sleep_us, ticks_ms, ticks_us, ticks_diff, ticks_add
from math import sqrt
import struct
import os
import sys
import gc
import json
import delta_proto as P

FW_VERSION = 1
CRASH_FILE = 'crash.txt'   # lỗi chương trình (nếu có) được ghi vào đây

PIN_EN    = 2     # A4988 EN active LOW
PIN_HOME  = 3     # cảm biến home: 1 chân GND, 1 chân IO3 (pull-up nội)
# A4988: MS1 = MS2 = 3.3 V (JP1, JP3 trên PCB), MS3 = IO4 LOW -> 1/8 bước
PIN_MS3   = 4     # TX_DRIVER trên sơ đồ
PIN_RESET = 5     # RX_DRIVER trên sơ đồ
PIN_SLEEP = 6     # CLK_DRIVER trên sơ đồ
PIN_STEP  = 7
PIN_DIR   = 8
# Theo sơ đồ Stepper_motor_control_V3
PIN_LED   = 1     # đèn NeoPixel WS2812B (LED1, qua R7 100R) báo trạng thái
PIN_VBUS  = 0     # đo VBUS qua cầu phân áp R9 100K / R8 5K (ADC1_CH0)

# ==== ĐO NGUỒN: VBUS = V_Motor (12 V từ cầu đấu J5; chỉ cắm USB thì ~4.3 V qua D3) ====
VBUS_DIV        = (100 + 5) / 5     # hệ số cầu phân áp R9 / R8
VBUS_LOW_MV     = 10500             # dưới mức này coi là sụt áp (nguồn 12 V)
VBUS_SAMPLE_MS  = 10                # đo VBUS mỗi 10 ms
POWER_PERIOD_MS = 2000              # gửi báo cáo điện áp cho master mỗi 2 s

# ==== ĐÈN TRẠNG THÁI ====
LED_COUNT     = 1
LED_MAX_PERCENT = 5   # độ sáng tối đa (%) - mọi hiệu ứng đều không vượt mức này
LED_BRIGHT    = int(255 * LED_MAX_PERCENT / 100)   # = 76 trên thang 0..255
LED_PERIOD_MS = 100    # cập nhật hiệu ứng ~10 lần/giây (ít lần ghi NeoPixel hơn)
LED_USE_RMT   = False  # False: ghi NeoPixel bằng bit-bang, không dùng bộ RMT của ESP32
# False = chế độ AN TOÀN: màu đứng yên theo trạng thái, không chớp / thở, không chớp khi nhận
# lệnh, không ghi đèn lúc motor đang chạy -> chỉ ghi đèn khi trạng thái đổi (vài lần mỗi phút).
# ESP32-C3 chỉ có 1 nhân: mỗi lần ghi NeoPixel làm Wi-Fi bị trễ (nghi gây panic / hỏng bộ nhớ).
LED_ANIMATE   = False
# Bảng màu (r, g, b) 0..1, nhân với LED_BRIGHT
C_RED    = (1.0, 0.0, 0.0)
C_GREEN  = (0.0, 1.0, 0.0)
C_BLUE   = (0.0, 0.2, 1.0)
C_YELLOW = (1.0, 0.6, 0.0)
C_ORANGE = (1.0, 0.25, 0.0)
C_CYAN   = (0.0, 1.0, 1.0)
C_PURPLE = (0.6, 0.0, 1.0)
C_WHITE  = (1.0, 1.0, 1.0)

# ==== CƠ KHÍ ====
# Driver A4988: MS1 = MS2 = 1 (cầu hàn trên PCB), MS3 = 0 -> 1/8 bước
MOTOR_FULL_STEPS    = 200
MICROSTEP           = 8
GEAR_RATIO          = 14
STEPS_PER_MOTOR_REV = MOTOR_FULL_STEPS * MICROSTEP            # 1600
STEPS_PER_DEG       = STEPS_PER_MOTOR_REV * GEAR_RATIO / 360.0  # ~62.22 bước / độ trục ra
USTEP_SCALE         = MICROSTEP / 8       # các hằng số bước bên dưới được chỉnh ở 1/8 bước

# ==== THÔNG SỐ CHỈNH ĐƯỢC TỪ MASTER (lệnh 'set'), lưu trong config.json trên board ====
# Hành trình tính bằng độ ở trục ra hộp số: 0 = điểm chạm công tắc home, dương = đi xa công tắc.
CFG_FILE = 'config.json'
CFG_DEFAULT = {
    'min': 0.5,          # giới hạn dưới (độ) - không về sát công tắc
    'max': 25.71,        # giới hạn trên (độ) - 25.71 = 1 vòng motor
    'homedir': 0,        # giá trị chân DIR khi quay VỀ PHÍA công tắc (0/1)
    'maxhz': 2000,       # tốc độ tối đa (bước/s)  (~32 độ/s trục ra ở 1/8 bước)
    'accel': 6000,       # gia tốc (bước/s^2)
    'homefast': 800,     # tốc độ dò home nhanh (bước/s)
    'homeslow': 150,     # tốc độ dò home chậm (bước/s)
    'led': 1,            # 1 = bật đèn NeoPixel, 0 = tắt hẳn
}
CFG_LIMITS = {           # khoảng hợp lệ - master gửi giá trị ngoài khoảng sẽ bị từ chối
    'min': (0.2, 20.0),
    'max': (2.0, 90.0),
    'homedir': (0, 1),
    'maxhz': (300, 8000),
    'accel': (500, 100000),
    'homefast': (100, 6000),
    'homeslow': (20, 2000),
    'led': (0, 1),
}
# Các thông số tính theo bước/s -> phải nhân lại khi đổi vi bước (config.json cũ lưu ở 1/8)
CFG_HZ_KEYS = ('maxhz', 'accel', 'homefast', 'homeslow')

# Khi đi về phía home, nếu cảm biến bị chạm ở vị trí > min + HOME_TOL thì coi là mất bước.
# (khoảng dung sai này để bỏ qua độ trễ nhả/đóng cơ khí của công tắc)
HOME_TOL_STEPS = int(48 * USTEP_SCALE)                # ~0.77 độ trục ra

# ==== HOMING ====
HOME_ACTIVE_LEVEL  = 0     # công tắc nối GND -> chạm = 0
HOME_BACKOFF_STEPS = int(160 * USTEP_SCALE)  # ~2.6 độ: lùi thêm sau khi công tắc đã NHẢ, rồi dò lại chậm
HOME_RELEASE_MAX   = int(800 * USTEP_SCALE)  # ~12.9 độ: lùi ngần này mà công tắc chưa nhả -> HOME_STUCK
SW_DEBOUNCE_US     = 500   # công tắc phải giữ nguyên mức liên tục 0.5 ms mới tính -> lọc nhiễu

# ==== CHUYỂN ĐỘNG ====
START_HZ     = int(300 * USTEP_SCALE)   # 600 ở 1/16 bước; tốc độ bắt đầu/kết thúc của đường tăng-giảm tốc
MIN_HZ       = 10          # tốc độ thấp nhất (bước rất ngắn mà thời gian dài -> chạy đều, chậm)
PULSE_US     = 3

# ==== WATCHDOG / LIÊN LẠC ====
ENABLE_WDT      = False    # đang thử nghiệm: tắt. Chạy ổn định thì đổi thành True
WDT_TIMEOUT_MS  = 5000     # vòng lặp chính treo quá 2s -> board tự reset
BOOT_DELAY_MS   = 3000     # chờ trước khi bật WDT, để kịp Ctrl+C khi cần nạp code
LINK_TIMEOUT_MS = 1000     # không nhận gì từ master trong 1s -> mất liên lạc
HELLO_PERIOD_MS = 1000
# Chỉ nhận lệnh từ master có MAC = P.MASTER_MAC (đặt trong delta_proto.py)

PRESSED  = HOME_ACTIVE_LEVEL
RELEASED = 1 - HOME_ACTIVE_LEVEL

# Các bước của homing
H_LEAVE, H_LEAVE_BACK, H_FAST, H_SLOW, H_PARK = range(5)


def load_cfg():
    cfg = dict(CFG_DEFAULT)
    try:
        with open(CFG_FILE) as f:
            saved = json.load(f)
        for k in CFG_DEFAULT:
            if k in saved:
                cfg[k] = saved[k]
        old = saved.get('ustep', 8)            # file cũ (chưa có 'ustep') được lưu ở 1/8 bước
        if old != MICROSTEP:
            for k in CFG_HZ_KEYS:
                if k in saved:
                    lo, hi = CFG_LIMITS[k]
                    cfg[k] = min(hi, max(lo, saved[k] * MICROSTEP / old))
            save_cfg(cfg)
            print("config.json: đổi vi bước 1/{} -> 1/{}, đã nhân lại tốc độ".format(old, MICROSTEP))
    except (OSError, ValueError):
        pass                                   # chưa có file -> dùng mặc định
    return cfg


def save_cfg(cfg):
    c = dict(cfg)
    c['ustep'] = MICROSTEP
    with open(CFG_FILE, 'w') as f:
        json.dump(c, f)


class PowerMon:
    """
    Đo VBUS (nguồn motor + nguồn vào bộ hạ áp 3.3 V) qua IO0.
    Mức thấp nhất từ lúc khởi động được ghi vào RTC memory (còn giữ qua HARD_RESET / WATCHDOG,
    mất khi rút điện) -> lần khởi động sau biết được VBUS có sụt ngay trước khi board reset không.
    """
    MAGIC = b'VB'

    def __init__(self):
        self.adc = ADC(Pin(PIN_VBUS), atten=ADC.ATTN_11DB)
        self.rtc = RTC()
        self.prev_min, self.prev_last = self._load()
        self.vbus = self.read_mv()
        self.vmin = self.vbus            # thấp nhất trong kỳ báo cáo hiện tại
        self.boot_min = self.vbus        # thấp nhất từ lúc khởi động
        self.low = self.vbus < VBUS_LOW_MV
        self._save()
        print("VBUS = {:.2f} V | trước lần reset này: thấp nhất {}, lần đo cuối {}".format(
            self.vbus / 1000, self._v(self.prev_min), self._v(self.prev_last)))

    @staticmethod
    def _v(mv):
        return '{:.2f} V'.format(mv / 1000) if mv else '?'

    def _load(self):
        try:
            m = self.rtc.memory()
            if len(m) >= 6 and m[:2] == self.MAGIC:
                return struct.unpack('<HH', m[2:6])
        except Exception:
            pass
        return 0, 0

    def _save(self):
        try:
            self.rtc.memory(self.MAGIC + struct.pack('<HH', min(self.boot_min, 65535),
                                                     min(self.vbus, 65535)))
        except Exception:
            pass

    def read_mv(self):
        return int(self.adc.read_uv() * VBUS_DIV / 1000)

    def sample(self):
        """Đo 1 lần. Trả về True nếu VBUS vừa tụt xuống dưới VBUS_LOW_MV."""
        v = self.read_mv()
        self.vbus = v
        if v < self.vmin:
            self.vmin = v
        if v < self.boot_min:
            self.boot_min = v
            self._save()
        was = self.low
        self.low = v < VBUS_LOW_MV
        return self.low and not was

    def report(self):
        """Payload POWER_FMT, rồi bắt đầu kỳ đo mới."""
        self._save()                     # lưu lần đo cuối (để biết nếu board reset ngay sau đó)
        p = struct.pack(P.POWER_FMT, min(self.vbus, 65535), min(self.vmin, 65535),
                        self.prev_min, self.prev_last)
        self.vmin = self.vbus
        return p


class Axis:
    """Phát xung STEP không chặn, có tăng/giảm tốc hình thang và dừng theo cảm biến."""

    def __init__(self, cfg):
        self.home_dir = int(cfg['homedir'])
        self.max_hz = cfg['maxhz']
        self.accel = cfg['accel']

        self.en = Pin(PIN_EN, Pin.OUT, value=1)
        self.ms3 = Pin(PIN_MS3, Pin.OUT, value=0)
        self.reset = Pin(PIN_RESET, Pin.OUT, value=1)
        self.sleep = Pin(PIN_SLEEP, Pin.OUT, value=1)
        self.dir = Pin(PIN_DIR, Pin.OUT, value=self.home_dir)
        self.step_pin = Pin(PIN_STEP, Pin.OUT, value=0)
        self.home_sw = Pin(PIN_HOME, Pin.IN, Pin.PULL_UP)

        self.pos = 0
        self.active = False
        self.enabled = False
        self.stopped_sw = False     # đoạn chạy vừa rồi kết thúc do cảm biến

        self._total = 0
        self._done = 0
        self._sign = 1
        self._v0 = START_HZ
        self._vmax = self.max_hz
        self._next = 0
        self._stop_level = None
        self._guard = None
        self._sw_since = None
        self._armed = True
        self._vs = START_HZ
        self.v = 0                  # tốc độ hiện tại (bước/s)

    def enable(self):
        # RESET, SLEEP active LOW -> HIGH; EN active LOW -> LOW
        self.reset.on()
        self.sleep.on()
        sleep_ms(2)
        self.en.off()
        self.enabled = True

    def disable(self):
        self.active = False
        self.en.on()
        self.enabled = False

    def at_home(self):
        """Đọc cảm biến có chống dội (dùng khi đứng yên): bị nhấn liên tục ~1 ms."""
        for _ in range(5):
            if self.home_sw.value() != PRESSED:
                return False
            sleep_us(250)
        return True

    def start(self, steps, toward_home, vmax, stop_when=None, guard=None, edge=False, vs=None):
        """
        Bắt đầu chạy 'steps' bước.
        stop_when: None / PRESSED / RELEASED - dừng sớm khi cảm biến ở mức đó.
        guard: chỉ xét cảm biến khi pos > guard (None = luôn xét).
        edge: chỉ dừng khi cảm biến CHUYỂN sang mức stop_when (phải thấy mức ngược lại trước).
              Dùng khi công tắc có thể vẫn đang bị nhấn sẵn do hành trình nhả dài.
        vs: tốc độ hiện tại khi đổi đích lúc đang chạy CÙNG CHIỀU (None = xuất phát từ đứng yên).
        """
        self._armed = not edge or self.home_sw.value() != stop_when
        self.stopped_sw = False
        if steps <= 0:
            self.active = False
            return
        continuing = vs is not None and self.active
        self.dir.value(self.home_dir if toward_home else 1 - self.home_dir)
        self._sign = -1 if toward_home else 1
        self._total = steps
        self._done = 0
        self._vmax = max(MIN_HZ, min(vmax, CFG_LIMITS['maxhz'][1]))
        self._v0 = min(START_HZ, self._vmax)
        self._vs = max(self._v0, vs) if continuing else self._v0
        self.v = self._vs
        self._stop_level = stop_when
        self._guard = guard
        self._sw_since = None
        if not continuing:
            self._next = ticks_add(ticks_us(), 20)   # thời gian setup DIR
        self.active = True

    def stop_steps(self):
        """Số bước cần để giảm tốc từ tốc độ hiện tại xuống dừng."""
        if not self.active:
            return 0
        return int((self.v * self.v - self._v0 * self._v0) / (2 * self.accel)) + 1

    def brake(self):
        """Giảm tốc mượt rồi dừng (không dừng gấp): rút ngắn đoạn đang chạy."""
        if self.active:
            self._total = min(self._total, self._done + self.stop_steps())

    def abort(self):
        self.active = False
        self.step_pin.off()

    def update(self):
        """Gọi liên tục trong vòng lặp chính. Phát tối đa 1 xung mỗi lần gọi."""
        if not self.active:
            return

        now = ticks_us()
        if self._stop_level is not None and not self._armed:
            if self.home_sw.value() != self._stop_level:
                self._armed = True             # đã thấy công tắc nhả -> từ giờ mới canh
        elif self._stop_level is not None and (self._guard is None or self.pos > self._guard):
            if self.home_sw.value() == self._stop_level:
                if self._sw_since is None:
                    self._sw_since = now
                elif ticks_diff(now, self._sw_since) >= SW_DEBOUNCE_US:
                    self.active = False        # giữ mức đủ lâu -> thật, không phải nhiễu
                    self.stopped_sw = True
                    return
            else:
                self._sw_since = None

        late = ticks_diff(now, self._next)
        if late < 0:
            return

        self.step_pin.on()
        sleep_us(PULSE_US)
        self.step_pin.off()
        self._done += 1
        self.pos += self._sign

        if self._done >= self._total:
            self.active = False
            return

        # Tốc độ = min(đường tăng tốc từ vs, đường giảm tốc về v0 tại đích)
        a = self.accel
        vs = self._vs
        vmax = self._vmax
        if vs <= vmax:
            v = sqrt(vs * vs + 2 * a * self._done)
            if v > vmax:
                v = vmax
        else:                                        # đang nhanh hơn vmax mới -> giảm dần về vmax
            v = vs * vs - 2 * a * self._done
            v = sqrt(v) if v > vmax * vmax else vmax
        vd = sqrt(self._v0 * self._v0 + 2 * a * (self._total - self._done))
        if vd < v:
            v = vd
        if v < MIN_HZ:
            v = MIN_HZ
        self.v = v
        period = int(1000000 / v)
        # Bị trễ hơn 1 chu kỳ (vd đang xử lý ESP-NOW) -> đặt lại mốc, không phát dồn xung
        if late > period:
            self._next = ticks_add(now, period)
        else:
            self._next = ticks_add(self._next, period)


class StatusLed:
    """
    Đèn NeoPixel báo trạng thái node (ưu tiên từ trên xuống):
      trắng nháy nhanh    : nhận diện (lệnh 'blink' từ master)
      trắng chớp 1 cái    : vừa nhận 1 lệnh
      đỏ nháy nhanh       : LỖI (mất bước, không tìm thấy home...)
      đỏ / xanh dương xen : LỖI mất liên lạc master khi đang chạy
      tím thở chậm        : đã nhả động cơ (release)
      vàng nháy           : đang homing
      xanh ngọc sáng      : đang chạy
      xanh dương thở      : chưa có liên lạc với master
      vàng thở            : chưa home
      cam                 : đứng yên và công tắc home đang bị nhấn
      xanh lá             : sẵn sàng (đã home, đứng yên)
    """

    def __init__(self, enabled=True):
        self.np = None
        self.enabled = False
        self.last = None
        self.set_enabled(enabled)
        self.t_upd = ticks_ms()
        self.flash_until = self.t_upd
        self.ident_until = self.t_upd

    def set_enabled(self, on):
        """Bật / tắt đèn. Tắt: ghi màu đen 1 lần rồi không đụng tới chân đèn nữa."""
        on = bool(on)
        if on and self.np is None:
            if not LED_USE_RMT:
                try:
                    # Ghi NeoPixel bằng bit-bang thay vì bộ RMT: tránh lỗi RMT + Wi-Fi/ESP-NOW
                    # (nghi gây hỏng bộ nhớ -> TypeError lạ / HARD_RESET)
                    import esp32
                    esp32.RMT.bitstream_channel(None)
                except Exception as ex:
                    print("Không tắt được RMT cho NeoPixel:", ex)
            try:
                from neopixel import NeoPixel
                self.np = NeoPixel(Pin(PIN_LED, Pin.OUT), LED_COUNT)
            except Exception as ex:      # board không có neopixel -> bỏ qua đèn
                print("Không dùng được NeoPixel:", ex)
                self.np = None
        if not on and self.enabled and self.np is not None:
            for i in range(LED_COUNT):
                self.np[i] = (0, 0, 0)
            self.np.write()
        self.enabled = on and self.np is not None
        self.last = None

    def flash(self, ms=60):
        if LED_ANIMATE:                      # chế độ an toàn: không chớp khi nhận lệnh
            self.flash_until = ticks_add(ticks_ms(), ms)

    def ident(self, ms=4000):
        self.ident_until = ticks_add(ticks_ms(), ms)

    def show(self, color, level=1.0):
        if not self.enabled:
            return
        k = LED_BRIGHT * level
        rgb = (int(color[0] * k), int(color[1] * k), int(color[2] * k))
        if rgb != self.last:             # chỉ ghi khi đổi màu -> không làm giật xung STEP
            for i in range(LED_COUNT):
                self.np[i] = rgb
            self.np.write()
            self.last = rgb

    @staticmethod
    def _blink(now, period):
        return (now % period) < period // 2

    @staticmethod
    def _breathe(now, period):
        x = (now % period) / period                  # 0..1
        x = 2 * x if x < 0.5 else 2 * (1 - x)        # tam giác 0..1..0
        return 0.05 + 0.95 * x * x                   # bình phương -> sáng tắt mượt hơn

    def update(self, node):
        if not self.enabled:
            return
        if not LED_ANIMATE and node.ax.active:
            return                           # chế độ an toàn: không ghi đèn khi đang phát xung
        now = ticks_ms()
        if ticks_diff(now, self.t_upd) < LED_PERIOD_MS:
            return
        self.t_upd = now
        if LED_ANIMATE:
            blink, breathe = self._blink, self._breathe
        else:                                # màu đứng yên: chỉ đổi khi trạng thái đổi
            blink, breathe = (lambda t, p: True), (lambda t, p: 0.5)
        st = node.state
        if ticks_diff(self.ident_until, now) > 0:
            self.show(C_WHITE, 1.0 if self._blink(now, 150) else 0.0)   # 'blink': vẫn nháy 4 s
        elif ticks_diff(self.flash_until, now) > 0:
            self.show(C_WHITE, 0.6)
        elif st == P.ST_ERROR:
            if node.err == P.E_LINK_LOST:
                self.show(C_RED if blink(now, 500) else C_BLUE)
            else:
                self.show(C_RED, 1.0 if blink(now, 250) else 0.0)
        elif st == P.ST_DISABLED:
            self.show(C_PURPLE, breathe(now, 3000))
        elif st == P.ST_HOMING:
            self.show(C_YELLOW, 1.0 if blink(now, 400) else 0.1)
        elif st == P.ST_MOVING:
            self.show(C_CYAN)
        elif not node.link_ok:
            self.show(C_BLUE, breathe(now, 2000))
        elif not node.homed:
            self.show(C_YELLOW, breathe(now, 2000))
        elif node.ax.home_sw.value() == PRESSED:
            self.show(C_ORANGE, 0.5)
        else:
            self.show(C_GREEN, 0.3)


def steps_to_mdeg(steps):
    return int(round(steps * 1000 / STEPS_PER_DEG))


class Node:
    def __init__(self, node_id, led):
        self.led = led
        self.id = node_id
        self.bit = 1 << (node_id - 1)
        self.cfg = load_cfg()
        self.ax = Axis(self.cfg)
        self.pwr = PowerMon()
        self.apply_cfg()
        self.ax.enable()
        print("Cấu hình:", self.cfg)

        self.state = P.ST_IDLE
        self.err = P.E_NONE
        self.homed = False
        self.target = 0
        self.hphase = 0
        self.deadline = 0
        self.pending = False            # đang giảm tốc để đổi đích, xong sẽ chạy tới self.target
        self._last_status = None        # (state, err, flags) đã gửi lần trước
        self.move_t0 = 0
        self.move_dur = 0

        self.e, mac = P.init_espnow()
        self.master = None
        self.link_ok = False
        self.last_rx = ticks_ms()
        self.last_hello = ticks_ms() - HELLO_PERIOD_MS
        self.last_sup = ticks_ms()
        self.tx_seq = 0
        self.tx_err = 0
        self.recent = []                # [(seq, ack_pkt)] các lệnh vừa xử lý
        self.reset_cause = reset_cause()
        self.boot_id = struct.unpack('<H', os.urandom(2))[0]   # để master biết node đã reset
        print("Node {} MAC {} | reset: {}".format(
            node_id, P.mac_str(mac), P.name(P.RESET_NAMES, self.reset_cause)))
        self.last_vbus = ticks_ms()
        self.last_pwr = ticks_ms()

    # ---------------- cấu hình ----------------
    def apply_cfg(self):
        """Tính lại các giá trị dùng khi chạy từ self.cfg."""
        c = self.cfg
        self.min_steps = int(c['min'] * STEPS_PER_DEG)
        self.max_steps = int(c['max'] * STEPS_PER_DEG)
        self.search_steps = int(self.max_steps * 1.3)     # dò home: hết hành trình + 30%
        self.home_fast = c['homefast']
        self.home_slow = c['homeslow']
        self.home_timeout_ms = int(self.search_steps / self.home_fast * 2000) + 10000
        self.ax.home_dir = int(c['homedir'])
        self.ax.max_hz = c['maxhz']
        self.ax.accel = c['accel']
        self.led.set_enabled(c['led'])

    def set_cfg(self, payload):
        if len(payload) != struct.calcsize(P.SETCFG_FMT):
            return P.R_BAD_CMD
        idx, val = struct.unpack(P.SETCFG_FMT, payload)
        if idx >= len(P.CFG_KEYS):
            return P.R_BAD_CMD
        key = P.CFG_KEYS[idx]
        lo, hi = CFG_LIMITS[key]
        if not lo <= val <= hi:
            return P.R_BAD_VALUE
        new = dict(self.cfg)
        new[key] = int(val) if key in ('homedir', 'led') else round(val, 3)
        if new['max'] - new['min'] < 2:
            return P.R_BAD_VALUE                       # hành trình phải >= 2 độ
        if key == 'homedir' and new[key] != self.cfg[key]:
            self.homed = False                         # đổi chiều home -> phải home lại
        self.cfg = new
        self.apply_cfg()
        save_cfg(self.cfg)
        print("Đổi cấu hình: {} = {}".format(key, new[key]))
        return P.R_OK

    def send_cfg(self):
        self._tx(P.MSG_CFG, struct.pack(P.CFG_FMT, *[float(self.cfg[k]) for k in P.CFG_KEYS]))

    # ---------------- gửi ----------------
    def _tx(self, typ, payload, dest=None):
        self.tx_seq = (self.tx_seq + 1) & 0xFFFF
        pkt = P.pack(typ, self.id, self.tx_seq, payload)
        self._send(pkt, dest)
        return pkt

    def _send(self, pkt, dest=None):
        try:
            # sync=False: không chờ ACK lớp MAC để không làm giật xung STEP
            self.e.send(dest or self.master or P.BROADCAST, pkt, False)
        except OSError as ex:
            self.tx_err += 1
            if self.tx_err % 50 == 1:
                print("Lỗi gửi ESP-NOW:", ex)

    def send_status(self, force=False):
        """Gửi STATUS. force=False: chỉ gửi khi state / lỗi / cờ đổi (bớt phát sóng)."""
        flags = 0
        if self.homed:
            flags |= P.F_HOMED
        if self.ax.home_sw.value() == PRESSED:
            flags |= P.F_HOME_SW
        if self.ax.enabled:
            flags |= P.F_ENABLED
        key = (self.state, self.err, flags)
        if not force and key == self._last_status:
            return                       # không đổi gì -> chờ lần trả lời PING kế tiếp
        self._last_status = key
        self._tx(P.MSG_STATUS, struct.pack(P.STATUS_FMT, self.state, self.err, flags,
                                           steps_to_mdeg(self.ax.pos),
                                           steps_to_mdeg(self.target)))

    # ---------------- nhận ----------------
    def poll_rx(self):
        while self.e.any():
            mac, msg = self.e.irecv(0)
            if msg is None:
                return
            pkt = P.unpack(msg)
            if pkt is None:
                continue
            typ, mask, seq, payload = pkt
            if typ & 0x80:
                continue                      # bản tin của node khác
            mac = bytes(mac)
            if not P.is_master(mac):
                continue                      # không phải master đã khai báo
            self._learn_master(mac)
            self.last_rx = ticks_ms()
            if not self.link_ok:
                self.link_ok = True
                print("Đã kết nối master", P.mac_str(mac))

            if not (mask & self.bit):
                continue
            if typ == P.CMD_PING:
                self.send_status(force=True)     # nhịp tim: luôn trả lời (kèm vị trí mới nhất)
                continue

            ack = self._recent_ack(seq)
            if ack is None:                   # lệnh mới -> thực hiện
                self.led.flash()
                res = self.execute(typ, payload)
                if res not in (P.R_OK, P.R_CLAMPED):
                    print("Từ chối {}: {}".format(P.name(P.CMD_NAMES, typ),
                                                  P.name(P.RES_NAMES, res)))
                ack = P.pack(P.MSG_ACK, self.id, seq, struct.pack(P.ACK_FMT, seq, typ, res))
                self.recent.append((seq, ack))
                if len(self.recent) > 6:
                    self.recent.pop(0)
            self._send(ack)                   # lệnh gửi lại -> chỉ ACK lại, không làm lại
            if typ in (P.CMD_GETCFG, P.CMD_SETCFG):
                self.send_cfg()
            self.send_status()

    def _recent_ack(self, seq):
        for s, a in self.recent:
            if s == seq:
                return a
        return None

    def _learn_master(self, mac):
        if mac == self.master:
            return
        try:
            self.e.add_peer(mac)
        except OSError:
            pass                              # peer đã tồn tại
        self.master = mac

    # ---------------- xử lý lệnh ----------------
    def execute(self, typ, payload):
        ax = self.ax

        # Các lệnh luôn được chấp nhận
        if typ == P.CMD_STOP:
            ax.abort()
            self.pending = False
            if self.state in (P.ST_MOVING, P.ST_HOMING):
                self.state = P.ST_IDLE
            self.target = ax.pos
            return P.R_OK
        if typ == P.CMD_RELEASE:
            ax.disable()
            self.homed = False
            self.state = P.ST_DISABLED
            self.err = P.E_NONE
            self.target = ax.pos
            return P.R_OK
        if typ == P.CMD_ENABLE:
            ax.enable()
            if self.state == P.ST_DISABLED:
                self.state = P.ST_IDLE
            return P.R_OK
        if typ == P.CMD_CLEAR:
            if self.state == P.ST_ERROR:
                self.state = P.ST_IDLE if ax.enabled else P.ST_DISABLED
            self.err = P.E_NONE
            return P.R_OK
        if typ == P.CMD_IDENT:
            self.led.ident()
            return P.R_OK
        if typ == P.CMD_GETCFG:
            return P.R_OK                     # MSG_CFG được gửi kèm sau ACK
        if typ == P.CMD_SETCFG:
            if self.state in (P.ST_HOMING, P.ST_MOVING):
                return P.R_BUSY
            return self.set_cfg(payload)
        if typ == P.CMD_SPEED:                # lệnh cũ: đổi tạm, không lưu
            if len(payload) != struct.calcsize(P.SPEED_FMT):
                return P.R_BAD_CMD
            hz, acc = struct.unpack(P.SPEED_FMT, payload)
            ax.max_hz = max(START_HZ, min(hz, CFG_LIMITS['maxhz'][1]))
            ax.accel = max(500, acc)
            return P.R_OK

        # Các lệnh chuyển động
        if self.state == P.ST_ERROR:
            return P.R_IN_ERROR
        if self.state == P.ST_DISABLED:
            return P.R_DISABLED
        if self.state == P.ST_HOMING:
            return P.R_BUSY
        if self.state == P.ST_MOVING and typ != P.CMD_MOVE:
            return P.R_BUSY                   # đang chạy chỉ nhận MOVE (đổi đích)

        if typ == P.CMD_HOME:
            self.start_homing()
            return P.R_OK
        if typ == P.CMD_MOVE:
            if len(payload) != struct.calcsize(P.MOVE_FMT):
                return P.R_BAD_CMD
            if not self.homed:
                return P.R_NOT_HOMED
            vals = struct.unpack(P.MOVE_FMT, payload)
            return self.start_move(vals[self.id], vals[0])
        return P.R_BAD_CMD

    def start_move(self, mdeg, dur_ms):
        res = P.R_OK
        target = int(round(mdeg * STEPS_PER_DEG / 1000))
        if target < self.min_steps or target > self.max_steps:
            target = min(max(target, self.min_steps), self.max_steps)
            res = P.R_CLAMPED
        self.target = target
        self.move_t0 = ticks_ms()
        self.move_dur = dur_ms
        ax = self.ax

        if self.state == P.ST_MOVING and ax.active:
            # ĐỔI ĐÍCH khi đang chạy
            delta = target - ax.pos
            same_dir = delta != 0 and (delta > 0) == (ax._sign > 0)
            if same_dir and abs(delta) >= ax.stop_steps():
                self.pending = False
                self._run_to(target, dur_ms, ax.v)    # chạy tiếp, giữ nguyên tốc độ hiện tại
            else:
                # ngược chiều / quá gần để dừng kịp -> giảm tốc dừng mượt rồi mới đi tới đích mới
                ax.brake()
                self.pending = True
            return res

        self.pending = False
        if self._run_to(target, dur_ms, None):
            self.state = P.ST_MOVING
        return res

    def _run_to(self, target, dur_ms, vs):
        """Chạy tới target. vs = tốc độ hiện tại nếu đang chạy cùng chiều. Trả về False nếu đã ở đích."""
        ax = self.ax
        delta = target - ax.pos
        if delta == 0:
            return False
        steps = abs(delta)
        vmax = ax.max_hz if dur_ms <= 0 else self._vmax_for(steps, dur_ms, vs)
        if delta < 0:
            # Đi về phía home thì canh cảm biến: chạm sớm = mất bước
            ax.start(steps, True, vmax, PRESSED, self.min_steps + HOME_TOL_STEPS, edge=True, vs=vs)
        else:
            ax.start(steps, False, vmax, vs=vs)
        est_ms = int((steps / vmax + vmax / ax.accel) * 1000)
        self.deadline = ticks_add(ticks_ms(), est_ms * 2 + 500)
        return True

    def _vmax_for(self, steps, dur_ms, vs=None):
        """
        Tốc độ đỉnh v để chạy 'steps' bước trong đúng 'dur_ms', xuất phát ở tốc độ vs,
        kết thúc ở v0 = START_HZ, gia tốc a.
        Tăng tốc vs -> v:  v^2 - (vs + v0 + aT) v + ((vs^2 + v0^2)/2 + aS) = 0  (nghiệm nhỏ)
        Giảm tốc vs -> v:  v = (S - (vs^2 - v0^2)/2a) / (T - (vs - v0)/a)
        """
        T = dur_ms / 1000
        S = steps
        a = self.ax.accel
        v0 = START_HZ
        vs = v0 if vs is None or vs < v0 else vs
        if vs == v0 and S / T <= v0:
            return max(MIN_HZ, S / T)       # quãng ngắn: chạy đều, không cần tăng tốc
        k = vs + v0 + a * T
        disc = k * k - 4 * ((vs * vs + v0 * v0) / 2 + a * S)
        if disc < 0:
            return self.ax.max_hz           # không kịp -> chạy nhanh nhất có thể
        v = (k - sqrt(disc)) / 2
        if v >= vs:
            return min(self.ax.max_hz, v)
        den = T - (vs - v0) / a             # đang nhanh hơn cần -> giảm tốc về v rồi chạy đều
        if den > 0:
            v = (S - (vs * vs - v0 * v0) / (2 * a)) / den
            if v >= MIN_HZ:
                return min(self.ax.max_hz, v)
        return max(MIN_HZ, min(self.ax.max_hz, S / T))

    # ---------------- homing ----------------
    def start_homing(self):
        self.homed = False
        self.state = P.ST_HOMING
        self.deadline = ticks_add(ticks_ms(), self.home_timeout_ms)
        print("Homing...")
        # Trình tự: [đang nhấn -> lùi đến khi nhả] -> chạy nhanh tới công tắc
        #           -> lùi đến khi NHẢ -> lùi thêm HOME_BACKOFF_STEPS -> dò lại chậm = điểm 0
        self.hfound = False
        if self.ax.at_home():
            self._home_leave()
        else:
            self._home_fast()

    def _home_leave(self):
        # Lùi ra cho tới khi công tắc nhả (không lùi cố định -> chịu được
        # công tắc cần gạt có hành trình nhả dài và độ rơ hộp số)
        self.hphase = H_LEAVE
        self.ax.start(HOME_RELEASE_MAX, False, self.home_slow, RELEASED)

    def _home_fast(self):
        self.hphase = H_FAST
        self.ax.start(self.search_steps, True, self.home_fast, PRESSED)

    def _homing_step(self):
        ax = self.ax
        ph = self.hphase
        if ph == H_LEAVE:
            if not ax.stopped_sw:
                print("Lùi {} bước mà công tắc vẫn chưa nhả".format(ax._done))
                return self.fail(P.E_HOME_STUCK)
            print("Công tắc nhả sau khi lùi {} bước".format(ax._done))
            self.hphase = H_LEAVE_BACK
            ax.start(HOME_BACKOFF_STEPS, False, self.home_fast)
        elif ph == H_LEAVE_BACK:
            if ax.at_home():
                print("Công tắc lại bị nhấn sau khi đã nhả -> nghi nhiễu dây IO3")
                return self.fail(P.E_HOME_STUCK)
            if self.hfound:
                self.hphase = H_SLOW
                ax.start(HOME_RELEASE_MAX + HOME_BACKOFF_STEPS * 2, True, self.home_slow, PRESSED)
            else:
                self._home_fast()
        elif ph == H_FAST:
            if not ax.stopped_sw:
                return self.fail(P.E_HOME_NOT_FOUND)
            self.hfound = True
            self._home_leave()
        elif ph == H_SLOW:
            if not ax.stopped_sw:
                return self.fail(P.E_HOME_NOT_FOUND)
            ax.pos = 0
            self.homed = True
            self.hphase = H_PARK
            self.target = self.min_steps
            ax.start(self.min_steps, False, self.home_fast)
        elif ph == H_PARK:
            self.state = P.ST_IDLE
            print("Home OK")
            self.send_status()

    # ---------------- giám sát ----------------
    def fail(self, err):
        self.ax.abort()
        self.state = P.ST_ERROR
        self.err = err
        self.pending = False
        if err in (P.E_LOST_STEP, P.E_HOME_NOT_FOUND, P.E_HOME_STUCK):
            self.homed = False
        self.target = self.ax.pos
        print("LỖI:", P.name(P.ERR_NAMES, err), "pos =", self.ax.pos)
        self.send_status()

    def supervise(self):
        now = ticks_ms()
        ax = self.ax

        # Kết thúc chuyển động
        if self.state == P.ST_MOVING and not ax.active:
            if ax.stopped_sw:
                self.fail(P.E_LOST_STEP)
            elif self.pending:
                # vừa giảm tốc dừng xong do đổi đích -> đi tiếp tới đích mới trong thời gian còn lại
                self.pending = False
                remain = 0
                if self.move_dur > 0:
                    remain = max(1, self.move_dur - ticks_diff(now, self.move_t0))
                if not self._run_to(self.target, remain, None):
                    self.state = P.ST_IDLE
                    self.send_status()
            else:
                self.state = P.ST_IDLE
                self.send_status()
        elif self.state == P.ST_HOMING and not ax.active:
            self._homing_step()

        # Watchdog chuyển động
        if self.state in (P.ST_MOVING, P.ST_HOMING) and ticks_diff(now, self.deadline) > 0:
            self.fail(P.E_MOTION_TIMEOUT)

        # Đo VBUS; báo master định kỳ, hoặc ngay khi vừa sụt áp
        if ticks_diff(now, self.last_vbus) >= VBUS_SAMPLE_MS:
            self.last_vbus = now
            dropped = self.pwr.sample()
            if dropped:
                print("VBUS sụt: {:.2f} V".format(self.pwr.vbus / 1000))
            if self.link_ok and (dropped or ticks_diff(now, self.last_pwr) >= POWER_PERIOD_MS):
                self.last_pwr = now
                self._tx(P.MSG_POWER, self.pwr.report())

        # Watchdog liên lạc
        if self.link_ok and ticks_diff(now, self.last_rx) > LINK_TIMEOUT_MS:
            self.link_ok = False
            print("Mất liên lạc master!")
            if self.state in (P.ST_MOVING, P.ST_HOMING):
                self.fail(P.E_LINK_LOST)

        # Chưa có liên lạc -> phát HELLO để master tìm thấy
        if not self.link_ok and ticks_diff(now, self.last_hello) >= HELLO_PERIOD_MS:
            self.last_hello = now
            self._tx(P.MSG_HELLO, struct.pack(P.HELLO_FMT, self.reset_cause, FW_VERSION,
                                                 self.boot_id),
                     P.BROADCAST)

    def service(self):
        if self.e.any():
            self.poll_rx()
        now = ticks_ms()
        if ticks_diff(now, self.last_sup) >= 2:
            self.last_sup = now
            self.supervise()
            self.led.update(self)


def load_node_id():
    """Tra ID theo MAC của board trong bảng NODE_MACS. Trả về None nếu không có."""
    import network
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    mac = sta.config('mac')
    nid = P.node_id_of(mac)
    if nid is None:
        print("LỖI: MAC {} chưa có trong NODE_MACS (file delta_proto.py).".format(P.mac_str(mac)))
        print("Thêm dòng  '{}': <số node>,  vào NODE_MACS rồi nạp lại delta_proto.py".format(
            P.mac_str(mac)))
        return None
    if not 1 <= nid <= P.NUM_NODES:
        print("LỖI: node_id = {} không hợp lệ, phải từ 1 đến {}".format(nid, P.NUM_NODES))
        return None
    return nid


def main():
    led = StatusLed(load_cfg()['led'])     # 'set all led 0' -> đèn tắt hẳn ngay từ lúc khởi động
    led.show(C_WHITE, 0.3)                 # trắng mờ: đang khởi động
    node_id = load_node_id()
    if node_id is None:
        led.show(C_RED, 0.5)               # đỏ đứng yên: MAC chưa có trong NODE_MACS
        return
    use_wdt = ENABLE_WDT and P.wdt_allowed()
    if use_wdt:
        print("Node {} khởi động sau {} s (Ctrl+C để dừng trước khi bật watchdog)".format(
            node_id, BOOT_DELAY_MS // 1000))
        sleep_ms(BOOT_DELAY_MS)

    node = Node(node_id, led)
    wdt = WDT(timeout=WDT_TIMEOUT_MS) if use_wdt else None
    ax = node.ax
    last_gc = ticks_ms()

    while True:
        try:
            while True:
                ax.update()
                node.service()
                if wdt:
                    wdt.feed()
                if not ax.active:
                    # dọn bộ nhớ chủ động lúc đứng yên, để GC không tự chạy giữa lúc phát xung
                    if ticks_diff(ticks_ms(), last_gc) > 2000:
                        gc.collect()
                        last_gc = ticks_ms()
                    sleep_ms(1)
        except KeyboardInterrupt:
            ax.abort()
            print("Đã dừng. Động cơ vẫn giữ lực.",
                  "Watchdog sẽ reset board sau {} s.".format(WDT_TIMEOUT_MS // 1000) if wdt else "")
            return
        except Exception as ex:
            # Lỗi chương trình: dừng motor, ghi crash.txt, báo lỗi lên master rồi chạy tiếp
            # (không để thoát vòng lặp -> watchdog reset -> mất home mà không biết lý do)
            ax.abort()
            log_crash(ex)
            try:
                node.fail(P.E_INTERNAL)
            except Exception:
                pass


def log_crash(ex):
    """In lỗi và lưu vào crash.txt (đọc lại bằng: print(open('crash.txt').read()))."""
    import io
    buf = io.StringIO()
    try:
        sys.print_exception(ex, buf)                 # MicroPython
    except AttributeError:
        import traceback                             # CPython (khi mô phỏng trên máy tính)
        traceback.print_exception(type(ex), ex, ex.__traceback__, file=buf)
    text = '[{} ms] {}'.format(ticks_ms(), buf.getvalue())
    print(text)
    try:
        if os.stat(CRASH_FILE)[6] > 8000:            # giữ file nhỏ
            os.remove(CRASH_FILE)
    except OSError:
        pass
    try:
        with open(CRASH_FILE, 'a') as f:
            f.write(text + '\n')
    except OSError:
        pass


main()
