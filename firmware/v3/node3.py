# node3.py - firmware node THẾ HỆ 3 (hai mắt, 6 node). Giống node2.py, chỉ khác giao thức proto3.
# Nạp lên CẢ 6 board động cơ với tên main.py, kèm proto3.py. ID 1..6 lấy theo MAC (proto3.NODE_MACS).
# (Bản 1 / bản 2 vẫn giữ nguyên trong firmware/ và firmware/v2/.)
#
# Ý tưởng:
#   1. CHỈ HOME LÚC ĐẦU (lần cấp điện đầu tiên). Khi home, đặt điểm 0 cho encoder.
#   2. Sau đó vị trí thật lấy từ encoder (đếm nhiều vòng). Đếm xung STEP chỉ là "dự kiến".
#      - Đang chạy mà lệch > STALL_STEPS (mất bước): sửa bộ đếm theo encoder, chạy tiếp.
#      - Dừng xong mà lệch đích > HOLD_TOL_STEPS (thiếu bước / bị đẩy lệch): tự chạy bù.
#   3. Board bị reset (HARD_RESET / WATCHDOG / soft reset): vị trí lưu trong RTC memory
#      + góc tuyệt đối của encoder -> KHÔNG cần home lại. Chỉ mất điện hẳn mới phải home.
#   4. Node KHÔNG gửi gì định kỳ. Chỉ gửi: HELLO lúc khởi động, EVENT khi có chuyện,
#      STATUS khi master hỏi, CFG khi master đọc/đổi cấu hình.
#   5. Encoder hỏng / không có nam châm -> tự chạy hở như bản 1 (vẫn dùng công tắc home
#      để phát hiện mất bước) và báo master.
#
# Encoder AS5600 trên PCB (I2C: SCL = IO20, SDA = IO21, trở kéo lên R12/R13).
# Mặc định coi nam châm gắn ĐUÔI TRỤC MOTOR (trước hộp số): 4096 số / vòng motor.
# Nếu gắn ở trục ra sau hộp số: đổi ENC_ON_MOTOR = False.

from machine import Pin, ADC, RTC, I2C, WDT, reset_cause
from time import sleep_ms, sleep_us, ticks_ms, ticks_us, ticks_diff, ticks_add
from math import sqrt
import struct
import os
import sys
import gc
import json
import proto3 as P

FW_VERSION = 3
CRASH_FILE = 'crash.txt'

# ==== CHÂN (sơ đồ Stepper_motor_control_V3, driver A4988) ====
PIN_EN    = 2     # A4988 EN active LOW
PIN_HOME  = 3     # công tắc home: 1 chân GND, 1 chân IO3 (pull-up nội)
PIN_MS3   = 4     # MS1 = MS2 = 3.3 V (JP1, JP3), MS3 = LOW -> 1/8 bước
PIN_RESET = 5
PIN_SLEEP = 6
PIN_STEP  = 7
PIN_DIR   = 8
PIN_LED   = 1     # WS2812B
PIN_VBUS  = 0     # cầu phân áp R9 100K / R8 5K
PIN_SCL   = 20    # AS5600
PIN_SDA   = 21

# ==== CƠ KHÍ ====
MOTOR_FULL_STEPS    = 200
MICROSTEP           = 8
GEAR_RATIO          = 14
STEPS_PER_MOTOR_REV = MOTOR_FULL_STEPS * MICROSTEP            # 1600
STEPS_PER_DEG       = STEPS_PER_MOTOR_REV * GEAR_RATIO / 360.0  # ~62.22 bước / độ trục ra

# ==== ENCODER ====
ENC_ON_MOTOR     = True       # nam châm ở đuôi trục motor (False: ở trục ra sau hộp số)
ENC_COUNTS       = 4096       # AS5600: 12 bit / vòng
ENC_PER_STEP     = ENC_COUNTS / (STEPS_PER_MOTOR_REV if ENC_ON_MOTOR
                                 else STEPS_PER_MOTOR_REV * GEAR_RATIO)   # 2.56 số / bước
ENC_PERIOD_MOVE  = 5          # ms giữa 2 lần đọc khi đang chạy (phải < nửa vòng encoder)
ENC_PERIOD_IDLE  = 20         # ms giữa 2 lần đọc khi đứng yên
ENC_FAIL_MAX     = 5          # đọc lỗi liên tiếp ngần này lần -> coi encoder hỏng
STALL_STEPS      = 3 * MICROSTEP   # đang chạy lệch > 3 bước đủ -> mất bước, sửa bộ đếm
HOLD_TOL_STEPS   = 4          # dừng rồi mà lệch đích > 4 vi bước (~0.06 độ trục ra) -> chạy bù
SETTLE_MS        = 40         # chờ motor đứng hẳn rồi mới so encoder
MAX_CORR_PER_TARGET = 3       # tối đa số lần chạy bù cho 1 đích (tránh rung qua lại)
STALL_WINDOW_MS  = 3000       # trong 3 s mà phải sửa mất bước quá STALL_MAX lần -> kẹt cơ khí
STALL_MAX        = 6
ENC_LEARN_MIN    = 200        # đoạn chạy khi home đủ dài (bước) để dò chiều encoder
ENC_RATIO_TOL    = 0.25       # tỷ lệ số encoder / bước được lệch tối đa 25% so với lý thuyết
RESTORE_MAX      = ENC_COUNTS // 4   # sau reset, trục quay quá 1/4 vòng encoder -> không tin, home lại
RTC_SAVE_MS      = 50         # lưu vị trí vào RTC memory mỗi 50 ms

# ==== ĐO NGUỒN ====
VBUS_DIV        = (100 + 5) / 5
VBUS_LOW_MV     = 10500
VBUS_SAMPLE_MS  = 20

# ==== CẤU HÌNH (lưu config3.json, đổi từ master bằng 'set') ====
CFG_FILE = 'config3.json'
CFG_DEFAULT = {
    'min': 0.5, 'max': 25.71, 'homedir': 0,
    'maxhz': 2000, 'accel': 6000, 'homefast': 800, 'homeslow': 150,
    'led': 1,
    'enc': 1,        # 1 = vòng kín bằng encoder
    'encdir': 0,     # +1 / -1, 0 = tự dò ở lần home tới
}
CFG_LIMITS = {
    'min': (0.2, 20.0), 'max': (2.0, 90.0), 'homedir': (0, 1),
    'maxhz': (300, 8000), 'accel': (500, 100000), 'homefast': (100, 6000), 'homeslow': (20, 2000),
    'led': (0, 1), 'enc': (0, 1), 'encdir': (-1, 1),
}
CFG_INT_KEYS = ('homedir', 'led', 'enc', 'encdir')

HOME_TOL_STEPS     = 48
HOME_ACTIVE_LEVEL  = 0
HOME_BACKOFF_STEPS = 160
HOME_RELEASE_MAX   = 800
SW_DEBOUNCE_US     = 500

START_HZ = 300
MIN_HZ   = 10
PULSE_US = 3

# ==== LIÊN LẠC / WATCHDOG ====
ENABLE_WDT      = False
WDT_TIMEOUT_MS  = 5000
BOOT_DELAY_MS   = 3000
LINK_TIMEOUT_MS = 1500     # master phát SYNC 4 lần/s -> 1.5 s không nghe gì = mất liên lạc
HELLO_PERIOD_MS = 1000     # gửi HELLO đến khi master xác nhận
EVT_RETRY_MS    = 300      # gửi lại EVENT đến khi master xác nhận
REJECT_EVT_MS   = 1000     # báo "từ chối lệnh" tối đa 1 lần / giây

# ==== ĐÈN ====
LED_COUNT = 1
LED_MAX_PERCENT = 5
LED_BRIGHT = int(255 * LED_MAX_PERCENT / 100)
LED_PERIOD_MS = 100
LED_USE_RMT = False
LED_ANIMATE = False
C_RED    = (1.0, 0.0, 0.0)
C_GREEN  = (0.0, 1.0, 0.0)
C_BLUE   = (0.0, 0.2, 1.0)
C_YELLOW = (1.0, 0.6, 0.0)
C_ORANGE = (1.0, 0.25, 0.0)
C_CYAN   = (0.0, 1.0, 1.0)
C_PURPLE = (0.6, 0.0, 1.0)
C_WHITE  = (1.0, 1.0, 1.0)

PRESSED  = HOME_ACTIVE_LEVEL
RELEASED = 1 - HOME_ACTIVE_LEVEL
H_LEAVE, H_LEAVE_BACK, H_FAST, H_SLOW, H_PARK = range(5)


def load_cfg():
    cfg = dict(CFG_DEFAULT)
    try:
        with open(CFG_FILE) as f:
            saved = json.load(f)
        for k in CFG_DEFAULT:
            if k in saved:
                cfg[k] = saved[k]
    except (OSError, ValueError):
        pass
    return cfg


def save_cfg(cfg):
    with open(CFG_FILE, 'w') as f:
        json.dump(cfg, f)


def wrap_counts(d):
    """Hiệu 2 lần đọc encoder, đưa về (-2048, 2048]."""
    d %= ENC_COUNTS
    return d - ENC_COUNTS if d > ENC_COUNTS // 2 else d


class RtcStore:
    """
    Dữ liệu giữ qua reset (mất khi mất điện) trong RTC memory:
      VBUS thấp nhất / lần đo cuối, bộ đếm encoder nhiều vòng, điểm 0, góc thô, đã home chưa.
    """
    MAGIC = b'D3'
    FMT = '<2sHHiiHB'

    def __init__(self):
        self.rtc = RTC()
        self.saved = None
        try:
            m = self.rtc.memory()
            if len(m) >= struct.calcsize(self.FMT) and m[:2] == self.MAGIC:
                self.saved = struct.unpack(self.FMT, m[:struct.calcsize(self.FMT)])
        except Exception:
            pass

    def save(self, vmin, vlast, total, zero, raw, homed):
        try:
            self.rtc.memory(struct.pack(self.FMT, self.MAGIC, min(vmin, 65535), min(vlast, 65535),
                                        total, zero, raw, 1 if homed else 0))
        except Exception:
            pass


class Encoder:
    """AS5600: đọc góc thô 12 bit, cộng dồn thành bộ đếm nhiều vòng (total)."""
    ADDR = 0x36

    def __init__(self):
        self.ok = False
        self.total = 0
        self.raw = 0
        self.fails = 0
        self.buf = bytearray(2)
        self.i2c = None
        try:
            self.i2c = I2C(0, scl=Pin(PIN_SCL), sda=Pin(PIN_SDA), freq=400000)
            st = self.i2c.readfrom_mem(self.ADDR, 0x0B, 1)[0]
            md, ml, mh = st & 0x20, st & 0x10, st & 0x08
            if not md:
                print("AS5600: KHÔNG thấy nam châm (STATUS={:02X}) -> chạy hở".format(st))
                return
            if ml or mh:
                print("AS5600: nam châm {} (STATUS={:02X}) - vẫn dùng nhưng nên chỉnh khoảng cách".format(
                    "quá yếu" if ml else "quá mạnh", st))
            self.raw = self.read_raw()
            self.ok = True
            print("AS5600 OK, góc thô = {}".format(self.raw))
        except Exception as ex:
            print("AS5600: không đọc được ({}) -> chạy hở".format(ex))

    def read_raw(self):
        self.i2c.readfrom_mem_into(self.ADDR, 0x0C, self.buf)
        return ((self.buf[0] & 0x0F) << 8) | self.buf[1]

    def update(self):
        """Đọc 1 lần, cộng dồn. Trả về False nếu đọc lỗi."""
        if not self.ok:
            return False
        try:
            raw = self.read_raw()
        except OSError:
            self.fails += 1
            if self.fails >= ENC_FAIL_MAX:
                self.ok = False
                print("AS5600: đọc lỗi {} lần liên tiếp -> bỏ encoder, chạy hở".format(self.fails))
            return False
        self.fails = 0
        self.total += wrap_counts(raw - self.raw)
        self.raw = raw
        return True


class PowerMon:
    def __init__(self, prev):
        self.adc = ADC(Pin(PIN_VBUS), atten=ADC.ATTN_11DB)
        self.prev_min, self.prev_last = (prev[1], prev[2]) if prev else (0, 0)
        self.vbus = self.read_mv()
        self.vmin = self.vbus          # thấp nhất từ lần báo cáo trước
        self.boot_min = self.vbus
        self.low = self.vbus < VBUS_LOW_MV
        print("VBUS = {:.2f} V | trước reset: thấp nhất {}, lần đo cuối {}".format(
            self.vbus / 1000, self._v(self.prev_min), self._v(self.prev_last)))

    @staticmethod
    def _v(mv):
        return '{:.2f} V'.format(mv / 1000) if mv else '?'

    def read_mv(self):
        return int(self.adc.read_uv() * VBUS_DIV / 1000)

    def sample(self):
        """Trả về True nếu VBUS vừa tụt xuống dưới VBUS_LOW_MV."""
        v = self.read_mv()
        self.vbus = v
        self.vmin = min(self.vmin, v)
        self.boot_min = min(self.boot_min, v)
        was = self.low
        self.low = v < VBUS_LOW_MV
        return self.low and not was


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
        self.store = RtcStore()
        self.pwr = PowerMon(self.store.saved)
        self.enc = Encoder()
        self.apply_cfg()
        self.ax.enable()
        print("Cấu hình:", self.cfg)

        self.state = P.ST_IDLE
        self.err = P.E_NONE
        self.homed = False
        self.restored = False
        self.target = 0
        self.hphase = 0
        self.deadline = 0
        self.pending = False
        self.move_t0 = 0
        self.move_dur = 0

        # vòng kín
        self.enc_zero = 0
        self.enc_err = 0                # bước: bộ đếm - encoder (lần đọc gần nhất)
        self.fixes = 0                  # tổng số lần sửa theo encoder từ lúc khởi động
        self.fix_times = []
        self.corr_target = None         # đích đang được chạy bù
        self.corr_count = 0
        self.t_stop = ticks_ms()        # lúc motor vừa dừng
        self.last_enc = ticks_ms()
        self.last_rtc = ticks_ms()
        self.last_vbus = ticks_ms()
        self.h_enc0 = 0                 # encoder lúc bắt đầu 1 pha home (để dò chiều)
        self.h_dir = 0

        self.reset_cause = reset_cause()
        self.boot_id = struct.unpack('<H', os.urandom(2))[0] or 1
        self.try_restore()

        # liên lạc
        self.e, mac = P.init_espnow()
        self.master = P.mac_bytes(P.MASTER_MAC)
        try:
            self.e.add_peer(self.master)
        except OSError:
            pass
        self.link_ok = False
        self.last_rx = ticks_ms()
        self.hello_acked = False
        self.last_hello = ticks_ms() - HELLO_PERIOD_MS
        self.evt_seq = 0
        self.evt_pending = False
        self.evt_pkt = None
        self.last_evt_tx = 0
        self.last_reject = ticks_ms() - REJECT_EVT_MS
        self.last_move_seq = None
        self.recent = []                # seq các lệnh (không phải MOVE) đã làm -> bỏ bản phát lặp
        self.tx_seq = 0
        self.tx_err = 0
        self.last_sup = ticks_ms()
        print("Node {} MAC {} | reset: {} | {}".format(
            node_id, P.mac_str(mac), P.name(P.RESET_NAMES, self.reset_cause),
            "khôi phục vị trí, KHÔNG cần home" if self.restored else "cần home"))

    # ---------------- cấu hình ----------------
    def apply_cfg(self):
        c = self.cfg
        self.min_steps = int(c['min'] * STEPS_PER_DEG)
        self.max_steps = int(c['max'] * STEPS_PER_DEG)
        self.search_steps = int(self.max_steps * 1.3)
        self.home_fast = c['homefast']
        self.home_slow = c['homeslow']
        self.home_timeout_ms = int(self.search_steps / self.home_fast * 2000) + 10000
        self.ax.home_dir = int(c['homedir'])
        self.ax.max_hz = c['maxhz']
        self.ax.accel = c['accel']
        self.led.set_enabled(c['led'])

    def set_cfg(self, payload):
        if len(payload) != struct.calcsize(P.SETCFG_FMT):
            return False
        idx, val = struct.unpack(P.SETCFG_FMT, payload)
        if idx >= len(P.CFG_KEYS):
            return False
        key = P.CFG_KEYS[idx]
        lo, hi = CFG_LIMITS[key]
        if not lo <= val <= hi:
            print("Từ chối set {} = {} (khoảng {}..{})".format(key, val, lo, hi))
            return False
        new = dict(self.cfg)
        new[key] = int(round(val)) if key in CFG_INT_KEYS else round(val, 3)
        if new['max'] - new['min'] < 2:
            return False
        if key == 'homedir' and new[key] != self.cfg[key]:
            self.homed = False
            self.raise_event()
        self.cfg = new
        self.apply_cfg()
        save_cfg(self.cfg)
        print("Đổi cấu hình: {} = {}".format(key, new[key]))
        return True

    # ---------------- encoder / vòng kín ----------------
    def closed_loop(self):
        """Vòng kín đang có hiệu lực?"""
        return (self.enc.ok and self.cfg['enc'] and self.cfg['encdir'] != 0 and self.homed)

    def enc_steps(self):
        """Vị trí thật (vi bước, 0 = home) theo encoder."""
        return int(round((self.enc.total - self.enc_zero) * self.cfg['encdir'] / ENC_PER_STEP))

    def try_restore(self):
        """Sau reset: lấy lại vị trí từ RTC memory + góc tuyệt đối của encoder."""
        s = self.store.saved
        if not s or not s[6] or not self.enc.ok or not self.cfg['enc'] or self.cfg['encdir'] == 0:
            return
        _, _, _, total, zero, raw, _ = s
        d = wrap_counts(self.enc.raw - raw)
        if abs(d) > RESTORE_MAX:
            print("Encoder lệch {} số so với lúc reset -> không tin, cần home".format(d))
            return
        self.enc.total = total + d
        self.enc_zero = zero
        self.homed = True
        self.restored = True
        self.ax.pos = self.enc_steps()
        self.target = self.ax.pos
        print("Khôi phục vị trí: {:.3f} độ (encoder trôi {} số trong lúc reset)".format(
            self.ax.pos / STEPS_PER_DEG, d))

    def _note_fix(self, now, why, n):
        self.fixes += 1
        self.fix_times.append(now)
        while self.fix_times and ticks_diff(now, self.fix_times[0]) > STALL_WINDOW_MS:
            self.fix_times.pop(0)
        print("Encoder: {} {} vi bước".format(why, n))
        if len(self.fix_times) > STALL_MAX:
            self.fix_times = []
            self.fail(P.E_STALL)            # sửa liên tục mà không hết -> kẹt / quá tải
            return False
        return True

    def enc_service(self, now):
        ax = self.ax
        period = ENC_PERIOD_MOVE if ax.active else ENC_PERIOD_IDLE
        if ticks_diff(now, self.last_enc) < period:
            return
        self.last_enc = now
        was_ok = self.enc.ok
        self.enc.update()
        if was_ok and not self.enc.ok:
            self.raise_event()              # encoder vừa hỏng -> báo master (vẫn chạy hở)
        if not self.closed_loop():
            return
        real = self.enc_steps()
        self.enc_err = ax.pos - real
        if self.state == P.ST_MOVING and ax.active:
            if abs(self.enc_err) > STALL_STEPS:
                # mất bước khi đang chạy: bộ đếm theo encoder; khi dừng sẽ tự chạy bù phần thiếu
                if self._note_fix(now, "mất bước khi chạy, sửa bộ đếm", self.enc_err):
                    ax.pos = real
        elif self.state == P.ST_IDLE and not ax.active \
                and ticks_diff(now, self.t_stop) >= SETTLE_MS:
            miss = self.target - real
            if abs(miss) > HOLD_TOL_STEPS:
                if self.corr_target != self.target:
                    self.corr_target = self.target
                    self.corr_count = 0
                if self.corr_count >= MAX_CORR_PER_TARGET:
                    return                  # đã bù đủ số lần cho đích này -> thôi, tránh rung
                self.corr_count += 1
                if self._note_fix(now, "lệch đích, chạy bù", miss):
                    ax.pos = real
                    if self._run_to(self.target, 0, None):
                        self.state = P.ST_MOVING

    def save_rtc(self, now):
        if ticks_diff(now, self.last_rtc) < RTC_SAVE_MS:
            return
        self.last_rtc = now
        p = self.pwr
        self.store.save(p.boot_min, p.vbus, self.enc.total, self.enc_zero, self.enc.raw,
                        self.homed and self.enc.ok and self.state != P.ST_HOMING)

    # ---------------- gửi ----------------
    def _send(self, typ, payload):
        self.tx_seq = (self.tx_seq + 1) & 0xFFFF
        try:
            self.e.send(self.master, P.pack(typ, self.id, self.tx_seq, payload), False)
        except OSError as ex:
            self.tx_err += 1
            if self.tx_err % 50 == 1:
                print("Lỗi gửi ESP-NOW:", ex)

    def flags(self):
        f = 0
        if self.homed:
            f |= P.F_HOMED
        if self.ax.home_sw.value() == PRESSED:
            f |= P.F_HOME_SW
        if self.ax.enabled:
            f |= P.F_ENABLED
        if self.enc.ok:
            f |= P.F_ENC_OK
        if self.restored:
            f |= P.F_RESTORED
        return f

    def status_payload(self, evt_seq):
        e = max(-32768, min(32767, self.enc_err))
        return struct.pack(P.STATUS_FMT, evt_seq, self.state, self.err, self.flags(),
                           steps_to_mdeg(self.ax.pos), steps_to_mdeg(self.target), e,
                           min(self.pwr.vbus, 65535), min(self.pwr.vmin, 65535),
                           min(self.fixes, 65535))

    def raise_event(self):
        """Có chuyện cần báo: gửi trạng thái mới nhất, lặp lại đến khi master xác nhận."""
        self.evt_seq = self.evt_seq % 255 + 1      # 1..255 (0 = trả lời QUERY)
        self.evt_pending = True
        self.last_evt_tx = ticks_add(ticks_ms(), -EVT_RETRY_MS)

    def comm_service(self, now):
        if not self.hello_acked and ticks_diff(now, self.last_hello) >= HELLO_PERIOD_MS:
            self.last_hello = now
            pos = steps_to_mdeg(self.ax.pos)
            self._send(P.M_HELLO, struct.pack(P.HELLO_FMT, self.reset_cause, FW_VERSION,
                                              self.boot_id, self.flags(), pos,
                                              self.pwr.prev_min, self.pwr.prev_last))
        if self.evt_pending and self.link_ok and self.hello_acked \
                and ticks_diff(now, self.last_evt_tx) >= EVT_RETRY_MS:
            self.last_evt_tx = now
            self._send(P.M_EVENT, self.status_payload(self.evt_seq))   # trạng thái MỚI NHẤT

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
            if typ & 0x80 or not P.is_master(bytes(mac)):
                continue
            self.last_rx = ticks_ms()
            if not self.link_ok:
                self.link_ok = True
                print("Đã nghe thấy master")
            if typ == P.C_SYNC:
                self.on_sync(payload)
                continue
            if not (mask & self.bit):
                continue
            if typ == P.C_MOVE:
                if seq == self.last_move_seq:
                    continue                  # bản phát lặp của lệnh vừa làm
                self.last_move_seq = seq
            else:
                if seq in self.recent:
                    continue
                self.recent.append(seq)
                if len(self.recent) > 8:
                    self.recent.pop(0)
            self.execute(typ, payload)

    def on_sync(self, payload):
        if len(payload) != struct.calcsize(P.SYNC_FMT):
            return
        v = struct.unpack(P.SYNC_FMT, payload)
        i = self.id - 1
        if v[i] == self.boot_id and not self.hello_acked:
            self.hello_acked = True
            print("Master đã nhận HELLO")
        if self.evt_pending and v[P.NUM_NODES + i] == self.evt_seq:
            self.evt_pending = False

    # ---------------- xử lý lệnh ----------------
    def reject(self, typ, why):
        print("Từ chối {}: {}".format(P.name(P.CMD_NAMES, typ), why))
        now = ticks_ms()
        if ticks_diff(now, self.last_reject) >= REJECT_EVT_MS:
            self.last_reject = now
            self.raise_event()                # master đang hiểu sai trạng thái node -> báo lại

    def execute(self, typ, payload):
        ax = self.ax
        if typ == P.C_STOP:
            ax.abort()
            self.pending = False
            if self.state in (P.ST_MOVING, P.ST_HOMING):
                self.state = P.ST_IDLE
                self.t_stop = ticks_ms()
            self.target = ax.pos
            return
        if typ == P.C_QUERY:
            self._send(P.M_STATUS, self.status_payload(0))
            return
        if typ == P.C_GETCFG:
            self._send(P.M_CFG, struct.pack(P.CFG_FMT, *[float(self.cfg[k]) for k in P.CFG_KEYS]))
            return
        if typ == P.C_SETCFG:
            if self.state in (P.ST_HOMING, P.ST_MOVING):
                self.reject(typ, "đang chạy")
            else:
                self.set_cfg(payload)
            self._send(P.M_CFG, struct.pack(P.CFG_FMT, *[float(self.cfg[k]) for k in P.CFG_KEYS]))
            return
        if typ == P.C_IDENT:
            self.led.ident()
            return
        if typ == P.C_RELEASE:
            ax.disable()
            self.homed = False                # nhả motor: trục có thể bị xoay tay -> phải home lại
            self.restored = False
            self.state = P.ST_DISABLED
            self.err = P.E_NONE
            self.target = ax.pos
            return
        if typ == P.C_ENABLE:
            ax.enable()
            if self.state == P.ST_DISABLED:
                self.state = P.ST_IDLE
            return
        if typ == P.C_CLEAR:
            if self.state == P.ST_ERROR:
                self.state = P.ST_IDLE if ax.enabled else P.ST_DISABLED
            self.err = P.E_NONE
            self.fix_times = []
            return

        if typ == P.C_HOME and self.state in (P.ST_ERROR, P.ST_DISABLED, P.ST_IDLE):
            # master ra lệnh home = bật motor + xóa lỗi + home (1 gói thay cho 3 lệnh)
            ax.enable()
            self.err = P.E_NONE
            self.state = P.ST_IDLE

        # lệnh chuyển động
        if self.state == P.ST_ERROR:
            return self.reject(typ, "đang lỗi " + P.name(P.ERR_NAMES, self.err))
        if self.state == P.ST_DISABLED:
            return self.reject(typ, "motor đang nhả")
        if self.state == P.ST_HOMING:
            return self.reject(typ, "đang home")
        if typ == P.C_HOME:
            self.start_homing()
            return
        if typ == P.C_MOVE:
            if len(payload) != struct.calcsize(P.MOVE_FMT):
                return
            if not self.homed:
                return self.reject(typ, "chưa home")
            vals = struct.unpack(P.MOVE_FMT, payload)
            self.start_move(vals[self.id], vals[0])

    # ---------------- chuyển động ----------------
    def start_move(self, mdeg, dur_ms):
        target = int(round(mdeg * STEPS_PER_DEG / 1000))
        target = min(max(target, self.min_steps), self.max_steps)
        if target != self.target:
            self.corr_target = None
        self.target = target
        self.move_t0 = ticks_ms()
        self.move_dur = dur_ms
        ax = self.ax
        if self.state == P.ST_MOVING and ax.active:
            delta = target - ax.pos
            same_dir = delta != 0 and (delta > 0) == (ax._sign > 0)
            if same_dir and abs(delta) >= ax.stop_steps():
                self.pending = False
                self._run_to(target, dur_ms, ax.v)
            else:
                ax.brake()
                self.pending = True
            return
        self.pending = False
        if self._run_to(target, dur_ms, None):
            self.state = P.ST_MOVING

    def _run_to(self, target, dur_ms, vs):
        ax = self.ax
        delta = target - ax.pos
        if delta == 0:
            return False
        steps = abs(delta)
        vmax = ax.max_hz if dur_ms <= 0 else self._vmax_for(steps, dur_ms, vs)
        if delta < 0 and not self.closed_loop():
            # chạy hở: đi về phía home thì canh công tắc, chạm sớm = mất bước (như bản 1)
            ax.start(steps, True, vmax, PRESSED, self.min_steps + HOME_TOL_STEPS, edge=True, vs=vs)
        else:
            ax.start(steps, delta < 0, vmax, vs=vs)
        est_ms = int((steps / vmax + vmax / ax.accel) * 1000)
        self.deadline = ticks_add(ticks_ms(), est_ms * 2 + 500)
        return True

    def _vmax_for(self, steps, dur_ms, vs=None):
        T = dur_ms / 1000
        S = steps
        a = self.ax.accel
        v0 = START_HZ
        vs = v0 if vs is None or vs < v0 else vs
        if vs == v0 and S / T <= v0:
            return max(MIN_HZ, S / T)
        k = vs + v0 + a * T
        disc = k * k - 4 * ((vs * vs + v0 * v0) / 2 + a * S)
        if disc < 0:
            return self.ax.max_hz
        v = (k - sqrt(disc)) / 2
        if v >= vs:
            return min(self.ax.max_hz, v)
        den = T - (vs - v0) / a
        if den > 0:
            v = (S - (vs * vs - v0 * v0) / (2 * a)) / den
            if v >= MIN_HZ:
                return min(self.ax.max_hz, v)
        return max(MIN_HZ, min(self.ax.max_hz, S / T))

    # ---------------- homing ----------------
    def _hstart(self, steps, toward_home, v, stop_when=None):
        """Bắt đầu 1 pha home, ghi lại encoder để dò chiều encoder."""
        self.h_enc0 = self.enc.total
        self.h_dir = -1 if toward_home else 1
        self.ax.start(steps, toward_home, v, stop_when)

    def _learn_encdir(self):
        """Sau 1 pha home: so số encoder với số bước -> biết chiều encoder, kiểm tra encoder."""
        n = self.ax._done
        if not self.enc.ok or not self.cfg['enc'] or n < ENC_LEARN_MIN:
            return
        ratio = (self.enc.total - self.h_enc0) / (n * self.h_dir)
        expect = ENC_PER_STEP
        if abs(abs(ratio) - expect) > expect * ENC_RATIO_TOL:
            print("Encoder: {:.3f} số/bước, lý thuyết {:.3f} -> sai (nam châm? ENC_ON_MOTOR?)".format(
                ratio, expect))
            self.enc.ok = False
            self.raise_event()
            return
        d = 1 if ratio > 0 else -1
        if d != self.cfg['encdir']:
            print("Encoder: chiều = {} ({:.3f} số/bước) -> lưu cấu hình".format(d, ratio))
            self.cfg['encdir'] = d
            save_cfg(self.cfg)

    def start_homing(self):
        self.homed = False
        self.restored = False
        self.state = P.ST_HOMING
        self.deadline = ticks_add(ticks_ms(), self.home_timeout_ms)
        print("Homing...")
        self.hfound = False
        if self.ax.at_home():
            self._home_leave()
        else:
            self._home_fast()

    def _home_leave(self):
        self.hphase = H_LEAVE
        self._hstart(HOME_RELEASE_MAX, False, self.home_slow, RELEASED)

    def _home_fast(self):
        self.hphase = H_FAST
        self._hstart(self.search_steps, True, self.home_fast, PRESSED)

    def _homing_step(self):
        ax = self.ax
        ph = self.hphase
        self._learn_encdir()
        if ph == H_LEAVE:
            if not ax.stopped_sw:
                print("Lùi {} bước mà công tắc vẫn chưa nhả".format(ax._done))
                return self.fail(P.E_HOME_STUCK)
            self.hphase = H_LEAVE_BACK
            self._hstart(HOME_BACKOFF_STEPS, False, self.home_fast)
        elif ph == H_LEAVE_BACK:
            if ax.at_home():
                print("Công tắc lại bị nhấn sau khi đã nhả -> nghi nhiễu dây IO3")
                return self.fail(P.E_HOME_STUCK)
            if self.hfound:
                self.hphase = H_SLOW
                self._hstart(HOME_RELEASE_MAX + HOME_BACKOFF_STEPS * 2, True, self.home_slow, PRESSED)
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
            self.enc_zero = self.enc.total          # điểm 0 của encoder = điểm chạm công tắc
            self.homed = True
            self.hphase = H_PARK
            self.target = self.min_steps
            self._hstart(self.min_steps, False, self.home_fast)
        elif ph == H_PARK:
            self.state = P.ST_IDLE
            self.t_stop = ticks_ms()
            self.corr_target = None
            print("Home OK{}".format(" (vòng kín encoder)" if self.closed_loop() else " (chạy hở)"))
            self.raise_event()

    # ---------------- giám sát ----------------
    def fail(self, err):
        self.ax.abort()
        self.state = P.ST_ERROR
        self.err = err
        self.pending = False
        if err in (P.E_LOST_STEP, P.E_HOME_NOT_FOUND, P.E_HOME_STUCK):
            self.homed = False
            self.restored = False
        self.target = self.ax.pos
        print("LỖI:", P.name(P.ERR_NAMES, err), "pos =", self.ax.pos)
        self.raise_event()

    def supervise(self):
        now = ticks_ms()
        ax = self.ax

        if self.state == P.ST_MOVING and not ax.active:
            if ax.stopped_sw:
                self.fail(P.E_LOST_STEP)
            elif self.pending:
                self.pending = False
                remain = 0
                if self.move_dur > 0:
                    remain = max(1, self.move_dur - ticks_diff(now, self.move_t0))
                if not self._run_to(self.target, remain, None):
                    self.state = P.ST_IDLE
                    self.t_stop = now
            else:
                self.state = P.ST_IDLE           # chạy xong: KHÔNG báo master (im lặng)
                self.t_stop = now
        elif self.state == P.ST_HOMING and not ax.active:
            self._homing_step()

        if self.state in (P.ST_MOVING, P.ST_HOMING) and ticks_diff(now, self.deadline) > 0:
            self.fail(P.E_MOTION_TIMEOUT)

        self.enc_service(now)
        self.save_rtc(now)

        if ticks_diff(now, self.last_vbus) >= VBUS_SAMPLE_MS:
            self.last_vbus = now
            if self.pwr.sample():
                print("VBUS sụt: {:.2f} V".format(self.pwr.vbus / 1000))
                self.raise_event()

        if self.link_ok and ticks_diff(now, self.last_rx) > LINK_TIMEOUT_MS:
            self.link_ok = False
            print("Mất liên lạc master!")
            if self.state in (P.ST_MOVING, P.ST_HOMING):
                self.fail(P.E_LINK_LOST)

        self.comm_service(now)

    def service(self):
        if self.e.any():
            self.poll_rx()
        now = ticks_ms()
        if ticks_diff(now, self.last_sup) >= 2:
            self.last_sup = now
            self.supervise()
            self.led.update(self)


def load_node_id():
    import network
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    mac = sta.config('mac')
    nid = P.node_id_of(mac)
    if nid is None or not 1 <= nid <= P.NUM_NODES:
        print("LỖI: MAC {} chưa có trong NODE_MACS (proto3.py)".format(P.mac_str(mac)))
        return None
    return nid


def log_crash(ex):
    import io
    buf = io.StringIO()
    try:
        sys.print_exception(ex, buf)
    except AttributeError:
        import traceback
        traceback.print_exception(type(ex), ex, ex.__traceback__, file=buf)
    text = '[{} ms] {}'.format(ticks_ms(), buf.getvalue())
    print(text)
    try:
        if os.stat(CRASH_FILE)[6] > 8000:
            os.remove(CRASH_FILE)
    except OSError:
        pass
    try:
        with open(CRASH_FILE, 'a') as f:
            f.write(text + '\n')
    except OSError:
        pass


def main():
    led = StatusLed(load_cfg()['led'])
    led.show(C_WHITE, 0.3)
    node_id = load_node_id()
    if node_id is None:
        led.show(C_RED, 0.5)
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
                    if ticks_diff(ticks_ms(), last_gc) > 2000:
                        gc.collect()
                        last_gc = ticks_ms()
                    sleep_ms(1)
        except KeyboardInterrupt:
            ax.abort()
            print("Đã dừng. Động cơ vẫn giữ lực.")
            return
        except Exception as ex:
            ax.abort()
            log_crash(ex)
            try:
                node.fail(P.E_INTERNAL)
            except Exception:
                pass


if __name__ == '__main__':
    main()
