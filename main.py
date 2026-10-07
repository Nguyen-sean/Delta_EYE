from machine import Pin
from time import sleep, sleep_us, sleep_ms
from math import sqrt
import sys
import select

PIN_EN    = 2
PIN_HOME  = 3     # cảm biến home: 1 chân GND, 1 chân IO3 (dùng pull-up nội)
PIN_MS3   = 4
PIN_RESET = 5
PIN_SLEEP = 6
PIN_STEP  = 7
PIN_DIR   = 8

# ==== CƠ KHÍ ====
# MS1 = MS2 = 1 cố định phần cứng, MS3 = 0  ->  1/8 bước
MOTOR_FULL_STEPS = 200
MICROSTEP        = 8
GEAR_RATIO       = 14                                    # hộp số 14:1
STEPS_PER_MOTOR_REV = MOTOR_FULL_STEPS * MICROSTEP       # 1600 bước / 1 vòng động cơ
STEPS_PER_DEG       = STEPS_PER_MOTOR_REV * GEAR_RATIO / 360.0   # ~62.22 bước / 1 độ trục ra

# ==== GIỚI HẠN HÀNH TRÌNH ====
# Động cơ quay tối đa 1 vòng = 360/14 ~ 25.71 độ ở trục ra.
# Vị trí 0 = đúng điểm chạm cảm biến home, chiều dương = đi ra xa cảm biến.
MAX_MOTOR_REV  = 1.0
SOFT_MIN_STEPS = 16                                      # chừa khoảng nhỏ để không đè lên công tắc
SOFT_MAX_STEPS = int(STEPS_PER_MOTOR_REV * MAX_MOTOR_REV)  # 1600
MAX_DEG        = SOFT_MAX_STEPS / STEPS_PER_DEG          # ~25.71 độ

# ==== HOMING ====
HOME_DIR            = 0      # giá trị chân DIR khi quay VỀ PHÍA cảm biến (đổi 0/1 nếu sai chiều)
HOME_ACTIVE_LEVEL   = 0      # công tắc nối GND -> khi chạm đọc được 0
HOME_FAST_HZ        = 800
HOME_SLOW_HZ        = 150
HOME_BACKOFF_STEPS  = 160    # lùi ra 1/10 vòng động cơ rồi dò lại chậm
HOME_SEARCH_STEPS   = int(SOFT_MAX_STEPS * 1.3)   # quá số bước này mà chưa chạm -> báo lỗi
HOME_PARK_STEPS     = SOFT_MIN_STEPS              # vị trí đứng chờ sau khi home

# ==== CHUYỂN ĐỘNG ====
START_HZ  = 300      # tốc độ khởi động (bước/giây)
MAX_HZ    = 2000     # tốc độ tối đa (bước/giây)
ACCEL     = 6000     # gia tốc (bước/giây^2)
PULSE_US  = 3        # độ rộng xung STEP (A4988 cần >= 1 us)


class HomingError(Exception):
    pass


class GearedStepper:
    def __init__(self):
        # Tạm tắt driver trong lúc cấu hình chân
        self.en = Pin(PIN_EN, Pin.OUT, value=1)
        self.ms3 = Pin(PIN_MS3, Pin.OUT, value=0)
        self.reset = Pin(PIN_RESET, Pin.OUT, value=1)
        self.sleep = Pin(PIN_SLEEP, Pin.OUT, value=1)
        self.dir = Pin(PIN_DIR, Pin.OUT, value=HOME_DIR)
        self.step_pin = Pin(PIN_STEP, Pin.OUT, value=0)
        self.home_sw = Pin(PIN_HOME, Pin.IN, Pin.PULL_UP)

        self.pos = 0           # vị trí hiện tại (bước), chỉ có nghĩa khi homed = True
        self.homed = False
        self.max_hz = MAX_HZ

        sleep(0.002)
        self.enable()

    # ---------- phần cứng ----------
    def enable(self):
        # RESET, SLEEP active LOW -> kéo HIGH; EN active LOW -> kéo LOW
        self.reset.on()
        self.sleep.on()
        sleep_ms(2)
        self.en.off()

    def release(self):
        """Nhả động cơ. Có thể bị trôi vị trí -> phải home lại."""
        self.en.on()
        self.homed = False

    def at_home(self):
        """Đọc cảm biến có chống dội: phải 3 lần liên tiếp cùng mức."""
        lvl = HOME_ACTIVE_LEVEL
        for _ in range(3):
            if self.home_sw.value() != lvl:
                return False
            sleep_us(20)
        return True

    def _set_dir(self, toward_home):
        self.dir.value(HOME_DIR if toward_home else 1 - HOME_DIR)
        sleep_us(5)  # thời gian setup DIR trước xung STEP

    def _pulse(self, period_us):
        self.step_pin.on()
        sleep_us(PULSE_US)
        self.step_pin.off()
        sleep_us(max(period_us - PULSE_US, 2))

    # ---------- chuyển động cơ bản ----------
    def _run(self, steps, toward_home, max_hz, stop_on_home):
        """
        Phát đúng 'steps' xung (đếm từng xung, không dùng PWM + sleep để
        không bị lệch số bước). Có tăng/giảm tốc hình thang.
        Trả về số bước thực sự đã chạy.
        """
        if steps <= 0:
            return 0
        self._set_dir(toward_home)

        v0 = min(START_HZ, max_hz)
        # số bước cần để tăng tốc từ v0 lên max_hz
        ramp = int((max_hz * max_hz - v0 * v0) / (2 * ACCEL)) if max_hz > v0 else 0
        ramp = min(ramp, steps // 2)

        done = 0
        for i in range(steps):
            # khoảng cách tới đầu/cuối hành trình -> chọn tốc độ
            n = min(i, steps - 1 - i, ramp)
            v = sqrt(v0 * v0 + 2 * ACCEL * n) if ramp else v0
            if v > max_hz:
                v = max_hz

            if stop_on_home and self.at_home():
                break
            self._pulse(int(1_000_000 / v))
            done += 1
        return done

    # ---------- homing ----------
    def home(self):
        print("Homing...")
        self.enable()
        self.homed = False

        # 1) Nếu đang đè lên công tắc -> lùi ra cho đến khi nhả
        if self.at_home():
            n = 0
            self._set_dir(False)
            while self.at_home():
                self._pulse(int(1_000_000 / HOME_SLOW_HZ))
                n += 1
                if n > HOME_SEARCH_STEPS:
                    raise HomingError("Cảm biến home không nhả - kiểm tra dây/công tắc")
            self._run(HOME_BACKOFF_STEPS, False, HOME_FAST_HZ, False)

        # 2) Chạy nhanh về phía cảm biến
        n = self._run(HOME_SEARCH_STEPS, True, HOME_FAST_HZ, True)
        if not self.at_home():
            self.hold()
            raise HomingError("Không tìm thấy home sau {} bước - kiểm tra HOME_DIR".format(n))

        # 3) Lùi ra một đoạn rồi dò lại thật chậm để lấy điểm chính xác
        self._run(HOME_BACKOFF_STEPS, False, HOME_FAST_HZ, False)
        self._run(HOME_BACKOFF_STEPS * 2, True, HOME_SLOW_HZ, True)
        if not self.at_home():
            raise HomingError("Mất tín hiệu home khi dò chậm")

        self.pos = 0
        self.homed = True

        # 4) Đi ra vị trí chờ, rời khỏi công tắc
        self.move_to_steps(HOME_PARK_STEPS)
        print("Home OK. Hành trình: 0 .. {:.2f} độ".format(MAX_DEG))

    # ---------- điều khiển vị trí ----------
    def move_to_steps(self, target, max_hz=None):
        if not self.homed:
            print("Chưa home! Gửi lệnh H trước.")
            return False

        if target < SOFT_MIN_STEPS or target > SOFT_MAX_STEPS:
            clamped = min(max(target, SOFT_MIN_STEPS), SOFT_MAX_STEPS)
            print("Vượt giới hạn ({} bước) -> giới hạn về {}".format(target, clamped))
            target = clamped

        delta = target - self.pos
        if delta == 0:
            return True

        toward_home = delta < 0
        steps = abs(delta)
        # Khi đi về phía home thì theo dõi cảm biến: chạm bất ngờ = mất bước
        done = self._run(steps, toward_home, max_hz or self.max_hz, toward_home)
        self.pos += -done if toward_home else done

        if done < steps:
            self.homed = False
            print("CẢNH BÁO: chạm home sớm tại pos={} -> mất bước, cần home lại".format(self.pos))
            return False
        return True

    def move_to_deg(self, deg, max_hz=None):
        """Góc ở TRỤC RA hộp số, 0 = vị trí cảm biến home."""
        return self.move_to_steps(int(round(deg * STEPS_PER_DEG)), max_hz)

    def deg(self):
        return self.pos / STEPS_PER_DEG

    def hold(self):
        """Dừng phát xung nhưng vẫn cấp dòng để khóa động cơ."""
        self.step_pin.off()
        self.reset.on()
        self.sleep.on()
        self.en.off()

    def test_sweep(self, cycles=1):
        """Quét hết hành trình qua lại để kiểm tra."""
        for _ in range(cycles):
            if not self.move_to_steps(SOFT_MAX_STEPS):
                return
            sleep(0.3)
            if not self.move_to_steps(SOFT_MIN_STEPS):
                return
            sleep(0.3)


# ================= GIAO TIẾP QUA USB SERIAL =================
HELP = """Lệnh:
  H          : home
  A <độ>     : đi tới góc trục ra (0 .. {:.2f})
  S <bước>   : đi tới vị trí theo bước ({} .. {})
  V <Hz>     : đặt tốc độ tối đa (bước/giây)
  T [n]      : quét hết hành trình n lần
  P          : in vị trí
  R          : nhả động cơ (mất home)
  E          : khóa động cơ lại
""".format(MAX_DEG, SOFT_MIN_STEPS, SOFT_MAX_STEPS)


def handle(motor, line):
    parts = line.strip().split()
    if not parts:
        return
    cmd = parts[0].upper()
    arg = parts[1] if len(parts) > 1 else None
    try:
        if cmd == "H":
            motor.home()
        elif cmd == "A" and arg:
            motor.move_to_deg(float(arg))
        elif cmd == "S" and arg:
            motor.move_to_steps(int(arg))
        elif cmd == "V" and arg:
            motor.max_hz = max(START_HZ, int(arg))
        elif cmd == "T":
            motor.test_sweep(int(arg) if arg else 1)
        elif cmd == "P":
            pass
        elif cmd == "R":
            motor.release()
        elif cmd == "E":
            motor.enable()
        else:
            print(HELP)
            return
    except HomingError as e:
        print("LỖI HOME:", e)
    except ValueError:
        print("Tham số không hợp lệ")
    print("pos={} bước | {:.3f} độ | homed={} | home_sw={}".format(
        motor.pos, motor.deg(), motor.homed, motor.at_home()))


motor = GearedStepper()
print(HELP)

try:
    motor.home()
except HomingError as e:
    print("LỖI HOME:", e)

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

try:
    while True:
        if poll.poll(50):
            handle(motor, sys.stdin.readline())
except KeyboardInterrupt:
    motor.hold()
    print("Đã dừng chương trình nhưng động cơ vẫn được khóa.")
