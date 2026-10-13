#!/usr/bin/env python3
"""
eye_movie.py - giả lập mắt người xem phim, điều khiển robot Delta_EYE qua board master.

Chạy trên máy tính, gửi lệnh 'look az tilt twist ms' xuống master qua cổng USB serial.

Nguồn chuyển động mắt:
  * synthetic : mô hình mắt người xem phim (nhìn chăm chú, giật mắt, bám theo vật, chuyển cảnh)
  * video     : đọc file phim (cần opencv-python), nhìn theo vùng có chuyển động trong khung hình

Ví dụ:
  python3 eye_movie.py --sim                         # không cần phần cứng, dùng master giả lập
  python3 eye_movie.py --port /dev/cu.usbmodem1101   # chạy thật, mô hình synthetic
  python3 eye_movie.py --port /dev/cu.usbmodem1101 --video phim.mp4 --show
  python3 eye_movie.py --list-ports

Cài thư viện:  pip3 install pyserial        (thêm opencv-python nếu dùng --video)
"""

import argparse
import math
import random
import re
import sys
import time

# ==== CẤU HÌNH MẶC ĐỊNH ====
SEND_FPS = 10            # tối đa 15 lệnh/giây (master cũng giới hạn 15)
DEADBAND_DEG = 1      # hướng nhìn đổi ít hơn ngần này thì không gửi lại
MAX_TILT = 12.0          # độ nghiêng tối đa (robot chịu được >= 13.6 độ mọi hướng khi calib giữa)
LOOP_HZ = 100            # tần số cập nhật mô hình

# Thông số robot - để tự tính thời gian giật mắt mà motor theo kịp
STEPS_PER_DEG = 200 * 8 * 14 / 360.0      # A4988 1/8 bước
MOTOR_PER_GAZE = 1.0     # 1 độ hướng nhìn cần tối đa ~1 độ motor (IK: nghiêng 10 độ -> motor 9.1 độ)
ROBOT_START_HZ = 300
ROBOT_MAX_HZ = 2000
ROBOT_ACCEL = 6000


# ======================================================================
# Tiện ích
# ======================================================================
def to_polar(x, y, az_offset=0.0, flip_x=False):
    """
    Hướng nhìn trên màn hình (x sang phải, y lên trên, đơn vị độ) -> (az, tilt) của lệnh look.
    az = 0 hướng phải, 90 hướng lên.  az_offset / flip_x để khớp hướng lắp robot.
    """
    if flip_x:
        x = -x
    tilt = math.hypot(x, y)
    if tilt < 1e-9:
        return 0.0, 0.0
    az = (math.degrees(math.atan2(y, x)) + az_offset) % 360.0
    return az, tilt


def clamp_circle(x, y, r):
    d = math.hypot(x, y)
    if d <= r or d < 1e-12:
        return x, y
    return x * r / d, y * r / d


def robot_min_ms(gaze_deg, max_hz=ROBOT_MAX_HZ, accel=ROBOT_ACCEL):
    """Thời gian ngắn nhất (ms) để robot đổi hướng nhìn gaze_deg độ (cùng công thức với master)."""
    S = abs(gaze_deg) * MOTOR_PER_GAZE * STEPS_PER_DEG
    v0, vm, a = ROBOT_START_HZ, max_hz, accel
    if S <= (vm * vm - v0 * v0) / a:
        t = 2 * (math.sqrt(v0 * v0 + a * S) - v0) / a
    else:
        t = 2 * (vm - v0) / a + (S - (vm * vm - v0 * v0) / a) / vm
    return int(t * 1000 * 1.1) + 20


# ======================================================================
# Nguồn 1: mô hình mắt người xem phim
# ======================================================================
class SyntheticGaze:
    """
    Mô hình mắt người khi xem phim (đơn vị độ, (0,0) = nhìn thẳng giữa màn hình):
      - fixation  : nhìn chăm chú 0.12..1.5 s (trung bình ~0.33 s), có trôi nhẹ + giật nhỏ (microsaccade)
      - saccade   : giật mắt tới điểm mới, biên độ trung bình ~5 độ, thiên về giữa màn hình
      - pursuit   : bám theo vật chuyển động 4..12 độ/s trong 0.8..2.5 s
      - cut       : chuyển cảnh mỗi 4..10 s -> giật về gần giữa màn hình
    step(t) trả về (x, y, kind); kind in 'fix', 'micro', 'sacc', 'pursuit', 'cut'.
    """

    def __init__(self, max_tilt=MAX_TILT, seed=None, center_bias=0.5, pursuit_prob=0.15):
        self.rng = random.Random(seed)
        self.max_tilt = max_tilt
        self.center_bias = center_bias
        self.pursuit_prob = pursuit_prob
        self.x = self.y = 0.0
        self.t = None
        self.state = 'fix'
        self.state_end = 0.0
        self.next_cut = 0.0
        self.next_micro = 0.0
        self.vx = self.vy = 0.0
        self.stats = {'sacc': 0, 'cut': 0, 'pursuit': 0, 'micro': 0, 'fix_durations': []}

    # --- phân bố thời gian / biên độ ---
    def _fix_duration(self):
        d = self.rng.lognormvariate(math.log(0.30), 0.40)
        return min(1.5, max(0.12, d))

    def _sacc_amplitude(self):
        return min(self.max_tilt, self.rng.gammavariate(2.0, 2.5))

    def _begin_fixation(self, t, delay=0.0):
        d = self._fix_duration()
        fx = self.stats['fix_durations']
        fx.append(d)
        if len(fx) > 2000:                       # chạy nhiều giờ không tốn thêm bộ nhớ
            del fx[:1000]
        self.state = 'fix'
        self.state_end = t + delay + d          # delay = thời gian robot cần để tới điểm mới
        self.next_micro = t + delay + self.rng.uniform(0.15, 0.6)

    def _saccade(self, t, tx, ty, kind):
        tx, ty = clamp_circle(tx, ty, self.max_tilt * 0.95)
        amp = math.hypot(tx - self.x, ty - self.y)
        self.x, self.y = tx, ty
        self.stats[kind] += 1
        self._begin_fixation(t, robot_min_ms(amp) / 1000.0)
        return self.x, self.y, kind

    def step(self, t):
        if self.t is None:                       # lần đầu
            self.t = t
            self.next_cut = t + self.rng.uniform(4, 10)
            self._begin_fixation(t)
            return self.x, self.y, 'fix'
        dt = max(0.0, t - self.t)
        self.t = t

        # Chuyển cảnh: giật về gần giữa màn hình
        if t >= self.next_cut:
            self.next_cut = t + self.rng.uniform(4, 10)
            return self._saccade(t, self.rng.gauss(0, 2.0), self.rng.gauss(0, 1.5), 'cut')

        if self.state == 'pursuit':
            self.x += self.vx * dt
            self.y += self.vy * dt
            r = self.max_tilt * 0.95
            if math.hypot(self.x, self.y) > r:   # chạm mép -> quay đầu
                self.x, self.y = clamp_circle(self.x, self.y, r)
                self.vx, self.vy = -self.vx, -self.vy
            if t >= self.state_end:
                self._begin_fixation(t)
            return self.x, self.y, 'pursuit'

        # Đang fixation
        if t >= self.state_end:
            if self.rng.random() < self.pursuit_prob:
                spd = self.rng.uniform(4, 12)
                ang = self.rng.uniform(0, 2 * math.pi)
                self.vx, self.vy = spd * math.cos(ang), spd * math.sin(ang)
                self.state = 'pursuit'
                self.state_end = t + self.rng.uniform(0.8, 2.5)
                self.stats['pursuit'] += 1
                return self.x, self.y, 'pursuit'
            amp = self._sacc_amplitude()
            ang = self.rng.uniform(0, 2 * math.pi)
            tx = self.x + amp * math.cos(ang)
            ty = self.y + amp * math.sin(ang)
            pull = self.center_bias * self.rng.random()     # thiên về giữa màn hình
            return self._saccade(t, tx * (1 - pull), ty * (1 - pull), 'sacc')

        if t >= self.next_micro:                 # giật nhỏ khi đang nhìn chăm chú (~1-2 lần/giây)
            self.next_micro = t + self.rng.uniform(0.4, 1.0)
            a = self.rng.uniform(0, 2 * math.pi)
            m = self.rng.uniform(0.2, 0.5)
            self.x, self.y = clamp_circle(self.x + m * math.cos(a), self.y + m * math.sin(a), self.max_tilt)
            self.stats['micro'] += 1
            return self.x, self.y, 'micro'

        # trôi rất chậm
        self.x += self.rng.gauss(0, 0.1) * dt
        self.y += self.rng.gauss(0, 0.1) * dt
        return self.x, self.y, 'fix'


# ======================================================================
# Nguồn 2: nhìn theo phim thật (vùng có chuyển động)
# ======================================================================
class VideoGaze:
    """
    Nhận từng khung hình xám (numpy HxW), tìm vùng chuyển động mạnh (có ưu tiên giữa màn hình),
    rồi điều khiển hướng nhìn như mắt người: giật mắt khi mục tiêu nhảy xa, bám mượt khi gần.
    Không cần OpenCV (chỉ numpy); OpenCV chỉ dùng để đọc file phim.
    """

    def __init__(self, max_tilt=MAX_TILT, sacc_thresh=2.0, min_fix=0.15, cut_thresh=40.0,
                 motion_thresh=3.0, pursuit_gain=6.0):
        self.max_tilt = max_tilt
        self.sacc_thresh = sacc_thresh      # mục tiêu cách xa hơn -> giật mắt
        self.min_fix = min_fix              # nhìn ít nhất ngần này giây trước lần giật tiếp
        self.cut_thresh = cut_thresh        # chênh lệch sáng trung bình > ngưỡng -> chuyển cảnh
        self.motion_thresh = motion_thresh  # chuyển động yếu hơn -> coi như cảnh tĩnh
        self.pursuit_gain = pursuit_gain    # tốc độ bám (1/s)
        self.prev = None
        self.x = self.y = 0.0
        self.last_sacc_t = -1e9
        self.target = (0.0, 0.0)

    def pixel_to_gaze(self, u, v, w, h):
        """Pixel (u sang phải, v xuống dưới) -> độ. Bề ngang màn hình = +-max_tilt."""
        x = (u / (w - 1) - 0.5) * 2 * self.max_tilt
        y = -(v / (h - 1) - 0.5) * 2 * self.max_tilt * h / w
        return clamp_circle(x, y, self.max_tilt)

    def saliency_target(self, frame):
        """Trả về (x, y, cut) - điểm đáng nhìn trong khung hình."""
        import numpy as np
        f = np.asarray(frame, dtype=np.float32)
        h, w = f.shape
        if self.prev is None or self.prev.shape != f.shape:
            self.prev = f
            return 0.0, 0.0, False
        diff = np.abs(f - self.prev)
        self.prev = f
        if diff.mean() > self.cut_thresh:
            return 0.0, 0.0, True                # chuyển cảnh
        yy, xx = np.mgrid[0:h, 0:w]
        sig = 0.35 * max(h, w)
        center = np.exp(-(((xx - w / 2) ** 2) + ((yy - h / 2) ** 2)) / (2 * sig * sig))
        wgt = np.where(diff > self.motion_thresh, diff, 0) * (0.3 + 0.7 * center)
        s = wgt.sum()
        if s < 1e-6:
            return self.target[0], self.target[1], False   # cảnh tĩnh -> giữ mục tiêu cũ
        u = float((wgt * xx).sum() / s)
        v = float((wgt * yy).sum() / s)
        x, y = self.pixel_to_gaze(u, v, w, h)
        return x, y, False

    def process(self, frame, t, dt):
        tx, ty, cut = self.saliency_target(frame)
        if cut:
            self.target = (0.0, 0.0)
            self.x, self.y = 0.0, 0.0
            self.last_sacc_t = t
            return self.x, self.y, 'cut'
        self.target = (tx, ty)
        d = math.hypot(tx - self.x, ty - self.y)
        if d > self.sacc_thresh and t - self.last_sacc_t >= self.min_fix:
            self.x, self.y = tx, ty
            self.last_sacc_t = t + robot_min_ms(d) / 1000.0
            return self.x, self.y, 'sacc'
        k = min(1.0, self.pursuit_gain * dt)     # bám mượt
        self.x += (tx - self.x) * k
        self.y += (ty - self.y) * k
        return self.x, self.y, 'pursuit' if d > 0.3 else 'fix'


# ======================================================================
# Gửi lệnh: giới hạn tốc độ, vùng chết, thời gian chạy phù hợp
# ======================================================================
class CommandSender:
    def __init__(self, link, fps=SEND_FPS, deadband=DEADBAND_DEG, az_offset=0.0, flip_x=False,
                 max_tilt=MAX_TILT):
        self.link = link
        self.period = 1.0 / fps
        self.deadband = deadband
        self.az_offset = az_offset
        self.flip_x = flip_x
        self.max_tilt = max_tilt
        self.last_tx = -1e9
        self.last = None          # (x, y) đã gửi
        self.pending_sacc = False
        self.sent = 0
        self.skipped = 0

    def update(self, now, x, y, kind):
        if kind in ('sacc', 'cut'):
            self.pending_sacc = True             # giật mắt không được bỏ, gửi ở lượt kế tiếp
        if now - self.last_tx < self.period:
            self.skipped += 1
            return None
        x, y = clamp_circle(x, y, self.max_tilt)
        if self.last is not None and not self.pending_sacc and \
                math.hypot(x - self.last[0], y - self.last[1]) < self.deadband:
            return None
        amp = 0.0 if self.last is None else math.hypot(x - self.last[0], y - self.last[1])
        if self.pending_sacc:
            ms = robot_min_ms(amp)               # giật mắt: nhanh nhất robot làm được
        else:
            ms = max(int(self.period * 1000 * 1.5), robot_min_ms(amp))   # bám mượt qua 1.5 khung
        az, tilt = to_polar(x, y, self.az_offset, self.flip_x)
        line = 'look {:.1f} {:.2f} 0 {}'.format(az, tilt, ms)
        note = '{:7s} x={:+6.2f} y={:+6.2f} đổi {:5.2f}°'.format(
            'sacc' if self.pending_sacc and kind not in ('cut',) else kind, x, y, amp)
        self.link.send(line, note=note)          # note chỉ ghi vào log, không gửi xuống master
        self.last = (x, y)
        self.last_tx = now
        self.pending_sacc = False
        self.sent += 1
        return line


# ======================================================================
# Ghi log trao đổi với master + bắt "sự cố" kèm những gì vừa gửi trước đó
# ======================================================================
class TrafficLog:
    """
    - Ghi MỌI dòng vào file:   giờ  >> lệnh gửi (kèm loại chuyển động)
                                    giờ  << dòng master in ra
                                    giờ  ## sự kiện của chương trình
    - Nhớ WINDOW_S giây gần nhất. Khi master báo sự cố (FAULT, OFFLINE, node khởi động lại,
      INTERNAL, mất ACK...) -> in ra màn hình + ghi file khối "SỰ CỐ" gồm các lệnh vừa gửi
      và các dòng master báo ngay trước đó.
    """
    TRIGGERS = ('FAULT', 'OFFLINE', ' BOOT ', 'INTERNAL', 'không nhận được ACK', 'watchdog',
                'LOST_STEP', 'HOME_NOT_FOUND', 'HOME_STUCK', 'MOTION_TIMEOUT', 'LINK_LOST',
                'VBUS sụt', 'VBUS đang thấp', 'DRIVER')
    WINDOW_S = 5.0
    MERGE_S = 3.0             # các dòng lỗi trong vòng 3 s tính chung 1 sự cố

    def __init__(self, path=None, clock=time.monotonic, wall=time.time, console=print,
                 show_sent=6, show_recv=8, verbose=False):
        self.path = path
        self.verbose = verbose    # True: in cả mọi dòng gửi / nhận ra màn hình
        self.f = None
        if path:
            import os
            d = os.path.dirname(path)
            if d:
                os.makedirs(d, exist_ok=True)
            self.f = open(path, 'a', encoding='utf-8')
        self.clock = clock
        self.wall = wall
        self.console = console
        self.show_sent = show_sent
        self.show_recv = show_recv
        self.ring = []            # (t, wall, dir, text)
        self.incidents = []       # (wall, trigger, số lệnh gửi trong cửa sổ)
        self._last_incident = -1e9

    def _stamp(self, w):
        lt = time.localtime(w)
        return '{:02d}:{:02d}:{:02d}.{:03d}'.format(lt.tm_hour, lt.tm_min, lt.tm_sec,
                                                     int((w % 1) * 1000))

    MAX_LOG_BYTES = 20 * 1024 * 1024          # chạy không ngừng: quá 20 MB thì sang file mới

    def _write(self, text):
        if self.f:
            try:
                self.f.write(text + '\n')
                self.f.flush()
                self._bytes = getattr(self, '_bytes', 0) + len(text) + 1
                if self._bytes > self.MAX_LOG_BYTES:
                    self._rotate()
            except OSError as ex:
                # Ổ đĩa / OneDrive treo khi ghi (vd [Errno 60] Operation timed out):
                # KHÔNG được làm dừng robot -> bỏ ghi file, chạy tiếp
                f, self.f = self.f, None
                try:
                    f.close()
                except OSError:
                    pass
                self.console('!! Không ghi được file log ({}) -> tắt ghi log, robot vẫn chạy. '
                             'Dùng --log-dir <thư mục ngoài OneDrive>'.format(ex))

    def _rotate(self):
        """Đóng file log hiện tại, mở file mới cùng tên + _2, _3 ..."""
        self._part = getattr(self, '_part', 1) + 1
        base = self.path[:-4] if self.path.endswith('.log') else self.path
        base = base.rsplit('_part', 1)[0]
        self.f.close()
        self.path = '{}_part{}.log'.format(base, self._part)
        self.f = open(self.path, 'a', encoding='utf-8')
        self._bytes = 0
        self.console('.. Log tiếp theo: ' + self.path)

    def record(self, direction, text, note=None):
        t, w = self.clock(), self.wall()
        if note:
            text = '{:32s} # {}'.format(text, note)
        self.ring.append((t, w, direction, text))
        while self.ring and t - self.ring[0][0] > self.WINDOW_S * 2:
            self.ring.pop(0)
        self._write('{} {} {}'.format(self._stamp(w), direction, text))
        if self.verbose and direction != '##':
            self.console('{} {} {}'.format(self._stamp(w), direction, text))
        if direction == '<<' and any(k in text for k in self.TRIGGERS):
            self.incident(text)

    def event(self, msg):
        """Thông báo của chương trình: in ra màn hình và ghi file."""
        self.console(msg)
        self.record('##', msg)

    def incident(self, trigger):
        t, w = self.clock(), self.wall()
        if t - self._last_incident < self.MERGE_S:
            return                                   # cùng 1 sự cố, đã in rồi
        self._last_incident = t
        recent = [r for r in self.ring if t - r[0] <= self.WINDOW_S]
        sent = [r for r in recent if r[2] == '>>']
        recv = [r for r in recent if r[2] == '<<']
        self.total_incidents = getattr(self, 'total_incidents', 0) + 1
        n = self.total_incidents
        self.incidents.append((w, trigger, len(sent)))
        if len(self.incidents) > 5000:               # chạy nhiều ngày: giữ 5000 sự cố gần nhất
            del self.incidents[:1000]
        head = '===== SỰ CỐ #{} lúc {}: {} ====='.format(n, self._stamp(w), trigger)
        block = [head,
                 '  {} lệnh gửi trong {:.0f} s trước đó (mới nhất ở dưới):'.format(len(sent), self.WINDOW_S)]
        block += ['    {} >> {}'.format(self._stamp(r[1]), r[3]) for r in sent[-self.show_sent:]]
        block.append('  Master báo gần nhất:')
        block += ['    {} << {}'.format(self._stamp(r[1]), r[3]) for r in recv[-self.show_recv:]]
        self._write('\n'.join(block))
        for line in block:
            self.console(line)
        if self.path:
            self.console('  (toàn bộ log: {})'.format(self.path))

    def summary(self):
        lines = ['Tổng kết: {} sự cố.'.format(len(self.incidents))]
        kinds = {}
        for _, trig, _ in self.incidents:
            m = re.search(r'(n\d)', trig)
            key = (m.group(1) + ' ' if m else '') + next((k.strip() for k in self.TRIGGERS if k in trig), '?')
            kinds[key] = kinds.get(key, 0) + 1
        for k, v in sorted(kinds.items(), key=lambda kv: -kv[1]):
            lines.append('   {:30s} {} lần'.format(k, v))
        for line in lines:
            self.event(line)

    def close(self):
        if self.f:
            self.f.close()
            self.f = None


# ======================================================================
# Liên kết với master qua serial: gửi lệnh, đọc phản hồi, phát hiện lỗi
# ======================================================================
class MasterLink:
    """
    Bọc 1 đối tượng giống serial (write, read, in_waiting). Không dùng thread:
    gọi poll() thường xuyên để đọc và phân tích các dòng master in ra.
    """
    STAT_RE = re.compile(r'^STAT n(\d) (ON |OFF) (\S+)\s+err=(\S+)\s+homed=(\d)')

    # Lệnh bị từ chối vì robot CHƯA SẴN SÀNG (khác với vượt hành trình)
    NOT_READY_KEYS = ('chưa home', 'offline', 'đang ', 'FAULT')
    RANGE_KEYS = ('ngoài hành trình', 'không với tới')

    def __init__(self, port, log=print, echo=False, traffic=None):
        self.port = port
        self._log = log
        self.echo = echo
        self.traffic = traffic    # TrafficLog (tùy chọn): ghi mọi dòng gửi / nhận
        self.buf = b''
        self.fault = False
        self.fault_reason = ''
        self.err_look = 0         # số lần master báo vượt hành trình
        self.not_ready = False    # master báo robot chưa sẵn sàng (chưa home, offline...)
        self.lines = []
        self.nodes = {}           # id -> dict(online, state, err, homed)
        self.rehome_done = False
        self._last_msg = None
        self._repeat = 0

    def log(self, msg):
        """In log, gộp các dòng giống hệt nhau liên tiếp để không spam màn hình."""
        if msg == self._last_msg:
            self._repeat += 1
            return
        if self._repeat:
            self._log('   (dòng trên lặp lại thêm {} lần)'.format(self._repeat))
        self._last_msg = msg
        self._repeat = 0
        self._log(msg)

    def send(self, line, note=None):
        if self.echo:
            self._log('>> ' + line)
        if self.traffic:
            self.traffic.record('>>', line, note)
        self.port.write((line + '\n').encode())

    def poll(self):
        n = getattr(self.port, 'in_waiting', 0)
        if n:
            self.buf += self.port.read(n)
        while b'\n' in self.buf:
            raw, self.buf = self.buf.split(b'\n', 1)
            line = raw.decode('utf-8', 'replace').strip()
            if line:
                self._handle(line)

    def _handle(self, line):
        if self.traffic:
            self.traffic.record('<<', line)
        self.lines.append(line)
        if len(self.lines) > 500:
            self.lines = self.lines[-200:]
        if line.startswith('FAULT'):
            self.fault = True
            self.fault_reason = line[6:]
            self.log('!! ' + line)
        elif line.startswith('ERR look') or line.startswith('ERR move'):
            if any(k in line for k in self.RANGE_KEYS):
                self.err_look += 1
            elif any(k in line for k in self.NOT_READY_KEYS):
                self.not_ready = True
            self.log('!! ' + line)
        elif line.startswith('OK đã home lại xong'):
            self.rehome_done = True
            self.log('.. ' + line)
        elif line.startswith('ERR') or line.startswith('WARN'):
            self.log('!! ' + line)
        m = self.STAT_RE.match(line)
        if m:
            self.nodes[int(m.group(1))] = {
                'online': m.group(2) == 'ON ', 'state': m.group(3),
                'err': m.group(4), 'homed': m.group(5) == '1'}

    def wait(self, cond, timeout, clock=time.monotonic, sleep=time.sleep):
        end = clock() + timeout
        while clock() < end:
            self.poll()
            if cond():
                return True
            sleep(0.02)
        self.poll()
        return cond()

    def query_status(self, timeout=1.5, **kw):
        self.nodes = {}
        self.send('status')
        self.wait(lambda: len(self.nodes) >= 3, timeout, **kw)
        return dict(self.nodes)

    def all_ready(self):
        return len(self.nodes) >= 3 and all(
            n['online'] and n['homed'] and n['state'] == 'IDLE' for n in self.nodes.values())

    def ensure_ready(self, home_timeout=40, online_timeout=10, **kw):
        """Đảm bảo 3 trục online, đã home, đứng yên. Tự gửi 'clear' / 'home' nếu cần."""
        clock = kw.get('clock', time.monotonic)
        sleep = kw.get('sleep', time.sleep)
        # Mở cổng serial thường làm master reset: chờ nó boot xong và 3 node online lại
        end = clock() + online_timeout
        while True:
            st = self.query_status(**kw)
            if len(st) >= 3 and all(n['online'] for n in st.values()):
                break
            if clock() >= end:
                break
            sleep(0.5)
        if len(st) < 3 or not all(n['online'] for n in st.values()):
            self.log('!! Không đủ 3 node online: {}'.format(st))
            return False
        if self.all_ready():
            return True
        self.send('clear')                        # master đang FAULT sẽ từ chối 'home' nếu chưa clear
        if not all(n['homed'] for n in st.values()):
            self.log('.. Chưa home -> gửi lệnh home')
            self.send('home')
        end = clock() + home_timeout
        while clock() < end:
            sleep(0.5)
            self.query_status(**kw)
            if self.all_ready():
                return True
        self.log('!! Home không xong sau {} s'.format(home_timeout))
        return False


# ======================================================================
# Master giả lập (để chạy --sim và để test, không cần phần cứng)
# ======================================================================
class FakeMaster:
    """
    Giả lập phản hồi của board master (giống firmware master.py):
    status / home / look / clear / stop. Kiểm tra hành trình bằng spm_ik nếu tìm thấy.
    """

    def __init__(self, homed=True, neutral=(12.857, 12.857, 12.857), lo=0.5, hi=25.71,
                 use_ik=True, rate_limit_fps=15, clock=time.monotonic):
        self.homed = [homed] * 3
        self.state = ['IDLE'] * 3
        self.err = ['NONE'] * 3
        self.neutral = neutral
        self.lo, self.hi = lo, hi
        self.out = b''
        self.clock = clock
        self.received = []        # (thời điểm, dòng)
        self.looks = []           # (thời điểm, az, tilt, twist, ms, góc 3 trục)
        self.fault = False
        self.min_gap = 1.0 / rate_limit_fps
        self.ik = None
        if use_ik:
            try:
                import os
                sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                '..', 'firmware'))
                import spm_ik
                self.ik = spm_ik
                self.th0 = spm_ik.ik(0, 0, 0)
            except Exception:
                self.ik = None

    # --- giao diện giống pyserial ---
    @property
    def in_waiting(self):
        return len(self.out)

    def read(self, n):
        d, self.out = self.out[:n], self.out[n:]
        return d

    def write(self, data):
        for line in data.decode().splitlines():
            if line.strip():
                self._cmd(line.strip())
        return len(data)

    def close(self):
        pass

    def _p(self, s):
        self.out += (s + '\n').encode()

    # --- xử lý lệnh ---
    def _cmd(self, line):
        self.received.append((self.clock(), line))
        p = line.split()
        c = p[0]
        if c == 'status':
            self._p('STAT fault={} queue=0 inflight=-'.format(self.fault))
            for i in range(3):
                self._p('STAT n{} ON  {:8} err={:14} homed={} sw=0 pos=  12.857 target=  12.857'.format(
                    i + 1, self.state[i], self.err[i], int(self.homed[i])))
        elif c == 'home':
            self.homed = [True] * 3
            self.state = ['IDLE'] * 3
            for i in range(3):
                self._p('EVT n{} HOMING -> IDLE pos=0.498'.format(i + 1))
            self._p('OK HOME')
        elif c == 'clear':
            self.fault = False
            self.err = ['NONE'] * 3
            self.state = ['IDLE'] * 3
            self._p('OK CLEAR')
        elif c == 'stop':
            self._p('WARN STOP tất cả (lệnh người dùng)')
        elif c == 'look':
            self._look(p[1:])
        else:
            self._p('ERR lệnh không rõ: ' + c)

    def _look(self, a):
        if self.fault:
            self._p("ERR move: hệ thống đang FAULT, gõ 'clear' trước")
            return
        if not all(self.homed):
            self._p('ERR move: n{} chưa home'.format(self.homed.index(False) + 1))
            return
        az, tilt = float(a[0]), float(a[1])
        tw = float(a[2]) if len(a) > 2 else 0.0
        ms = int(a[3]) if len(a) > 3 else None
        degs = None
        if self.ik:
            try:
                th = self.ik.ik(az, tilt, tw)
            except ValueError as ex:
                self._p('ERR look: ' + str(ex))
                return
            degs = []
            for i in range(3):
                d = self.neutral[i] + self.ik.wrap180(th[i] - self.th0[i])
                if not self.lo <= d <= self.hi:
                    self._p('ERR look: n{} cần {:.2f} độ, ngoài hành trình {:g}..{:g}'.format(
                        i + 1, d, self.lo, self.hi))
                    return
                degs.append(d)
            self._p('IK az={} tilt={} twist={} -> '.format(az, tilt, tw) +
                    ' '.join('n{}={:.3f}'.format(i + 1, d) for i, d in enumerate(degs)))
        self.looks.append((self.clock(), az, tilt, tw, ms, degs))
        self._p('OK MOVE')

    def inject_fault(self, reason='n2 LOST_STEP'):
        self.fault = True
        self.homed[1] = False
        self.err[1] = 'LOST_STEP'
        self.state[1] = 'ERROR'
        self._p('ERR n2 LOST_STEP pos=4.002')
        self._p('FAULT ' + reason)
        self._p('WARN STOP tất cả ({})'.format(reason))

    def reboot_node(self, i, cause='WATCHDOG'):
        """Node i tự khởi động lại (mất home) mà master KHÔNG báo FAULT - như log thật đã gặp."""
        self.homed[i - 1] = False
        self._p('ERR n{} OFFLINE'.format(i))
        self._p('EVT n{} BOOT fw=1 reset={}'.format(i, cause))
        if cause == 'WATCHDOG':
            self._p('WARN n{} vừa bị watchdog reset!'.format(i))

    def finish_rehome(self):
        self.fault = False
        self.homed = [True] * 3
        self.err = ['NONE'] * 3
        self.state = ['IDLE'] * 3
        self._p('OK đã home lại xong tất cả')


# ======================================================================
# Cửa sổ xem hướng nhìn (tkinter, tùy chọn)
# ======================================================================
class GazeView:
    def __init__(self, max_tilt, size=360):
        import tkinter as tk
        self.tk = tk.Tk()
        self.tk.title('Delta_EYE - hướng nhìn')
        self.size = size
        self.scale = (size / 2 - 20) / max_tilt
        self.cv = tk.Canvas(self.tk, width=size, height=size, bg='#111')
        self.cv.pack()
        c = size / 2
        r = max_tilt * self.scale
        self.cv.create_oval(c - r, c - r, c + r, c + r, outline='#555')
        self.cv.create_line(c - 6, c, c + 6, c, fill='#555')
        self.cv.create_line(c, c - 6, c, c + 6, fill='#555')
        self.dot = self.cv.create_oval(0, 0, 0, 0, fill='#3cf', outline='')
        self.txt = self.cv.create_text(10, 10, anchor='nw', fill='#aaa', text='')
        self.alive = True
        self.tk.protocol('WM_DELETE_WINDOW', self._close)

    def _close(self):
        self.alive = False

    def update(self, x, y, kind, info=''):
        if not self.alive:
            return False
        c = self.size / 2
        px, py = c + x * self.scale, c - y * self.scale
        r = 9 if kind in ('sacc', 'cut') else 6
        self.cv.coords(self.dot, px - r, py - r, px + r, py + r)
        self.cv.itemconfig(self.dot, fill={'sacc': '#fc3', 'cut': '#f55', 'pursuit': '#6f6'}.get(kind, '#3cf'))
        self.cv.itemconfig(self.txt, text='{}  x={:+.1f} y={:+.1f}  {}'.format(kind, x, y, info))
        self.tk.update()
        return self.alive


# ======================================================================
# Chương trình chính
# ======================================================================
PYSERIAL_HELP = """Chưa cài pyserial cho Python này ({exe}).
Cài bằng 1 trong 2 cách:
  {exe} -m pip install pyserial
hoặc (nếu báo 'externally-managed-environment'):
  {exe} -m venv ~/delta_venv && ~/delta_venv/bin/pip install pyserial
  rồi chạy:  ~/delta_venv/bin/python {script} ..."""


def import_serial():
    try:
        import serial
        from serial.tools import list_ports
        return serial, list_ports
    except ImportError:
        raise SystemExit(PYSERIAL_HELP.format(exe=sys.executable, script=sys.argv[0]))


def find_master_port(quiet=False):
    """Tìm cổng USB của board ESP32. Trả về tên cổng, hoặc None nếu không chắc chắn."""
    _, list_ports = import_serial()
    keys = ('usbmodem', 'usbserial', 'wchusbserial', 'SLAB_USBtoUART', 'ttyUSB', 'ttyACM')
    found = []
    for p in list_ports.comports():
        desc = '{} {}'.format(p.description or '', p.manufacturer or '')
        if any(k in p.device for k in keys) or 'Espressif' in desc or 'CP210' in desc \
                or 'CH34' in desc or (p.device.startswith('COM') and p.vid):
            found.append(p)
    if len(found) == 1:
        return found[0].device
    if quiet:
        return None
    if found:
        print('Có nhiều cổng USB, hãy chọn bằng --port:')
        for p in found:
            print('  ', p.device, '-', p.description)
    else:
        print('Không thấy board nào cắm USB. Kiểm tra dây cáp, và đóng Thonny nếu đang mở.')
    return None


def open_serial(port, baud=115200):
    serial, _ = import_serial()
    s = serial.Serial()
    s.port = port
    s.baudrate = baud
    s.timeout = 0
    s.dtr = False            # tránh reset board khi mở cổng (board dùng chip USB-UART)
    s.rts = False
    s.open()
    s.reset_input_buffer()   # bỏ các dòng cũ còn tồn từ lần chạy trước
    return s


def run(link, source, sender, duration, clock=time.monotonic, sleep=time.sleep,
        view=None, auto_clear=False, log=print, video=None):
    """Vòng lặp chính. Trả về mã thoát: 0 = xong, 2 = dừng vì FAULT."""
    t0 = clock()
    last = t0
    paused_at = None
    nr_since = None           # thời điểm bắt đầu "robot chưa sẵn sàng"
    next_check = 0.0
    home_tries = 0
    frame_dt = 1.0 / LOOP_HZ
    while True:
        now = clock()
        if duration and now - t0 >= duration:
            break
        link.poll()

        # --- xử lý FAULT: ngừng gửi lệnh ---
        if link.fault:
            if paused_at is None:
                paused_at = now
                log('.. Tạm dừng gửi lệnh vì FAULT: {}'.format(link.fault_reason))
            if link.rehome_done:                      # master đã tự home lại xong
                link.fault = link.rehome_done = False
                link.not_ready = True                 # kiểm tra lại trạng thái trước khi chạy tiếp
                paused_at = None
                log('.. Master đã home lại xong')
            elif auto_clear and now - paused_at > 2.0:
                link.send('clear')
                link.fault = False
                link.not_ready = True                 # clear xong phải kiểm tra: có thể cần home
                paused_at = None
                log('.. Đã gửi clear')
            elif not auto_clear and now - paused_at > 20.0:
                log('!! FAULT không tự hết sau 20 s -> thoát. Kiểm tra robot rồi gõ clear/home.')
                return 2
            sleep(frame_dt)
            continue

        # --- robot chưa sẵn sàng (chưa home, offline, đang lỗi): ngừng gửi, chờ / tự home ---
        if link.not_ready:
            if nr_since is None:
                nr_since = now
                next_check = now
                log('.. Robot chưa sẵn sàng -> tạm dừng gửi lệnh, kiểm tra trạng thái')
            if now >= next_check:
                next_check = now + 3.0
                link.query_status(clock=clock, sleep=sleep)
                st = link.nodes
                if link.all_ready():
                    link.not_ready = False
                    nr_since = None
                    home_tries = 0
                    sender.last = None
                    log('.. Robot sẵn sàng -> chạy tiếp')
                    continue
                busy = any(n['state'] in ('HOMING', 'MOVING') for n in st.values())
                offline = len(st) < 3 or not all(n['online'] for n in st.values())
                if auto_clear and not busy and not offline and not link.fault and home_tries < 3:
                    link.send('clear')
                    home_tries += 1
                    link.send('home')
                    log('.. Gửi home (lần {})'.format(home_tries))
            if now - nr_since > (120 if auto_clear else 60):
                log('!! Robot không sẵn sàng lại được -> thoát. Kiểm tra nguồn / dây / log của master.')
                return 2
            sleep(frame_dt)
            continue

        # --- tự thu hẹp vùng nhìn nếu master báo vượt hành trình ---
        if link.err_look:
            new = max(4.0, sender.max_tilt - 1.0 * link.err_look)
            if new < sender.max_tilt:
                log('.. Master báo vượt hành trình -> giảm độ nghiêng tối đa còn {:.1f} độ'.format(new))
                sender.max_tilt = new
                if hasattr(source, 'max_tilt'):
                    source.max_tilt = new
            link.err_look = 0

        # --- lấy hướng nhìn ---
        if video is not None:
            frame = video.next_frame()
            if frame is None:
                break
            x, y, kind = source.process(frame, now, now - last)
        else:
            x, y, kind = source.step(now)        # đồng hồ tuyệt đối: tạm dừng rồi chạy tiếp vẫn đúng
        last = now
        sender.update(now, x, y, kind)
        if view is not None and not view.update(x, y, kind, 'gửi {}'.format(sender.sent)):
            break
        if video is not None:
            video.show(x, y, kind)
            sleep(max(0.0, video.frame_period - (clock() - now)))
        else:
            sleep(max(0.0, frame_dt - (clock() - now)))
    return 0


class RobotNotReady(Exception):
    pass


def run_forever(open_port, source, sender, duration=0, forever=False, auto_clear=False,
                no_home=False, traffic=None, log=print, clock=time.monotonic, sleep=time.sleep,
                view=None, video=None, echo=False, max_restarts=None, eyes=None):
    """
    Kết nối -> [chọn mắt] -> kiểm tra / home robot -> chạy.
    eyes: '1' / '2' / '12' -> gửi 'eyes ...' cho master3 (2 mắt) mỗi lần kết nối, None = giữ nguyên. forever=True: lỗi gì cũng không thoát mà tự
    kết nối lại cổng USB, clear + home rồi chạy tiếp; chờ giữa các lần thử 5, 10, 20, 40, 60 s.
    Trả về (mã thoát, link cuối cùng). Mã: 0 xong, 1 robot không sẵn sàng, 2 bỏ cuộc, 3 lỗi cổng USB.
    """
    t0 = clock()
    restarts = 0
    link = None
    while True:
        port = None
        try:
            port = open_port()
            link = MasterLink(port, log=log, echo=echo, traffic=traffic)
            sender.link = link
            sender.last = None
            sleep(0.5)
            link.poll()
            if eyes:
                link.send('eyes ' + eyes)             # master3 lưu lại + hỏi trạng thái mắt đó
                sleep(1.0)
                link.poll()
            if not no_home and not link.ensure_ready(clock=clock, sleep=sleep):
                raise RobotNotReady('Robot chưa sẵn sàng (thiếu node online hoặc home không xong)')
            link.fault = link.not_ready = False      # bỏ các thông báo cũ trước lúc bắt đầu
            link.err_look = 0
            log('Robot sẵn sàng. Bắt đầu "xem phim"... (Ctrl+C để dừng)')
            left = 0
            if duration:
                left = duration - (clock() - t0)
                if left <= 0:
                    return 0, link
            t_run = clock()
            code = run(link, source, sender, left, clock=clock, sleep=sleep, view=view,
                       auto_clear=auto_clear or forever, log=log, video=video)
            if code == 0 or not forever:
                return code, link
            if clock() - t_run > 300:
                restarts = 0                          # đã chạy tốt một lúc -> bắt đầu lại từ 5 s
            log('!! Robot không tự hồi phục -> kết nối lại, clear và home lại')
        except (OSError, RobotNotReady) as ex:
            usb = isinstance(ex, OSError)
            msg = ('Mất kết nối USB với master: {}'.format(ex) if usb else str(ex))
            log('!! ' + msg)
            if traffic:
                traffic.incident(msg)
            if link is not None:
                link.fault = True                     # không gửi gì thêm qua cổng này
            if not forever:
                return (3 if usb else 1), link
        # Tới đây = sẽ thử lại
        try:
            if port is not None:
                port.close()
        except Exception:
            pass
        restarts += 1
        if max_restarts is not None and restarts > max_restarts:
            return 2, link
        wait = min(60, 5 * 2 ** min(restarts - 1, 4))
        log('.. Thử lại sau {} s (lần {}). Ctrl+C để dừng.'.format(wait, restarts))
        sleep(wait)


class VideoReader:
    """Đọc file phim bằng OpenCV, trả về khung xám thu nhỏ; có thể hiện phim kèm điểm nhìn."""

    def __init__(self, path, show=False, max_tilt=MAX_TILT, width=160, loop=False):
        import cv2
        self.cv2 = cv2
        self.loop = loop                     # True: hết phim thì phát lại từ đầu
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise SystemExit('Không mở được file phim: ' + path)
        fps = self.cap.get(cv2.CAP_PROP_FPS) or 25
        self.frame_period = 1.0 / fps
        self.width = width
        self.show_win = show
        self.max_tilt = max_tilt
        self.last = None

    def next_frame(self):
        ok, frame = self.cap.read()
        if not ok and self.loop:
            self.cap.set(self.cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        if not ok:
            return None
        self.last = frame
        h, w = frame.shape[:2]
        small = self.cv2.resize(frame, (self.width, int(self.width * h / w)))
        return self.cv2.cvtColor(small, self.cv2.COLOR_BGR2GRAY)

    def show(self, x, y, kind):
        if not self.show_win or self.last is None:
            return
        f = self.last
        h, w = f.shape[:2]
        u = int((x / (2 * self.max_tilt) + 0.5) * w)
        v = int((-y / (2 * self.max_tilt * h / w) + 0.5) * h)
        color = {'sacc': (0, 200, 255), 'cut': (0, 0, 255)}.get(kind, (0, 255, 0))
        self.cv2.circle(f, (u, v), 18, color, 3)
        self.cv2.imshow('Delta_EYE - phim', f)
        if self.cv2.waitKey(1) & 0xFF == 27:      # ESC để thoát
            raise KeyboardInterrupt


def main(argv=None):
    ap = argparse.ArgumentParser(description='Giả lập mắt xem phim, điều khiển Delta_EYE.')
    ap.add_argument('--port', help='cổng serial của board master, vd /dev/cu.usbmodem1101 hoặc COM5')
    ap.add_argument('--sim', action='store_true', help='không cần phần cứng: dùng master giả lập')
    ap.add_argument('--list-ports', action='store_true', help='liệt kê cổng serial')
    ap.add_argument('--video', help='file phim để nhìn theo (cần opencv-python)')
    ap.add_argument('--show', action='store_true', help='hiện phim kèm điểm nhìn (với --video)')
    ap.add_argument('--view', action='store_true', help='cửa sổ hiện hướng nhìn (tkinter)')
    ap.add_argument('--duration', type=float, default=0, help='thời gian chạy (s), 0 = chạy mãi')
    ap.add_argument('--max-tilt', type=float, default=MAX_TILT, help='độ nghiêng tối đa (độ)')
    ap.add_argument('--fps', type=float, default=SEND_FPS, help='số lệnh tối đa mỗi giây (<=15)')
    ap.add_argument('--az-offset', type=float, default=0.0, help='xoay hướng cho khớp robot (độ)')
    ap.add_argument('--flip-x', action='store_true', help='đảo trái/phải')
    ap.add_argument('--seed', type=int, help='hạt giống ngẫu nhiên (lặp lại được)')
    ap.add_argument('--no-home', action='store_true', help='không kiểm tra / home khi bắt đầu')
    ap.add_argument('--auto-clear', action='store_true', help='tự gửi clear khi FAULT (cẩn thận!)')
    ap.add_argument('--echo', action='store_true', help='in mọi lệnh gửi đi')
    ap.add_argument('--verbose', action='store_true', help='in mọi dòng gửi / nhận ra màn hình')
    ap.add_argument('--log-dir', help='thư mục lưu log (mặc định: ~/Library/Logs/Delta_EYE, '
                                      'NGOÀI OneDrive để ghi không bị treo)')
    ap.add_argument('--no-log', action='store_true', help='không ghi file log')
    ap.add_argument('--eyes', choices=('1', '2', '12'),
                    help='firmware 2 mắt (master3): chỉ chạy mắt 1, mắt 2, hoặc cả hai (12)')
    ap.add_argument('--forever', action='store_true',
                    help='chạy không ngừng: tự kết nối lại USB, tự clear + home khi lỗi, phim tự lặp')
    args = ap.parse_args(argv)

    if args.list_ports:
        _, list_ports = import_serial()
        for p in list_ports.comports():
            print(p.device, '-', p.description)
        return 0
    if not args.port and not args.sim:
        args.port = find_master_port(quiet=args.forever)   # không ghi --port -> tự tìm cổng USB
        if not args.port and not args.forever:
            print('Hoặc chạy thử không cần robot:  {} {} --sim'.format(sys.executable, sys.argv[0]))
            return 1
        print('Dùng cổng', args.port or '(chưa thấy board - sẽ chờ)')

    log_path = None
    if not args.no_log:
        import os
        # Mặc định ghi NGOÀI OneDrive: thư mục đồng bộ có lúc treo khi ghi -> TimeoutError
        log_dir = args.log_dir or os.path.join(os.path.expanduser('~'), 'Library', 'Logs', 'Delta_EYE')
        log_path = os.path.join(log_dir, time.strftime('eye_%Y%m%d_%H%M%S.log'))
    traffic = TrafficLog(log_path, verbose=args.verbose)
    log = traffic.event
    if log_path:
        print('Ghi log vào:', log_path)
    traffic.record('##', 'Bắt đầu: ' + ' '.join(sys.argv))
    if args.forever:
        log('Chế độ CHẠY KHÔNG NGỪNG: lỗi gì cũng tự kết nối lại + home lại. Ctrl+C để dừng.')

    def open_port():
        if args.sim:
            return FakeMaster()
        if not args.port:
            args.port = find_master_port(quiet=True)
            if not args.port:
                raise OSError('chưa thấy board master cắm USB')
        try:
            return open_serial(args.port)
        except OSError:
            if not args.forever:
                raise
            other = find_master_port(quiet=True)  # tên cổng có thể đổi sau khi cắm lại
            if other and other != args.port:
                log('.. Cổng {} không còn, thử cổng {}'.format(args.port, other))
                args.port = other
                return open_serial(other)
            raise

    fps = min(args.fps, SEND_FPS)
    sender = CommandSender(None, fps=fps, az_offset=args.az_offset, flip_x=args.flip_x,
                           max_tilt=args.max_tilt)
    view = GazeView(args.max_tilt) if args.view else None
    video = None
    if args.video:
        video = VideoReader(args.video, show=args.show, max_tilt=args.max_tilt, loop=args.forever)
        source = VideoGaze(max_tilt=args.max_tilt)
    else:
        source = SyntheticGaze(max_tilt=args.max_tilt, seed=args.seed)

    link = None
    code = 0
    try:
        code, link = run_forever(open_port, source, sender, duration=args.duration,
                                 forever=args.forever, auto_clear=args.auto_clear,
                                 no_home=args.no_home, traffic=traffic, log=log, eyes=args.eyes,
                                 view=view, video=video, echo=args.echo)
        if code == 3 and link is None:
            print('-> Board master bị rút / khởi động lại / Thonny đang chiếm cổng.')
            p = find_master_port()
            if p:
                print('   ', p, '  <- thử lại với --port', p)
        log('Đã gửi {} lệnh look.'.format(sender.sent))
        return code
    except KeyboardInterrupt:
        log('Dừng (Ctrl+C).')
        return 0
    finally:
        try:
            if link is not None and not link.fault:
                link.send('look 0 0 0 800')  # về nhìn thẳng (không gửi khi đang FAULT)
                time.sleep(0.2)
                link.poll()
        except Exception:
            pass
        traffic.summary()
        if traffic.path:
            print('Log đầy đủ:', traffic.path)
        traffic.close()
        try:
            if link is not None:
                link.port.close()
        except Exception:
            pass


if __name__ == '__main__':
    sys.exit(main())
