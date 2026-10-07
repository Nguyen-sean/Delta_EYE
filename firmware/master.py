# master.py - code cho ESP32 master. Nạp lên board master với tên main.py, kèm delta_proto.py.
#
# Master nhận lệnh dạng chữ từ PC qua USB serial, gửi xuống các node bằng ESP-NOW,
# theo dõi trạng thái từng node và in kết quả / lỗi về PC.
# Mỗi dòng in ra bắt đầu bằng một từ khóa để PC dễ đọc tự động:
#   OK / ERR / WARN / EVT / FAULT / STAT

import sys
import select
import struct
import os
from math import sqrt
from machine import WDT
from time import ticks_ms, ticks_diff, ticks_add, sleep_ms
import delta_proto as P
import spm_ik as K

# ==== CẤU HÌNH ====
ENABLE_WDT        = False    # đang thử nghiệm: tắt. Chạy ổn định thì đổi thành True
WDT_TIMEOUT_MS    = 3000
BOOT_DELAY_MS     = 3000     # chờ trước khi bật WDT, để kịp Ctrl+C khi cần nạp code
PING_PERIOD_MS    = 200      # nhịp tim gửi tới các node
NODE_TIMEOUT_MS   = 2000     # node im lặng quá lâu -> OFFLINE
ACK_TIMEOUT_MS    = 200
MAX_RETRY         = 5
STOP_REPEAT       = 3        # lệnh STOP gửi lặp nhiều lần cho chắc
DEFAULT_SPEED_DPS = 15.0     # tốc độ (độ/giây ở trục ra) khi lệnh move không ghi thời gian
ACCEL_MARGIN_MS   = 200
# Lệnh move/look/goto mới thay lệnh đang chạy NGAY (đổi đích). Gửi tối đa MOVE_MAX_FPS lệnh/giây;
# lệnh đến nhanh hơn thì chỉ giữ lệnh mới nhất, các lệnh ở giữa bị bỏ.
MOVE_MAX_FPS      = 15

# Một trục mất home (mất bước, khởi động lại...) -> dừng hết rồi TẤT CẢ tự về home
AUTO_REHOME       = True
REHOME_DELAY_MS   = 1000     # chờ sau khi dừng rồi mới bắt đầu home lại
REHOME_MAX_TRIES  = 10        # home lại thất bại quá số lần này -> chờ người xử lý

# ==== IK (lệnh look) ====
# Góc động cơ gửi xuống node = NEUTRAL[i] + MOTOR_SIGN[i] * (theta_i - theta_i lúc trung tính)
#   NEUTRAL[i] : góc node (độ, tính từ home) khi khâu cuối nhìn thẳng - đo bằng lệnh 'calib'
#   MOTOR_SIGN : +1 nếu chiều dương của node (đi xa công tắc home) trùng chiều dương IK, ngược lại -1
MOTOR_SIGN   = (1, 1, 1)
NEUTRAL_FILE = 'neutral.txt'

# Hành trình (min/max), tốc độ, gia tốc: master ĐỌC TỪ TỪNG NODE (lệnh 'cfg', sửa bằng 'set').
# Các giá trị dưới đây chỉ dùng khi chưa đọc được cấu hình của node.
CFG_FALLBACK = {'min': 0.5, 'max': 25.71, 'maxhz': 2000, 'accel': 6000}
STEPS_PER_DEG  = 200 * 8 * 14 / 360.0   # PHẢI GIỐNG motor_node.py
NODE_START_HZ  = 300                    # START_HZ trong motor_node.py

HELP = """Lệnh (gõ qua USB serial):
  look az tilt [twist] [ms] : nhìn theo tọa độ cực (độ): hướng az, nghiêng tilt, xoay twist,
                              chạy trong ms mili-giây (3 trục cùng xuất phát, cùng tới đích)
  ik az tilt [twist]  : chỉ tính và in góc 3 động cơ, không chạy
  calib               : lưu vị trí hiện tại làm vị trí nhìn thẳng (az=0 tilt=0 twist=0)
  range [twist]       : in góc nghiêng tối đa theo từng hướng (theo calib hiện tại)
  move d1 d2 d3 [ms]  : 3 trục tới góc d1,d2,d3 (độ), cùng xuất phát và cùng dừng
  goto n deg [ms]     : chỉ node n tới góc deg
  home [n]            : home tất cả hoặc node n
  stop                : dừng khẩn cấp tất cả
  release [n] / enable [n]
  clear [n]           : xóa lỗi
  speed hz [accel]    : tốc độ tối đa (bước/s) và gia tốc (bước/s^2) cho cả 3 node (có lưu)
  status              : bảng trạng thái
  blink [n]           : đèn của node n (hoặc tất cả) nháy trắng 4 giây để nhận diện board
  cfg                 : đọc và in cấu hình của các node
  set <n|all> <tên> <giá trị> : đổi cấu hình node, node tự lưu vào flash
        tên: min max homedir maxhz accel homefast homeslow led
        vd:  set all max 40      set 2 homedir 1      set all maxhz 1500
"""


class NodeInfo:
    def __init__(self, nid):
        self.id = nid
        self.bit = 1 << (nid - 1)
        self.mac = None
        self.online = False
        self.ever_seen = False
        self.last_seen = 0
        self.state = None
        self.err = P.E_NONE
        self.flags = 0
        self.pos = 0          # mili-độ
        self.target = 0
        self.boot_id = None
        self.cfg = None       # cấu hình đọc từ node (dict theo P.CFG_KEYS)
        self.no_rehome = False   # mất home do người dùng đổi homedir -> không tự home lại

    def c(self, key):
        """Thông số cấu hình của node, hoặc giá trị dự phòng nếu chưa đọc được."""
        return self.cfg[key] if self.cfg else CFG_FALLBACK[key]

    @property
    def homed(self):
        return bool(self.flags & P.F_HOMED)

    @property
    def busy(self):
        return self.state in (P.ST_MOVING, P.ST_HOMING)


class Master:
    def __init__(self):
        self.e, mac = P.init_espnow()
        print("OK master MAC", P.mac_str(mac))
        if not P.is_master(mac):
            print("WARN MAC này khác MASTER_MAC trong delta_proto.py -> các node sẽ bỏ qua lệnh!")
        self.nodes = [NodeInfo(i) for i in range(1, P.NUM_NODES + 1)]
        self.unknown_macs = set()
        self.rehome_at = None    # thời điểm sẽ home lại toàn bộ (None = không có)
        self.rehome_tries = 0
        self.th0 = K.ik(0, 0, 0)             # góc IK ở vị trí nhìn thẳng
        self.neutral = self.load_neutral()
        # seq khởi đầu ngẫu nhiên: master reset không bị node coi là lệnh cũ
        self.seq = struct.unpack('<H', os.urandom(2))[0]
        self.queue = []          # các lệnh chờ gửi (typ, mask, payload)
        self.inflight = None     # lệnh đang chờ ACK
        self.fault = False
        self.last_ping = ticks_ms()
        # Lệnh di chuyển: chỉ giữ lệnh MỚI NHẤT, gửi tối đa MOVE_MAX_FPS lệnh/giây
        self.move_slot = None    # (mask, payload) đang chờ gửi
        self.last_move_tx = ticks_ms() - 1000
        self.dropped = 0
        self.last_drop_report = ticks_ms()

    # ---------------- gửi ----------------
    def _next_seq(self):
        self.seq = (self.seq + 1) & 0xFFFF
        return self.seq

    def _tx(self, pkt):
        try:
            self.e.send(P.BROADCAST, pkt, False)
        except OSError as ex:
            print("WARN lỗi gửi ESP-NOW:", ex)

    def online_mask(self):
        m = 0
        for n in self.nodes:
            if n.online:
                m |= n.bit
        return m

    def command(self, typ, mask, payload=b''):
        """Xếp lệnh vào hàng đợi. Mỗi lần chỉ gửi 1 lệnh, đợi đủ ACK mới gửi lệnh sau."""
        self.queue.append((typ, mask, payload))

    def _start(self, typ, mask, payload):
        waiting = mask & self.online_mask()
        if waiting == 0:
            print("ERR {}: không có node nào online trong {}".format(
                P.name(P.CMD_NAMES, typ), self.mask_str(mask)))
            return
        seq = self._next_seq()
        pkt = P.pack(typ, mask, seq, payload)
        self._tx(pkt)
        self.inflight = {'seq': seq, 'pkt': pkt, 'typ': typ,
                         'waiting': waiting, 'tries': 1, 't': ticks_ms()}

    def emergency_stop(self, reason):
        self.queue = []
        self.inflight = None
        self.move_slot = None
        seq = self._next_seq()
        pkt = P.pack(P.CMD_STOP, P.ALL_NODES, seq)
        for _ in range(STOP_REPEAT):
            self._tx(pkt)
        print("WARN STOP tất cả ({})".format(reason))
        waiting = self.online_mask()
        if waiting:
            self.inflight = {'seq': seq, 'pkt': pkt, 'typ': P.CMD_STOP,
                             'waiting': waiting, 'tries': 1, 't': ticks_ms()}

    def on_fault(self, reason):
        print("FAULT", reason)
        self.fault = True
        self.emergency_stop(reason)

    # ---------------- nhận ----------------
    def poll_rx(self):
        while self.e.any():
            mac, msg = self.e.irecv(0)
            if msg is None:
                return
            pkt = P.unpack(msg)
            if pkt is None:
                continue
            typ, nid, seq, payload = pkt
            if not (typ & 0x80):
                continue
            # ID của node lấy theo MAC; phải khớp với ID trong gói tin
            mac = bytes(mac)
            mac_id = P.node_id_of(mac)
            if mac_id is None or mac_id != nid:
                if mac not in self.unknown_macs:
                    self.unknown_macs.add(mac)
                    print("WARN bỏ qua MAC {} (gói ghi node {}, bảng NODE_MACS ghi {})".format(
                        P.mac_str(mac), nid, mac_id))
                continue
            n = self.nodes[nid - 1]
            n.mac = mac
            n.last_seen = ticks_ms()
            if not n.online:
                n.online = True
                n.ever_seen = True
                print("EVT n{} ONLINE {}".format(nid, P.mac_str(n.mac)))
                self.command(P.CMD_GETCFG, n.bit)      # đọc cấu hình (hành trình, tốc độ...)

            if typ == P.MSG_STATUS and len(payload) == struct.calcsize(P.STATUS_FMT):
                self._on_status(n, payload)
            elif typ == P.MSG_CFG and len(payload) == struct.calcsize(P.CFG_FMT):
                self._on_cfg(n, payload)
            elif typ == P.MSG_ACK and len(payload) == struct.calcsize(P.ACK_FMT):
                self._on_ack(n, payload)
            elif typ == P.MSG_HELLO and len(payload) == struct.calcsize(P.HELLO_FMT):
                self._on_hello(n, payload)

    def _on_cfg(self, n, payload):
        vals = struct.unpack(P.CFG_FMT, payload)
        cfg = {k: round(v, 3) for k, v in zip(P.CFG_KEYS, vals)}
        if cfg != n.cfg:
            n.cfg = cfg
            self.print_cfg(n)

    def print_cfg(self, n):
        if n.cfg is None:
            print("CFG n{} (chưa đọc được)".format(n.id))
            return
        print("CFG n{} ".format(n.id) + ' '.join('{}={:g}'.format(k, n.cfg[k]) for k in P.CFG_KEYS))

    def _on_status(self, n, payload):
        state, err, flags, pos, target = struct.unpack(P.STATUS_FMT, payload)
        old_state, old_err, was_homed = n.state, n.err, n.homed
        n.state, n.err, n.flags, n.pos, n.target = state, err, flags, pos, target
        # Mất home ngoài ý muốn (không phải do lệnh home / release). Gồm cả trường hợp node
        # OFFLINE rồi quay lại sau khi tự khởi động lại (cờ homed vẫn nhớ từ trước khi OFFLINE).
        lost_home = (was_homed and not n.homed
                     and state not in (P.ST_HOMING, P.ST_DISABLED))

        if state != old_state:
            print("EVT n{} {} -> {} pos={:.3f}".format(
                n.id, P.name(P.STATE_NAMES, old_state) if old_state is not None else '-',
                P.name(P.STATE_NAMES, state), pos / 1000))
        if err != old_err and err != P.E_NONE:
            print("ERR n{} {} pos={:.3f}".format(n.id, P.name(P.ERR_NAMES, err), pos / 1000))
            # robot song song: 1 trục lỗi thì các trục còn lại cũng phải dừng
            self.on_fault("n{} {}".format(n.id, P.name(P.ERR_NAMES, err)))
        if lost_home and n.no_rehome:
            n.no_rehome = False
            print("WARN n{} đã đổi homedir -> chưa home. Sẵn sàng tay ở nút nguồn rồi gõ 'home'".format(n.id))
        elif lost_home:
            self.request_rehome("n{} mất home".format(n.id))

    def _on_ack(self, n, payload):
        ack_seq, cmd, res = struct.unpack(P.ACK_FMT, payload)
        f = self.inflight
        if f is None or ack_seq != f['seq'] or not (f['waiting'] & n.bit):
            return                                   # ACK trùng / cũ
        f['waiting'] &= ~n.bit
        cname = P.name(P.CMD_NAMES, cmd)
        if res == P.R_CLAMPED:
            print("WARN n{} {}: đích vượt hành trình, đã giới hạn lại".format(n.id, cname))
        elif res != P.R_OK:
            f['rejected'] = True
            print("ERR n{} từ chối {}: {}".format(n.id, cname, P.name(P.RES_NAMES, res)))
            if cmd in (P.CMD_MOVE, P.CMD_HOME):
                # các node khác có thể đã chạy -> dừng hết cho an toàn
                self.on_fault("n{} từ chối {}".format(n.id, cname))
                return
        if f['waiting'] == 0:
            if not f.get('rejected'):
                print("OK", cname)
            self.inflight = None

    def _on_hello(self, n, payload):
        cause, fw, boot_id = struct.unpack(P.HELLO_FMT, payload)
        if boot_id == n.boot_id:
            return                    # chỉ là kết nối lại sau khi mất sóng, node không reset
        cname = P.name(P.RESET_NAMES, cause)
        print("EVT n{} BOOT fw={} reset={}".format(n.id, fw, cname))
        if cause == P.machine.WDT_RESET:
            print("WARN n{} vừa bị watchdog reset!".format(n.id))
        rebooted = n.boot_id is not None
        n.boot_id = boot_id
        # node khởi động lại giữa chừng -> mất home -> báo lỗi hệ thống
        if rebooted:
            n.flags = 0
            n.state = None
            self.on_fault("n{} khởi động lại ({})".format(n.id, cname))
            self.request_rehome("n{} khởi động lại".format(n.id))

    # ---------------- home lại toàn bộ ----------------
    def request_rehome(self, reason):
        if not AUTO_REHOME:
            print("WARN {} -> gõ 'clear' rồi 'home' để home lại".format(reason))
            return
        if self.rehome_tries >= REHOME_MAX_TRIES:
            print("ERR {} nhưng đã home lại {} lần không thành -> cần kiểm tra, rồi gõ 'clear', 'home'".format(
                reason, self.rehome_tries))
            return
        if self.rehome_at is None:
            print("WARN {} -> sẽ dừng hết và HOME LẠI TẤT CẢ".format(reason))
        self.rehome_at = ticks_add(ticks_ms(), REHOME_DELAY_MS)

    def _rehome_service(self, now):
        if self.rehome_at is None or ticks_diff(now, self.rehome_at) < 0:
            return
        # chờ: mọi node online, không node nào đang chạy, không còn lệnh đang chờ ACK
        if self.inflight or self.queue:
            return
        for n in self.nodes:
            if not n.online or n.state is None or n.busy:
                return
        self.rehome_at = None
        self.rehome_tries += 1
        self.fault = False
        print("EVT HOME LẠI TẤT CẢ (lần {})".format(self.rehome_tries))
        self.command(P.CMD_ENABLE, P.ALL_NODES)
        self.command(P.CMD_CLEAR, P.ALL_NODES)
        self.command(P.CMD_HOME, P.ALL_NODES)

    # ---------------- giám sát ----------------
    def service(self):
        now = ticks_ms()

        if ticks_diff(now, self.last_ping) >= PING_PERIOD_MS:
            self.last_ping = now
            self._tx(P.pack(P.CMD_PING, P.ALL_NODES, self._next_seq()))

        # Watchdog từng node
        for n in self.nodes:
            if n.online and ticks_diff(now, n.last_seen) > NODE_TIMEOUT_MS:
                n.online = False
                was_busy = n.busy
                n.state = None
                print("ERR n{} OFFLINE".format(n.id))
                if self.inflight:
                    self.inflight['waiting'] &= ~n.bit
                if was_busy or any(m.busy for m in self.nodes):
                    self.on_fault("n{} mất liên lạc".format(n.id))

        # Gửi lại lệnh chưa đủ ACK
        f = self.inflight
        if f is not None:
            if f['waiting'] == 0:
                self.inflight = None
            elif ticks_diff(now, f['t']) > ACK_TIMEOUT_MS:
                if f['tries'] >= MAX_RETRY:
                    cname = P.name(P.CMD_NAMES, f['typ'])
                    print("ERR {}: không nhận được ACK từ {}".format(cname, self.mask_str(f['waiting'])))
                    self.inflight = None
                    if f['typ'] in (P.CMD_MOVE, P.CMD_HOME, P.CMD_STOP):
                        self.on_fault("thiếu ACK lệnh " + cname)
                else:
                    f['tries'] += 1
                    f['t'] = now
                    self._tx(f['pkt'])

        self._rehome_service(now)

        # Cả hệ đã home xong -> reset bộ đếm số lần home lại
        if self.rehome_tries and all(n.homed and n.state == P.ST_IDLE for n in self.nodes):
            print("OK đã home lại xong tất cả")
            self.rehome_tries = 0

        if self.inflight is None and self.queue:
            self._start(*self.queue.pop(0))

        self._move_service(now)

    # ---------------- lệnh từ PC ----------------
    def mask_str(self, mask):
        return ','.join('n{}'.format(n.id) for n in self.nodes if mask & n.bit) or '-'

    def parse_mask(self, args):
        if not args:
            return P.ALL_NODES
        nid = int(args[0])
        if not 1 <= nid <= P.NUM_NODES:
            raise ValueError
        return 1 << (nid - 1)

    def check_ready(self, mask, need_homed, allow_moving=False):
        """Kiểm tra trước khi gửi lệnh chuyển động. Trả về None nếu OK, hoặc lý do."""
        if self.fault:
            return "hệ thống đang FAULT, gõ 'clear' trước"
        for n in self.nodes:
            if not mask & n.bit:
                continue
            if not n.online:
                return "n{} offline".format(n.id)
            if n.state != P.ST_IDLE and not (allow_moving and n.state == P.ST_MOVING):
                return "n{} đang {}".format(n.id, P.name(P.STATE_NAMES, n.state))
            if need_homed and not n.homed:
                return "n{} chưa home".format(n.id)
        return None

    def auto_duration(self, targets_mdeg, mask):
        d = 0
        for n in self.nodes:
            if mask & n.bit:
                d = max(d, abs(targets_mdeg[n.id - 1] - n.pos))
        return int(d / 1000 / DEFAULT_SPEED_DPS * 1000) + ACCEL_MARGIN_MS

    def min_duration_ms(self, targets_mdeg, mask):
        """Thời gian ngắn nhất để trục chậm nhất tới đích (tăng/giảm tốc hình thang)."""
        T = 0
        v0 = NODE_START_HZ
        for n in self.nodes:
            if not mask & n.bit:
                continue
            S = abs(targets_mdeg[n.id - 1] - n.pos) / 1000 * STEPS_PER_DEG
            vm, a = n.c('maxhz'), n.c('accel')
            if S <= (vm * vm - v0 * v0) / a:       # không kịp đạt tốc độ tối đa
                t = 2 * (sqrt(v0 * v0 + a * S) - v0) / a
            else:
                t = 2 * (vm - v0) / a + (S - (vm * vm - v0 * v0) / a) / vm
            T = max(T, t)
        return int(T * 1000 * 1.05) + 20           # +5% và 20 ms dự phòng

    def send_move(self, targets_mdeg, mask, dur_ms):
        # Cho phép gửi khi đang chạy: node sẽ đổi đích ngay
        why = self.check_ready(mask, True, allow_moving=True)
        if why:
            print("ERR move:", why)
            return
        moving = any(n.state == P.ST_MOVING for n in self.nodes if mask & n.bit)
        tmin = self.min_duration_ms(targets_mdeg, mask)
        if dur_ms is None:
            dur_ms = max(self.auto_duration(targets_mdeg, mask), tmin)
        elif 0 < dur_ms < tmin:
            if not moving:      # khi bám mục tiêu liên tục thì không in, tránh tràn màn hình
                print("WARN {} ms quá nhanh với tốc độ hiện tại -> dùng {} ms để 3 trục tới đích cùng lúc".format(
                    dur_ms, tmin))
            dur_ms = tmin
        dur_ms = max(0, min(dur_ms, 65535))
        payload = struct.pack(P.MOVE_FMT, dur_ms, *targets_mdeg)
        if self.move_slot is not None:
            if self.move_slot[0] == mask:
                self.dropped += 1               # lệnh cũ chưa kịp gửi -> bỏ, giữ lệnh mới nhất
            else:
                self.command(P.CMD_MOVE, *self.move_slot)   # khác trục -> không được bỏ, gửi theo hàng đợi
        self.move_slot = (mask, payload)

    def _move_service(self, now):
        """Gửi lệnh di chuyển mới nhất, tối đa MOVE_MAX_FPS lệnh/giây."""
        if self.move_slot is not None and not self.queue \
                and ticks_diff(now, self.last_move_tx) >= 1000 // MOVE_MAX_FPS \
                and (self.inflight is None or self.inflight['typ'] == P.CMD_MOVE):
            # lệnh MOVE trước còn chờ ACK cũng được thay luôn: lệnh mới đã chứa đích mới nhất
            mask, payload = self.move_slot
            self.move_slot = None
            self.last_move_tx = now
            self._start(P.CMD_MOVE, mask, payload)
        if self.dropped and ticks_diff(now, self.last_drop_report) >= 1000:
            print("INFO bỏ {} lệnh di chuyển (gửi nhanh hơn {} lệnh/giây, chỉ giữ lệnh mới nhất)".format(
                self.dropped, MOVE_MAX_FPS))
            self.dropped = 0
            self.last_drop_report = now

    # ---------------- IK ----------------
    def load_neutral(self):
        try:
            with open(NEUTRAL_FILE) as f:
                vals = [float(x) for x in f.read().split()]
            if len(vals) == P.NUM_NODES:
                print("OK neutral = {}".format(' '.join('{:.3f}'.format(v) for v in vals)))
                return vals
        except (OSError, ValueError):
            pass
        print("WARN chưa calib: chỉnh mắt nhìn thẳng bằng 'goto', rồi gõ 'calib'")
        return None

    def calib(self):
        for n in self.nodes:
            if not (n.online and n.homed and n.state == P.ST_IDLE):
                print("ERR calib: n{} phải online, đã home và đứng yên".format(n.id))
                return
        self.neutral = [n.pos / 1000 for n in self.nodes]
        with open(NEUTRAL_FILE, 'w') as f:
            f.write(' '.join('{:.3f}'.format(v) for v in self.neutral))
        print("OK calib neutral = {}".format(' '.join('{:.3f}'.format(v) for v in self.neutral)))

    def look_targets(self, az, tilt, twist):
        """Góc node (độ) cho hướng nhìn. Trả về (list, None) hoặc (None, lý do lỗi)."""
        if self.neutral is None:
            return None, "chưa calib"
        try:
            th = K.ik(az, tilt, twist)
        except ValueError as ex:
            return None, str(ex)
        out = []
        for i, n in enumerate(self.nodes):
            d = self.neutral[i] + MOTOR_SIGN[i] * K.wrap180(th[i] - self.th0[i])
            lo, hi = n.c('min'), n.c('max')         # hành trình đọc từ node
            if not lo <= d <= hi:
                # không giới hạn (clamp) 1 trục: robot song song sẽ bị kẹt / sai hướng
                return None, "n{} cần {:.2f} độ, ngoài hành trình {:g}..{:g}".format(
                    i + 1, d, lo, hi)
            out.append(d)
        return out, None

    def _max_ok(self, f, hi=60.0):
        """Tìm giá trị lớn nhất x trong [0, hi] mà f(x) còn hợp lệ (chia đôi)."""
        lo = 0.0
        for _ in range(12):
            m = (lo + hi) / 2
            if f(m):
                lo = m
            else:
                hi = m
        return lo

    def print_range(self, twist=0.0):
        """In góc nghiêng tối đa theo từng hướng, tính với neutral đã calib."""
        if self.neutral is None:
            print("ERR range: chưa calib")
            return
        print("RANGE twist={} (nghiêng tối đa theo hướng):".format(twist))
        for az in range(0, 360, 30):
            t = self._max_ok(lambda x: self.look_targets(az, x, twist)[0] is not None)
            print("RANGE az={:3d} tilt_max={:5.1f}".format(az, t))
            self.poll_rx()          # giữ liên lạc với các node trong lúc tính
            self.service()
        tp = self._max_ok(lambda x: self.look_targets(0, 0, x)[0] is not None)
        tn = self._max_ok(lambda x: self.look_targets(0, 0, -x)[0] is not None)
        print("RANGE twist khi nhìn thẳng: -{:.1f} .. +{:.1f}".format(tn, tp))

    def set_cfg(self, mask, key, val):
        if key not in P.CFG_KEYS:
            print("ERR set: không có thông số '{}'. Chọn: {}".format(key, ' '.join(P.CFG_KEYS)))
            return
        for n in self.nodes:
            if mask & n.bit and n.busy:
                print("ERR set: n{} đang chạy, đợi dừng hẳn".format(n.id))
                return
        if key in ('min', 'max', 'homedir'):
            print("WARN đổi '{}' -> kiểm tra lại cơ khí; nên 'home' và 'calib' lại sau khi đổi".format(key))
        if key == 'homedir':
            # đổi chiều home -> node mất home; KHÔNG tự home lại, chờ người dùng gõ 'home'
            for n in self.nodes:
                if mask & n.bit:
                    n.no_rehome = True
        self.command(P.CMD_SETCFG, mask, struct.pack(P.SETCFG_FMT, P.CFG_KEYS.index(key), val))

    def handle_line(self, line):
        p = line.strip().split()
        if not p:
            return
        c = p[0].lower()
        a = p[1:]
        N = P.NUM_NODES
        try:
            if c in ('help', '?'):
                print(HELP)
            elif c == 'status':
                self.print_status()
            elif c == 'stop':
                self.emergency_stop("lệnh người dùng")
            elif c in ('look', 'ik'):
                if len(a) < 2:
                    raise ValueError
                az, tilt = float(a[0]), float(a[1])
                twist = float(a[2]) if len(a) > 2 else 0.0
                degs, why = self.look_targets(az, tilt, twist)
                if why:
                    print("ERR {}: {}".format(c, why))
                    return
                print("IK az={} tilt={} twist={} -> {}".format(
                    az, tilt, twist, ' '.join('n{}={:.3f}'.format(i + 1, d) for i, d in enumerate(degs))))
                if c == 'look':
                    ms = int(a[3]) if len(a) > 3 else None
                    self.send_move([int(round(d * 1000)) for d in degs], P.ALL_NODES, ms)
            elif c == 'calib':
                self.calib()
            elif c == 'range':
                self.print_range(float(a[0]) if a else 0.0)
            elif c == 'move':
                if len(a) < N:
                    raise ValueError
                targets = [int(round(float(x) * 1000)) for x in a[:N]]
                self.send_move(targets, P.ALL_NODES, int(a[N]) if len(a) > N else None)
            elif c == 'goto':
                mask = self.parse_mask(a[:1])
                nid = int(a[0])
                targets = [n.target for n in self.nodes]   # node khác không đọc ô của mình
                targets[nid - 1] = int(round(float(a[1]) * 1000))
                self.send_move(targets, mask, int(a[2]) if len(a) > 2 else None)
            elif c == 'home':
                mask = self.parse_mask(a)
                why = self.check_ready(mask, False)
                if why:
                    print("ERR home:", why)
                else:
                    self.command(P.CMD_HOME, mask)
            elif c == 'release':
                self.command(P.CMD_RELEASE, self.parse_mask(a))
            elif c == 'enable':
                self.command(P.CMD_ENABLE, self.parse_mask(a))
            elif c == 'clear':
                self.fault = False
                self.rehome_tries = 0        # người dùng đã xử lý -> cho phép tự home lại tiếp
                self.command(P.CMD_CLEAR, self.parse_mask(a))
            elif c == 'speed':
                self.set_cfg(P.ALL_NODES, 'maxhz', float(a[0]))
                if len(a) > 1:
                    self.set_cfg(P.ALL_NODES, 'accel', float(a[1]))
            elif c == 'blink':
                self.command(P.CMD_IDENT, self.parse_mask(a))
            elif c == 'cfg':
                for n in self.nodes:
                    self.print_cfg(n)
                self.command(P.CMD_GETCFG, P.ALL_NODES)   # đọc lại để chắc chắn mới nhất
            elif c == 'set':
                mask = P.ALL_NODES if a[0].lower() == 'all' else self.parse_mask(a[:1])
                self.set_cfg(mask, a[1].lower(), float(a[2]))
            else:
                print("ERR lệnh không rõ:", c)
        except (ValueError, IndexError):
            print("ERR sai tham số. Gõ 'help'")

    def print_status(self):
        print("STAT fault={} queue={} inflight={}".format(
            self.fault, len(self.queue),
            P.name(P.CMD_NAMES, self.inflight['typ']) if self.inflight else '-'))
        for n in self.nodes:
            print("STAT n{} {} {:8} err={:14} homed={} sw={} pos={:8.3f} target={:8.3f}".format(
                n.id, 'ON ' if n.online else 'OFF',
                P.name(P.STATE_NAMES, n.state) if n.state is not None else '-',
                P.name(P.ERR_NAMES, n.err),
                int(n.homed), int(bool(n.flags & P.F_HOME_SW)),
                n.pos / 1000, n.target / 1000))


class LineReader:
    """Đọc stdin không chặn, từng ký tự -> không làm treo vòng lặp (và watchdog)."""

    def __init__(self):
        self.poll = select.poll()
        self.poll.register(sys.stdin, select.POLLIN)
        self.buf = ''

    def read(self):
        while self.poll.poll(0):
            ch = sys.stdin.read(1)
            if ch in ('\r', '\n'):
                if self.buf:
                    line, self.buf = self.buf, ''
                    return line
            else:
                self.buf += ch
        return None


def main():
    if ENABLE_WDT:
        print("Master khởi động sau {} s (Ctrl+C để dừng trước khi bật watchdog)".format(
            BOOT_DELAY_MS // 1000))
        sleep_ms(BOOT_DELAY_MS)

    m = Master()
    reader = LineReader()
    wdt = WDT(timeout=WDT_TIMEOUT_MS) if ENABLE_WDT else None
    print(HELP)

    try:
        while True:
            m.poll_rx()
            m.service()
            line = reader.read()
            if line:
                m.handle_line(line)
            if wdt:
                wdt.feed()
            sleep_ms(1)
    except KeyboardInterrupt:
        m.emergency_stop("master dừng")
        print("Master đã dừng.",
              "Watchdog sẽ reset board sau {} s.".format(WDT_TIMEOUT_MS // 1000) if wdt else "")


main()
