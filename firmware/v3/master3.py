# master3.py - firmware master THẾ HỆ 3: HAI MẮT (6 node) trên 1 master.
# Nạp lên board master với tên main.py, kèm proto3.py và spm_ik.py.
#
# Thêm so với bản 2:
#   - 2 mắt, mỗi mắt 3 node (proto3.EYES). 1 gói MOVE broadcast chứa đích của cả 6 node
#     -> 2 mắt xuất phát cùng lúc.
#   - Chọn mắt hoạt động để test:  eyes 1 | eyes 2 | eyes 12   (lưu trong eyes.txt)
#     Mắt không hoạt động: không nhận lệnh chạy, không cần online, lỗi của nó không làm FAULT.
#   - Calib riêng từng mắt:        calib [1|2]                (lưu trong neutral3.txt)
#   - Quy tụ (2 mắt cùng nhìn vào 1 điểm gần): verg <độ>  hoặc  dist <mm>   (0 / 'dist 0' = tắt)
#   - Lệnh 'look' giữ nguyên cú pháp -> pc/eye_movie.py dùng được ngay.
#   - 2 mắt là 2 cơ cấu độc lập: 1 node mất home -> chỉ home lại MẮT CỦA NODE ĐÓ.

import sys
import select
import struct
import os
from math import sqrt, atan2, degrees, radians, sin, cos, atan   # MicroPython không có hypot
from machine import WDT
from time import ticks_ms, ticks_diff, ticks_add, sleep_ms
import proto3 as P
import spm_ik as K

# ==== CẤU HÌNH ====
ENABLE_WDT        = False
WDT_TIMEOUT_MS    = 3000
BOOT_DELAY_MS     = 3000
SYNC_PERIOD_MS    = 250
CMD_REPEAT        = 3
STOP_REPEAT       = 5
REPEAT_GAP_MS     = 30
MOVE_RESEND_MS    = (30, 300, 1000)
MOVE_MAX_FPS      = 15
STATUS_WAIT_MS    = 300
ALIVE_CHECK_MS    = 5000     # 0 = tắt kiểm tra node còn sống
ALIVE_RETRY       = 3
ALIVE_MISS_MAX    = 2
DEFAULT_SPEED_DPS = 15.0
ACCEL_MARGIN_MS   = 200

AUTO_HOME_AT_START = True
AUTO_REHOME        = True
REHOME_DELAY_MS    = 1000
REHOME_MAX_TRIES   = 3

# ==== HÌNH HỌC 2 MẮT ====
# MOTOR_SIGN: +1 nếu chiều dương của node (đi xa công tắc) trùng chiều dương IK (theo node 1..6)
MOTOR_SIGN = (1, 1, 1, 1, 1, 1)
# Mỗi mắt:
#   side      : 'L' = mắt trái, 'R' = mắt phải (nhìn từ phía robot) -> chiều quy tụ
#   az_offset : mắt lắp xoay so với mắt kia bao nhiêu độ quanh trục nhìn (cộng vào az trước IK)
EYE_CFG = {
    1: {'side': 'L', 'az_offset': 0.0},
    2: {'side': 'R', 'az_offset': 0.0},
}
# az (trong hệ IK) của hướng "sang PHẢI của robot". Quy tụ = mắt trái quay sang phải,
# mắt phải quay sang trái. Kiểm tra bằng 'look <RIGHT_AZ> 10' -> cả 2 mắt phải nhìn sang phải robot.
RIGHT_AZ = 0.0
IPD_MM   = 65.0          # khoảng cách 2 tâm mắt (dùng cho lệnh 'dist')

NEUTRAL_FILE = 'neutral3.txt'
EYES_FILE    = 'eyes.txt'
CFG_FALLBACK = {'min': 0.5, 'max': 25.71, 'maxhz': 2000, 'accel': 6000}
VBUS_WARN_MV   = 10500
STEPS_PER_DEG  = 200 * 8 * 14 / 360.0   # PHẢI GIỐNG node3.py
NODE_START_HZ  = 300

HELP = """Lệnh (thế hệ 3 - hai mắt):
  eyes 1 | eyes 2 | eyes 12 : chọn mắt hoạt động (lưu lại). Mắt tắt không cần cắm điện.
  look az tilt [twist] [ms] : các mắt đang bật cùng nhìn theo tọa độ cực (độ)
  verg <độ>           : góc quy tụ 2 mắt (tổng, mỗi mắt quay vào trong một nửa). 0 = song song
  dist <mm>           : nhìn vật cách <mm> (tự tính góc quy tụ theo IPD). 0 = vô cực
  ik az tilt [twist]  : chỉ tính góc 6 động cơ
  calib [1|2]         : lưu vị trí hiện tại làm vị trí nhìn thẳng của mắt (mặc định: mắt đang bật)
  range [twist]       : góc nghiêng tối đa theo hướng (mỗi mắt đang bật)
  move d1 d2 d3 [ms]  : 3 trục của MỖI mắt đang bật tới góc d1,d2,d3 (độ)
  goto n deg [ms]     : chỉ node n (1..6) tới góc deg
  home [n|e1|e2]      : home (mặc định: các mắt đang bật)
  stop                : dừng khẩn cấp tất cả
  release [n|e1|e2] / enable [n|e1|e2]
  clear [n|e1|e2]     : xóa lỗi
  speed hz [accel]    : tốc độ tối đa / gia tốc (mắt đang bật)
  status              : hỏi trạng thái node của các mắt đang bật
  blink [n|e1|e2]     : đèn node nháy trắng 4 giây
  cfg                 : đọc cấu hình các node
  set <n|e1|e2|all> <tên> <giá trị> : min max homedir maxhz accel homefast homeslow led enc encdir
"""


class NodeInfo:
    def __init__(self, nid):
        self.id = nid
        self.eye = P.eye_of(nid)
        self.leg = P.EYES[self.eye].index(nid)      # chân 0..2 của IK
        self.bit = 1 << (nid - 1)
        self.online = False
        self.ever_seen = False
        self.state = None
        self.err = P.E_NONE
        self.flags = 0
        self.pos = 0
        self.target = 0
        self.boot_id = 0
        self.evt_ack = 0
        self.cfg = None
        self.no_rehome = False
        self.vbus = 0
        self.vmin = 0
        self.vmin_all = 0
        self.last_vwarn = 0
        self.enc_err = 0
        self.fixes = 0
        self.replied = False
        self.miss = 0

    def c(self, key):
        return self.cfg[key] if self.cfg else CFG_FALLBACK[key]

    @property
    def homed(self):
        return bool(self.flags & P.F_HOMED)


class Master:
    def __init__(self):
        self.e, mac = P.init_espnow()
        print("OK master3 MAC", P.mac_str(mac))
        if not P.is_master(mac):
            print("WARN MAC này khác MASTER_MAC trong proto3.py -> các node sẽ bỏ qua lệnh!")
        self.nodes = [NodeInfo(i) for i in range(1, P.NUM_NODES + 1)]
        self.unknown_macs = set()
        self.th0 = K.ik(0, 0, 0)
        self.neutral = self.load_neutral()        # {mắt: [3 góc] hoặc None}
        self.active = self.load_eyes()            # bitmask node của các mắt đang bật
        self.verg = 0.0
        self.seq = struct.unpack('<H', os.urandom(2))[0]
        self.repeats = []
        self.fault = False
        self.sysflags = 0
        now = ticks_ms()
        self.last_sync = now
        self.move_slot = None
        self.last_move_tx = ticks_add(now, -1000)
        self.dropped = 0
        self.last_drop_report = now
        self.rehome_at = None
        self.rehome_mask = 0     # node của các mắt cần home lại
        self.rehome_tries = 0
        self.rehoming = False
        self.start_home_done = not AUTO_HOME_AT_START
        self.status_due = None
        self.alive_due = None
        self.alive_try = 0
        self.last_alive = now
        self.query(silent=True)

    # ---------------- mắt đang bật ----------------
    def act(self):
        """Các node thuộc mắt đang bật."""
        return [n for n in self.nodes if self.active & n.bit]

    def active_eyes(self):
        return [e for e in P.EYES if self.active & P.eye_mask(e) == P.eye_mask(e)]

    def load_eyes(self):
        eyes = (1, 2)
        try:
            with open(EYES_FILE) as f:
                eyes = tuple(int(c) for c in f.read().strip() if c in '12') or eyes
        except (OSError, ValueError):
            pass
        m = 0
        for e in eyes:
            m |= P.eye_mask(e)
        print("OK mắt đang bật: {}".format(' + '.join(str(e) for e in eyes)))
        return m

    def set_eyes(self, eyes):
        m = 0
        for e in eyes:
            m |= P.eye_mask(e)
        stop = self.active & ~m                   # mắt vừa tắt: dừng lại cho an toàn
        if stop:
            self.bcast(P.C_STOP, stop, repeat=STOP_REPEAT)
        self.active = m
        self.move_slot = None
        self.fault = False
        self.sysflags &= ~P.SYS_FAULT
        self.rehome_at = None
        self.rehome_mask = 0
        with open(EYES_FILE, 'w') as f:
            f.write(''.join(str(e) for e in eyes))
        print("OK mắt đang bật: {}".format(' + '.join(str(e) for e in eyes)))
        if AUTO_HOME_AT_START:
            self.start_home_done = False          # mắt mới bật chưa home -> tự home
        self.query(silent=True)

    # ---------------- gửi ----------------
    def _next_seq(self):
        self.seq = (self.seq + 1) & 0xFFFF
        return self.seq

    def _tx(self, pkt):
        try:
            self.e.send(P.BROADCAST, pkt, False)
        except OSError as ex:
            print("WARN lỗi gửi ESP-NOW:", ex)

    def bcast(self, typ, mask, payload=b'', repeat=CMD_REPEAT, offsets=None):
        pkt = P.pack(typ, mask, self._next_seq(), payload)
        self._tx(pkt)
        if offsets is None:
            offsets = [REPEAT_GAP_MS * k for k in range(1, repeat)]
        t0 = ticks_ms()
        for off in offsets:
            self.repeats.append([ticks_add(t0, off), pkt, typ])

    def _repeat_service(self, now):
        keep = []
        for r in self.repeats:
            if ticks_diff(now, r[0]) >= 0:
                self._tx(r[1])
            else:
                keep.append(r)
        self.repeats = keep

    def send_sync(self):
        v = [n.boot_id for n in self.nodes] + [n.evt_ack for n in self.nodes] + [self.sysflags]
        self.bcast(P.C_SYNC, P.ALL_NODES, struct.pack(P.SYNC_FMT, *v), repeat=1)

    def emergency_stop(self, reason):
        self.move_slot = None
        self.repeats = [r for r in self.repeats if r[2] != P.C_MOVE]
        self.bcast(P.C_STOP, P.ALL_NODES, repeat=STOP_REPEAT)
        for n in self.nodes:
            if n.state == P.ST_HOMING:
                n.state = P.ST_IDLE
        print("WARN STOP tất cả ({})".format(reason))

    def on_fault(self, reason):
        print("FAULT", reason)
        self.fault = True
        self.sysflags |= P.SYS_FAULT
        self.emergency_stop(reason)

    def query(self, silent=False, mask=None):
        mask = self.active if mask is None else mask
        for n in self.nodes:
            if mask & n.bit:
                n.replied = False
        self.bcast(P.C_QUERY, mask, repeat=2)
        due = ticks_add(ticks_ms(), STATUS_WAIT_MS)
        if silent:
            self.alive_due = due
        else:
            self.status_due = due

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
            mac = bytes(mac)
            mac_id = P.node_id_of(mac)
            if mac_id is None or mac_id != nid:
                if mac not in self.unknown_macs:
                    self.unknown_macs.add(mac)
                    print("WARN bỏ qua MAC {} (gói ghi node {}, bảng ghi {})".format(
                        P.mac_str(mac), nid, mac_id))
                continue
            n = self.nodes[nid - 1]
            n.miss = 0
            if not n.online:
                n.online = True
                print("EVT n{} ONLINE {} (mắt {}{})".format(
                    nid, P.mac_str(mac), n.eye, '' if self.active & n.bit else ', đang TẮT'))
            if not n.ever_seen or n.cfg is None:
                n.ever_seen = True
                self.bcast(P.C_GETCFG, n.bit, repeat=1)
            if typ == P.M_HELLO and len(payload) == struct.calcsize(P.HELLO_FMT):
                self._on_hello(n, payload)
            elif typ in (P.M_EVENT, P.M_STATUS) and len(payload) == struct.calcsize(P.STATUS_FMT):
                self._on_status(n, payload, typ == P.M_EVENT)
            elif typ == P.M_CFG and len(payload) == struct.calcsize(P.CFG_FMT):
                self._on_cfg(n, payload)

    def _on_hello(self, n, payload):
        cause, fw, boot_id, flags, pos, prev_min, prev_last = struct.unpack(P.HELLO_FMT, payload)
        if boot_id == n.boot_id:
            return
        rebooted = n.boot_id != 0 or n.homed
        n.boot_id = boot_id
        n.evt_ack = 0
        cname = P.name(P.RESET_NAMES, cause)
        print("EVT n{} BOOT fw={} reset={}".format(n.id, fw, cname))
        if prev_min:
            print("PWR n{} trước lần reset vừa rồi: VBUS thấp nhất {:.2f} V, lần đo cuối {:.2f} V".format(
                n.id, prev_min / 1000, prev_last / 1000))
            if prev_last < VBUS_WARN_MV:
                print("WARN n{} VBUS đang thấp ({:.2f} V) ngay trước khi reset -> nghi do nguồn".format(
                    n.id, prev_last / 1000))
        old_target = n.target
        n.flags = flags
        n.pos = n.target = pos
        if n.state != P.ST_HOMING:
            n.state = P.ST_IDLE
        n.err = P.E_NONE
        if not self.active & n.bit:
            return                                  # mắt đang tắt: chỉ ghi nhận
        if flags & P.F_RESTORED:
            print("EVT n{} khôi phục vị trí {:.3f} độ từ encoder -> không cần home".format(n.id, pos / 1000))
            if rebooted:
                n.target = old_target
                self.resend_last_move()
        elif rebooted:
            self.on_fault("n{} khởi động lại ({}), mất vị trí".format(n.id, cname))
            self.request_rehome("n{} khởi động lại".format(n.id), n.eye)

    def _on_status(self, n, payload, is_event):
        (evt, state, err, flags, pos, target, enc_err, vbus, vmin, fixes) = \
            struct.unpack(P.STATUS_FMT, payload)
        if is_event:
            if evt == n.evt_ack:
                return
            n.evt_ack = evt
        else:
            n.replied = True
        old_state, old_err, was_homed = n.state, n.err, n.homed
        n.state, n.err, n.flags = state, err, flags
        n.pos, n.enc_err = pos, enc_err
        if is_event:
            n.target = target
        n.vbus, n.vmin = vbus, vmin
        if not n.vmin_all or vmin < n.vmin_all:
            n.vmin_all = vmin
        if fixes > n.fixes and is_event:
            print("INFO n{} encoder đã bù {} lần".format(n.id, fixes))
        n.fixes = fixes
        if vmin < VBUS_WARN_MV:
            now = ticks_ms()
            if ticks_diff(now, n.last_vwarn) > 1000:
                n.last_vwarn = now
                print("WARN n{} VBUS sụt {:.2f} V (hiện {:.2f} V)".format(n.id, vmin / 1000, vbus / 1000))
        if state != old_state and (is_event or old_state is None or state == P.ST_ERROR):
            print("EVT n{} {} -> {} pos={:.3f}".format(
                n.id, P.name(P.STATE_NAMES, old_state) if old_state is not None else '-',
                P.name(P.STATE_NAMES, state), pos / 1000))
        active = self.active & n.bit
        if err != old_err and err != P.E_NONE:
            print("ERR n{} {} pos={:.3f}".format(n.id, P.name(P.ERR_NAMES, err), pos / 1000))
            if active:
                self.on_fault("n{} {}".format(n.id, P.name(P.ERR_NAMES, err)))
        if is_event and not (flags & P.F_ENC_OK) and n.cfg and n.cfg.get('enc'):
            print("WARN n{} encoder không dùng được -> chạy hở".format(n.id))
        lost_home = was_homed and not n.homed and state not in (P.ST_HOMING, P.ST_DISABLED)
        if lost_home and n.no_rehome:
            n.no_rehome = False
            print("WARN n{} đã đổi homedir -> chưa home. Gõ 'home' khi sẵn sàng".format(n.id))
        elif lost_home and active:
            self.request_rehome("n{} mất home".format(n.id), n.eye)

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

    # ---------------- home ----------------
    def request_rehome(self, reason, eye):
        if not AUTO_REHOME:
            print("WARN {} -> gõ 'clear' rồi 'home' để home lại".format(reason))
            return
        if self.rehome_tries >= REHOME_MAX_TRIES:
            print("ERR {} nhưng đã home lại {} lần không thành -> kiểm tra, rồi gõ 'clear', 'home'".format(
                reason, self.rehome_tries))
            return
        if not self.rehome_mask & P.eye_mask(eye):
            print("WARN {} -> sẽ dừng hết và HOME LẠI MẮT {}".format(reason, eye))
        self.rehome_mask |= P.eye_mask(eye)
        self.rehome_at = ticks_add(ticks_ms(), REHOME_DELAY_MS)

    def start_home(self, mask):
        self.bcast(P.C_HOME, mask)
        for n in self.nodes:
            if mask & n.bit:
                n.state = P.ST_HOMING
                n.err = P.E_NONE
                n.flags &= ~P.F_HOMED

    def _home_service(self, now):
        act = self.act()
        if not act:
            return
        if not self.start_home_done and all(n.online and n.state is not None for n in act):
            self.start_home_done = True
            need = [e for e in self.active_eyes()
                    if not all(self.nodes[i - 1].homed for i in P.EYES[e])]
            if need:
                print("EVT mắt {} chưa biết vị trí -> HOME TẤT CẢ node của mắt đó "
                      "(chỉ cần lần đầu cấp điện)".format('+'.join(str(e) for e in need)))
                self.fault = False
                self.sysflags &= ~P.SYS_FAULT
                self.rehoming = True
                m = 0
                for e in need:
                    m |= P.eye_mask(e)
                self.start_home(m)
        if self.rehome_at is not None and ticks_diff(now, self.rehome_at) >= 0:
            mask = self.rehome_mask & self.active
            if all(n.online for n in self.nodes if mask & n.bit):
                self.rehome_at = None
                self.rehome_mask = 0
                self.rehome_tries += 1
                self.fault = False
                self.sysflags &= ~P.SYS_FAULT
                if mask:
                    self.rehoming = True
                    print("EVT HOME LẠI TẤT CẢ node {} (lần {})".format(self.mask_str(mask),
                                                                       self.rehome_tries))
                    self.start_home(mask)
        if self.rehoming and all(n.homed and n.state == P.ST_IDLE for n in act):
            self.rehoming = False
            self.rehome_tries = 0
            print("OK đã home lại xong tất cả")

    def resend_last_move(self):
        if all(n.homed for n in self.act()):
            targets = [n.target for n in self.nodes]
            self.move_slot = (self.active, struct.pack(P.MOVE_FMT, 0, *targets))

    # ---------------- giám sát ----------------
    def service(self):
        now = ticks_ms()
        if ticks_diff(now, self.last_sync) >= SYNC_PERIOD_MS:
            self.last_sync = now
            self.send_sync()
        self._repeat_service(now)

        if self.status_due is not None and (ticks_diff(now, self.status_due) >= 0
                                            or all(n.replied for n in self.act())):
            self.status_due = None
            for n in self.act():
                if not n.replied and n.online:
                    n.online = False
                    print("ERR n{} OFFLINE (không trả lời)".format(n.id))
            self.print_status()
        if self.alive_due is not None and ticks_diff(now, self.alive_due) >= 0:
            self.alive_due = None
            missing = 0
            for n in self.act():
                if not n.replied and n.online:
                    missing |= n.bit
            if missing and self.alive_try < ALIVE_RETRY:
                self.alive_try += 1
                self.query(silent=True, mask=missing)
            else:
                for n in self.act():
                    if not n.replied and n.online:
                        n.miss += 1
                        if n.miss >= ALIVE_MISS_MAX:
                            n.online = False
                            print("ERR n{} OFFLINE".format(n.id))
                            self.on_fault("n{} mất liên lạc".format(n.id))
        if ALIVE_CHECK_MS and self.status_due is None and self.alive_due is None \
                and ticks_diff(now, self.last_alive) >= ALIVE_CHECK_MS:
            self.last_alive = now
            self.alive_try = 0
            self.query(silent=True)

        self._home_service(now)
        self._move_service(now)

    # ---------------- lệnh từ PC ----------------
    def mask_str(self, mask):
        return ','.join('n{}'.format(n.id) for n in self.nodes if mask & n.bit) or '-'

    def parse_mask(self, args, default=None):
        """'' -> mắt đang bật, '3' -> node 3, 'e2' -> mắt 2, 'all' -> cả 6 node."""
        if not args:
            return self.active if default is None else default
        a = args[0].lower()
        if a == 'all':
            return P.ALL_NODES
        if a in ('e1', 'e2'):
            return P.eye_mask(int(a[1]))
        nid = int(a)
        if not 1 <= nid <= P.NUM_NODES:
            raise ValueError
        return 1 << (nid - 1)

    def check_ready(self, mask, need_homed):
        if self.fault:
            return "hệ thống đang FAULT, gõ 'clear' trước"
        if not mask:
            return "không có mắt nào đang bật (gõ 'eyes 1', 'eyes 2' hoặc 'eyes 12')"
        for n in self.nodes:
            if not mask & n.bit:
                continue
            if not n.online:
                return "n{} offline".format(n.id)
            if n.state in (P.ST_HOMING, P.ST_ERROR, P.ST_DISABLED):
                return "n{} đang {}".format(n.id, P.name(P.STATE_NAMES, n.state))
            if need_homed and not n.homed:
                return "n{} chưa home".format(n.id)
        return None

    def auto_duration(self, targets_mdeg, mask):
        d = 0
        for n in self.nodes:
            if mask & n.bit:
                d = max(d, abs(targets_mdeg[n.id - 1] - n.target))
        return int(d / 1000 / DEFAULT_SPEED_DPS * 1000) + ACCEL_MARGIN_MS

    def min_duration_ms(self, targets_mdeg, mask):
        T = 0
        v0 = NODE_START_HZ
        for n in self.nodes:
            if not mask & n.bit:
                continue
            S = abs(targets_mdeg[n.id - 1] - n.target) / 1000 * STEPS_PER_DEG
            vm, a = n.c('maxhz'), n.c('accel')
            if S <= (vm * vm - v0 * v0) / a:
                t = 2 * (sqrt(v0 * v0 + a * S) - v0) / a
            else:
                t = 2 * (vm - v0) / a + (S - (vm * vm - v0 * v0) / a) / vm
            T = max(T, t)
        return int(T * 1000 * 1.05) + 20

    def send_move(self, targets_mdeg, mask, dur_ms):
        """targets_mdeg: list 6 phần tử (ô của node không thuộc mask bị bỏ qua)."""
        why = self.check_ready(mask, True)
        if why:
            print("ERR move:", why)
            return
        tmin = self.min_duration_ms(targets_mdeg, mask)
        if dur_ms is None:
            dur_ms = max(self.auto_duration(targets_mdeg, mask), tmin)
        elif 0 < dur_ms < tmin:
            dur_ms = tmin
        dur_ms = max(0, min(dur_ms, 65535))
        for n in self.nodes:
            if mask & n.bit:
                n.target = n.pos = targets_mdeg[n.id - 1]
        full = [n.target for n in self.nodes]
        payload = struct.pack(P.MOVE_FMT, dur_ms, *full)
        if self.move_slot is not None:
            self.dropped += 1
        self.move_slot = (mask, payload)

    def _move_service(self, now):
        if self.move_slot is not None and ticks_diff(now, self.last_move_tx) >= 1000 // MOVE_MAX_FPS:
            mask, payload = self.move_slot
            self.move_slot = None
            self.last_move_tx = now
            self.repeats = [r for r in self.repeats if r[2] != P.C_MOVE]
            self.bcast(P.C_MOVE, mask, payload, offsets=MOVE_RESEND_MS)
        if self.dropped and ticks_diff(now, self.last_drop_report) >= 1000:
            print("INFO bỏ {} lệnh di chuyển (gửi nhanh hơn {} lệnh/giây, chỉ giữ lệnh mới nhất)".format(
                self.dropped, MOVE_MAX_FPS))
            self.dropped = 0
            self.last_drop_report = now

    # ---------------- IK 2 mắt ----------------
    def load_neutral(self):
        out = {1: None, 2: None}
        try:
            with open(NEUTRAL_FILE) as f:
                for line in f:
                    p = line.split()
                    if len(p) == 4:
                        out[int(p[0])] = [float(x) for x in p[1:]]
        except (OSError, ValueError):
            pass
        for e in (1, 2):
            if out[e]:
                print("OK neutral mắt {} = {}".format(e, ' '.join('{:.3f}'.format(v) for v in out[e])))
            else:
                print("WARN mắt {} chưa calib: chỉnh nhìn thẳng bằng 'goto', rồi 'calib {}'".format(e, e))
        return out

    def calib(self, eyes):
        for e in eyes:
            for nid in P.EYES[e]:
                n = self.nodes[nid - 1]
                if not (n.online and n.homed and n.state == P.ST_IDLE):
                    print("ERR calib: n{} (mắt {}) phải online, đã home và đứng yên".format(n.id, e))
                    return
        for e in eyes:
            self.neutral[e] = [self.nodes[nid - 1].target / 1000 for nid in P.EYES[e]]
            print("OK calib mắt {} neutral = {}".format(
                e, ' '.join('{:.3f}'.format(v) for v in self.neutral[e])))
        with open(NEUTRAL_FILE, 'w') as f:
            for e in (1, 2):
                if self.neutral[e]:
                    f.write('{} {}\n'.format(e, ' '.join('{:.3f}'.format(v) for v in self.neutral[e])))

    def eye_gaze(self, eye, az, tilt):
        """Hướng nhìn riêng của 1 mắt sau khi cộng quy tụ và độ xoay lắp đặt."""
        if self.verg:
            # đổi sang tọa độ phẳng (x = sang phải robot, y = vuông góc), cộng quy tụ theo x
            a = radians(az - RIGHT_AZ)
            x, y = tilt * cos(a), tilt * sin(a)
            x += (self.verg / 2) * (1 if EYE_CFG[eye]['side'] == 'L' else -1)
            tilt = sqrt(x * x + y * y)
            az = RIGHT_AZ + (degrees(atan2(y, x)) if tilt > 1e-9 else 0.0)
        return az + EYE_CFG[eye]['az_offset'], tilt

    def eye_targets(self, eye, az, tilt, twist):
        """Góc 3 node (độ) của 1 mắt. Trả về (list, None) hoặc (None, lý do)."""
        if self.neutral[eye] is None:
            return None, "mắt {} chưa calib".format(eye)
        a, t = self.eye_gaze(eye, az, tilt)
        try:
            th = K.ik(a, t, twist)
        except ValueError as ex:
            return None, "mắt {}: {}".format(eye, ex)
        out = []
        for leg, nid in enumerate(P.EYES[eye]):
            n = self.nodes[nid - 1]
            d = self.neutral[eye][leg] + MOTOR_SIGN[nid - 1] * K.wrap180(th[leg] - self.th0[leg])
            lo, hi = n.c('min'), n.c('max')
            if not lo <= d <= hi:
                return None, "mắt {} n{} cần {:.2f} độ, ngoài hành trình {:g}..{:g}".format(
                    eye, nid, d, lo, hi)
            out.append(d)
        return out, None

    def look_targets(self, az, tilt, twist, eyes=None):
        """Góc 6 node (độ, ô của mắt tắt = đích cũ) cho các mắt. (list, None) hoặc (None, lý do)."""
        eyes = self.active_eyes() if eyes is None else eyes
        if not eyes:
            return None, "không có mắt nào đang bật"
        out = [n.target / 1000 for n in self.nodes]
        for e in eyes:
            degs, why = self.eye_targets(e, az, tilt, twist)
            if why:
                return None, why                  # 1 mắt không tới được -> cả 2 mắt không chạy
            for leg, nid in enumerate(P.EYES[e]):
                out[nid - 1] = degs[leg]
        return out, None

    def _max_ok(self, f, hi=60.0):
        lo = 0.0
        for _ in range(12):
            m = (lo + hi) / 2
            if f(m):
                lo = m
            else:
                hi = m
        return lo

    def print_range(self, twist=0.0):
        eyes = [e for e in self.active_eyes() if self.neutral[e]]
        if not eyes:
            print("ERR range: mắt đang bật chưa calib")
            return
        print("RANGE twist={} verg={} (nghiêng tối đa, các mắt đang bật cùng tới được):".format(
            twist, self.verg))
        for az in range(0, 360, 30):
            t = self._max_ok(lambda x: self.look_targets(az, x, twist, eyes)[0] is not None)
            print("RANGE az={:3d} tilt_max={:5.1f}".format(az, t))
            self.poll_rx()
            self.service()

    def set_cfg(self, mask, key, val):
        if key not in P.CFG_KEYS:
            print("ERR set: không có thông số '{}'. Chọn: {}".format(key, ' '.join(P.CFG_KEYS)))
            return
        if key in ('min', 'max', 'homedir'):
            print("WARN đổi '{}' -> nên 'home' và 'calib' lại sau khi đổi".format(key))
        if key == 'homedir':
            for n in self.nodes:
                if mask & n.bit:
                    n.no_rehome = True
        self.bcast(P.C_SETCFG, mask, struct.pack(P.SETCFG_FMT, P.CFG_KEYS.index(key), val))

    def handle_line(self, line):
        p = line.strip().split()
        if not p:
            return
        c = p[0].lower()
        a = p[1:]
        try:
            if c in ('help', '?'):
                print(HELP)
            elif c == 'eyes':
                eyes = sorted(set(int(ch) for ch in ''.join(a) if ch in '12'))
                if not eyes:
                    raise ValueError
                self.set_eyes(eyes)
            elif c == 'verg':
                self.verg = float(a[0])
                print("OK quy tụ = {:.2f} độ (mỗi mắt quay vào {:.2f} độ)".format(self.verg, self.verg / 2))
            elif c == 'dist':
                d = float(a[0])
                self.verg = 0.0 if d <= 0 else 2 * degrees(atan(IPD_MM / 2 / d))
                print("OK nhìn vật cách {} mm -> quy tụ {:.2f} độ".format(a[0], self.verg))
            elif c == 'status':
                self.query()
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
                print("IK az={} tilt={} twist={} -> {}".format(az, tilt, twist, ' '.join(
                    'n{}={:.3f}'.format(i + 1, d) for i, d in enumerate(degs) if self.active & (1 << i))))
                if c == 'look':
                    ms = int(a[3]) if len(a) > 3 else None
                    self.send_move([int(round(d * 1000)) for d in degs], self.active, ms)
            elif c == 'calib':
                eyes = [int(a[0])] if a else self.active_eyes()
                self.calib(eyes)
            elif c == 'range':
                self.print_range(float(a[0]) if a else 0.0)
            elif c == 'move':
                if len(a) < 3:
                    raise ValueError
                three = [int(round(float(x) * 1000)) for x in a[:3]]
                targets = [n.target for n in self.nodes]
                for e in self.active_eyes():
                    for leg, nid in enumerate(P.EYES[e]):
                        targets[nid - 1] = three[leg]
                self.send_move(targets, self.active, int(a[3]) if len(a) > 3 else None)
            elif c == 'goto':
                nid = int(a[0])
                mask = self.parse_mask(a[:1])
                targets = [n.target for n in self.nodes]
                targets[nid - 1] = int(round(float(a[1]) * 1000))
                self.send_move(targets, mask, int(a[2]) if len(a) > 2 else None)
            elif c == 'home':
                mask = self.parse_mask(a)
                if self.fault:
                    print("ERR home: hệ thống đang FAULT, gõ 'clear' trước")
                else:
                    self.rehoming = True
                    self.start_home(mask)
            elif c == 'release':
                mask = self.parse_mask(a)
                self.bcast(P.C_RELEASE, mask)
                for n in self.nodes:
                    if mask & n.bit:
                        n.state = P.ST_DISABLED
                        n.flags &= ~P.F_HOMED
            elif c == 'enable':
                mask = self.parse_mask(a)
                self.bcast(P.C_ENABLE, mask)
                for n in self.nodes:
                    if mask & n.bit and n.state == P.ST_DISABLED:
                        n.state = P.ST_IDLE
            elif c == 'clear':
                mask = self.parse_mask(a)
                self.fault = False
                self.sysflags &= ~P.SYS_FAULT
                self.rehome_tries = 0
                self.bcast(P.C_CLEAR, mask)
                for n in self.nodes:
                    if mask & n.bit:
                        n.err = P.E_NONE
                        if n.state == P.ST_ERROR:
                            n.state = P.ST_IDLE
                print("OK CLEAR")
            elif c == 'speed':
                self.set_cfg(self.active, 'maxhz', float(a[0]))
                if len(a) > 1:
                    self.set_cfg(self.active, 'accel', float(a[1]))
            elif c == 'blink':
                self.bcast(P.C_IDENT, self.parse_mask(a))
            elif c == 'cfg':
                for n in self.nodes:
                    self.print_cfg(n)
                self.bcast(P.C_GETCFG, P.ALL_NODES, repeat=1)
            elif c == 'set':
                self.set_cfg(self.parse_mask(a[:1]), a[1].lower(), float(a[2]))
            else:
                print("ERR lệnh không rõ:", c)
        except (ValueError, IndexError):
            print("ERR sai tham số. Gõ 'help'")

    def print_status(self):
        print("STAT fault={} eyes={} verg={:.2f}".format(
            self.fault, '+'.join(str(e) for e in self.active_eyes()) or '-', self.verg))
        for n in self.act():                     # chỉ in node của mắt đang bật (PC đếm theo đây)
            print("STAT n{} {} {:8} err={:14} homed={} sw={} pos={:8.3f} target={:8.3f}".format(
                n.id, 'ON ' if n.online else 'OFF',
                P.name(P.STATE_NAMES, n.state) if n.state is not None else '-',
                P.name(P.ERR_NAMES, n.err),
                int(n.homed), int(bool(n.flags & P.F_HOME_SW)),
                n.pos / 1000, n.target / 1000))
        for n in self.act():
            print("ENC n{} encoder={} lệch={} vi bước  đã bù={} lần  khôi phục sau reset={}".format(
                n.id, 'OK' if n.flags & P.F_ENC_OK else 'KHÔNG', n.enc_err, n.fixes,
                int(bool(n.flags & P.F_RESTORED))))
        for n in self.act():
            print("PWR n{} VBUS={:.2f} V  thấp nhất={:.2f} V  thấp nhất từ đầu={:.2f} V".format(
                n.id, n.vbus / 1000, n.vmin / 1000, n.vmin_all / 1000))


class LineReader:
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
    use_wdt = ENABLE_WDT and P.wdt_allowed()
    if use_wdt:
        print("Master khởi động sau {} s (Ctrl+C để dừng trước khi bật watchdog)".format(
            BOOT_DELAY_MS // 1000))
        sleep_ms(BOOT_DELAY_MS)
    m = Master()
    reader = LineReader()
    wdt = WDT(timeout=WDT_TIMEOUT_MS) if use_wdt else None
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
        print("Master đã dừng.")


if __name__ == '__main__':
    main()
