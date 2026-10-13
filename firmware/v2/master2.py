# master2.py - firmware master THẾ HỆ 2 ("master nói, node im lặng").
# Nạp lên board master với tên main.py, kèm proto2.py, delta_proto.py, spm_ik.py.
# Lệnh gõ từ PC và các dòng in ra GIỐNG bản 1 (master.py) -> pc/eye_movie.py dùng được ngay.
#
# Khác bản 1:
#   - Mọi lệnh gửi BROADCAST, không chờ ACK. Lệnh quan trọng phát lặp CMD_REPEAT lần
#     (node bỏ bản trùng theo seq). MOVE: lệnh mới thay lệnh cũ, lệnh cuối được phát lại thêm.
#   - SYNC 4 lần/giây = nhịp tim cho node + xác nhận HELLO / EVENT node gửi lên.
#   - Master không biết từng node đang chạy hay đứng; chỉ biết qua EVENT (lỗi, home xong...)
#     và khi hỏi ('status', hoặc kiểm tra định kỳ ALIVE_CHECK_MS).
#   - Node bị reset mà khôi phục được vị trí (RTC + encoder) -> KHÔNG home lại.

import sys
import select
import struct
import os
from math import sqrt
from machine import WDT
from time import ticks_ms, ticks_diff, ticks_add, sleep_ms
import proto2 as P
import spm_ik as K

# ==== CẤU HÌNH ====
ENABLE_WDT        = False
WDT_TIMEOUT_MS    = 3000
BOOT_DELAY_MS     = 3000
SYNC_PERIOD_MS    = 250      # nhịp tim (node mất liên lạc sau 1 s không nghe gì)
CMD_REPEAT        = 3        # lệnh thường phát lặp 3 lần
STOP_REPEAT       = 5
REPEAT_GAP_MS     = 30
# MOVE phát lại sau các mốc này (ms) nếu chưa có MOVE mới hơn: khi đang bám liên tục chỉ kịp
# phát lại 1 lần (30 ms); lệnh CUỐI của chuỗi được phát lại thêm ở 300 ms và 1 s -> không bị kẹt
# ở đích cũ nếu gói cuối bị rớt (node bỏ bản trùng theo seq).
MOVE_RESEND_MS    = (30, 300, 1000)
MOVE_MAX_FPS      = 15
STATUS_WAIT_MS    = 300      # 'status': chờ node trả lời tối đa ngần này
# Hỏi trạng thái định kỳ để biết node còn sống (mất điện hẳn thì node không tự báo được).
# Mỗi node trả 1 gói nhỏ / lần. Đặt 0 để tắt hẳn (khi đó chỉ biết node chết lúc gõ 'status').
ALIVE_CHECK_MS    = 5000
ALIVE_RETRY       = 3        # mỗi lần kiểm tra: node chưa trả lời được hỏi lại riêng tối đa 3 lần
ALIVE_MISS_MAX    = 2        # 2 lần kiểm tra liên tiếp vẫn không trả lời -> OFFLINE
DEFAULT_SPEED_DPS = 15.0
ACCEL_MARGIN_MS   = 200

AUTO_HOME_AT_START = True    # 3 node cùng online mà có node chưa biết vị trí -> tự home tất cả
AUTO_REHOME        = True
REHOME_DELAY_MS    = 1000
REHOME_MAX_TRIES   = 3

MOTOR_SIGN   = (1, 1, 1)
NEUTRAL_FILE = 'neutral.txt'
CFG_FALLBACK = {'min': 0.5, 'max': 25.71, 'maxhz': 2000, 'accel': 6000}
VBUS_WARN_MV   = 10500
STEPS_PER_DEG  = 200 * 8 * 14 / 360.0   # PHẢI GIỐNG node2.py
NODE_START_HZ  = 300

HELP = """Lệnh (thế hệ 2 - vòng kín encoder):
  look az tilt [twist] [ms] : nhìn theo tọa độ cực (độ), chạy trong ms mili-giây
  ik az tilt [twist]  : chỉ tính góc 3 động cơ
  calib               : lưu vị trí hiện tại làm vị trí nhìn thẳng
  range [twist]       : góc nghiêng tối đa theo từng hướng
  move d1 d2 d3 [ms]  : 3 trục tới góc d1,d2,d3 (độ)
  goto n deg [ms]     : chỉ node n tới góc deg
  home [n]            : home (chỉ cần lần đầu cấp điện)
  stop                : dừng khẩn cấp tất cả
  release [n] / enable [n]
  clear [n]           : xóa lỗi
  speed hz [accel]    : tốc độ tối đa / gia tốc cả 3 node (có lưu)
  status              : hỏi trạng thái 3 node (vị trí, encoder, nguồn)
  blink [n]           : đèn node nháy trắng 4 giây
  cfg                 : đọc cấu hình các node
  set <n|all> <tên> <giá trị> : tên: min max homedir maxhz accel homefast homeslow led enc encdir
        enc 0 = tắt vòng kín (chạy hở như bản 1); encdir 0 = tự dò chiều encoder lần home tới
"""


class NodeInfo:
    def __init__(self, nid):
        self.id = nid
        self.bit = 1 << (nid - 1)
        self.online = False
        self.ever_seen = False
        self.state = None
        self.err = P.E_NONE
        self.flags = 0
        self.pos = 0          # mili-độ (báo về, hoặc đích lệnh gần nhất)
        self.target = 0
        self.boot_id = 0
        self.evt_ack = 0      # evt_seq gần nhất đã nhận -> gửi lại trong SYNC
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

    @property
    def busy(self):
        return self.state == P.ST_HOMING


class Master:
    def __init__(self):
        self.e, mac = P.init_espnow()
        print("OK master2 MAC", P.mac_str(mac))
        if not P.is_master(mac):
            print("WARN MAC này khác MASTER_MAC trong delta_proto.py -> các node sẽ bỏ qua lệnh!")
        self.nodes = [NodeInfo(i) for i in range(1, P.NUM_NODES + 1)]
        self.unknown_macs = set()
        self.th0 = K.ik(0, 0, 0)
        self.neutral = self.load_neutral()
        self.seq = struct.unpack('<H', os.urandom(2))[0]
        self.repeats = []        # [hạn, gói, số lần còn lại, typ]
        self.fault = False
        self.sysflags = 0
        now = ticks_ms()
        self.last_sync = now
        self.move_slot = None
        self.last_move_tx = ticks_add(now, -1000)
        self.dropped = 0
        self.last_drop_report = now
        self.rehome_at = None
        self.rehome_tries = 0
        self.rehoming = False
        self.start_home_done = not AUTO_HOME_AT_START
        self.status_due = None   # đang chờ trả lời 'status' của người dùng
        self.alive_due = None    # đang chờ trả lời kiểm tra định kỳ
        self.alive_try = 0
        self.last_alive = now
        # Master vừa khởi động (vd PC mở cổng làm master reset): hỏi ngay để biết các node
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
        """Phát 1 lệnh. Phát lại cùng seq sau các mốc offsets (ms), mặc định cách đều REPEAT_GAP_MS."""
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

    def query(self, silent=False, mask=P.ALL_NODES):
        """Hỏi trạng thái node (QUERY). silent: kiểm tra định kỳ, không in bảng."""
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
                print("EVT n{} ONLINE {}".format(nid, P.mac_str(mac)))
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
        # Node khởi động lại GIỮA CHỪNG = trước đó master đã biết node ở trạng thái đã home.
        # (Cấp điện cùng lúc: master có thể nghe trả lời 'status' trước HELLO -> không tính là reset.)
        rebooted = n.boot_id != 0 or n.homed
        n.boot_id = boot_id                     # SYNC kế tiếp sẽ xác nhận HELLO này
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
        if n.state != P.ST_HOMING:            # có thể master đã ra lệnh home trước khi HELLO tới
            n.state = P.ST_IDLE
        n.err = P.E_NONE
        if flags & P.F_RESTORED:
            print("EVT n{} khôi phục vị trí {:.3f} độ từ encoder -> không cần home".format(n.id, pos / 1000))
            if rebooted:
                # node dừng ở chỗ lúc reset -> gửi lại đích master đã ra lệnh gần nhất
                n.target = old_target
                self.resend_last_move()
        elif rebooted:
            self.on_fault("n{} khởi động lại ({}), mất vị trí".format(n.id, cname))
            self.request_rehome("n{} khởi động lại".format(n.id))

    def _on_status(self, n, payload, is_event):
        (evt, state, err, flags, pos, target, enc_err, vbus, vmin, fixes) = \
            struct.unpack(P.STATUS_FMT, payload)
        if is_event:
            if evt == n.evt_ack:
                return                          # bản gửi lại của EVENT đã xử lý
            n.evt_ack = evt
        else:
            n.replied = True
        old_state, old_err, was_homed = n.state, n.err, n.homed
        n.state, n.err, n.flags = state, err, flags
        n.pos, n.enc_err = pos, enc_err
        if is_event:
            # chỉ EVENT mới đáng tin hơn đích master đã ra lệnh (vd đích đỗ sau khi home);
            # trả lời QUERY có thể rời node TRƯỚC khi node nhận MOVE mới nhất
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
        if err != old_err and err != P.E_NONE:
            print("ERR n{} {} pos={:.3f}".format(n.id, P.name(P.ERR_NAMES, err), pos / 1000))
            self.on_fault("n{} {}".format(n.id, P.name(P.ERR_NAMES, err)))
        if is_event and not (flags & P.F_ENC_OK) and n.cfg and n.cfg.get('enc'):
            print("WARN n{} encoder không dùng được -> chạy hở".format(n.id))
        lost_home = was_homed and not n.homed and state not in (P.ST_HOMING, P.ST_DISABLED)
        if lost_home and n.no_rehome:
            n.no_rehome = False
            print("WARN n{} đã đổi homedir -> chưa home. Gõ 'home' khi sẵn sàng".format(n.id))
        elif lost_home:
            self.request_rehome("n{} mất home".format(n.id))

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
    def request_rehome(self, reason):
        if not AUTO_REHOME:
            print("WARN {} -> gõ 'clear' rồi 'home' để home lại".format(reason))
            return
        if self.rehome_tries >= REHOME_MAX_TRIES:
            print("ERR {} nhưng đã home lại {} lần không thành -> kiểm tra, rồi gõ 'clear', 'home'".format(
                reason, self.rehome_tries))
            return
        if self.rehome_at is None:
            print("WARN {} -> sẽ dừng hết và HOME LẠI TẤT CẢ".format(reason))
        self.rehome_at = ticks_add(ticks_ms(), REHOME_DELAY_MS)

    def start_home(self, mask):
        # C_HOME trên node tự bật motor + xóa lỗi trước khi home
        self.bcast(P.C_HOME, mask)
        for n in self.nodes:
            if mask & n.bit:
                n.state = P.ST_HOMING
                n.err = P.E_NONE
                n.flags &= ~P.F_HOMED

    def _home_service(self, now):
        if not self.start_home_done and all(n.online for n in self.nodes) \
                and all(n.state is not None for n in self.nodes):
            self.start_home_done = True
            if not all(n.homed for n in self.nodes):
                print("EVT có node chưa biết vị trí -> HOME TẤT CẢ (chỉ cần lần đầu cấp điện)")
                self.fault = False
                self.sysflags &= ~P.SYS_FAULT
                self.rehoming = True
                self.start_home(P.ALL_NODES)
        if self.rehome_at is not None and ticks_diff(now, self.rehome_at) >= 0:
            if all(n.online for n in self.nodes):
                self.rehome_at = None
                self.rehome_tries += 1
                self.fault = False
                self.sysflags &= ~P.SYS_FAULT
                self.rehoming = True
                print("EVT HOME LẠI TẤT CẢ (lần {})".format(self.rehome_tries))
                self.start_home(P.ALL_NODES)
        if self.rehoming and all(n.homed and n.state == P.ST_IDLE for n in self.nodes):
            self.rehoming = False
            self.rehome_tries = 0
            print("OK đã home lại xong tất cả")

    def resend_last_move(self):
        targets = [n.target for n in self.nodes]
        if all(n.homed for n in self.nodes):
            self.move_slot = (P.ALL_NODES, struct.pack(P.MOVE_FMT, 0, *targets))

    # ---------------- giám sát ----------------
    def service(self):
        now = ticks_ms()
        if ticks_diff(now, self.last_sync) >= SYNC_PERIOD_MS:
            self.last_sync = now
            self.send_sync()
        self._repeat_service(now)

        if self.status_due is not None and (ticks_diff(now, self.status_due) >= 0
                                            or all(n.replied for n in self.nodes)):
            self.status_due = None
            for n in self.nodes:
                if not n.replied and n.online:
                    n.online = False
                    print("ERR n{} OFFLINE (không trả lời)".format(n.id))
            self.print_status()
        if self.alive_due is not None and ticks_diff(now, self.alive_due) >= 0:
            self.alive_due = None
            missing = 0
            for n in self.nodes:
                if not n.replied and n.online:
                    missing |= n.bit
            if missing and self.alive_try < ALIVE_RETRY:
                self.alive_try += 1
                self.query(silent=True, mask=missing)     # hỏi lại riêng node chưa trả lời
                return
            for n in self.nodes:
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

    def parse_mask(self, args):
        if not args:
            return P.ALL_NODES
        nid = int(args[0])
        if not 1 <= nid <= P.NUM_NODES:
            raise ValueError
        return 1 << (nid - 1)

    def check_ready(self, mask, need_homed):
        if self.fault:
            return "hệ thống đang FAULT, gõ 'clear' trước"
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
        for n in self.nodes:                    # master không nghe node báo vị trí -> giả định tới đích
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
            self.repeats = [r for r in self.repeats if r[2] != P.C_MOVE]   # bỏ bản lặp của MOVE cũ
            self.bcast(P.C_MOVE, mask, payload, offsets=MOVE_RESEND_MS)
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
                print("ERR calib: n{} phải online, đã home và đứng yên (gõ 'status' trước)".format(n.id))
                return
        self.neutral = [n.target / 1000 for n in self.nodes]
        with open(NEUTRAL_FILE, 'w') as f:
            f.write(' '.join('{:.3f}'.format(v) for v in self.neutral))
        print("OK calib neutral = {}".format(' '.join('{:.3f}'.format(v) for v in self.neutral)))

    def look_targets(self, az, tilt, twist):
        if self.neutral is None:
            return None, "chưa calib"
        try:
            th = K.ik(az, tilt, twist)
        except ValueError as ex:
            return None, str(ex)
        out = []
        for i, n in enumerate(self.nodes):
            d = self.neutral[i] + MOTOR_SIGN[i] * K.wrap180(th[i] - self.th0[i])
            lo, hi = n.c('min'), n.c('max')
            if not lo <= d <= hi:
                return None, "n{} cần {:.2f} độ, ngoài hành trình {:g}..{:g}".format(i + 1, d, lo, hi)
            out.append(d)
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
        if self.neutral is None:
            print("ERR range: chưa calib")
            return
        print("RANGE twist={} (nghiêng tối đa theo hướng):".format(twist))
        for az in range(0, 360, 30):
            t = self._max_ok(lambda x: self.look_targets(az, x, twist)[0] is not None)
            print("RANGE az={:3d} tilt_max={:5.1f}".format(az, t))
            self.poll_rx()
            self.service()
        tp = self._max_ok(lambda x: self.look_targets(0, 0, x)[0] is not None)
        tn = self._max_ok(lambda x: self.look_targets(0, 0, -x)[0] is not None)
        print("RANGE twist khi nhìn thẳng: -{:.1f} .. +{:.1f}".format(tn, tp))

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
        N = P.NUM_NODES
        try:
            if c in ('help', '?'):
                print(HELP)
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
                self.bcast(P.C_RELEASE, self.parse_mask(a))
                for n in self.nodes:
                    if self.parse_mask(a) & n.bit:
                        n.state = P.ST_DISABLED
                        n.flags &= ~P.F_HOMED
            elif c == 'enable':
                self.bcast(P.C_ENABLE, self.parse_mask(a))
                for n in self.nodes:
                    if self.parse_mask(a) & n.bit and n.state == P.ST_DISABLED:
                        n.state = P.ST_IDLE
            elif c == 'clear':
                self.fault = False
                self.sysflags &= ~P.SYS_FAULT
                self.rehome_tries = 0
                self.bcast(P.C_CLEAR, self.parse_mask(a))
                for n in self.nodes:
                    if self.parse_mask(a) & n.bit:
                        n.err = P.E_NONE
                        if n.state == P.ST_ERROR:
                            n.state = P.ST_IDLE
                print("OK CLEAR")
            elif c == 'speed':
                self.set_cfg(P.ALL_NODES, 'maxhz', float(a[0]))
                if len(a) > 1:
                    self.set_cfg(P.ALL_NODES, 'accel', float(a[1]))
            elif c == 'blink':
                self.bcast(P.C_IDENT, self.parse_mask(a))
            elif c == 'cfg':
                for n in self.nodes:
                    self.print_cfg(n)
                self.bcast(P.C_GETCFG, P.ALL_NODES, repeat=1)
            elif c == 'set':
                mask = P.ALL_NODES if a[0].lower() == 'all' else self.parse_mask(a[:1])
                self.set_cfg(mask, a[1].lower(), float(a[2]))
            else:
                print("ERR lệnh không rõ:", c)
        except (ValueError, IndexError):
            print("ERR sai tham số. Gõ 'help'")

    def print_status(self):
        print("STAT fault={} queue=0 inflight=-".format(self.fault))
        for n in self.nodes:
            print("STAT n{} {} {:8} err={:14} homed={} sw={} pos={:8.3f} target={:8.3f}".format(
                n.id, 'ON ' if n.online else 'OFF',
                P.name(P.STATE_NAMES, n.state) if n.state is not None else '-',
                P.name(P.ERR_NAMES, n.err),
                int(n.homed), int(bool(n.flags & P.F_HOME_SW)),
                n.pos / 1000, n.target / 1000))
        for n in self.nodes:
            print("ENC n{} encoder={} lệch={} vi bước  đã bù={} lần  khôi phục sau reset={}".format(
                n.id, 'OK' if n.flags & P.F_ENC_OK else 'KHÔNG', n.enc_err, n.fixes,
                int(bool(n.flags & P.F_RESTORED))))
        for n in self.nodes:
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
