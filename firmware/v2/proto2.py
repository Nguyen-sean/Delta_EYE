# proto2.py - giao thức ESP-NOW thế hệ 2 ("master nói, node im lặng").
# Nạp lên TẤT CẢ các board, cùng với delta_proto.py (lấy bảng MAC, kênh, công suất phát từ đó).
#
# Khác bản 1:
#   - Master -> node: CHỈ BROADCAST, không chờ ACK. Lệnh quan trọng được phát lặp vài lần
#     (cùng seq, node bỏ bản trùng). MOVE là "mới nhất thắng": lỡ 1 gói thì gói sau thay thế.
#   - Node -> master: chỉ gửi khi CẦN:
#       HELLO  : vừa khởi động (lặp 1 s / lần đến khi master xác nhận qua SYNC)
#       EVENT  : sự kiện quan trọng (home xong, lỗi, mất home, sụt nguồn...) - lặp đến khi được xác nhận
#       STATUS : trả lời lệnh QUERY (lệnh 'status' của master)
#       CFG    : trả lời GETCFG / SETCFG
#     Không còn ACK từng lệnh, không còn trả lời PING 5 lần/giây.
#   - Master phát SYNC 4 lần/giây: vừa là nhịp tim, vừa XÁC NHẬN HELLO / EVENT của từng node.
#
# Gói tin = header 6 byte + payload (little-endian):
#   magic(1) | version(1) | type(1) | node(1) | seq(2) | payload...
#   Master -> node: 'node' = bitmask node nhận. Node -> master: 'node' = ID node gửi.

import struct
import delta_proto as P0          # bảng MAC, kênh Wi-Fi, init_espnow... (dùng lại, không sửa)

MAGIC     = 0xD6                  # khác bản 1 (0xD5) -> 2 thế hệ không lẫn gói của nhau
VERSION   = 2
NUM_NODES = P0.NUM_NODES
ALL_NODES = P0.ALL_NODES
BROADCAST = P0.BROADCAST
MASTER_MAC = P0.MASTER_MAC

# ---- Master -> node (broadcast) ----
C_SYNC    = 0x10   # SYNC_FMT: nhịp tim + xác nhận HELLO/EVENT
C_MOVE    = 0x11   # MOVE_FMT: thời gian + góc đích cả 3 node (node chỉ đọc ô của mình)
C_HOME    = 0x12
C_STOP    = 0x13
C_CLEAR   = 0x14
C_RELEASE = 0x15
C_ENABLE  = 0x16
C_IDENT   = 0x17
C_SETCFG  = 0x18   # SETCFG_FMT
C_GETCFG  = 0x19
C_QUERY   = 0x1A   # node trả STATUS

# ---- Node -> master (unicast tới MASTER_MAC) ----
M_HELLO   = 0x90   # HELLO_FMT
M_EVENT   = 0x91   # STATUS_FMT (có evt_seq)
M_STATUS  = 0x92   # STATUS_FMT (trả lời QUERY, evt_seq = 0)
M_CFG     = 0x93   # CFG_FMT

CMD_NAMES = {C_SYNC: 'SYNC', C_MOVE: 'MOVE', C_HOME: 'HOME', C_STOP: 'STOP', C_CLEAR: 'CLEAR',
             C_RELEASE: 'RELEASE', C_ENABLE: 'ENABLE', C_IDENT: 'IDENT', C_SETCFG: 'SETCFG',
             C_GETCFG: 'GETCFG', C_QUERY: 'QUERY'}

# boot_id đã biết của 3 node, evt_seq đã nhận của 3 node, cờ hệ thống
SYNC_FMT   = '<' + 'H' * NUM_NODES + 'B' * NUM_NODES + 'B'
MOVE_FMT   = '<H' + 'i' * NUM_NODES          # dur_ms, đích (mili-độ trục ra)
# reset_cause, fw, boot_id, flags (F_*), pos_mdeg, vbus_min_trước_reset, vbus_cuối_trước_reset (mV)
HELLO_FMT  = '<BBHBiHH'
# evt_seq, state, err, flags, pos_mdeg, target_mdeg, enc_err (bước), vbus_mV, vbus_min_mV, fixes
STATUS_FMT = '<BBBBiihHHH'
SETCFG_FMT = '<Bf'

# ---- Cấu hình node chỉnh được từ master (lưu trong config2.json trên node) ----
#   enc    : 1 = dùng encoder AS5600 (vòng kín), 0 = chạy hở như bản 1
#   encdir : chiều encoder so với chiều bước: +1 / -1, 0 = tự dò lần home tới
CFG_KEYS = ('min', 'max', 'homedir', 'maxhz', 'accel', 'homefast', 'homeslow', 'led',
            'enc', 'encdir')
CFG_FMT  = '<' + 'f' * len(CFG_KEYS)

# ---- Trạng thái / cờ / lỗi ----
ST_DISABLED, ST_IDLE, ST_HOMING, ST_MOVING, ST_ERROR = range(5)
STATE_NAMES = ('DISABLED', 'IDLE', 'HOMING', 'MOVING', 'ERROR')

F_HOMED    = 0x01   # biết vị trí tuyệt đối
F_HOME_SW  = 0x02   # công tắc home đang bị nhấn
F_ENABLED  = 0x04
F_ENC_OK   = 0x08   # encoder đọc được, có nam châm
F_RESTORED = 0x10   # vị trí được khôi phục sau reset (RTC + encoder), không cần home lại

E_NONE, E_HOME_NOT_FOUND, E_HOME_STUCK, E_LOST_STEP, E_LINK_LOST, E_MOTION_TIMEOUT, \
    E_INTERNAL, E_STALL, E_ENCODER = range(9)
ERR_NAMES = ('NONE', 'HOME_NOT_FOUND', 'HOME_STUCK', 'LOST_STEP', 'LINK_LOST',
             'MOTION_TIMEOUT', 'INTERNAL', 'STALL', 'ENCODER')

SYS_FAULT = 0x01    # cờ trong SYNC: hệ thống đang FAULT

RESET_NAMES = P0.RESET_NAMES
name = P0.name
mac_str = P0.mac_str
node_id_of = P0.node_id_of
is_master = P0.is_master
init_espnow = P0.init_espnow
wdt_allowed = P0.wdt_allowed


def pack(typ, node, seq, payload=b''):
    return struct.pack('<BBBBH', MAGIC, VERSION, typ, node, seq & 0xFFFF) + payload


def unpack(msg):
    if msg is None or len(msg) < 6:
        return None
    magic, ver, typ, node, seq = struct.unpack('<BBBBH', msg[:6])
    if magic != MAGIC or ver != VERSION:
        return None
    return typ, node, seq, msg[6:]


def mac_bytes(s):
    """'F8:B3:B7:50:DD:D0' -> b'\\xf8\\xb3...'"""
    return bytes(int(x, 16) for x in s.split(':'))
