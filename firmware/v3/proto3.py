# proto3.py - giao thức ESP-NOW thế hệ 3: HAI MẮT (6 node), 1 master.
# Nạp CÙNG file này lên master và cả 6 node. Không cần delta_proto.py / proto2.py.
#
# Cách liên lạc giống bản 2 ("master nói, node im lặng"):
#   Master -> node: chỉ BROADCAST (1 gói tới cả 6 node -> 2 mắt xuất phát cùng lúc),
#                   lệnh quan trọng phát lặp, MOVE "mới nhất thắng" + phát lại lệnh cuối.
#   Node -> master: unicast, chỉ khi cần (HELLO, EVENT, trả lời QUERY / CFG).
#   SYNC 4 lần/giây: nhịp tim + xác nhận HELLO / EVENT.
#
# Gói tin = header 6 byte + payload:  magic | version | type | node | seq(2) | payload
#   Master -> node: 'node' = bitmask (bit0 = node 1 ... bit5 = node 6)

import os
import struct
import machine

MAGIC     = 0xD7                  # khác bản 1 (D5), bản 2 (D6)
VERSION   = 3
CHANNEL   = 1                     # kênh Wi-Fi, mọi board phải giống nhau
TX_POWER_DBM = 8.5
BROADCAST = b'\xff' * 6

# ==== MẮT VÀ NODE ====
# Mắt 1 = node 1, 2, 3 (chân 1, 2, 3 của IK). Mắt 2 = node 4, 5, 6 (chân 1, 2, 3 của IK).
EYES = {1: (1, 2, 3), 2: (4, 5, 6)}
NUM_NODES = 6
ALL_NODES = (1 << NUM_NODES) - 1

# ==== BẢNG ĐỊA CHỈ MAC ====
# Lấy MAC của 1 board: trong REPL gõ   import proto3; proto3.my_mac()
MASTER_MAC = 'F8:B3:B7:50:DD:D0'
NODE_MACS = {
    'F8:5B:1B:4C:81:C0': 1,    # mắt 1 - motor 1
    'F8:5B:1B:4B:83:6C': 2,    # mắt 1 - motor 2
    'F8:5B:1B:49:DA:70': 3,    # mắt 1 - motor 3
    'F8:5B:1B:4C:A1:A8': 4,    # mắt 2 - motor 1
    'F8:5B:1B:4B:D2:D0': 5,    # mắt 2 - motor 2
    'F8:5B:1B:4A:E2:8C': 6,    # mắt 2 - motor 3
}


def eye_of(nid):
    for e, ids in EYES.items():
        if nid in ids:
            return e
    return None


def eye_mask(eye):
    m = 0
    for nid in EYES[eye]:
        m |= 1 << (nid - 1)
    return m


# ---- Master -> node (broadcast) ----
C_SYNC    = 0x10
C_MOVE    = 0x11   # MOVE_FMT: thời gian + đích của cả 6 node (node chỉ đọc ô của mình)
C_HOME    = 0x12   # node tự bật motor + xóa lỗi rồi home
C_STOP    = 0x13
C_CLEAR   = 0x14
C_RELEASE = 0x15
C_ENABLE  = 0x16
C_IDENT   = 0x17
C_SETCFG  = 0x18
C_GETCFG  = 0x19
C_QUERY   = 0x1A

# ---- Node -> master (unicast) ----
M_HELLO   = 0x90
M_EVENT   = 0x91
M_STATUS  = 0x92
M_CFG     = 0x93

CMD_NAMES = {C_SYNC: 'SYNC', C_MOVE: 'MOVE', C_HOME: 'HOME', C_STOP: 'STOP', C_CLEAR: 'CLEAR',
             C_RELEASE: 'RELEASE', C_ENABLE: 'ENABLE', C_IDENT: 'IDENT', C_SETCFG: 'SETCFG',
             C_GETCFG: 'GETCFG', C_QUERY: 'QUERY'}

SYNC_FMT   = '<' + 'H' * NUM_NODES + 'B' * NUM_NODES + 'B'   # boot_id, evt_ack từng node, cờ
MOVE_FMT   = '<H' + 'i' * NUM_NODES                          # dur_ms, đích (mili-độ)
HELLO_FMT  = '<BBHBiHH'
STATUS_FMT = '<BBBBiihHHH'
SETCFG_FMT = '<Bf'

CFG_KEYS = ('min', 'max', 'homedir', 'maxhz', 'accel', 'homefast', 'homeslow', 'led',
            'enc', 'encdir')
CFG_FMT  = '<' + 'f' * len(CFG_KEYS)

ST_DISABLED, ST_IDLE, ST_HOMING, ST_MOVING, ST_ERROR = range(5)
STATE_NAMES = ('DISABLED', 'IDLE', 'HOMING', 'MOVING', 'ERROR')

F_HOMED    = 0x01
F_HOME_SW  = 0x02
F_ENABLED  = 0x04
F_ENC_OK   = 0x08
F_RESTORED = 0x10

E_NONE, E_HOME_NOT_FOUND, E_HOME_STUCK, E_LOST_STEP, E_LINK_LOST, E_MOTION_TIMEOUT, \
    E_INTERNAL, E_STALL, E_ENCODER = range(9)
ERR_NAMES = ('NONE', 'HOME_NOT_FOUND', 'HOME_STUCK', 'LOST_STEP', 'LINK_LOST',
             'MOTION_TIMEOUT', 'INTERNAL', 'STALL', 'ENCODER')

SYS_FAULT = 0x01

RESET_NAMES = {
    machine.PWRON_RESET: 'POWER_ON',
    machine.HARD_RESET: 'HARD_RESET',
    machine.WDT_RESET: 'WATCHDOG',
    machine.DEEPSLEEP_RESET: 'DEEPSLEEP',
    machine.SOFT_RESET: 'SOFT_RESET',
}


def name(table, idx):
    try:
        return table[idx]
    except (IndexError, KeyError):
        return '?{}'.format(idx)


def pack(typ, node, seq, payload=b''):
    return struct.pack('<BBBBH', MAGIC, VERSION, typ, node, seq & 0xFFFF) + payload


def unpack(msg):
    if msg is None or len(msg) < 6:
        return None
    magic, ver, typ, node, seq = struct.unpack('<BBBBH', bytes(msg[:6]))
    if magic != MAGIC or ver != VERSION:
        return None
    return typ, node, seq, bytes(msg[6:])


def mac_str(mac):
    return ':'.join('{:02X}'.format(b) for b in mac)


def mac_bytes(s):
    return bytes([int(x, 16) for x in s.split(':')])


def node_id_of(mac):
    s = mac_str(mac)
    for k, nid in NODE_MACS.items():
        if k.upper() == s:
            return nid
    return None


def is_master(mac):
    return MASTER_MAC is None or mac_str(mac) == MASTER_MAC.upper()


def my_mac():
    import network
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    mac = sta.config('mac')
    nid = node_id_of(mac)
    role = "node {} (mắt {})".format(nid, eye_of(nid)) if nid else \
        ("master" if is_master(mac) else "(chưa có trong NODE_MACS)")
    print("MAC:", mac_str(mac), "->", role)
    return mac_str(mac)


def init_espnow():
    import network
    import espnow
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    try:
        sta.disconnect()
    except OSError:
        pass
    sta.config(channel=CHANNEL)
    try:
        sta.config(pm=sta.PM_NONE)
    except (AttributeError, ValueError):
        pass
    if TX_POWER_DBM is not None:
        try:
            sta.config(txpower=TX_POWER_DBM)
        except (AttributeError, ValueError, OSError) as ex:
            print("Không đặt được txpower:", ex)
    e = espnow.ESPNow()
    try:
        e.config(rxbuf=2048)
    except (ValueError, OSError):
        pass
    e.active(True)
    try:
        e.add_peer(BROADCAST)
    except OSError:
        pass
    return e, sta.config('mac')


NOWDT_FILE = 'nowdt'


def wdt_allowed():
    """False nếu có file 'nowdt' hoặc đang giữ nút BOOT lúc khởi động."""
    try:
        os.stat(NOWDT_FILE)
        return False
    except OSError:
        pass
    pin = 9 if 'C3' in os.uname().machine else 0
    return machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP).value() != 0
