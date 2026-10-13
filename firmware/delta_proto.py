# delta_proto.py - giao thức ESP-NOW dùng chung cho master và các board động cơ.
# Nạp file này lên TẤT CẢ các board (cùng một nội dung).
#
# Gói tin = header 6 byte + payload (little-endian):
#   magic(1) | version(1) | type(1) | node(1) | seq(2) | payload...
#
#   Master -> node : 'node' là BITMASK node nhận lệnh (bit0 = node 1, bit1 = node 2 ...)
#                    -> 1 gói broadcast điều khiển được nhiều node cùng lúc,
#                       cả 3 trục bắt đầu chạy cùng một thời điểm.
#   Node -> master : 'node' là ID của node gửi (1..NUM_NODES)
#
# Độ tin cậy: mọi lệnh (trừ PING) đều được node trả ACK kèm 'seq' của lệnh.
# Master gửi lại nếu chưa đủ ACK; node nhớ các seq gần nhất để không thực hiện
# một lệnh 2 lần khi nhận bản gửi lại.

import os
import struct
import machine

MAGIC     = 0xD5
VERSION   = 1
CHANNEL   = 1                 # kênh Wi-Fi, tất cả board phải giống nhau
# Công suất phát Wi-Fi (dBm). ESP32-C3 phát mặc định ~20 dBm kéo dòng đỉnh rất lớn, hay làm
# board C3 cỡ nhỏ sụt áp / panic. Các board cách nhau < 1 m nên 8.5 dBm là thừa. None = mặc định.
TX_POWER_DBM = 8.5
BROADCAST = b'\xff' * 6
NUM_NODES = 3
ALL_NODES = (1 << NUM_NODES) - 1

# ==== BẢNG ĐỊA CHỈ MAC ====
# ID của mỗi board động cơ được xác định bằng địa chỉ MAC của chính nó.
# Lấy MAC của 1 board: trong REPL gõ
#     import delta_proto; delta_proto.my_mac()
# rồi điền vào bảng dưới. Nạp CÙNG file này lên cả 4 board.
MASTER_MAC = 'F8:B3:B7:50:DD:D0'   # None = node nhận lệnh từ master bất kỳ
NODE_MACS = {
    'F8:5B:1B:4C:81:C0': 1,    # motor 1
    'F8:5B:1B:4B:83:6C': 2,    # motor 2
    'F8:5B:1B:49:DA:70': 3,    # motor 3
}

HDR_FMT = '<BBBBH'
HDR_LEN = struct.calcsize(HDR_FMT)

# ---- Lệnh: master -> node ----
CMD_PING    = 0x01   # nhịp tim; node trả lời bằng STATUS (không ACK)
CMD_HOME    = 0x02
CMD_MOVE    = 0x03   # payload MOVE_FMT
CMD_STOP    = 0x04   # dừng ngay, vẫn giữ lực
CMD_RELEASE = 0x05   # nhả động cơ (mất home)
CMD_ENABLE  = 0x06
CMD_CLEAR   = 0x07   # xóa trạng thái lỗi
CMD_SPEED   = 0x08   # payload SPEED_FMT (cũ, không lưu - dùng CMD_SETCFG)
CMD_SETCFG  = 0x09   # payload SETCFG_FMT: đặt 1 thông số, node lưu vào flash rồi gửi lại MSG_CFG
CMD_GETCFG  = 0x0A   # node gửi lại MSG_CFG
CMD_IDENT   = 0x0B   # đèn NeoPixel của node nháy trắng vài giây để nhận diện board

# ---- Bản tin: node -> master ----
MSG_ACK    = 0x81    # payload ACK_FMT
MSG_STATUS = 0x82    # payload STATUS_FMT
MSG_HELLO  = 0x83    # payload HELLO_FMT, gửi broadcast khi chưa có liên kết
MSG_CFG    = 0x84    # payload CFG_FMT: toàn bộ thông số hiện tại của node
MSG_POWER  = 0x85    # payload POWER_FMT: điện áp nguồn VBUS (12 V motor) của node

# Thời gian chạy (ms, 0 = chạy tốc độ tối đa) + góc đích của từng node (mili-độ ở trục ra).
# Node chỉ đọc ô của chính mình.
MOVE_FMT   = '<H' + 'i' * NUM_NODES
SPEED_FMT  = '<HH'          # max_hz, accel (bước/s, bước/s^2)
ACK_FMT    = '<HBB'         # seq của lệnh, type của lệnh, mã kết quả
STATUS_FMT = '<BBBii'       # state, err, flags, pos_mdeg, target_mdeg
HELLO_FMT  = '<BBH'         # reset_cause, fw_version, boot_id (ngẫu nhiên mỗi lần khởi động)
# vbus_mv (lúc gửi), vmin_mv (thấp nhất từ lần gửi trước), prev_min_mv / prev_last_mv (thấp nhất /
# lần đo cuối TRƯỚC lần reset vừa rồi, 0 = không biết, vd sau POWER_ON)
POWER_FMT  = '<HHHH'

# ---- Thông số node chỉnh được từ master (lưu trong config.json trên node) ----
#   min      : giới hạn dưới hành trình (độ ở trục ra, tính từ công tắc home)
#   max      : giới hạn trên hành trình (độ ở trục ra). 1 vòng motor = 360/14 = 25.71 độ
#   homedir  : giá trị chân DIR khi quay VỀ PHÍA công tắc home (0 / 1)
#   maxhz    : tốc độ tối đa (bước/s)
#   accel    : gia tốc (bước/s^2)
#   homefast : tốc độ dò home nhanh (bước/s)
#   homeslow : tốc độ dò home chậm (bước/s)
#   led      : 1 = bật đèn NeoPixel báo trạng thái, 0 = tắt hẳn (không ghi gì ra chân đèn)
CFG_KEYS   = ('min', 'max', 'homedir', 'maxhz', 'accel', 'homefast', 'homeslow', 'led')
CFG_FMT    = '<' + 'f' * len(CFG_KEYS)
SETCFG_FMT = '<Bf'          # chỉ số trong CFG_KEYS, giá trị

# ---- Trạng thái node ----
ST_DISABLED = 0
ST_IDLE     = 1
ST_HOMING   = 2
ST_MOVING   = 3
ST_ERROR    = 4
STATE_NAMES = ('DISABLED', 'IDLE', 'HOMING', 'MOVING', 'ERROR')

# ---- Cờ trong STATUS ----
F_HOMED   = 0x01
F_HOME_SW = 0x02
F_ENABLED = 0x04

# ---- Mã lỗi (lưu lại ở node đến khi CLEAR) ----
E_NONE           = 0
E_HOME_NOT_FOUND = 1   # chạy hết hành trình mà không chạm cảm biến
E_HOME_STUCK     = 2   # cảm biến không nhả khi lùi ra
E_LOST_STEP      = 3   # chạm cảm biến khi không mong đợi -> mất bước
E_LINK_LOST      = 4   # mất liên lạc master khi đang chạy
E_MOTION_TIMEOUT = 5   # chuyển động lâu bất thường
E_INTERNAL       = 6   # lỗi chương trình trên node (chi tiết trong crash.txt của node)
E_DRIVER         = 7   # (dành cho driver cấu hình qua UART, hiện không dùng)
ERR_NAMES = ('NONE', 'HOME_NOT_FOUND', 'HOME_STUCK', 'LOST_STEP',
             'LINK_LOST', 'MOTION_TIMEOUT', 'INTERNAL', 'DRIVER')

# ---- Mã kết quả trong ACK ----
R_OK        = 0
R_CLAMPED   = 1   # đã thực hiện nhưng đích bị giới hạn lại trong hành trình
R_BUSY      = 2
R_NOT_HOMED = 3
R_IN_ERROR  = 4
R_DISABLED  = 5
R_BAD_CMD   = 6
R_BAD_VALUE = 7   # thông số ngoài khoảng cho phép
RES_NAMES = ('OK', 'CLAMPED', 'BUSY', 'NOT_HOMED', 'IN_ERROR', 'DISABLED', 'BAD_CMD', 'BAD_VALUE')

CMD_NAMES = {
    CMD_PING: 'PING', CMD_HOME: 'HOME', CMD_MOVE: 'MOVE', CMD_STOP: 'STOP',
    CMD_RELEASE: 'RELEASE', CMD_ENABLE: 'ENABLE', CMD_CLEAR: 'CLEAR',
    CMD_SPEED: 'SPEED', CMD_SETCFG: 'SETCFG', CMD_GETCFG: 'GETCFG', CMD_IDENT: 'IDENT',
}

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
    return struct.pack(HDR_FMT, MAGIC, VERSION, typ, node, seq & 0xFFFF) + payload


def unpack(msg):
    """Trả về (type, node, seq, payload) hoặc None nếu gói không hợp lệ."""
    if msg is None or len(msg) < HDR_LEN:
        return None
    magic, ver, typ, node, seq = struct.unpack_from(HDR_FMT, msg)
    if magic != MAGIC or ver != VERSION:
        return None
    return typ, node, seq, bytes(msg[HDR_LEN:])


def init_espnow():
    """Bật Wi-Fi STA (không kết nối AP) + ESP-NOW trên kênh CHANNEL."""
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
        sta.config(pm=sta.PM_NONE)   # tắt tiết kiệm điện để nhận tin nhanh
    except (AttributeError, ValueError):
        pass
    if TX_POWER_DBM is not None:
        try:
            sta.config(txpower=TX_POWER_DBM)   # giảm dòng đỉnh khi phát
        except (AttributeError, ValueError, OSError) as ex:
            print("Không đặt được txpower:", ex)

    e = espnow.ESPNow()
    try:
        e.config(rxbuf=2048)         # mặc định 526 byte -> dễ tràn khi 3 node cùng gửi
    except (ValueError, OSError):
        pass
    e.active(True)
    try:
        e.add_peer(BROADCAST)
    except OSError:
        pass
    return e, sta.config('mac')


def mac_str(mac):
    return ':'.join('{:02X}'.format(b) for b in mac) if mac else '-'


def node_id_of(mac):
    """ID của node có địa chỉ MAC này (bytes), hoặc None nếu không có trong bảng."""
    s = mac_str(mac)
    for k, nid in NODE_MACS.items():
        if k.upper() == s:
            return nid
    return None


def is_master(mac):
    return MASTER_MAC is None or mac_str(mac) == MASTER_MAC.upper()


def my_mac():
    """In và trả về MAC của board đang chạy (dạng 'AA:BB:CC:DD:EE:FF')."""
    import network
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    mac = sta.config('mac')
    nid = node_id_of(mac)
    if nid:
        role = "node {}".format(nid)
    elif MASTER_MAC and is_master(mac):
        role = "master"
    else:
        role = "(chưa có trong NODE_MACS)"
    print("MAC:", mac_str(mac), "->", role)
    return mac_str(mac)


# ==== TẮT / BẬT WATCHDOG LÚC KHỞI ĐỘNG ====
# machine.WDT một khi đã bật thì KHÔNG tắt được -> chỉ có thể quyết định lúc boot có bật hay không.
# WDT bị bỏ qua (dù ENABLE_WDT = True) nếu:
#   - trên board có file 'nowdt'      (tạo bằng: import delta_proto as P; P.set_wdt(False))
#   - hoặc đang GIỮ nút BOOT lúc cắm điện / nhấn RST (ESP32-C3: IO9, ESP32: IO0)
NOWDT_FILE = 'nowdt'


def _boot_button_pin():
    return 9 if 'C3' in os.uname().machine else 0


def wdt_allowed():
    """True nếu được phép bật watchdog ở lần khởi động này."""
    try:
        os.stat(NOWDT_FILE)
        print("WDT: TẮT (có file '{}'). Bật lại: import delta_proto as P; P.set_wdt(True)".format(
            NOWDT_FILE))
        return False
    except OSError:
        pass
    if machine.Pin(_boot_button_pin(), machine.Pin.IN, machine.Pin.PULL_UP).value() == 0:
        print("WDT: TẮT cho lần chạy này (đang giữ nút BOOT)")
        return False
    return True


def set_wdt(on):
    """Bật/tắt watchdog cho các lần khởi động sau (lưu trên flash). Có hiệu lực sau khi reset."""
    if on:
        try:
            os.remove(NOWDT_FILE)
        except OSError:
            pass
    else:
        with open(NOWDT_FILE, 'w') as f:
            f.write('1')
    print("WDT sẽ {} từ lần khởi động sau (nhấn RST hoặc machine.reset())".format(
        "BẬT" if on else "TẮT"))
