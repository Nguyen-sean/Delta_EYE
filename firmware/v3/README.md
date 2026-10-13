# Delta_EYE v3: hai mắt, 6 node, một master

Firmware thế hệ 3 cho robot mắt cầu song song 3-RRR. Một board **master** (ESP32, cắm USB vào máy tính) điều khiển **2 mắt × 3 node** (ESP32-C3 + A4988 + Nema17 hộp số 14:1 + encoder AS5600) qua ESP-NOW.

> Trạng thái: logic đã kiểm tra bằng bộ giả lập trên máy tính (`sim_v3.py`, 23/23 kịch bản qua). **Chưa chạy trên phần cứng thật.**
> Các bản cũ vẫn giữ nguyên: bản 1 ở `firmware/`, bản 2 (1 mắt, encoder) ở `firmware/v2/`.

---

## 1. Tổng quan

```
            USB (lệnh chữ)
 Máy tính ─────────────────► MASTER (ESP32)
 (Thonny / eye_movie.py)          │  broadcast ESP-NOW (1 gói → cả 6 node)
                                  ▼
         ┌──────── MẮT 1 ────────┐   ┌──────── MẮT 2 ────────┐
         │ node 1  node 2  node 3│   │ node 4  node 5  node 6│
         └───────────────────────┘   └───────────────────────┘
                  ▲  unicast, chỉ khi cần (HELLO / EVENT / trả lời)
```

Các điểm chính:

| Tính năng | Mô tả |
|---|---|
| **Master nói, node im lặng** | Master chỉ phát broadcast. Node chỉ gửi khi khởi động, khi có sự cố, hoặc khi được hỏi. |
| **2 mắt cùng xuất phát** | Một gói MOVE chứa đích của cả 6 node. |
| **Chọn mắt để test** | `eyes 1`, `eyes 2`, `eyes 12`. Mắt đang tắt không cần cắm điện. |
| **Calib riêng từng mắt** | `calib 1`, `calib 2`, lưu trong `neutral3.txt`. |
| **Quy tụ (tùy chọn)** | `verg <độ>` hoặc `dist <mm>`: hai mắt cùng nhìn vào một điểm gần. |
| **Vòng kín encoder** | Mất bước hoặc bị đẩy lệch thì node tự bù. Kẹt cứng thì báo `STALL`. |
| **Chỉ home lần đầu** | Node bị reset vẫn khôi phục được vị trí từ RTC memory và encoder, không cần home lại. |
| **Lỗi theo từng mắt** | Node mất vị trí thì chỉ home lại **mắt của node đó**. |

---

## 2. Các file

| File | Nạp lên | Tên trên board | Vai trò |
|---|---|---|---|
| `proto3.py` | **cả 7 board** | `proto3.py` | Giao thức, bảng MAC, chia mắt |
| `node3.py` | 6 node | **`main.py`** | Phát xung, homing, encoder, liên lạc |
| `master3.py` | master | **`main.py`** | Nhận lệnh từ PC, IK 2 mắt, điều phối |
| `../spm_ik.py` | master | `spm_ik.py` | Động học ngược cơ cấu 3-RRR (dùng chung với bản 1, 2) |
| `sim_v3.py` | không nạp | | Giả lập toàn hệ thống trên máy tính |

File do firmware tự tạo trên board:

| File | Board | Nội dung |
|---|---|---|
| `config3.json` | node | Hành trình, tốc độ, chiều home, chiều encoder |
| `crash.txt` | node | Lỗi chương trình (nếu có) |
| `neutral3.txt` | master | Calib nhìn thẳng của từng mắt |
| `eyes.txt` | master | Mắt nào đang bật |

---

## 3. Phần cứng (sơ đồ `Stepper_motor_control_V3`)

| Chân ESP32-C3 | Nối tới | Ghi chú |
|---|---|---|
| IO0 | Cầu phân áp VBUS (R9 100K / R8 5K) | Đo điện áp nguồn 12 V |
| IO1 | WS2812B (qua R7 100 Ω) | Đèn trạng thái |
| IO2 | A4988 EN | LOW = bật motor |
| IO3 | Công tắc home | Nối GND khi nhấn, dùng pull-up nội |
| IO4 | A4988 MS3 | LOW → 1/8 bước (MS1, MS2 lên 3,3 V qua JP1, JP3) |
| IO5 | A4988 RESET | HIGH |
| IO6 | A4988 SLEEP | HIGH |
| IO7 | A4988 STEP | |
| IO8 | A4988 DIR | |
| IO20 / IO21 | AS5600 SCL / SDA | Trở kéo lên R12 / R13 |

Cơ khí: 200 bước × 1/8 × 14 = **62,22 vi bước / độ** ở trục ra.

Encoder AS5600 cho 4096 số mỗi vòng. Mặc định coi **nam châm gắn ở đuôi trục motor**, tức 2,56 số / vi bước. Nếu nam châm gắn ở trục ra sau hộp số, đặt `ENC_ON_MOTOR = False` trong `node3.py`.

**Nguồn:** 12 V vào cầu đấu J5. Khi chỉ cắm USB, VBUS chỉ còn khoảng 4,3 V qua D3, motor không chạy đúng. Jumper **JP5 phải để hở** khi có 12 V.

---

## 4. Cài đặt lần đầu

### 4.1 Lấy địa chỉ MAC

Nạp `proto3.py` lên từng board, rồi trong REPL (Thonny) gõ:

```python
import proto3; proto3.my_mac()
```

### 4.2 Điền bảng MAC trong `proto3.py`

```python
MASTER_MAC = 'F8:B3:B7:50:DD:D0'
NODE_MACS = {
    'F8:5B:1B:4C:81:C0': 1,    # mắt 1 - motor 1
    'F8:5B:1B:4B:83:6C': 2,    # mắt 1 - motor 2
    'F8:5B:1B:49:DA:70': 3,    # mắt 1 - motor 3
    'xx:xx:xx:xx:xx:xx': 4,    # mắt 2 - motor 1   <- ĐIỀN MAC THẬT
    'xx:xx:xx:xx:xx:xx': 5,    # mắt 2 - motor 2
    'xx:xx:xx:xx:xx:xx': 6,    # mắt 2 - motor 3
}
```

Node của mắt 2 phải theo **cùng thứ tự chân IK** với mắt 1: node 4 ↔ chân 1, node 5 ↔ chân 2, node 6 ↔ chân 3.

Nạp **cùng một file `proto3.py`** lên cả 7 board.

### 4.3 Nạp firmware

| Board | File cần có |
|---|---|
| 6 node | `proto3.py`, `node3.py` → đổi tên thành `main.py` |
| master | `proto3.py`, `spm_ik.py`, `master3.py` → đổi tên thành `main.py` |

### 4.4 Chỉnh hình học 2 mắt trong `master3.py`

```python
EYE_CFG = {
    1: {'side': 'L', 'az_offset': 0.0},   # 'L' = mắt trái (nhìn từ phía robot)
    2: {'side': 'R', 'az_offset': 0.0},   # az_offset: mắt lắp xoay khác mắt kia bao nhiêu độ
}
RIGHT_AZ = 0.0     # az ứng với hướng "sang phải của robot"
IPD_MM   = 65.0    # khoảng cách 2 tâm mắt (mm), dùng cho lệnh 'dist'
MOTOR_SIGN = (1, 1, 1, 1, 1, 1)
```

**Kiểm tra `RIGHT_AZ`:** gõ `look <RIGHT_AZ> 10`. Cả hai mắt phải cùng nhìn sang phải robot. Nếu đặt sai, lệnh quy tụ sẽ làm hai mắt quay **ra ngoài** thay vì vào trong.

### 4.5 Lần cấp điện đầu tiên

1. Cấp 12 V cho các node, cắm USB master vào máy tính, mở Thonny.
2. Master tự hỏi trạng thái node. Mắt nào chưa biết vị trí sẽ **tự home**:
   ```
   EVT mắt 1+2 chưa biết vị trí -> HOME TẤT CẢ node của mắt đó (chỉ cần lần đầu cấp điện)
   ...
   OK đã home lại xong tất cả
   ```
3. Khi home, mỗi node **tự dò chiều encoder** và lưu vào `config3.json`. Log node in ra:
   ```
   AS5600 OK, góc thô = 1234
   Encoder: chiều = 1 (2.561 số/bước) -> lưu cấu hình
   Home OK (vòng kín encoder)
   ```
4. Nếu trục nào chạy **ra xa** công tắc khi home: gõ `stop`, rồi `set <n> homedir 1` cho trục đó, rồi `home`.

### 4.6 Calib từng mắt

```
eyes 1
goto 1 12.8      (chỉnh từng node đến khi mắt 1 nhìn thẳng)
goto 2 12.9
goto 3 12.7
calib 1
eyes 2
goto 4 ...       (tương tự cho mắt 2)
calib 2
eyes 12
```

Kiểm tra: `range` in góc nghiêng tối đa theo từng hướng, tính sao cho các mắt đang bật đều tới được.

---

## 5. Lệnh trên master

| Lệnh | Ý nghĩa |
|---|---|
| `eyes 1` / `eyes 2` / `eyes 12` | Chọn mắt hoạt động. Lựa chọn được lưu, khởi động lại vẫn giữ. |
| `look az tilt [twist] [ms]` | Các mắt đang bật nhìn theo tọa độ cực (độ), trong `ms` mili-giây |
| `ik az tilt [twist]` | Chỉ tính góc motor, không chạy |
| `verg <độ>` | Góc quy tụ tổng. Mỗi mắt quay vào trong `độ/2`. `verg 0` = song song |
| `dist <mm>` | Nhìn vật cách `<mm>`, tự tính góc quy tụ theo `IPD_MM`. `dist 0` = nhìn xa vô cực |
| `calib [1\|2]` | Lưu vị trí hiện tại làm vị trí nhìn thẳng (mặc định: các mắt đang bật) |
| `range [twist]` | Góc nghiêng tối đa theo hướng |
| `move d1 d2 d3 [ms]` | 3 trục của **mỗi** mắt đang bật tới góc d1, d2, d3 |
| `goto n deg [ms]` | Chỉ node `n` (1..6) tới góc `deg` |
| `home [n\|e1\|e2]` | Home (mặc định: các mắt đang bật) |
| `stop` | Dừng khẩn cấp **cả 6 node** |
| `release` / `enable [n\|e1\|e2]` | Nhả / bật motor. Nhả xong phải home lại. |
| `clear [n\|e1\|e2]` | Xóa lỗi và FAULT |
| `speed hz [accel]` | Tốc độ tối đa / gia tốc của các mắt đang bật (có lưu) |
| `status` | Bảng trạng thái node của các mắt đang bật |
| `blink [n\|e1\|e2]` | Đèn node nháy trắng 4 giây, để nhận diện board |
| `cfg` | Đọc cấu hình cả 6 node |
| `set <n\|e1\|e2\|all> <tên> <giá trị>` | Đổi cấu hình node (node tự lưu) |

Cách chọn node trong các lệnh: số `1`..`6` = một node, `e1` / `e2` = cả mắt, `all` = cả 6 node. Bỏ trống = các mắt đang bật.

Tên thông số cho `set`:

| Tên | Mặc định | Khoảng | Ý nghĩa |
|---|---|---|---|
| `min` / `max` | 0.5 / 25.71 | 0.2–20 / 2–90 | Hành trình (độ trục ra, 0 = công tắc home) |
| `homedir` | 0 | 0 / 1 | Mức chân DIR khi quay về phía công tắc |
| `maxhz` | 2000 | 300–8000 | Tốc độ tối đa (vi bước/s), ~32 độ/s |
| `accel` | 6000 | 500–100000 | Gia tốc (vi bước/s²) |
| `homefast` / `homeslow` | 800 / 150 | | Tốc độ dò home |
| `led` | 1 | 0 / 1 | Bật / tắt đèn |
| `enc` | 1 | 0 / 1 | 1 = vòng kín encoder, 0 = chạy hở như bản 1 |
| `encdir` | 0 | -1 / 0 / 1 | Chiều encoder, 0 = tự dò ở lần home tới |

### Đọc `status`

```
STAT fault=False eyes=1+2 verg=0.00
STAT n1 ON  IDLE     err=NONE           homed=1 sw=0 pos=  12.857 target=  12.857
...
ENC n1 encoder=OK lệch=1 vi bước  đã bù=0 lần  khôi phục sau reset=0
PWR n1 VBUS=12.05 V  thấp nhất=11.98 V  thấp nhất từ đầu=11.90 V
```

Chỉ in node của **mắt đang bật**. Chương trình PC đếm số dòng `STAT n…` để biết robot đã sẵn sàng.

---

## 6. Chương trình PC

`pc/eye_movie.py` dùng được với master3 mà không cần sửa, vì cú pháp `look`, `status`, `home`, `clear` và các dòng `STAT`, `FAULT`, `ERR` giữ nguyên.

```bash
caffeinate -i .venv/bin/python pc/eye_movie.py --port /dev/cu.usbserial-2110 --forever
```

Muốn test chỉ một mắt: gõ `eyes 1` trên master (qua Thonny) trước, rồi đóng Thonny và chạy chương trình PC. Lựa chọn mắt đã được lưu trên master.

---

## 7. Giao thức (proto3)

Mỗi gói có header 6 byte: `magic(0xD7) | version(3) | type | node | seq(2)`, sau đó là payload.

- **Master → node:** trường `node` là bitmask (bit0 = node 1 … bit5 = node 6).
- **Node → master:** trường `node` là ID của node gửi.

### Master → node (broadcast, không ACK)

| Gói | Phát | Nội dung |
|---|---|---|
| `SYNC` | 4 lần/giây | `boot_id` và `evt_ack` của 6 node, cờ FAULT. Là nhịp tim, đồng thời xác nhận HELLO và EVENT |
| `MOVE` | tối đa 15 lần/giây | `dur_ms` + 6 đích (mili-độ). Phát lại sau 30 ms; **lệnh cuối** phát lại thêm sau 300 ms và 1 s |
| `HOME`, `STOP`, `CLEAR`, `RELEASE`, `ENABLE`, `IDENT`, `SETCFG`, `GETCFG`, `QUERY` | lặp 3 lần (STOP lặp 5 lần) | Node bỏ bản trùng theo `seq` |

### Node → master (unicast tới `MASTER_MAC`, có ACK phần cứng)

| Gói | Khi nào |
|---|---|
| `HELLO` | Vừa khởi động. Gửi lại mỗi 1 s đến khi `SYNC` chứa `boot_id` của node |
| `EVENT` | Home xong, lỗi, mất home, encoder hỏng, VBUS sụt. Gửi lại mỗi 0,3 s đến khi `SYNC` xác nhận |
| `STATUS` | Trả lời `QUERY` (lệnh `status`, hoặc kiểm tra định kỳ) |
| `CFG` | Trả lời `GETCFG` / `SETCFG` |

Node **không** báo khi bắt đầu hay kết thúc một chuyển động bình thường.

**Kiểm tra node còn sống:** cứ mỗi `ALIVE_CHECK_MS` (5 s), master hỏi các node của mắt đang bật. Node không trả lời được hỏi lại riêng tối đa 3 lần. Hai lượt kiểm tra liên tiếp không trả lời thì node bị coi là OFFLINE và gây FAULT. Đặt `ALIVE_CHECK_MS = 0` để tắt kiểm tra này: node hoàn toàn im lặng, nhưng master sẽ không phát hiện được node mất điện hẳn.

---

## 8. Vòng kín encoder

| Tình huống | Node xử lý |
|---|---|
| Đang chạy mà lệch > 3 bước đủ (24 vi bước) | Mất bước: sửa bộ đếm theo encoder, chạy tiếp; khi dừng thì bù phần thiếu |
| Dừng rồi mà lệch đích > 4 vi bước (~0,06°) | Tự chạy bù, tối đa 3 lần cho một đích (tránh rung qua lại) |
| Phải sửa > 6 lần trong 3 s | Báo `STALL` → master FAULT, dừng tất cả |
| Encoder đọc lỗi 5 lần liên tiếp / không có nam châm | Chuyển sang chạy hở (như bản 1), báo master |
| Lúc home: tỉ lệ số encoder / bước lệch > 25% so với lý thuyết | Báo sai, chuyển sang chạy hở (kiểm tra nam châm, `ENC_ON_MOTOR`) |

**Khôi phục sau reset:** mỗi 50 ms, node ghi bộ đếm encoder, điểm 0 và góc thô vào **RTC memory**. Bộ nhớ này còn giữ qua HARD_RESET, WATCHDOG và soft reset, nhưng **mất khi rút điện**. Khi khởi động lại:

- Trục trôi < 1/4 vòng encoder trong lúc reset: lấy lại vị trí, **không home**, master gửi lại đích gần nhất.
- Trục trôi nhiều hơn, hoặc vừa mất điện hẳn: master **home lại mắt của node đó**.

---

## 9. Xử lý sự cố

| Hiện tượng | Nguyên nhân / cách làm |
|---|---|
| `WARN bỏ qua MAC …` | MAC chưa có trong `NODE_MACS`, hoặc `proto3.py` trên các board không giống nhau |
| Node in `LỖI: MAC … chưa có trong NODE_MACS` | Điền MAC vào `proto3.py`, nạp lại cho **cả 7 board** |
| `AS5600: KHÔNG thấy nam châm` | Nam châm thiếu, lệch tâm hoặc quá xa chip |
| `Encoder: … số/bước, lý thuyết … -> sai` | Sai `ENC_ON_MOTOR`, hoặc nam châm trượt trên trục |
| `ERR nX STALL` | Trục kẹt hoặc quá tải. Kiểm tra cơ khí, rồi `clear` |
| `ERR look: mắt 2 n5 cần … ngoài hành trình` | Hướng nhìn vượt hành trình của mắt 2. Giảm `tilt`/`verg`, hoặc kiểm tra calib mắt 2 |
| `PWR nX trước lần reset vừa rồi: VBUS thấp nhất …` thấp | Board reset do sụt nguồn. Thêm tụ 470–1000 µF / 35 V ở đầu nguồn, 220 µF ở ngõ ra 3,3 V |
| Mắt quay **ra ngoài** khi `verg` | Sai `RIGHT_AZ` hoặc sai `side` trong `EYE_CFG` |
| Một mắt nhìn lệch so với mắt kia | Calib lại mắt đó; kiểm tra `az_offset` |
| Motor kêu / rung hơn bản 1 | Tăng `ENC_PERIOD_MOVE` (node3.py) từ 5 lên 10 ms |

---

## 10. Giả lập trên máy tính

```bash
python3 firmware/v3/sim_v3.py
```

Bộ giả lập chạy đúng `node3.py` và `master3.py`. Phần cứng được giả lập gồm: thời gian ảo, GPIO, motor + công tắc home, AS5600, RTC memory, ESP-NOW có rớt gói. Các kịch bản kiểm tra:

- Tự home cả 6 node; 2 mắt tới đúng hướng với calib khác nhau, kể cả khi **rớt 20% gói**.
- Node gần như im lặng (6 node gửi 6–10 gói trong 4,5 s chạy liên tục).
- Quy tụ `verg`, `dist`.
- Chỉ bật một mắt, mắt kia không cắm điện: không FAULT, không bị báo OFFLINE.
- Mắt tắt không nhận lệnh chạy; `eyes 12` chỉ home mắt vừa bật.
- Node mắt 2 HARD_RESET: khôi phục vị trí. Mất điện: chỉ home lại mắt 2, mắt 1 đứng yên.

Bản 2 (1 mắt) có bộ giả lập riêng: `python3 firmware/v2/sim_v2.py`.

---

## 11. Giới hạn đã biết

- Chưa chạy thử trên phần cứng thật.
- Mỗi lần đọc AS5600 qua I2C mất khoảng 0,2 ms. Khi motor chạy, node đọc mỗi 5 ms, nên xung STEP có thể lệch nhịp nhẹ.
- IO20/IO21 cũng là chân UART0 của ESP32-C3. Lúc khởi động, chip in vài ký tự lên các chân này, thường không ảnh hưởng tới AS5600.
- Quy tụ được tính gần đúng trong mặt phẳng góc. Sai số nhỏ khi tổng `tilt` + `verg` dưới khoảng 15°.
- Master là điểm hỏng duy nhất: master dừng thì cả 2 mắt dừng (node tự dừng sau 1,5 s không nghe thấy master).
- Mắt tắt bằng `eyes` vẫn nhận `SYNC`, `STOP` và `cfg`. Nó chỉ không nhận lệnh chạy, `home` hay kiểm tra định kỳ.
