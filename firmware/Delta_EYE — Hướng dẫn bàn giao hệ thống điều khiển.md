# Delta\_EYE — Hướng dẫn bàn giao hệ thống điều khiển

Oct 7, 2026 · @Sơn

## 1. Tổng quan

Delta\_EYE là một con mắt robot dạng **cơ cấu cầu song song 3-RRR**: 3 động cơ bước (qua hộp số 14:1) cùng xoay một khâu cuối (mắt) quanh một tâm. Máy tính ra lệnh hướng nhìn theo **tọa độ cực** (hướng, độ nghiêng, xoay); board master tính động học ngược (IK) rồi gửi góc cho 3 board động cơ qua **ESP-NOW** (sóng Wi-Fi không cần router).

**Trạng thái tại 07/10/2026:** toàn bộ phần mềm đã viết và chạy được (homing, IK, đồng bộ 3 trục, đổi đích liên tục, tự xử lý lỗi, chương trình PC giả lập mắt xem phim có log). **Vấn đề còn mở:** các board ESP32-C3 thỉnh thoảng tự khởi động lại / báo lỗi bộ nhớ khi chạy lâu — đang khoanh vùng giữa đèn NeoPixel, công suất phát Wi-Fi và nguồn 3.3V (mục 8).

Các thành phần, từ trên xuống:

| Tầng | Thiết bị | Phần mềm | Nhiệm vụ |
| --- | --- | --- | --- |
| Máy tính | Mac / PC, cáp USB tới master | `pc/eye_movie.py` (Python 3 + pyserial) | Sinh hướng nhìn (giả lập hoặc từ phim), gửi lệnh `look` ≤ 15 lần/s, đọc phản hồi, ghi log |
| Master | ESP32 (bản gốc), cắm USB vào máy tính | `firmware/master.py` (lưu thành `main.py`) + `delta_proto.py` + `spm_ik.py` | Nhận lệnh chữ qua USB, tính IK, gửi lệnh ESP-NOW, giám sát 3 node, dừng khẩn cấp, tự home lại |
| Node 1, 2, 3 | ESP32-C3 + driver A4988 + Nema17 hộp số 14:1 + công tắc home + NeoPixel | `firmware/motor_node.py` (lưu thành `main.py`) + `delta_proto.py` | Phát xung bước có tăng/giảm tốc, homing, báo trạng thái, tự dừng khi mất liên lạc |

&#91;embedded content: kiến trúc Delta\_EYE · 1 máy tính, 1 master, 3 node\]

Lệnh đi từ trên xuống: máy tính gửi hướng nhìn qua USB, master tính góc 3 motor và phát một gói ESP-NOW cho cả 3 node; trạng thái và lỗi đi ngược lên.

Thư mục dự án: `OneDrive-Personal/MakerLab.vn/Delta_EYE/` gồm `firmware/` (code nạp lên board), `pc/` (code chạy trên máy tính, log trong `pc/logs/`) và file CAD gốc `delta.step` (ở thư mục Downloads của máy Mac).

Lưu ý OneDrive: thư mục nằm trong OneDrive nên file có thể bị đẩy lên mây (Thonny báo *Operation timed out*). Chuột phải thư mục `Delta_EYE` → **Always Keep on This Device**.

## 2. Phần cứng

Mỗi node là một board riêng gồm **ESP32-C3** (MicroPython v1.28.0), driver **A4988** (vi bước 1/8), động cơ **Nema17 có hộp số hành tinh 14:1**, một **công tắc home** và một đèn **NeoPixel**. ESP32-C3 được cấp 3.3V từ một bộ hạ áp xung 12V → 3.3V, bộ này cấp luôn cho đèn.

**Sơ đồ chân (giống nhau trên cả 3 node):**

| Chân ESP32-C3 | Nối tới | Ghi chú |
| --- | --- | --- |
| IO1 | NeoPixel (data) | WS2812, 1 đèn |
| IO2 | A4988 EN | mức thấp = cấp dòng giữ motor |
| IO3 | Công tắc home (chân còn lại nối GND) | pull-up nội, bị nhấn = 0 |
| IO4 | A4988 MS3 | = 0 → 1/8 bước (MS1 = MS2 = 1 cố định trên mạch) |
| IO5 | A4988 RESET | giữ mức cao |
| IO6 | A4988 SLEEP | giữ mức cao |
| IO7 | A4988 STEP | xung bước |
| IO8 | A4988 DIR | chiều quay |

**Cơ khí và đơn vị:** 1 vòng motor = 200 × 8 = **1600 bước**; trục ra hộp số = 1600 × 14 / 360 ≈ **62,22 bước/độ**. Mọi góc của node (“vị trí”, min/max) tính bằng **độ ở trục ra**, 0° = điểm chạm công tắc home. Hành trình hiện đặt trong cấu hình node (`max=30` theo log gần nhất; mặc định 25,71° = 1 vòng motor).

**Địa chỉ MAC (định danh board, khai báo trong `delta_proto.py`):**

| Board | MAC |
| --- | --- |
| Master (ESP32) | F8:B3:B7:50:DD:D0 |
| Node 1 | F8:5B:1B:4C:81:C0 |
| Node 2 | F8:5B:1B:4B:83:6C |
| Node 3 | F8:5B:1B:49:DA:70 |

Theo file CAD, node 1 là motor `Nema17_PLANETARY v4:1` / chân `Part1:1`, node 2 là `v4:2` / `Part1:10`, node 3 là `v4:3` / `Part1:11`. Nếu lắp ra thực tế khác thứ tự này thì IK sẽ sai — kiểm tra bằng lệnh `blink n` (đèn node n nháy 4 giây).

**Khuyến nghị nguồn (chưa làm, xem mục 8):** tụ 470–1000 µF + 10 µF + 100 nF sát chân 3V3 của mỗi ESP32-C3; tụ 100–470 µF + 100 nF sát VMOT mỗi A4988; tốt nhất hạ 12V → 5V rồi cấp vào chân 5V của board (LDO trên board tạo 3.3V) và cấp NeoPixel từ 5V; dây GND của bộ hạ áp về thẳng cực âm nguồn 12V.

## 3. Firmware và cách nạp

Cả 3 node chạy **cùng một file code**; mỗi board tự biết mình là node mấy nhờ tra địa chỉ MAC của chính nó trong bảng `NODE_MACS`. `delta_proto.py` chứa định dạng gói tin dùng chung, nên **phải giống hệt nhau trên cả 4 board** — lệch phiên bản là mất liên lạc hoặc không đọc được cấu hình.

| File (trong `firmware/`) | Nạp lên | Tên trên board | Nội dung |
| --- | --- | --- | --- |
| `delta_proto.py` | Cả 4 board | `delta_proto.py` | Giao thức, mã lỗi, bảng MAC, kênh và công suất phát Wi-Fi |
| `motor_node.py` | Node 1, 2, 3 | `main.py` | Phát xung, homing, đèn, xử lý lệnh, ghi `crash.txt` |
| `master.py` | Master | `main.py` | Giao tiếp USB, IK, giám sát, dừng khẩn cấp, tự home lại |
| `spm_ik.py` | Master | `spm_ik.py` | Động học ngược, hình học đo từ CAD |

File tự sinh trên board (không nạp tay): `config.json` trên mỗi node (cấu hình đặt từ master), `crash.txt` trên node (lỗi chương trình nếu có), `neutral.txt` trên master (vị trí nhìn thẳng, tạo bằng lệnh `calib`).

**Nạp bằng Thonny:**

1. Cắm board vào máy, mở Thonny, chọn interpreter *MicroPython (ESP32)* đúng cổng.
2. Bấm Stop; nếu board đang chạy không dừng được, bấm Ctrl+C liên tục ngay sau khi reset.
3. Mở file trên máy tính → *File → Save as… → MicroPython device* → gõ đúng tên ở bảng trên. **Không** mở `main.py` trực tiếp trên board khi board đang chạy (Thonny lỗi `TclError`).
4. Bấm Ctrl+D để khởi động lại.
5. Kiểm tra: trên master gõ `cfg` — 3 dòng `CFG n1/n2/n3` phải hiện, có đủ trường `led=`.

**Thêm board mới / thay board:** trên board đó gõ `import delta_proto; delta_proto.my_mac()` để lấy MAC, sửa bảng `NODE_MACS` (hoặc `MASTER_MAC`) trong `delta_proto.py`, rồi nạp lại `delta_proto.py` cho cả 4 board. Board có MAC lạ sẽ không chạy và đèn đỏ đứng yên.

**Watchdog:** hiện để `ENABLE_WDT = False` ở cả master lẫn node cho dễ nạp code. Khi chạy ổn định không người trông thì đổi thành `True` (board treo sẽ tự reset); lúc đó muốn nạp code phải Ctrl+C trong 3 giây đầu sau khi khởi động.

## 4. Giao thức ESP-NOW và cơ chế an toàn

Master gửi mọi lệnh bằng **broadcast** kèm một mặt nạ bit chọn node; mỗi lệnh được node trả **ACK** theo số thứ tự, thiếu ACK thì master gửi lại (tối đa 5 lần, mỗi lần chờ 200 ms). Lệnh `MOVE` chứa đích của cả 3 trục trong **một gói**, nên 3 trục xuất phát cùng lúc; mỗi node tự tính tốc độ để tới đích đúng thời gian `ms` (sai lệch đo được trong mô phỏng < 5 ms).

**Gói tin** = header 6 byte (magic, version, loại, node/mặt nạ, số thứ tự) + dữ liệu. Loại gói:

| Hướng | Loại | Ý nghĩa |
| --- | --- | --- |
| Master → node | PING | Nhịp tim mỗi 200 ms; node trả STATUS |
| Master → node | HOME, MOVE, STOP, RELEASE, ENABLE, CLEAR | Lệnh điều khiển (có ACK) |
| Master → node | SETCFG, GETCFG, IDENT | Đặt / đọc cấu hình, nháy đèn nhận diện |
| Node → master | ACK | Kết quả: OK, CLAMPED, BUSY, NOT\_HOMED, IN\_ERROR, DISABLED, BAD\_CMD, BAD\_VALUE |
| Node → master | STATUS | Trạng thái, mã lỗi, đã home chưa, công tắc, vị trí, đích |
| Node → master | HELLO, CFG | Khi khởi động (kèm lý do reset, `boot_id`); cấu hình hiện tại |

**Trạng thái node:** DISABLED (đã nhả motor) · IDLE · HOMING · MOVING · ERROR. **Mã lỗi:** HOME\_NOT\_FOUND, HOME\_STUCK, LOST\_STEP, LINK\_LOST, MOTION\_TIMEOUT, INTERNAL.

**Các lớp an toàn:**

- **Một trục lỗi → dừng cả 3** (robot song song không được chạy thiếu trục). Master vào FAULT và từ chối lệnh di chuyển cho tới khi gõ `clear`.
- **Mất liên lạc:** node không nghe master > 1 s khi đang chạy → tự dừng, lỗi LINK\_LOST. Master không nghe node > 2 s → OFFLINE, dừng các trục còn lại.
- **Mất bước:** đang đi về phía home mà công tắc bị chạm sớm (chuyển từ nhả sang nhấn) → LOST\_STEP.
- **Tự home lại tất cả:** khi một trục mất home (mất bước, board khởi động lại — nhận ra nhờ `boot_id` đổi), master dừng hết, chờ 1 s, gửi ENABLE + CLEAR + HOME cho cả 3. Thất bại 2 lần thì dừng chờ người; `clear` đặt lại bộ đếm. Đổi `homedir` thì **không** tự home (chờ người gõ `home`).
- **Lỗi chương trình trên node:** bắt exception, dừng motor, ghi `crash.txt`, báo INTERNAL, vẫn chạy tiếp (không mất vị trí).
- **Hành trình:** master từ chối nguyên lệnh `look` nếu một trục vượt min/max của node đó (không cắt riêng một trục).

**Đổi đích khi đang chạy:** lệnh di chuyển mới thay lệnh cũ ngay. Cùng chiều → chạy tiếp từ tốc độ hiện tại; ngược chiều hoặc quá gần → giảm tốc mượt rồi quay lại. Master gửi tối đa **15 lệnh di chuyển/giây**, lệnh tới nhanh hơn thì chỉ giữ lệnh mới nhất (in `INFO bỏ N lệnh`). Yêu cầu thời gian ngắn hơn mức motor chạy kịp thì master tự kéo dài để 3 trục vẫn tới cùng lúc (`WARN … quá nhanh`).

## 5. Cơ khí, động học ngược (IK) và calib

Hướng nhìn được ra lệnh theo tọa độ cực: **az** = nghiêng về hướng nào (0–360°, 0° = trục X của CAD), **tilt** = nghiêng bao nhiêu độ, **twist** = xoay quanh trục nhìn. `spm_ik.py` đổi ra góc 3 motor; master cộng với vị trí nhìn thẳng đã calib: `góc node[i] = NEUTRAL[i] + MOTOR_SIGN[i] × (θ_i − θ_i lúc nhìn thẳng)`.

**Hình học đo từ `delta.step` (EYES\_DELTA v23):**

| Thông số | Giá trị | Ý nghĩa |
| --- | --- | --- |
| α1 | 65,6° | Cung khâu 1: trục motor → trục khớp giữa (Fusion báo 114,4° = góc bù) |
| α2 | 50,0° | Cung khâu 2: khớp giữa → ổ bi trên mắt |
| γ | 54,6° | Trục motor so với phương thẳng đứng; 3 trục motor vuông góc nhau từng đôi |
| β | 90° | Ổ bi trên mắt (`Center_ring`) nằm ngang |
| δ | 37,95° | Khớp trên mắt lệch phương vị so với motor cùng chân |

Trong code lưu dưới dạng vector hướng `U`, `V0` và `ALPHA1`, `ALPHA2`, `BRANCH` (đầu file `spm_ik.py`). IK tính ra khớp giữa trùng CAD sai < 0,001°.

**Sai số thiết kế cần biết:** các trục trong CAD **không cắt nhau tại một tâm**. 3 trục khớp trên mắt gặp nhau đúng tâm; 3 trục motor gặp nhau thấp hơn 8,5 mm (lệch 1,8 mm); trục khớp giữa lệch tâm \~13 mm và lệch trục motor 5,5 mm. IK dùng hướng trục nên vẫn gần đúng ở góc nhỏ, nhưng nghiêng xa cơ cấu có thể cấn. Sửa thiết kế cho mọi trục đi qua một tâm sẽ chạy êm và đúng hơn.

**Calib vị trí nhìn thẳng (làm 1 lần, làm lại khi đổi cơ khí hoặc hành trình):**

1. `home`
2. `move M M M 2000` với M = (min + max) / 2 (25,71° → 12,86; 30° → 15,25).
3. Chỉnh từng trục bằng `goto 1 …`, `goto 2 …`, `goto 3 …` tới khi mắt nhìn thẳng, không xoay.
4. `calib` → master lưu `neutral.txt`. Ba số nên gần giữa hành trình. Calib ngay sau `home` (≈ 0,5°) là sai — mọi hướng nghiêng đều báo số âm.
5. `look 0 0 5 1000` (chỉ xoay): 3 motor phải quay cùng chiều, cùng lượng; motor nào ngược thì đổi `MOTOR_SIGN` của nó thành -1 trong `master.py`.
6. `look 90 5 0 1000` → mắt phải nghiêng về phía +Y của CAD.

**Vùng làm việc** (gõ `range` để tính theo calib thật): với hành trình 25,71° và neutral ở giữa, nghiêng được **≥ 13,6° mọi hướng** (tối đa 17,5° ở các hướng 90°, 210°, 330°), xoay −16,3° … +10,8°. Nâng hành trình lên 40° thì ≥ 20,5° mọi hướng (mô phỏng). Phép tính **không xét va chạm** giữa các khâu — thử tăng dần.

## 6. Lệnh của master và cấu hình từ xa

Gõ vào cổng USB của master (ô Shell của Thonny, hoặc chương trình PC gửi), mỗi lệnh một dòng. Chữ trong `[ ]` là không bắt buộc; góc tính bằng độ, thời gian bằng mili-giây. Master in mỗi dòng phản hồi bắt đầu bằng `OK`, `ERR`, `WARN`, `EVT`, `FAULT`, `STAT`, `CFG`, `INFO`, `IK` để PC đọc tự động.

| Lệnh | Việc | Ví dụ |
| --- | --- | --- |
| `look az tilt [twist] [ms]` | Nhìn theo tọa độ cực; 3 trục cùng xuất phát, cùng tới đích | `look 90 10 0 800` |
| `ik az tilt [twist]` | Chỉ tính và in góc 3 motor, không chạy | `ik 45 12` |
| `range [twist]` | In độ nghiêng tối đa theo 12 hướng (theo calib hiện tại) | `range` |
| `calib` | Lưu vị trí hiện tại làm vị trí nhìn thẳng | `calib` |
| `move d1 d2 d3 [ms]` | 3 trục tới góc trực tiếp | `move 12.86 12.86 12.86 2000` |
| `goto n deg [ms]` | Một trục tới góc | `goto 2 13.5` |
| `home [n]` | Chạy về công tắc tìm điểm 0 | `home` |
| `stop` | Dừng khẩn cấp cả 3 (vẫn giữ lực) | `stop` |
| `release [n]` / `enable [n]` | Nhả motor (mất home) / cấp lực lại | `release 1` |
| `clear [n]` | Xóa lỗi, thoát FAULT, đặt lại bộ đếm tự home | `clear` |
| `status` | Bảng trạng thái 3 trục | `status` |
| `blink [n]` | Đèn node náy trắng 4 s để nhận diện board | `blink 2` |
| `cfg` | Đọc và in cấu hình của 3 node | `cfg` |
| `set <n\|all> <tên> <giá trị>` | Đổi cấu hình, node lưu vào `config.json` | `set all max 40` |
| `speed hz [accel]` | Tốc độ tối đa và gia tốc cho cả 3 (có lưu) | `speed 1500 4000` |

**Thông số cấu hình node** (`set` / `cfg`):

| Tên | Mặc định | Khoảng cho phép | Ý nghĩa |
| --- | --- | --- | --- |
| `min` | 0,5° | 0,2 – 20 | Giới hạn dưới, tính từ công tắc home |
| `max` | 25,71° | 2 – 90 | Giới hạn trên (25,71 = 1 vòng motor) |
| `homedir` | 0 | 0 / 1 | Chiều chân DIR khi quay về công tắc |
| `maxhz` | 2000 | 300 – 4000 | Tốc độ tối đa (bước/s) |
| `accel` | 6000 | 500 – 50000 | Gia tốc (bước/s²) |
| `homefast` / `homeslow` | 800 / 150 | 100–3000 / 20–1000 | Tốc độ dò home nhanh / chậm |
| `led` | 1 | 0 / 1 | Bật / tắt hẳn đèn NeoPixel |

Tăng `max` phải thử cơ khí từ từ (`goto` từng trục, tay sẵn `stop`) và calib lại ở giữa hành trình mới. Đổi `homedir` xong phải tự gõ `home`.

## 7. Chương trình máy tính `pc/eye_movie.py`

Chương trình cho mắt robot chuyển động như mắt người xem phim và là **khung mẫu cho mọi ứng dụng điều khiển từ máy tính**: mở cổng USB, kiểm tra / home robot, gửi `look` có giới hạn tốc độ, đọc phản hồi, tự xử lý lỗi, ghi log.

**Cài đặt (1 lần, trên Mac):**

```bash
cd ~/Library/CloudStorage/OneDrive-Personal/MakerLab.vn/Delta_EYE
/opt/homebrew/bin/python3 -m venv .venv
.venv/bin/pip install pyserial numpy opencv-python
```

`numpy` và `opencv-python` chỉ cần cho chế độ nhìn theo phim thật. Trước khi chạy phải **đóng Thonny** (nếu không cổng bị chiếm).

**Chạy:**

| Mục đích | Lệnh |
| --- | --- |
| Thử không cần robot | `.venv/bin/python pc/eye_movie.py --sim --view` |
| Liệt kê cổng USB | `.venv/bin/python pc/eye_movie.py --list-ports` |
| Chạy thật, tự tìm cổng | `.venv/bin/python pc/eye_movie.py --view` |
| Chạy 30 phút, không cho Mac ngủ | `caffeinate -i .venv/bin/python pc/eye_movie.py --port /dev/cu.usbserial-2110 --duration 1800` |
| Nhìn theo phim thật | `.venv/bin/python pc/eye_movie.py --video phim.mp4 --show` |

Tùy chọn khác: `--max-tilt` (mặc định 12°), `--fps`, `--az-offset` / `--flip-x` (khớp hướng màn hình với robot), `--seed`, `--auto-clear` (tự `clear` + `home` khi lỗi — chỉ dùng khi đã ổn định), `--echo` / `--verbose`, `--log-dir`, `--no-log`.

**Cách hoạt động:**

- **Mô hình mắt (mặc định):** nhìn chăm chú 0,12–1,5 s (TB \~0,3 s), giật nhỏ 0,2–0,5°, giật mắt TB \~5° thiên về giữa, bám vật 4–12°/s, chuyển cảnh mỗi 4–10 s. Mô hình chờ robot tới nơi rồi mới giật tiếp (\~65 lần giật/phút).
- **Gửi lệnh:** tối đa `SEND_FPS` = 10 lệnh/s; hướng đổi < `DEADBAND_DEG` = 0,3° thì không gửi; lệnh giật mắt không bao giờ bị bỏ; thời gian mỗi lệnh tự tính theo khả năng motor.
- **Xử lý lỗi:** FAULT → ngừng gửi; master tự home xong → kiểm tra `status` rồi chạy tiếp. Báo `chưa home` / `offline` → tạm dừng (có `--auto-clear` thì tự home, tối đa 3 lần). Báo vượt hành trình → thu hẹp vùng nhìn 1°. Chờ quá 60–120 s → thoát an toàn (mã 2).
- **Log:** mỗi lần chạy một file `pc/logs/eye_<ngày>_<giờ>.log`; `>>` lệnh gửi (kèm loại chuyển động, biên độ), `<<` master báo, `##` sự kiện. Khi có sự cố in khối **SỰ CỐ #n** với các lệnh và phản hồi 5 giây trước đó; kết thúc in bảng tổng kết theo node và loại lỗi.

**Test:** `cd pc && ../.venv/bin/python -m unittest -v test_eye_movie.py` — 41 test, không cần phần cứng (có master giả lập dùng IK thật trong `firmware/spm_ik.py`): tiện ích, thống kê mô hình mắt, giới hạn tốc độ, đọc phản hồi, xem phim 3 phút không lỗi, FAULT / node khởi động lại / tự home, log sự cố, nhìn theo phim tổng hợp.

## 8. Vấn đề đang mở: node tự khởi động lại và lỗi bộ nhớ

Chạy lâu, các node ESP32-C3 thỉnh thoảng **tự khởi động lại** (`HARD_RESET`, `WATCHDOG`, `POWER_ON`) hoặc báo **`INTERNAL`**; hệ thống tự dừng và home lại nên an toàn, nhưng không chạy liên tục được. Nguyên nhân chưa chốt; các bản sửa mới nhất **chưa được thử trên phần cứng**.

**Bằng chứng đã thu:**

- `crash.txt` cho thấy **bộ nhớ bị ghi đè**: hằng số `P.F_HOME_SW` đọc ra thành một hàm (`unsupported types for __or__: 'int', 'bound_method'`), hàm `int`/`round` đọc ra thành chuỗi rỗng (`'' object isn't callable`). Code Python không thể tự gây ra điều này → lỗi ở tầng firmware C, driver, hoặc điện áp.
- `HARD_RESET` xảy ra cả khi `ENABLE_WDT = False` và cả khi robot **đứng yên** → chip panic, không phải watchdog của chương trình.
- Quy luật trong log: **sau mỗi lệnh broadcast (vd STOP) là một node reset**. Mỗi lệnh nhận được, node cũ chớp đèn NeoPixel — cả 3 node ghi đèn cùng lúc đúng lúc xử lý Wi-Fi.
- **Mọi lần reset đều xuất hiện sau khi thêm đèn NeoPixel** vào firmware.
- ESP32-C3 chỉ có **1 nhân**: Wi-Fi và Python dùng chung; ghi NeoPixel (RMT hoặc bit-bang tắt ngắt) làm Wi-Fi trễ.
- Nguồn: ESP32-C3 và đèn lấy 3.3V trực tiếp từ bộ hạ áp xung 12V, dùng chung 12V với motor; ESP32-C3 phát Wi-Fi mặc định \~20 dBm kéo dòng đỉnh 250–500 mA. Flash cần ≥ 3,0V nhưng mạch brownout chỉ báo ở \~2,4–2,8V → sụt nhẹ gây chạy sai thay vì reset sạch.
- MicroPython trên node: **v1.28.0** (ESP32C3 module) — bản mới, khả năng thấp là lỗi firmware cũ.

**Đã sửa trong firmware (bản hiện tại):**

- Đèn ở **chế độ an toàn** `LED_ANIMATE = False`: màu đứng yên theo trạng thái, không chớp khi nhận lệnh, không ghi khi motor chạy → 0 lần ghi đèn trong 30 s bám mục tiêu (trước \~600). Ghi bằng bit-bang thay RMT (`LED_USE_RMT = False`). Thông số `led` để tắt hẳn từ master.
- **Công suất phát Wi-Fi 8,5 dBm** (`TX_POWER_DBM` trong `delta_proto.py`).
- Node chỉ gửi STATUS khi trạng thái đổi + khi trả lời ping: \~25 → \~16 gói/s mỗi node.
- Dọn bộ nhớ (GC) chủ động khi đứng yên.

**Việc tiếp theo, theo thứ tự:**

- [ ] Nạp bản hiện tại cho cả 4 board; trên mỗi node `import os; os.remove('crash.txt')`; `cfg` phải thấy `led=1`.
- [ ] Chạy `eye_movie.py --duration 600`, xem bảng tổng kết. Hết lỗi → chạy 30 phút rồi bật lại `ENABLE_WDT = True`.
- [ ] Còn lỗi → `set all led 0`, chạy lại. Hết → thủ phạm là đèn: thay bằng LED thường, hoặc cấp 5V riêng + tụ cho NeoPixel.
- [ ] Vẫn còn → nguồn: gắn tụ (mục 2), đổi bộ hạ áp giữa n1 và n2 xem lỗi có đi theo không, đo 3V3 khi motor chạy (phải ≥ 3,2V), cân nhắc cấp 5V vào chân 5V của board.
- [ ] Bắt thông báo panic: cắm USB node (cổng `/dev/cu.usbmodem…`), `.venv/bin/python -m serial.tools.miniterm <cổng> 115200`, chép đoạn `Guru Meditation … Backtrace` khi nó reset. Nếu cắm USB mà hết reset → bằng chứng nguồn 3.3V yếu.

## 9. Lỗi thường gặp

Khi có sự cố: nhìn đèn 3 node → gõ `status` → tìm dòng `FAULT` **đầu tiên** trong log (các lỗi sau thường là hệ quả) → xử lý → `clear` rồi `home`.

| Dấu hiệu | Nguyên nhân | Cách sửa |
| --- | --- | --- |
| `BOOT … reset=POWER_ON` khi motor chạy | Sụt áp nguồn ESP32 | Tụ, nguồn logic riêng, GND hình sao (mục 2, 8) |
| `reset=HARD_RESET` / `WATCHDOG`, `INTERNAL` với lỗi vô lý | Chip panic / hỏng bộ nhớ | Xem mục 8; đọc `crash.txt`: `print(open('crash.txt').read())` |
| `HOME_NOT_FOUND`, đèn đỏ | Sai `homedir`, công tắc hỏng, đứt dây IO3 | Trục quay ra xa công tắc → `set n homedir 1`; ấn tay công tắc phải thấy `sw=1` trong `status` |
| `HOME_STUCK` | Công tắc không nhả khi lùi 12,9°, hoặc nhiễu IO3 | Kiểm tra cần gạt; thêm pull-up 4,7–10 kΩ lên 3.3V + tụ 100 nF IO3–GND |
| `LOST_STEP` rồi `HOME LẠI TẤT CẢ` | Trượt bước: quá nhanh, kẹt, thiếu dòng, driver nóng | Giảm `maxhz` / `accel`, tăng Vref, tản nhiệt A4988 |
| `ERR look: nX cần … ngoài hành trình` | Hướng vượt min/max, hoặc calib sai | Giảm tilt, xem `range`; calib lại ở giữa hành trình |
| `look` báo góc âm dù nghiêng ít | Calib lúc trục ở vị trí home (\~0,5°) | Calib lại (mục 5) |
| `ERR move: nX chưa home` | Node vừa khởi động lại hoặc bị `release` | `clear` rồi `home` |
| `ERR nX OFFLINE`, đèn xanh dương | Node mất nguồn / đang khởi động lại / ngoài tầm | Kiểm tra nguồn, khoảng cách |
| `LINK_LOST` | Node đang chạy mất liên lạc master > 1 s | Kiểm tra master có bị reset, `clear` |
| `WARN bỏ qua MAC …` | MAC không có trong `NODE_MACS` | Thêm MAC, nạp lại `delta_proto.py` cho cả 4 board |
| Node không phản hồi, master vẫn chạy | `MASTER_MAC` sai hoặc `delta_proto.py` khác phiên bản | Nạp cùng một `delta_proto.py` cho cả 4 board |
| `AttributeError: … NUM_NODES` | Trên board có file khác trùng tên module | Lưu đúng tên `delta_proto.py`, xóa file trùng |
| Thonny `TimeoutError` khi mở file | OneDrive đẩy file lên mây | *Always Keep on This Device* cho `Delta_EYE` |
| Thonny `TclError` khi mở `main.py` | Board đang chạy / watchdog reset liên tục | Stop + Ctrl+C trong 3 s đầu; ghi đè từ máy tính bằng *Save as* |
| PC: `Không mở được cổng` / `No such file` | Master bị rút hoặc reset, Thonny chiếm cổng | Đóng Thonny, `--list-ports`, đổi cáp có dữ liệu |
| PC: `Chưa cài pyserial` | Sai bản Python | Dùng `.venv/bin/python` (mục 7) |
| `WARN … ms quá nhanh`, `INFO bỏ N lệnh` | Bình thường | Không cần làm gì |

## 10. Hướng làm tiếp: điều khiển từ máy tính

Giao diện giữa máy tính và robot đã đủ dùng: **gửi lệnh chữ `look az tilt twist ms` qua cổng USB của master (115200 baud), tối đa 15 lệnh/giây, đọc các dòng phản hồi**. Mọi ứng dụng mới (camera bám mặt, giao diện tay, kết nối phần mềm khác) chỉ cần thay phần *sinh hướng nhìn*; phần kết nối và xử lý lỗi dùng lại từ `eye_movie.py`.

**Lộ trình đề xuất** (mỗi bước có điều kiện qua cửa trước khi làm bước sau):

1. **Ổn định phần cứng** — làm xong danh sách ở mục 8. *Cửa:* `eye_movie.py --duration 1800` chạy 30 phút, bảng tổng kết 0 sự cố, rồi bật `ENABLE_WDT = True`.
2. **Tách thư viện điều khiển** — chuyển `MasterLink`, `CommandSender`, `TrafficLog`, `robot_min_ms`, `to_polar` từ `eye_movie.py` sang module `pc/delta_eye.py` với API gọn: `connect(port)`, `ensure_ready()`, `look(az, tilt, twist=0, ms=None)`, `look_xy(x_deg, y_deg)`, `stop()`, `status()`, callback `on_fault` / `on_ready`. Giữ master giả lập `FakeMaster` để test không cần robot. *Cửa:* 41 test hiện có vẫn đạt, `eye_movie.py` chuyển sang dùng thư viện.
3. **Điều khiển tay** — cửa sổ nhỏ (tkinter có sẵn, hoặc web): kéo chuột / joystick → `look_xy`, nút Home / Stop / Clear, hiện trạng thái 3 node và vùng nhìn cho phép (lấy từ lệnh `range`). *Cửa:* thao tác 10 phút không lỗi, Stop có tác dụng < 0,3 s.
4. **Bám mục tiêu bằng camera** — OpenCV (hoặc MediaPipe) tìm mặt / vật trong ảnh, đổi vị trí pixel sang độ theo góc nhìn của camera, lọc mượt, gửi 10–15 lệnh/s với `ms` ≈ 1–2 chu kỳ gửi (100–150 ms). Cần hiệu chỉnh: góc nhìn camera, vị trí camera so với mắt, chiều trục (`--az-offset`, `--flip-x`). `VideoGaze` trong `eye_movie.py` là điểm bắt đầu (đang theo vùng chuyển động). *Cửa:* mắt bám theo người đi ngang ở 2 m, trễ < 0,5 s.
5. **Kết nối phần mềm khác (nếu cần)** — một tiến trình duy nhất giữ cổng USB và mở UDP / WebSocket / OSC cho Unity, TouchDesigner, web… gửi hướng nhìn. Chỉ một tiến trình được mở cổng USB cùng lúc.
6. **Vận hành dài ngày** — tự khởi động khi bật máy (launchd trên Mac), xoay vòng file log, cảnh báo khi số sự cố vượt ngưỡng, kiểm tra nhiệt độ driver.

**Quy tắc khi viết ứng dụng mới:**

- Không gửi quá 15 lệnh/s; lệnh nào cũng ghi `ms`. Gửi dồn cũng không hỏng (master giữ lệnh mới nhất) nhưng phí băng thông.
- Giữ độ nghiêng trong vùng `range` trả về (hiện \~13,6° mọi hướng với hành trình 25,71°).
- Thấy `FAULT` → ngừng gửi; thấy `chưa home` / `offline` → tạm dừng, kiểm tra `status`. Không tự `clear` liên tục (mòn cơ khí, che mất lỗi phần cứng).
- Luôn ghi log trao đổi như `TrafficLog` — thiếu log thì không chẩn đoán được sự cố.
- Thoát chương trình thì gửi `look 0 0 0 800` (về nhìn thẳng), trừ khi đang FAULT.

## 11. Phụ lục: tham số và chỗ sửa

Giá trị là giá trị hiện tại trong file ngày 07/10/2026. Thông số của node (min, max, homedir, maxhz, accel, homefast, homeslow, led) nên đổi bằng lệnh `set` thay vì sửa code.

| File | Tham số | Giá trị | Khi nào sửa |
| --- | --- | --- | --- |
| `delta_proto.py` | `NODE_MACS`, `MASTER_MAC` | xem mục 2 | Thay / thêm board |
| `delta_proto.py` | `CHANNEL` | 1 | Nhiễu sóng Wi-Fi xung quanh |
| `delta_proto.py` | `TX_POWER_DBM` | 8,5 | Mất liên lạc → 11; `None` = mặc định |
| `motor_node.py` | `PIN_*` | IO1–IO8 (mục 2) | Đổi mạch |
| `motor_node.py` | `GEAR_RATIO`, `MICROSTEP` | 14, 8 | Đổi hộp số / vi bước (sửa cả `STEPS_PER_DEG` trong `master.py`) |
| `motor_node.py` | `HOME_TOL_STEPS`, `HOME_BACKOFF_STEPS`, `HOME_RELEASE_MAX` | 48, 160, 800 | Công tắc khác loại |
| `motor_node.py` | `LED_ANIMATE`, `LED_USE_RMT`, `LED_MAX_PERCENT` | False, False, 5 | Sau khi hết lỗi reset mới thử bật hiệu ứng |
| `motor_node.py` | `ENABLE_WDT`, `WDT_TIMEOUT_MS` | False, 5000 | Bật khi chạy không người trông |
| `motor_node.py` | `LINK_TIMEOUT_MS` | 1000 | Node tự dừng khi mất master |
| `master.py` | `MOTOR_SIGN` | (1, 1, 1) | Motor quay ngược khi thử xoay |
| `master.py` | `PING_PERIOD_MS`, `NODE_TIMEOUT_MS`, `ACK_TIMEOUT_MS`, `MAX_RETRY` | 200, 2000, 200, 5 | Điều chỉnh độ nhạy phát hiện mất liên lạc |
| `master.py` | `MOVE_MAX_FPS` | 15 | Giới hạn lệnh di chuyển/giây |
| `master.py` | `AUTO_REHOME`, `REHOME_MAX_TRIES` | True, 2 | Tắt tự home lại khi cần kiểm tra tay |
| `master.py` | `ENABLE_WDT` | False | Như trên |
| `spm_ik.py` | `U`, `V0`, `ALPHA1`, `ALPHA2`, `BRANCH` | từ CAD v23 | Đổi thiết kế cơ khí (đo lại từ file STEP) |
| `eye_movie.py` | `SEND_FPS`, `DEADBAND_DEG`, `MAX_TILT` | 10, 0,3°, 12° | Độ mượt / số lệnh / vùng nhìn |

**File trên board xem nhanh trong Thonny:** `print(open('config.json').read())` (node), `print(open('crash.txt').read())` (node), `print(open('neutral.txt').read())` (master), `import os; print(os.uname())` (phiên bản firmware).
