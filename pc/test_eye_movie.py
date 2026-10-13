#!/usr/bin/env python3
"""
Test cho eye_movie.py - không cần phần cứng.
Chạy:  python3 -m unittest -v test_eye_movie.py      (trong thư mục pc/)
"""

import math
import re
import statistics
import unittest

import eye_movie as E

LOOK_RE = re.compile(r'^look (\d+\.\d) (\d+\.\d\d) 0 (\d+)$')


class FakeClock:
    """Đồng hồ giả: sleep() chỉ cộng thời gian -> chạy vài phút 'xem phim' trong vài giây."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, d):
        self.t += max(d, 0.001)


class RecordLink:
    """Link giả chỉ ghi lại lệnh gửi đi."""

    def __init__(self):
        self.sent = []

    def send(self, line, note=None):
        self.sent.append(line)


def has_ik():
    return E.FakeMaster().ik is not None


# ----------------------------------------------------------------------
class TestHelpers(unittest.TestCase):
    def test_to_polar_directions(self):
        self.assertEqual(E.to_polar(0, 0), (0.0, 0.0))
        for (x, y), az in (((5, 0), 0), ((0, 5), 90), ((-5, 0), 180), ((0, -5), 270)):
            a, t = E.to_polar(x, y)
            self.assertAlmostEqual(a, az, places=6)
            self.assertAlmostEqual(t, 5, places=6)

    def test_to_polar_offset_and_flip(self):
        a, _ = E.to_polar(5, 0, az_offset=30)
        self.assertAlmostEqual(a, 30)
        a, _ = E.to_polar(5, 0, flip_x=True)
        self.assertAlmostEqual(a, 180)
        a, _ = E.to_polar(0, -5, az_offset=100)
        self.assertAlmostEqual(a, 10)          # 270 + 100 = 370 -> 10

    def test_clamp_circle(self):
        self.assertEqual(E.clamp_circle(3, 4, 10), (3, 4))
        x, y = E.clamp_circle(30, 40, 10)
        self.assertAlmostEqual(math.hypot(x, y), 10)
        self.assertAlmostEqual(y / x, 4 / 3)

    def test_robot_min_ms(self):
        self.assertGreater(E.robot_min_ms(10), E.robot_min_ms(2))
        self.assertGreater(E.robot_min_ms(0.1), 0)
        # 10 độ ~ 622 bước (1/8), tăng/giảm tốc 6000 -> ~0.55 s (+10% +20 ms)
        self.assertTrue(500 < E.robot_min_ms(10) < 700, E.robot_min_ms(10))


# ----------------------------------------------------------------------
class TestSyntheticGaze(unittest.TestCase):
    def simulate(self, seconds=120, seed=1, max_tilt=12.0):
        g = E.SyntheticGaze(max_tilt=max_tilt, seed=seed)
        pts = []
        t = 0.0
        while t < seconds:
            pts.append(g.step(t) + (t,))
            t += 0.01
        return g, pts

    def test_never_exceeds_max_tilt(self):
        for mt in (6.0, 12.0):
            g, pts = self.simulate(120, seed=2, max_tilt=mt)
            worst = max(math.hypot(x, y) for x, y, _, _ in pts)
            self.assertLessEqual(worst, mt + 1e-6, 'max_tilt={}'.format(mt))

    def test_human_like_statistics(self):
        g, pts = self.simulate(120)
        fx = g.stats['fix_durations']
        self.assertTrue(0.25 < statistics.mean(fx) < 0.50, statistics.mean(fx))
        self.assertTrue(min(fx) >= 0.12 and max(fx) <= 1.5)
        # Người xem phim giật mắt ~100-180 lần/phút, nhưng robot giật 5 độ mất ~0.4 s (người ~0.03 s)
        # và mô hình chờ robot tới nơi -> mong đợi thấp hơn: 40..200 lần/phút (tính cả chuyển cảnh).
        per_min = (g.stats['sacc'] + g.stats['cut']) / 2.0
        self.assertTrue(40 < per_min < 200, per_min)
        self.assertTrue(12 <= g.stats['cut'] <= 30, g.stats['cut'])   # chuyển cảnh 4..10 s
        self.assertGreater(g.stats['pursuit'], 3)
        self.assertGreater(g.stats['micro'], 10)

    def test_center_bias(self):
        g, pts = self.simulate(120)
        mean_r = statistics.mean(math.hypot(x, y) for x, y, _, _ in pts)
        self.assertLess(mean_r, 0.6 * 12.0, mean_r)

    def test_deterministic_with_seed(self):
        _, a = self.simulate(10, seed=7)
        _, b = self.simulate(10, seed=7)
        _, c = self.simulate(10, seed=8)
        self.assertEqual(a, b)
        self.assertNotEqual(a, c)

    def test_saccade_waits_for_robot(self):
        # sau mỗi lần giật mắt, lần giật kế tiếp phải cách ít nhất thời gian robot cần để tới nơi
        g, pts = self.simulate(60)
        last = None
        for x, y, k, t in pts:
            if k == 'sacc':
                if last is not None:
                    lx, ly, lt = last
                    need = E.robot_min_ms(0) / 1000.0
                    self.assertGreaterEqual(t - lt, need + 0.11)
                last = (x, y, t)


# ----------------------------------------------------------------------
class TestCommandSender(unittest.TestCase):
    def test_rate_limit_15fps(self):
        link = RecordLink()
        s = E.CommandSender(link)
        g = E.SyntheticGaze(seed=3)
        t = 0.0
        while t < 10.0:
            s.update(t, *g.step(t))
            t += 0.01
        self.assertLessEqual(len(link.sent), 10 * 15 + 1)
        self.assertGreater(len(link.sent), 15)          # vẫn gửi đều (tùy SEND_FPS / DEADBAND_DEG)

    def test_deadband_static_gaze(self):
        link = RecordLink()
        s = E.CommandSender(link)
        for i in range(300):
            s.update(i * 0.01, 3.0, 2.0, 'fix')
        self.assertEqual(len(link.sent), 1)

    def test_saccade_never_dropped(self):
        link = RecordLink()
        s = E.CommandSender(link, fps=15)     # cố định 15 lệnh/giây, không phụ thuộc SEND_FPS
        s.update(0.00, 0, 0, 'fix')
        s.update(0.01, 8, 0, 'sacc')          # trong thời gian chờ -> chưa gửi
        self.assertEqual(len(link.sent), 1)
        s.update(0.02, 8.05, 0, 'fix')        # vẫn trong thời gian chờ
        s.update(0.07, 8.05, 0, 'fix')        # tới lượt -> phải gửi dù gần điểm cũ? (lệch 8 độ)
        self.assertEqual(len(link.sent), 2)
        az, tilt, ms = LOOK_RE.match(link.sent[1]).groups()
        self.assertAlmostEqual(float(tilt), 8.05, places=2)
        self.assertGreaterEqual(int(ms), E.robot_min_ms(8.05))

    def test_format_and_limits(self):
        link = RecordLink()
        s = E.CommandSender(link, max_tilt=10)
        g = E.SyntheticGaze(seed=4, max_tilt=14)    # mô hình rộng hơn -> sender phải cắt về 10
        t = 0.0
        while t < 30:
            s.update(t, *g.step(t))
            t += 0.01
        for line in link.sent:
            m = LOOK_RE.match(line)
            self.assertIsNotNone(m, line)
            az, tilt, ms = float(m.group(1)), float(m.group(2)), int(m.group(3))
            self.assertTrue(0 <= az < 360)
            self.assertLessEqual(tilt, 10.0 + 0.005)
            self.assertGreaterEqual(ms, 20)

    def test_pursuit_uses_smooth_time(self):
        link = RecordLink()
        s = E.CommandSender(link)
        for i in range(100):
            s.update(i * 0.01, i * 0.05, 0, 'pursuit')
        ms = [int(LOOK_RE.match(l).group(3)) for l in link.sent[1:]]
        self.assertTrue(all(m >= 100 for m in ms), ms)      # 1.5 khung * 67 ms


# ----------------------------------------------------------------------
class TestMasterLink(unittest.TestCase):
    class Port:
        def __init__(self, chunks):
            self.chunks = list(chunks)
            self.written = b''

        @property
        def in_waiting(self):
            return len(self.chunks[0]) if self.chunks else 0

        def read(self, n):
            return self.chunks.pop(0)

        def write(self, d):
            self.written += d

    def test_parse_lines_split_across_reads(self):
        p = self.Port([b'STAT n1 ON  IDLE     err=NONE      ', b'     homed=1 sw=0 pos=1\nSTAT n2 OFF',
                       b' -        err=NONE           homed=0 sw=0 pos=0\n'])
        link = E.MasterLink(p, log=lambda s: None)
        for _ in range(3):
            link.poll()
        self.assertEqual(link.nodes[1], {'online': True, 'state': 'IDLE', 'err': 'NONE', 'homed': True})
        self.assertEqual(link.nodes[2]['online'], False)

    def test_not_homed_is_not_range_error(self):
        p = self.Port(['ERR move: n3 chưa home\n'.encode(), 'ERR move: n1 offline\n'.encode(),
                       'ERR look: n2 cần 26.60 độ, ngoài hành trình 0.5..25.71\n'.encode()])
        link = E.MasterLink(p, log=lambda s: None)
        link.poll()
        self.assertEqual((link.err_look, link.not_ready), (0, True))
        link.not_ready = False
        link.poll()
        self.assertTrue(link.not_ready)
        link.poll()
        self.assertEqual(link.err_look, 1)

    def test_fault_err_rehome(self):
        p = self.Port(['ERR look: n2 cần -0.82 độ, ngoài hành trình 0.5..25.71\n'.encode(),
                       b'FAULT n2 LOST_STEP\n',
                       'OK đã home lại xong tất cả\n'.encode()])
        link = E.MasterLink(p, log=lambda s: None)
        link.poll()
        self.assertEqual(link.err_look, 1)
        link.poll()
        self.assertTrue(link.fault)
        self.assertEqual(link.fault_reason, 'n2 LOST_STEP')
        link.poll()
        self.assertTrue(link.rehome_done)

    def test_ensure_ready_sends_home(self):
        fc = FakeClock()
        fm = E.FakeMaster(homed=False, clock=fc)
        link = E.MasterLink(fm, log=lambda s: None)
        self.assertTrue(link.ensure_ready(clock=fc, sleep=fc.sleep))
        cmds = [c for _, c in fm.received]
        self.assertIn('home', cmds)
        self.assertTrue(link.all_ready())

    def test_ensure_ready_already_homed(self):
        fc = FakeClock()
        fm = E.FakeMaster(homed=True, clock=fc)
        link = E.MasterLink(fm, log=lambda s: None)
        self.assertTrue(link.ensure_ready(clock=fc, sleep=fc.sleep))
        self.assertNotIn('home', [c for _, c in fm.received])


# ----------------------------------------------------------------------
@unittest.skipUnless(has_ik(), 'không tìm thấy firmware/spm_ik.py')
class TestEndToEnd(unittest.TestCase):
    """Chạy cả vòng lặp với master giả lập dùng IK thật của robot."""

    def setup(self, **fm_kw):
        self.fc = FakeClock()
        self.fm = E.FakeMaster(clock=self.fc, **fm_kw)
        self.logs = []
        self.link = E.MasterLink(self.fm, log=self.logs.append)
        self.sender = E.CommandSender(self.link)
        self.src = E.SyntheticGaze(seed=11)

    def go(self, seconds, **kw):
        return E.run(self.link, self.src, self.sender, seconds, clock=self.fc, sleep=self.fc.sleep,
                     log=self.logs.append, **kw)

    def test_movie_3_minutes_no_errors(self):
        self.setup()
        self.assertEqual(self.go(180), 0)
        errs = [l for l in self.link.lines if l.startswith('ERR') or l.startswith('FAULT')]
        self.assertEqual(errs, [])
        looks = self.fm.looks
        self.assertGreater(len(looks), 500)
        gaps = [b[0] - a[0] for a, b in zip(looks, looks[1:])]
        self.assertGreaterEqual(min(gaps), 1 / 15 - 1e-6)          # không quá 15 lệnh/giây
        for _, az, tilt, tw, ms, degs in looks:                    # mọi trục trong hành trình
            self.assertTrue(all(0.5 <= d <= 25.71 for d in degs))

    def test_pause_on_fault_and_resume_after_rehome(self):
        self.setup()
        self.go(10)
        self.fm.inject_fault()
        n_before = len(self.fm.looks)
        self.go(5)                                    # đang FAULT -> không được gửi look
        self.assertEqual(len(self.fm.looks), n_before)
        self.assertTrue(any('Tạm dừng' in l for l in self.logs))
        self.fm.finish_rehome()
        self.assertEqual(self.go(10), 0)
        self.assertGreater(len(self.fm.looks), n_before + 20)
        self.assertTrue(any('chạy tiếp' in l for l in self.logs))

    def test_fault_never_clears_exits(self):
        self.setup()
        self.go(3)
        self.fm.inject_fault()
        self.assertEqual(self.go(60), 2)

    def test_auto_clear(self):
        self.setup()
        self.go(3)
        self.fm.inject_fault()
        self.fm.homed[1] = True                       # giả sử trục vẫn còn home
        self.assertEqual(self.go(10, auto_clear=True), 0)
        self.assertIn('clear', [c for _, c in self.fm.received])

    # --- các tình huống lấy từ log chạy thật (node bị watchdog reset, mất home) ---
    def test_node_reboot_not_homed_auto_clear_rehomes_and_resumes(self):
        self.setup()
        self.go(10)
        self.fm.reboot_node(3)
        n_before = len(self.fm.looks)
        self.assertEqual(self.go(30, auto_clear=True), 0)
        cmds = [c for _, c in self.fm.received]
        self.assertIn('home', cmds)                                  # đã tự home lại
        self.assertEqual(self.sender.max_tilt, 12.0)                 # KHÔNG bị thu hẹp vùng nhìn
        self.assertGreater(len(self.fm.looks), n_before + 50)        # đã chạy tiếp
        not_home = [l for l in self.link.lines if 'chưa home' in l]
        self.assertLessEqual(len(not_home), 2)                       # không gửi lệnh dồn dập khi chưa home
        self.assertTrue(any('sẵn sàng -> chạy tiếp' in l for l in self.logs))

    def test_node_reboot_without_auto_clear_waits_then_exits(self):
        self.setup()
        self.go(5)
        self.fm.reboot_node(1)
        n_before = len(self.fm.looks)
        self.assertEqual(self.go(120), 2)                            # chờ tối đa 60 s rồi thoát an toàn
        self.assertEqual(len(self.fm.looks), n_before)
        self.assertNotIn('home', [c for _, c in self.fm.received][-50:])
        self.assertEqual(self.sender.max_tilt, 12.0)

    def test_node_reboot_master_rehomes_by_itself(self):
        self.setup()
        self.go(5)
        self.fm.reboot_node(2)
        self.go(4)                                                   # PC tạm dừng
        self.fm.finish_rehome()                                      # master tự home lại xong
        n_before = len(self.fm.looks)
        self.assertEqual(self.go(10), 0)
        self.assertGreater(len(self.fm.looks), n_before + 10)        # đã chạy tiếp

    def test_fault_auto_clear_then_rehome_if_needed(self):
        self.setup()
        self.go(5)
        self.fm.inject_fault()                                       # n2 LOST_STEP -> mất home
        self.assertEqual(self.go(30, auto_clear=True), 0)
        cmds = [c for _, c in self.fm.received]
        i_clear = cmds.index('clear')
        self.assertIn('home', cmds[i_clear:])                        # clear xong còn phải home
        self.assertTrue(all(self.fm.homed))

    def test_log_dedupe(self):
        self.setup()
        for _ in range(100):
            self.link.log('!! ERR move: n3 chưa home')
        self.link.log('khác')
        self.assertEqual(self.logs, ['!! ERR move: n3 chưa home',
                                     '   (dòng trên lặp lại thêm 99 lần)', 'khác'])

    def test_shrinks_range_when_master_rejects(self):
        self.setup(hi=17.0)                           # hành trình hẹp -> nhiều hướng bị từ chối
        self.go(60)
        self.assertLess(self.sender.max_tilt, 12.0)
        errs_late = [t for t, l in self.fm.received if False]
        n_err_first = sum(1 for l in self.link.lines if l.startswith('ERR look'))
        self.link.lines.clear()
        self.go(60)
        n_err_after = sum(1 for l in self.link.lines if l.startswith('ERR look'))
        self.assertLess(n_err_after, max(1, n_err_first))


# ----------------------------------------------------------------------
class TestTrafficLog(unittest.TestCase):
    def make(self, path=None):
        self.fc = FakeClock()
        self.out = []
        return E.TrafficLog(path, clock=self.fc, wall=lambda: 1700000000.0 + self.fc.t,
                            console=self.out.append)

    def test_log_disk_timeout_does_not_stop_program(self):
        class HangingFile:
            def write(self, t):
                pass

            def flush(self):
                raise TimeoutError(60, 'Operation timed out')   # OneDrive treo khi ghi

            def close(self):
                raise TimeoutError(60, 'Operation timed out')
        tl = self.make()
        tl.f = HangingFile()
        tl.record('>>', 'look 1.0 1.00 0 100')          # không được ném lỗi
        tl.event('vẫn chạy')
        self.assertIsNone(tl.f)
        self.assertTrue(any('Không ghi được file log' in l for l in self.out))

    def test_incident_shows_recent_commands_only(self):
        tl = self.make()
        tl.record('>>', 'look 1.0 1.00 0 100', 'cũ')
        self.fc.t = 10.0                                  # 10 s sau: ngoài cửa sổ 5 s
        for i in range(8):
            tl.record('>>', 'look {}.0 2.00 0 100'.format(i), 'sacc')
            self.fc.t += 0.1
        tl.record('<<', 'ERR MOVE: không nhận được ACK từ n1')
        text = '\n'.join(self.out)
        self.assertIn('SỰ CỐ #1', text)
        self.assertIn('không nhận được ACK từ n1', text)
        self.assertIn('8 lệnh gửi trong 5 s', text)
        self.assertIn('look 7.0', text)                   # lệnh mới nhất có mặt
        self.assertNotIn('look 1.0', text)                # lệnh 10 s trước thì không
        self.assertIn('# sacc', text)                     # có ghi loại chuyển động

    def test_related_lines_merge_into_one_incident(self):
        tl = self.make()
        for line in ('ERR MOVE: không nhận được ACK từ n1', 'FAULT thiếu ACK lệnh MOVE',
                     'ERR n1 OFFLINE', 'FAULT n1 khởi động lại (HARD_RESET)'):
            tl.record('<<', line)
            self.fc.t += 0.5
        self.assertEqual(len(tl.incidents), 1)
        self.fc.t += 5
        tl.record('<<', 'ERR n3 OFFLINE')
        self.assertEqual(len(tl.incidents), 2)

    def test_normal_lines_are_not_incidents(self):
        tl = self.make()
        for line in ('OK MOVE', 'WARN 100 ms quá nhanh với tốc độ hiện tại -> dùng 120 ms',
                     'EVT n1 IDLE -> MOVING pos=1.0', 'INFO bỏ 3 lệnh di chuyển'):
            tl.record('<<', line)
        self.assertEqual(tl.incidents, [])

    def test_file_contains_everything(self):
        import os
        import tempfile
        d = tempfile.mkdtemp()
        path = os.path.join(d, 'sub', 'x.log')            # tạo luôn thư mục con
        tl = self.make(path)
        tl.record('>>', 'look 10.0 3.00 0 100', 'fix')
        tl.record('<<', 'OK MOVE')
        tl.event('.. thông báo')
        tl.record('<<', 'FAULT n2 LOST_STEP')
        tl.summary()
        tl.close()
        with open(path, encoding='utf-8') as f:
            text = f.read()
        for s in ('>> look 10.0 3.00 0 100', '<< OK MOVE', '## .. thông báo',
                  '===== SỰ CỐ #1', 'Tổng kết: 1 sự cố', 'n2 LOST_STEP'):
            self.assertIn(s, text)
        self.assertRegex(text, r'\d\d:\d\d:\d\d\.\d{3} >> look')

    def test_summary_counts_by_node_and_type(self):
        tl = self.make()
        for line in ('ERR n1 OFFLINE', 'ERR n3 OFFLINE', 'ERR n1 OFFLINE', 'ERR n3 INTERNAL pos=1'):
            tl.record('<<', line)
            self.fc.t += 10
        self.out.clear()
        tl.summary()
        text = '\n'.join(self.out)
        self.assertIn('Tổng kết: 4 sự cố', text)
        self.assertRegex(text, r'n1 OFFLINE\s+2 lần')
        self.assertRegex(text, r'n3 INTERNAL\s+1 lần')

    @unittest.skipUnless(has_ik(), 'không tìm thấy firmware/spm_ik.py')
    def test_end_to_end_fault_logs_recent_looks(self):
        tl = self.make()
        fm = E.FakeMaster(clock=self.fc)
        link = E.MasterLink(fm, log=tl.event, traffic=tl)
        sender = E.CommandSender(link)
        src = E.SyntheticGaze(seed=5)
        E.run(link, src, sender, 10, clock=self.fc, sleep=self.fc.sleep, log=tl.event)
        fm.inject_fault()
        E.run(link, src, sender, 1, clock=self.fc, sleep=self.fc.sleep, log=tl.event)
        self.assertEqual(len(tl.incidents), 1)
        text = '\n'.join(self.out)
        self.assertIn('SỰ CỐ #1', text)
        self.assertRegex(text, r'>> look [\d.]+ [\d.]+ 0 \d+\s+# ')   # lệnh look ngay trước lỗi


# ----------------------------------------------------------------------
@unittest.skipUnless(has_ik(), 'không tìm thấy firmware/spm_ik.py')
class TestForever(unittest.TestCase):
    """Chế độ chạy không ngừng (--forever): không bao giờ tự thoát, tự kết nối lại."""

    class DyingMaster(E.FakeMaster):
        """Master giả 'mất cổng USB' (write lỗi) sau N lần ghi."""
        def __init__(self, die_after, **kw):
            super().__init__(**kw)
            self.left = die_after

        def write(self, data):
            self.left -= 1
            if self.left < 0:
                raise OSError(6, 'Device not configured')
            return super().write(data)

    class NeverHomes(E.FakeMaster):
        """Master giả: node không bao giờ home được."""
        def __init__(self, **kw):
            super().__init__(homed=False, **kw)

        def _cmd(self, line):
            if line == 'home':
                self.received.append((self.clock(), line))
                self._p('ERR n1 HOME_NOT_FOUND pos=-30.000')
                return
            super()._cmd(line)

    def setUp(self):
        self.fc = FakeClock()
        self.logs = []
        self.opened = []

    def go(self, factory, **kw):
        sender = E.CommandSender(None)
        src = E.SyntheticGaze(seed=21)

        attempts = [0]

        def open_port():
            attempts[0] += 1
            p = factory(attempts[0] - 1)          # đếm cả những lần mở cổng thất bại
            self.opened.append(p)
            return p
        return E.run_forever(open_port, src, sender, clock=self.fc, sleep=self.fc.sleep,
                             log=self.logs.append, **kw), sender

    def test_reconnects_after_usb_lost(self):
        # cổng đầu chết sau 300 lần ghi, cổng sau chạy bình thường
        def factory(i):
            if i == 0:
                return self.DyingMaster(300, clock=self.fc)
            return E.FakeMaster(clock=self.fc)
        (code, link), sender = self.go(factory, duration=600, forever=True)
        self.assertEqual(code, 0)
        self.assertEqual(len(self.opened), 2)
        self.assertTrue(any('Mất kết nối USB' in l for l in self.logs))
        self.assertTrue(any('Thử lại sau 5 s' in l for l in self.logs))
        self.assertGreater(len(self.opened[1].looks), 500)      # chạy tiếp lâu sau khi nối lại

    def test_without_forever_usb_lost_exits(self):
        (code, link), _ = self.go(lambda i: self.DyingMaster(300, clock=self.fc), duration=600)
        self.assertEqual(code, 3)
        self.assertEqual(len(self.opened), 1)

    def test_waits_for_board_to_appear(self):
        # 3 lần đầu không mở được cổng (board chưa cắm), lần 4 có
        def factory(i):
            if i < 3:
                raise OSError(2, 'No such file or directory')
            return E.FakeMaster(clock=self.fc)
        (code, link), _ = self.go(factory, duration=120, forever=True)
        self.assertEqual(code, 0)
        waits = [l for l in self.logs if 'Thử lại sau' in l]
        self.assertEqual([re.search(r'sau (\d+) s', w).group(1) for w in waits],
                         ['5', '10', '20'])                     # chờ tăng dần

    def test_robot_never_ready_keeps_retrying_with_backoff(self):
        (code, link), _ = self.go(lambda i: self.NeverHomes(clock=self.fc), forever=True,
                                  max_restarts=7)
        self.assertEqual(code, 2)                               # chỉ vì test đặt max_restarts
        waits = [int(re.search(r'sau (\d+) s', l).group(1)) for l in self.logs if 'Thử lại sau' in l]
        self.assertEqual(waits, [5, 10, 20, 40, 60, 60, 60])    # tối đa 60 s, không spam robot
        self.assertTrue(all(p.received[-1][1] != 'look' for p in self.opened))

    def test_without_forever_not_ready_exits(self):
        (code, link), _ = self.go(lambda i: self.NeverHomes(clock=self.fc))
        self.assertEqual(code, 1)

    def test_fault_that_never_clears_restarts_instead_of_exit(self):
        fm_list = []

        def factory():
            fm = E.FakeMaster(clock=self.fc)
            fm_list.append(fm)
            return fm
        sender = E.CommandSender(None)
        src = E.SyntheticGaze(seed=3)
        # chạy 30 s, rồi node 2 khởi động lại liên tục (không home được) trong lúc đang chạy
        orig_look = E.FakeMaster._look

        def flaky_look(fm, a):
            if self.fc.t > 30 and self.fc.t < 200:
                fm.homed[1] = False                             # n2 mất home liên tục
            orig_look(fm, a)
        E.FakeMaster._look = flaky_look
        try:
            code, link = E.run_forever(factory, src, sender, duration=400, forever=True,
                                       clock=self.fc, sleep=self.fc.sleep, log=self.logs.append)
        finally:
            E.FakeMaster._look = orig_look
        self.assertEqual(code, 0)                               # vẫn chạy hết 400 s
        self.assertGreater(len(fm_list[-1].looks), 50)

    def test_log_rotation(self):
        import os
        import tempfile
        d = tempfile.mkdtemp()
        tl = E.TrafficLog(os.path.join(d, 'eye_x.log'), console=lambda s: None)
        tl.MAX_LOG_BYTES = 2000
        for i in range(200):
            tl.record('>>', 'look {}.0 3.00 0 100'.format(i))
        tl.close()
        files = sorted(os.listdir(d))
        self.assertGreaterEqual(len(files), 3, files)
        self.assertIn('eye_x_part2.log', files)
        self.assertTrue(all(os.path.getsize(os.path.join(d, f)) < 2200 for f in files))


# ----------------------------------------------------------------------
try:
    import numpy as np
except ImportError:
    np = None


@unittest.skipIf(np is None, 'cần numpy')
class TestVideoGaze(unittest.TestCase):
    W, H = 160, 90

    def frame_with_square(self, cx, cy, bg=40, size=12):
        f = np.full((self.H, self.W), bg, dtype=np.float32)
        f[max(0, cy - size // 2):cy + size // 2, max(0, cx - size // 2):cx + size // 2] = 230
        return f

    def test_follows_moving_object(self):
        vg = E.VideoGaze(max_tilt=12)
        errs = []
        t = 0.0
        for i in range(120):                          # vật chạy từ trái sang phải trong 4.8 s
            cx = 15 + i
            x, y, k = vg.process(self.frame_with_square(cx, 45), t, 0.04)
            t += 0.04
            if i > 15:
                tx, ty = vg.pixel_to_gaze(cx, 45, self.W, self.H)
                errs.append(math.hypot(x - tx, y - ty))
        self.assertLess(statistics.mean(errs), 1.5, statistics.mean(errs))
        self.assertGreater(x, 5)                      # cuối cùng nhìn về bên phải

    def test_vertical_direction(self):
        vg = E.VideoGaze(max_tilt=12)
        t = 0.0
        for i in range(60):                           # vật đi từ dưới lên trên
            x, y, k = vg.process(self.frame_with_square(80, 80 - i), t, 0.04)
            t += 0.04
        self.assertGreater(y, 2)

    def test_static_scene_holds_gaze(self):
        vg = E.VideoGaze(max_tilt=12)
        f = self.frame_with_square(100, 30)
        out = [vg.process(f, i * 0.04, 0.04) for i in range(50)]
        self.assertTrue(all(abs(x) < 1e-9 and abs(y) < 1e-9 for x, y, _ in out))

    def test_scene_cut_detected(self):
        vg = E.VideoGaze(max_tilt=12)
        vg.process(self.frame_with_square(30, 20), 0.0, 0.04)
        x, y, k = vg.process(self.frame_with_square(30, 20, bg=200), 0.04, 0.04)
        self.assertEqual((x, y, k), (0.0, 0.0, 'cut'))

    def test_jump_is_saccade(self):
        vg = E.VideoGaze(max_tilt=12)
        t = 0.0
        for i in range(10):
            vg.process(self.frame_with_square(30 + (i % 2), 45), t, 0.04)
            t += 0.04
        kinds = []
        for i in range(10):                           # vật nhảy sang bên phải
            kinds.append(vg.process(self.frame_with_square(140 + (i % 2), 45), t, 0.04)[2])
            t += 0.04
        self.assertIn('sacc', kinds)

    def test_video_sender_rate(self):
        link = RecordLink()
        s = E.CommandSender(link)
        vg = E.VideoGaze(max_tilt=12)
        t = 0.0
        for i in range(250):                          # 10 s phim 25 fps
            s.update(t, *vg.process(self.frame_with_square(15 + (i % 130), 45), t, 0.04))
            t += 0.04
        self.assertLessEqual(len(link.sent), 10 * 15 + 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
