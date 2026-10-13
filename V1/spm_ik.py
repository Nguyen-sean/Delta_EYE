# spm_ik.py - động học ngược (IK) cho cơ cấu cầu song song 3-RRR (spherical parallel manipulator).
# Nạp lên board MASTER (cùng delta_proto.py và main.py).
#
# Hệ tọa độ robot: gốc O = tâm quay của mắt, trục z = hướng nhìn khi nhìn thẳng,
# trục x = trục X của bản CAD (chiếu lên mặt phẳng ngang).
#
#   u_i : trục động cơ chân i, hướng từ tâm ra động cơ   (cố định trên đế)
#   w_i : trục khớp giữa chân i (quay quanh u_i góc theta_i, cách u_i góc ALPHA1)
#   v_i : trục khớp trên mắt (gắn với mắt, cách w_i góc ALPHA2)
#
# Lệnh hướng nhìn theo TỌA ĐỘ CỰC:
#   azimuth (phi)  : hướng nghiêng, 0 độ = trục x, ngược chiều kim đồng hồ nhìn từ phía trước mắt
#   tilt (theta)   : độ nghiêng của hướng nhìn so với trục z
#   twist (psi)    : xoay quanh chính trục nhìn
# Ma trận hướng: R = Rz(phi) * Ry(theta) * Rz(psi - phi)   (kiểu "tilt-and-torsion",
# không bị lỗi khi tilt = 0 như góc Euler thông thường).

from math import sin, cos, radians, degrees, atan2, acos, sqrt

# ==== HÌNH HỌC - ĐO TỪ delta.step (EYES_DELTA v23), vị trí nhìn thẳng ====
# Chân 1 = motor Nema17 v4:1 / Part1:1, chân 2 = v4:2 / Part1:10, chân 3 = v4:3 / Part1:11
# Ghi chú: trong CAD các trục không cắt nhau đúng 1 điểm (khớp giữa lệch tâm ~13 mm,
# tâm 3 motor thấp hơn tâm mắt ~8.5 mm). IK dùng HƯỚNG các trục nên vẫn đúng gần đúng.
U = (                                   # trục động cơ (vector đơn vị, hướng ra phía động cơ)
    (0.57453, 0.57828, -0.57923),
    (0.21355, -0.78670, -0.57923),
    (-0.78807, 0.20841, -0.57923),
)
V0 = (                                  # trục khớp trên mắt khi nhìn thẳng
    (0.99204, 0.12595, 0.0),
    (-0.38694, -0.92210, 0.0),
    (-0.60510, 0.79615, 0.0),
)
ALPHA1 = (65.5955, 65.5955, 65.5955)   # góc giữa trục động cơ và trục khớp giữa (độ)
ALPHA2 = (50.0, 50.0, 50.0)            # góc giữa trục khớp giữa và trục khớp trên mắt (độ)
BRANCH = (1, 1, 1)                     # nghiệm khớp với cách lắp trong CAD (+1 / -1)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _unit(a):
    n = sqrt(_dot(a, a))
    return (a[0] / n, a[1] / n, a[2] / n)


def _matvec(m, v):
    return tuple(_dot(row, v) for row in m)


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def _rz(t):
    c, s = cos(t), sin(t)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _ry(t):
    c, s = cos(t), sin(t)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def wrap180(d):
    """Đưa góc (độ) về khoảng (-180, 180]."""
    d = (d + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def orientation(azimuth, tilt, twist=0.0):
    """Ma trận quay của mắt từ tọa độ cực (độ)."""
    p, t, s = radians(azimuth), radians(tilt), radians(twist)
    return _matmul(_matmul(_rz(p), _ry(t)), _rz(s - p))


def look_vector(azimuth, tilt):
    """Vector hướng nhìn (đơn vị) ứng với tọa độ cực."""
    p, t = radians(azimuth), radians(tilt)
    return (sin(t) * cos(p), sin(t) * sin(p), cos(t))


def _legs():
    legs = []
    for i in range(3):
        u = _unit(U[i])
        a = _unit(_cross((0.0, 0.0, 1.0), u))   # gốc đo theta: vuông góc u, nằm ngang
        b = _cross(u, a)
        legs.append((u, a, b, _unit(V0[i]),
                     cos(radians(ALPHA1[i])), sin(radians(ALPHA1[i])), cos(radians(ALPHA2[i]))))
    return legs


_LEGS = _legs()


def ik(azimuth, tilt, twist=0.0):
    """
    Trả về [theta1, theta2, theta3] (độ) - góc quay trục động cơ trong hệ IK.
    Ném ValueError nếu hướng này nằm ngoài vùng làm việc của cơ cấu.
    """
    R = orientation(azimuth, tilt, twist)
    out = []
    for i, (u, a, b, v0, c1, s1, c2) in enumerate(_LEGS):
        v = _matvec(R, v0)
        # w(theta) = c1*u + s1*(cos(theta)*a + sin(theta)*b),  điều kiện w . v = cos(ALPHA2)
        # -> A cos(theta) + B sin(theta) = C
        A = s1 * _dot(a, v)
        B = s1 * _dot(b, v)
        C = c2 - c1 * _dot(u, v)
        r = sqrt(A * A + B * B)
        if r < 1e-9 or abs(C) > r:
            raise ValueError("chân {} không với tới (ngoài vùng làm việc)".format(i + 1))
        th = atan2(B, A) + BRANCH[i] * acos(C / r)
        out.append(wrap180(degrees(th)))
    return out


def joint_w(i, theta):
    """Vector trục khớp giữa của chân i (0..2) khi động cơ ở góc theta (độ)."""
    u, a, b, v0, c1, s1, c2 = _LEGS[i]
    t = radians(theta)
    return tuple(c1 * u[k] + s1 * (cos(t) * a[k] + sin(t) * b[k]) for k in range(3))


def check(azimuth, tilt, twist, thetas):
    """Sai số ràng buộc w.v - cos(ALPHA2) lớn nhất (để kiểm tra, ~0 là đúng)."""
    R = orientation(azimuth, tilt, twist)
    err = 0.0
    for i, th in enumerate(thetas):
        v = _matvec(R, _LEGS[i][3])
        err = max(err, abs(_dot(joint_w(i, th), v) - _LEGS[i][6]))
    return err
