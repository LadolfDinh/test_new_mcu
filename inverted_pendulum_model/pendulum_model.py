"""
Mô hình con lắc ngược trên xe đẩy (cart-pendulum) - theo CTMS, Univ. of Michigan
"Inverted Pendulum: System Modeling"
https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling

Chỉ cần numpy. Gồm:
  - PendulumParams     : thông số vật lý (mặc định = giá trị trên trang CTMS)
  - accel_theta()      : phương trình phi tuyến (3),(6) giải ra x_ddot, theta_ddot
  - rhs_theta/rhs_phi  : vế phải ODE phi tuyến, trạng thái [x, x_dot, theta|phi, theta_dot|phi_dot]
  - reaction_forces()  : lực liên kết N (ngang), P (dọc) tại khớp xe - con lắc
  - linear_ss()        : mô hình tuyến tính A, B, C, D (20),(21)
  - tf_coeffs()        : hệ số hàm truyền P_pend(s), P_cart(s) (16)-(19)
  - c2d_zoh()          : rời rạc hóa ZOH (dùng cho vi điều khiển, vd Ts = 2 ms)
  - simulate()         : tích phân RK4 bước cố định, lực giữ không đổi trong 1 bước (ZOH)
  - export_json()      : xuất toàn bộ số liệu ra JSON

Quy ước góc (giống CTMS):
  theta : góc con lắc tính từ phương thẳng đứng HƯỚNG XUỐNG (theta = pi là thẳng đứng hướng lên)
  phi   : độ lệch so với vị trí thẳng đứng hướng lên, theta = pi + phi
          phi > 0 nghĩa là con lắc nghiêng về phía -x (trọng tâm x_p = x - l*sin(phi))
  F (=u): lực ngang tác dụng lên xe, chiều dương theo chiều x

Chạy:  python pendulum_model.py                 -> in số liệu + tự kiểm tra với số CTMS
       python pendulum_model.py --json out.json -> xuất số liệu
       python pendulum_model.py --plot          -> vẽ so sánh phi tuyến / tuyến tính (cần matplotlib)
"""
from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass

import numpy as np

STATE_NAMES = ["x", "x_dot", "phi", "phi_dot"]
OUTPUT_NAMES = ["x", "phi"]
INPUT_NAMES = ["u"]


# --------------------------------------------------------------------------- thông số
@dataclass
class PendulumParams:
    M: float = 0.5     # kg       khối lượng xe
    m: float = 0.2     # kg       khối lượng con lắc
    b: float = 0.1     # N/(m/s)  hệ số ma sát nhớt của xe
    l: float = 0.3     # m        khoảng cách từ khớp quay đến trọng tâm con lắc
    I: float = 0.006   # kg.m^2   mô men quán tính con lắc quanh trọng tâm
    g: float = 9.8     # m/s^2    gia tốc trọng trường

    @property
    def q(self) -> float:
        """q = (M+m)(I+ml^2) - (ml)^2   (mẫu số của hàm truyền)"""
        return (self.M + self.m) * (self.I + self.m * self.l**2) - (self.m * self.l) ** 2

    @property
    def p(self) -> float:
        """p = I(M+m) + M m l^2   (mẫu số của ma trận A, B). Đại số: p == q."""
        return self.I * (self.M + self.m) + self.M * self.m * self.l**2


# --------------------------------------------------------------------------- phi tuyến
def accel_theta(par: PendulumParams, theta, x_dot, theta_dot, F):
    """Giải hệ phi tuyến (3) và (6) theo x_ddot, theta_ddot.

    (3) (M+m) x_dd + b x_d + m l th_dd cos(th) - m l th_d^2 sin(th) = F
    (6) (I + m l^2) th_dd + m g l sin(th) = -m l x_dd cos(th)

    Dạng ma trận:
        [ M+m        m l cos th ] [x_dd ]   [ F - b x_d + m l th_d^2 sin th ]
        [ m l cos th  I + m l^2 ] [th_dd] = [ -m g l sin th                 ]
    """
    M, m, b, l, I, g = par.M, par.m, par.b, par.l, par.I, par.g
    c, s = np.cos(theta), np.sin(theta)
    a11, a12 = M + m, m * l * c
    a22 = I + m * l**2
    r1 = F - b * x_dot + m * l * theta_dot**2 * s
    r2 = -m * g * l * s
    det = a11 * a22 - a12 * a12            # = (M+m)(I+ml^2) - m^2 l^2 cos^2 th  > 0
    x_dd = (a22 * r1 - a12 * r2) / det
    th_dd = (a11 * r2 - a12 * r1) / det
    return x_dd, th_dd


def rhs_theta(par: PendulumParams, state, F):
    """d/dt [x, x_dot, theta, theta_dot] (phi tuyến, góc theta)."""
    x, xd, th, thd = state
    x_dd, th_dd = accel_theta(par, th, xd, thd, F)
    return np.array([xd, x_dd, thd, th_dd])


def rhs_phi(par: PendulumParams, state, F):
    """d/dt [x, x_dot, phi, phi_dot] (phi tuyến, góc phi = theta - pi).

    Tương đương:
        (M+m) x_dd + b x_d - m l phi_dd cos(phi) + m l phi_d^2 sin(phi) = F
        (I + m l^2) phi_dd - m g l sin(phi) = m l x_dd cos(phi)
    """
    x, xd, ph, phd = state
    x_dd, ph_dd = accel_theta(par, np.pi + ph, xd, phd, F)
    return np.array([xd, x_dd, phd, ph_dd])


def reaction_forces(par: PendulumParams, theta, theta_dot, x_dd, theta_dd):
    """Lực liên kết tại khớp (tác dụng lên con lắc):
        (2) N = m x_dd + m l th_dd cos th - m l th_d^2 sin th      (phương ngang)
            P = m g + m l th_dd sin th + m l th_d^2 cos th         (phương dọc)
    """
    m, l, g = par.m, par.l, par.g
    c, s = np.cos(theta), np.sin(theta)
    N = m * x_dd + m * l * theta_dd * c - m * l * theta_dot**2 * s
    P = m * g + m * l * theta_dd * s + m * l * theta_dot**2 * c
    return N, P


# --------------------------------------------------------------------------- tuyến tính
def linear_ss(par: PendulumParams):
    """Mô hình không gian trạng thái quanh theta = pi (phi = 0).

    x_vec = [x, x_dot, phi, phi_dot]^T, u = F, y = [x, phi]^T
    (20) x_vec_dot = A x_vec + B u
    (21) y = C x_vec + D u
    """
    M, m, b, l, I, g = par.M, par.m, par.b, par.l, par.I, par.g
    p = par.p
    A = np.array([
        [0, 1, 0, 0],
        [0, -(I + m * l**2) * b / p, (m**2 * g * l**2) / p, 0],
        [0, 0, 0, 1],
        [0, -(m * l * b) / p, m * g * l * (M + m) / p, 0],
    ], dtype=float)
    B = np.array([[0], [(I + m * l**2) / p], [0], [m * l / p]], dtype=float)
    C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]], dtype=float)
    D = np.zeros((2, 1))
    return A, B, C, D


def tf_coeffs(par: PendulumParams):
    """Hệ số hàm truyền (đa thức theo s, bậc giảm dần, mẫu monic).

    pend : Phi(s)/U(s) sau khi triệt cực-zero tại gốc (18)        [rad/N]
    pend_full : dạng chưa triệt (16)
    cart : X(s)/U(s) (19)                                          [m/N]
    """
    M, m, b, l, I, g = par.M, par.m, par.b, par.l, par.I, par.g
    q = par.q
    a2 = b * (I + m * l**2) / q
    a1 = -(M + m) * m * g * l / q
    a0 = -b * m * g * l / q
    return {
        "pend": {"num": [m * l / q, 0.0], "den": [1.0, a2, a1, a0]},
        "pend_full": {"num": [m * l / q, 0.0, 0.0], "den": [1.0, a2, a1, a0, 0.0]},
        "cart": {"num": [(I + m * l**2) / q, 0.0, -g * m * l / q], "den": [1.0, a2, a1, a0, 0.0]},
    }


def tf_from_ss(A, B, C, D, s):
    """G(s) = C (sI - A)^-1 B + D, đánh giá tại một giá trị phức s (dùng để kiểm tra)."""
    n = A.shape[0]
    return C @ np.linalg.solve(s * np.eye(n) - A, B) + D


def polyval_tf(tf, s):
    return np.polyval(tf["num"], s) / np.polyval(tf["den"], s)


# --------------------------------------------------------------------------- rời rạc
def _expm(Mat):
    """exp(Mat) bằng scaling & squaring + Taylor (đủ chính xác cho ma trận nhỏ)."""
    Mat = np.asarray(Mat, dtype=float)
    n = Mat.shape[0]
    nrm = np.linalg.norm(Mat, 1)
    sq = max(0, int(np.ceil(np.log2(nrm))) + 1) if nrm > 0 else 0
    X = Mat / (2.0**sq)
    E = np.eye(n)
    term = np.eye(n)
    for k in range(1, 30):
        term = term @ X / k
        E = E + term
    for _ in range(sq):
        E = E @ E
    return E


def c2d_zoh(A, B, Ts):
    """Rời rạc hóa ZOH: x[k+1] = Ad x[k] + Bd u[k]."""
    n, nu = B.shape
    Maug = np.zeros((n + nu, n + nu))
    Maug[:n, :n] = A
    Maug[:n, n:] = B
    E = _expm(Maug * Ts)
    return E[:n, :n], E[:n, n:]


# --------------------------------------------------------------------------- mô phỏng
def simulate(rhs, s0, t_end, dt, force=None):
    """RK4 bước cố định. rhs(state, F) -> d state/dt. force(t, state) -> F (giữ trong 1 bước).

    Trả về t (n+1,), S (n+1, 4), U (n+1,).
    """
    n = int(round(t_end / dt))
    t = np.arange(n + 1) * dt
    S = np.zeros((n + 1, len(s0)))
    U = np.zeros(n + 1)
    S[0] = s0
    for k in range(n):
        u = 0.0 if force is None else float(force(t[k], S[k]))
        U[k] = u
        x = S[k]
        k1 = rhs(x, u)
        k2 = rhs(x + 0.5 * dt * k1, u)
        k3 = rhs(x + 0.5 * dt * k2, u)
        k4 = rhs(x + dt * k3, u)
        S[k + 1] = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    U[n] = U[n - 1] if n > 0 else 0.0
    return t, S, U


def simulate_nonlinear(par, s0, t_end, dt, force=None):
    """Phi tuyến, trạng thái [x, x_dot, phi, phi_dot]."""
    return simulate(lambda s, F: rhs_phi(par, s, F), np.asarray(s0, float), t_end, dt, force)


def simulate_linear(par, s0, t_end, dt, force=None):
    """Tuyến tính A, B, trạng thái [x, x_dot, phi, phi_dot]."""
    A, B, _, _ = linear_ss(par)
    return simulate(lambda s, F: A @ s + B[:, 0] * F, np.asarray(s0, float), t_end, dt, force)


# --------------------------------------------------------------------------- xuất dữ liệu
EQUATIONS_TEXT = {
    "1_cart_horizontal": "M*x_dd + b*x_d + N = F",
    "2_pend_horizontal": "N = m*x_dd + m*l*th_dd*cos(th) - m*l*th_d^2*sin(th)",
    "3_combined_1": "(M+m)*x_dd + b*x_d + m*l*th_dd*cos(th) - m*l*th_d^2*sin(th) = F",
    "4_pend_perpendicular": "P*sin(th) + N*cos(th) - m*g*sin(th) = m*l*th_dd + m*x_dd*cos(th)",
    "5_pend_moment": "-P*l*sin(th) - N*l*cos(th) = I*th_dd",
    "6_combined_2": "(I + m*l^2)*th_dd + m*g*l*sin(th) = -m*l*x_dd*cos(th)",
    "P_vertical": "P = m*g + m*l*th_dd*sin(th) + m*l*th_d^2*cos(th)",
    "7_8_9_linearization": "th = pi + phi; cos(th) ~ -1; sin(th) ~ -phi; th_d^2 = phi_d^2 ~ 0",
    "10_linear_1": "(I + m*l^2)*phi_dd - m*g*l*phi = m*l*x_dd",
    "11_linear_2": "(M+m)*x_dd + b*x_d - m*l*phi_dd = u",
    "12_laplace_1": "(I + m*l^2)*Phi(s)*s^2 - m*g*l*Phi(s) = m*l*X(s)*s^2",
    "13_laplace_2": "(M+m)*X(s)*s^2 + b*X(s)*s - m*l*Phi(s)*s^2 = U(s)",
    "14_X_of_Phi": "X(s) = [ (I + m*l^2)/(m*l) - g/s^2 ] * Phi(s)",
    "17_q": "q = (M+m)*(I + m*l^2) - (m*l)^2",
    "18_P_pend": "Phi(s)/U(s) = (m*l/q)*s / ( s^3 + b*(I+m*l^2)/q*s^2 - (M+m)*m*g*l/q*s - b*m*g*l/q )",
    "19_P_cart": "X(s)/U(s) = ((I+m*l^2)*s^2 - g*m*l)/q / ( s^4 + b*(I+m*l^2)/q*s^3 - (M+m)*m*g*l/q*s^2 - b*m*g*l/q*s )",
    "p": "p = I*(M+m) + M*m*l^2   (= q)",
    "20_state": "d/dt[x; x_d; phi; phi_d] = A*[x; x_d; phi; phi_d] + B*u",
    "21_output": "y = [x; phi] = C*[x; x_d; phi; phi_d] + D*u",
}


def _roots_list(z):
    """Nghiệm thực -> list số thực (sắp xếp); có nghiệm phức -> list [re, im]."""
    z = np.asarray(z, dtype=complex)
    if np.all(np.abs(z.imag) < 1e-12):
        return sorted(float(v) for v in z.real)
    return [[float(v.real), float(v.imag)] for v in z]


def build_data(par: PendulumParams, Ts: float = 0.002) -> dict:
    A, B, C, D = linear_ss(par)
    Ad, Bd = c2d_zoh(A, B, Ts)
    tfs = tf_coeffs(par)
    return {
        "source": "CTMS Univ. of Michigan - Inverted Pendulum: System Modeling",
        "angle_convention": "theta tu phuong thang dung huong xuong; phi = theta - pi (lech khoi vi tri thang dung huong len)",
        "params": asdict(par),
        "units": {"M": "kg", "m": "kg", "b": "N/(m/s)", "l": "m", "I": "kg*m^2", "g": "m/s^2"},
        "derived": {"q": par.q, "p": par.p},
        "states": STATE_NAMES, "inputs": INPUT_NAMES, "outputs": OUTPUT_NAMES,
        "continuous": {"A": A.tolist(), "B": B.tolist(), "C": C.tolist(), "D": D.tolist(),
                       "eig_A": _roots_list(np.linalg.eigvals(A))},
        "discrete_zoh": {"Ts": Ts, "Ad": Ad.tolist(), "Bd": Bd.tolist()},
        "transfer_functions": {
            k: {**v, "poles": _roots_list(np.roots(v["den"])), "zeros": _roots_list(np.roots(v["num"]))}
            for k, v in tfs.items()
        },
        "equations_text": EQUATIONS_TEXT,
    }


def export_json(path: str, par: PendulumParams | None = None, Ts: float = 0.002) -> dict:
    data = build_data(par or PendulumParams(), Ts)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    return data


# --------------------------------------------------------------------------- tự kiểm tra
CTMS_REFERENCE = {  # số in trên trang CTMS (4 chữ số có nghĩa)
    "A": [[0, 1, 0, 0], [0, -0.1818, 2.673, 0], [0, 0, 0, 1], [0, -0.4545, 31.18, 0]],
    "B": [0, 1.818, 0, 4.545],
    "pend_num": [4.545, 0], "pend_den": [1, 0.1818, -31.18, -4.455],
    "cart_num": [1.818, 0, -44.55], "cart_den": [1, 0.1818, -31.18, -4.455, 0],
}


def self_check(par: PendulumParams, verbose: bool = True) -> bool:
    ok = True

    def check(name, got, ref, rtol=2e-3, atol=1e-9):
        nonlocal ok
        good = np.allclose(np.asarray(got, complex), np.asarray(ref, complex), rtol=rtol, atol=atol)
        ok &= bool(good)
        if verbose:
            print(f"  [{'OK ' if good else 'SAI'}] {name}")

    A, B, C, D = linear_ss(par)
    tfs = tf_coeffs(par)
    is_ctms = asdict(par) == asdict(PendulumParams())
    if is_ctms:
        check("A khớp CTMS", A, CTMS_REFERENCE["A"])
        check("B khớp CTMS", B[:, 0], CTMS_REFERENCE["B"])
        check("P_pend khớp CTMS (tử)", tfs["pend"]["num"], CTMS_REFERENCE["pend_num"])
        check("P_pend khớp CTMS (mẫu)", tfs["pend"]["den"], CTMS_REFERENCE["pend_den"])
        check("P_cart khớp CTMS (tử)", tfs["cart"]["num"], CTMS_REFERENCE["cart_num"])
        check("P_cart khớp CTMS (mẫu)", tfs["cart"]["den"], CTMS_REFERENCE["cart_den"])
    check("p == q", par.p, par.q, rtol=1e-12)

    # hàm truyền tính từ (A,B,C,D) == hàm truyền giải tích
    for s in (0.7 + 1.3j, -2.0 + 0.5j, 3.1j):
        G = tf_from_ss(A, B, C, D, s)
        check(f"C(sI-A)^-1B == TF tại s={s}", [G[0, 0], G[1, 0]],
              [polyval_tf(tfs["cart"], s), polyval_tf(tfs["pend"], s)], rtol=1e-9)

    # Jacobian số của mô hình phi tuyến tại điểm cân bằng == (A, B)
    eps = 1e-6
    J = np.zeros((4, 4))
    for i in range(4):
        e = np.zeros(4); e[i] = eps
        J[:, i] = (rhs_phi(par, e, 0.0) - rhs_phi(par, -e, 0.0)) / (2 * eps)
    Bn = (rhs_phi(par, np.zeros(4), eps) - rhs_phi(par, np.zeros(4), -eps)) / (2 * eps)
    check("Jacobian phi tuyến == A", J, A, rtol=1e-6, atol=1e-7)
    check("Jacobian phi tuyến == B", Bn, B[:, 0], rtol=1e-6, atol=1e-7)

    # lực liên kết N, P thỏa (4), (5)
    th, thd, F = 2.5, 1.7, 0.8
    xdd, thdd = accel_theta(par, th, 0.3, thd, F)
    N, P = reaction_forces(par, th, thd, xdd, thdd)
    m, l, g, I = par.m, par.l, par.g, par.I
    r4 = P * np.sin(th) + N * np.cos(th) - m * g * np.sin(th) - (m * l * thdd + m * xdd * np.cos(th))
    r5 = -P * l * np.sin(th) - N * l * np.cos(th) - I * thdd
    r1 = par.M * xdd + par.b * 0.3 + N - F
    check("N, P thỏa (1), (4), (5)", [r1, r4, r5], [0, 0, 0], atol=1e-12)

    # ZOH: so với tích phân RK4 bước rất nhỏ với u không đổi
    Ts = 0.002
    Ad, Bd = c2d_zoh(A, B, Ts)
    x0 = np.array([0.01, -0.02, 0.05, 0.1]); u = 0.3
    _, S, _ = simulate(lambda s, F: A @ s + B[:, 0] * F, x0, Ts, Ts / 200, lambda t, s: u)
    check("Rời rạc ZOH (Ts=2ms) == RK4", Ad @ x0 + Bd[:, 0] * u, S[-1], rtol=1e-8, atol=1e-12)

    # phi tuyến ~ tuyến tính với góc lệch nhỏ
    t, Sn, _ = simulate_nonlinear(par, [0, 0, 0.001, 0], 0.5, 1e-3)
    _, Sl, _ = simulate_linear(par, [0, 0, 0.001, 0], 0.5, 1e-3)
    rel = np.max(np.abs(Sn[:, 2] - Sl[:, 2])) / np.max(np.abs(Sl[:, 2]))
    good = rel < 1e-3
    ok &= bool(good)
    if verbose:
        print(f"  [{'OK ' if good else 'SAI'}] phi tuyến ~ tuyến tính khi phi0 = 0.001 rad (sai lệch tương đối {rel:.1e})")
    return ok


def _print_summary(par: PendulumParams):
    np.set_printoptions(precision=4, suppress=True, linewidth=110)
    A, B, C, D = linear_ss(par)
    tfs = tf_coeffs(par)
    print("Thông số:", asdict(par))
    print(f"q = {par.q:.6g}   p = {par.p:.6g}")
    print("A =\n", A, "\nB =\n", B[:, 0], "\nC =\n", C, "\nD =\n", D[:, 0])
    print("Trị riêng A:", np.sort(np.linalg.eigvals(A).real))
    for k in ("pend", "cart"):
        print(f"{k}: num = {np.round(tfs[k]['num'], 4)}  den = {np.round(tfs[k]['den'], 4)}")
        print(f"      cực = {np.sort(np.roots(tfs[k]['den']).real)}")
    Ad, Bd = c2d_zoh(A, B, 0.002)
    np.set_printoptions(precision=6)
    print("ZOH Ts = 2 ms: Ad =\n", Ad, "\nBd =\n", Bd[:, 0])


def _plot(par: PendulumParams):
    import matplotlib.pyplot as plt
    phi0 = 0.05
    t, Sn, _ = simulate_nonlinear(par, [0, 0, phi0, 0], 1.0, 1e-3)
    _, Sl, _ = simulate_linear(par, [0, 0, phi0, 0], 1.0, 1e-3)
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(7, 5))
    ax[0].plot(t, Sn[:, 2], label="phi tuyến"); ax[0].plot(t, Sl[:, 2], "--", label="tuyến tính")
    ax[0].set_ylabel("phi [rad]"); ax[0].legend(); ax[0].grid(alpha=.3)
    ax[1].plot(t, Sn[:, 0]); ax[1].plot(t, Sl[:, 0], "--")
    ax[1].set_ylabel("x [m]"); ax[1].set_xlabel("t [s]"); ax[1].grid(alpha=.3)
    fig.suptitle(f"Con lắc tự do, F = 0, phi0 = {phi0} rad")
    fig.tight_layout(); plt.show()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", help="xuất số liệu ra file JSON")
    ap.add_argument("--Ts", type=float, default=0.002, help="chu kỳ lấy mẫu cho ZOH (s)")
    ap.add_argument("--plot", action="store_true", help="vẽ phi tuyến vs tuyến tính")
    args = ap.parse_args()

    par = PendulumParams()
    _print_summary(par)
    print("\nTự kiểm tra:")
    print("=> TẤT CẢ OK" if self_check(par) else "=> CÓ LỖI")
    if args.json:
        export_json(args.json, par, args.Ts)
        print("Đã xuất", args.json)
    if args.plot:
        _plot(par)
