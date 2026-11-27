"""
Suy dẫn ký hiệu (sympy) mô hình con lắc ngược trên xe (CTMS - Inverted Pendulum: System Modeling)
và kiểm tra lại toàn bộ công thức dùng trong pendulum_model.py / README.md.

Các bước:
  1) Newton-Euler theo sơ đồ lực CTMS: (1),(2),(4),(5) -> (3),(6)
  2) Lagrange (cách độc lập) -> phải ra đúng (3),(6)
  3) Giải tường minh x_dd, theta_dd (dạng dùng để mô phỏng)
  4) Đổi biến theta = pi + phi, tuyến tính hóa -> (10),(11) và ma trận A, B (20)
  5) Laplace (12),(13) -> hàm truyền (16)-(19), so với C(sI-A)^-1 B
  6) Thay số CTMS

Chạy:  python pendulum_symbolic.py          -> kiểm tra (in OK/SAI)
       python pendulum_symbolic.py --latex  -> in thêm LaTeX của các phương trình
"""
import argparse

import sympy as sp

t, s = sp.symbols("t s")
M, m, b, l, I, g = sp.symbols("M m b l I g", positive=True)
F, N, P = sp.symbols("F N P")
x = sp.Function("x")(t)
th = sp.Function("theta")(t)
xd, xdd = x.diff(t), x.diff(t, 2)
thd, thdd = th.diff(t), th.diff(t, 2)

# ký hiệu thay cho đạo hàm (để giải đại số)
X1, X2, X3, T1, T2, T3 = sp.symbols("x x_d x_dd theta theta_d theta_dd")
phi, phid, phidd, u = sp.symbols("phi phi_d phi_dd u")
to_sym = {xdd: X3, thdd: T3, xd: X2, thd: T2, x: X1, th: T1}  # thứ tự: bậc cao trước

q = (M + m) * (I + m * l**2) - (m * l) ** 2
p = I * (M + m) + M * m * l**2

ALL_OK = True


def check(name, expr_zero):
    global ALL_OK
    if isinstance(expr_zero, sp.MatrixBase):
        good = all(sp.simplify(v) == 0 for v in expr_zero)
    else:
        good = sp.simplify(expr_zero) == 0
    ALL_OK &= bool(good)
    print(f"  [{'OK ' if good else 'SAI'}] {name}")


def sub(e):
    return e.subs(to_sym)


def main(show_latex: bool):
    print("1) Newton-Euler (sơ đồ lực CTMS)")
    eq1 = sp.Eq(M * xdd + b * xd + N, F)
    eq2 = sp.Eq(N, m * xdd + m * l * thdd * sp.cos(th) - m * l * thd**2 * sp.sin(th))
    eq4 = sp.Eq(P * sp.sin(th) + N * sp.cos(th) - m * g * sp.sin(th), m * l * thdd + m * xdd * sp.cos(th))
    eq5 = sp.Eq(-P * l * sp.sin(th) - N * l * sp.cos(th), I * thdd)

    res3 = sp.expand((eq1.lhs - eq1.rhs).subs(N, eq2.rhs))          # (3) = 0
    res3_ctms = (M + m) * xdd + b * xd + m * l * thdd * sp.cos(th) - m * l * thd**2 * sp.sin(th) - F
    check("(1)+(2) -> (3)", res3 - res3_ctms)

    res6 = sp.expand(l * (eq4.lhs - eq4.rhs) + (eq5.lhs - eq5.rhs))  # khử N, P
    res6_ctms = (I + m * l**2) * thdd + m * g * l * sp.sin(th) + m * l * xdd * sp.cos(th)
    check("l*(4)+(5) -> (6), không còn N, P", res6 + res6_ctms)

    # vị trí trọng tâm con lắc (theta tính từ phương thẳng đứng hướng xuống, y hướng lên)
    xp = x + l * sp.sin(th)
    yp = -l * sp.cos(th)
    N_kin = m * xp.diff(t, 2)
    P_kin = m * g + m * yp.diff(t, 2)
    check("(2) đúng là m*d2(x_p)/dt2", N_kin - eq2.rhs)
    check("(4) là đồng nhất thức động học với N, P = m*g + m*d2(y_p)/dt2",
          (eq4.lhs - eq4.rhs).subs({N: N_kin, P: P_kin}))

    print("2) Lagrange (kiểm tra độc lập)")
    T = sp.Rational(1, 2) * M * xd**2 + sp.Rational(1, 2) * m * (xp.diff(t) ** 2 + yp.diff(t) ** 2) \
        + sp.Rational(1, 2) * I * thd**2
    V = m * g * yp
    L = T - V
    EL_x = sp.diff(L.diff(xd), t) - L.diff(x) - (F - b * xd)
    EL_th = sp.diff(L.diff(thd), t) - L.diff(th)
    check("Lagrange theo x  == (3)", EL_x - res3_ctms)
    check("Lagrange theo th == (6)", EL_th - res6_ctms)

    print("3) Giải tường minh x_dd, theta_dd")
    sol = sp.solve([sub(res3_ctms), sub(res6_ctms)], [X3, T3], dict=True)[0]
    c, sn = sp.cos(T1), sp.sin(T1)
    det = (M + m) * (I + m * l**2) - (m * l * c) ** 2
    r1 = F - b * X2 + m * l * T2**2 * sn
    r2 = -m * g * l * sn
    xdd_closed = ((I + m * l**2) * r1 - m * l * c * r2) / det
    thdd_closed = ((M + m) * r2 - m * l * c * r1) / det
    check("x_dd khớp accel_theta()", sol[X3] - xdd_closed)
    check("theta_dd khớp accel_theta()", sol[T3] - thdd_closed)

    print("4) Tuyến tính hóa quanh theta = pi")
    to_phi = {T1: sp.pi + phi, T2: phid}
    f = sp.Matrix([X2, sol[X3].subs(to_phi), phid, sol[T3].subs(to_phi)])
    st = sp.Matrix([X1, X2, phi, phid])
    eq0 = {X1: 0, X2: 0, phi: 0, phid: 0, F: 0}
    A_sym = f.jacobian(st).subs(eq0)
    B_sym = f.diff(F).subs(eq0)
    A_ctms = sp.Matrix([[0, 1, 0, 0],
                        [0, -(I + m * l**2) * b / p, m**2 * g * l**2 / p, 0],
                        [0, 0, 0, 1],
                        [0, -m * l * b / p, m * g * l * (M + m) / p, 0]])
    B_ctms = sp.Matrix([0, (I + m * l**2) / p, 0, m * l / p])
    check("p == q", p - q)
    check("Jacobian -> A (20)", A_sym - A_ctms)
    check("Jacobian -> B (20)", B_sym - B_ctms)

    # (10), (11) từ (6), (3) với cos th ~ -1, sin th ~ -phi, th_d^2 ~ 0
    lin = {sp.cos(T1): -1, sp.sin(T1): -phi, T2: 0}
    e10 = sub(res6_ctms).subs(lin).subs({T3: phidd})
    e11 = sub(res3_ctms).subs(lin).subs({T3: phidd, F: u})
    e10_ctms = (I + m * l**2) * phidd - m * g * l * phi - m * l * X3
    e11_ctms = (M + m) * X3 + b * X2 - m * l * phidd - u
    check("(6) tuyến tính -> (10)", e10 - e10_ctms)
    check("(3) tuyến tính -> (11)", e11 - e11_ctms)

    print("5) Laplace & hàm truyền")
    Xs, Phis, Us = sp.symbols("X Phi U")
    e12 = sp.Eq((I + m * l**2) * Phis * s**2 - m * g * l * Phis, m * l * Xs * s**2)
    e13 = sp.Eq((M + m) * Xs * s**2 + b * Xs * s - m * l * Phis * s**2, Us)
    solL = sp.solve([e12, e13], [Xs, Phis], dict=True)[0]
    den4 = s**4 + b * (I + m * l**2) / q * s**3 - (M + m) * m * g * l / q * s**2 - b * m * g * l / q * s
    den3 = s**3 + b * (I + m * l**2) / q * s**2 - (M + m) * m * g * l / q * s - b * m * g * l / q
    P_pend_16 = (m * l / q * s**2) / den4
    P_pend_18 = (m * l / q * s) / den3
    P_cart_19 = (((I + m * l**2) * s**2 - g * m * l) / q) / den4
    check("(12),(13) -> P_pend (16)", solL[Phis] / Us - P_pend_16)
    check("(16) triệt s -> (18)", P_pend_16 - P_pend_18)
    check("(12),(13) -> P_cart (19)", solL[Xs] / Us - P_cart_19)
    X_of_Phi = ((I + m * l**2) / (m * l) - g / s**2) * Phis
    check("(12) -> (14) X(s) theo Phi(s)", solL[Xs] - X_of_Phi.subs(Phis, solL[Phis]))
    C = sp.Matrix([[1, 0, 0, 0], [0, 0, 1, 0]])
    G = sp.simplify(C * (s * sp.eye(4) - A_ctms).inv() * B_ctms)
    check("C(sI-A)^-1 B [x]   == (19)", G[0] - P_cart_19)
    check("C(sI-A)^-1 B [phi] == (18)", G[1] - P_pend_18)

    print("6) Thay số CTMS (M=0.5, m=0.2, b=0.1, l=0.3, I=0.006, g=9.8)")
    num = {M: sp.Rational(1, 2), m: sp.Rational(1, 5), b: sp.Rational(1, 10),
           l: sp.Rational(3, 10), I: sp.Rational(6, 1000), g: sp.Rational(98, 10)}
    print("  q = p =", sp.nsimplify(q.subs(num)), "=", sp.N(q.subs(num), 6))
    print("  A =", sp.N(A_ctms.subs(num), 4).tolist())
    print("  B =", sp.N(B_ctms.subs(num), 4).T.tolist())
    print("  P_pend(s) =", monic_tf(P_pend_18.subs(num)))
    print("  P_cart(s) =", monic_tf(P_cart_19.subs(num)))

    print("\n=> TẤT CẢ OK" if ALL_OK else "\n=> CÓ LỖI")

    if show_latex:
        print("\n---- LaTeX ----")
        disp = {xdd: sp.Symbol(r"\ddot{x}"), thdd: sp.Symbol(r"\ddot{\theta}"),
                xd: sp.Symbol(r"\dot{x}"), thd: sp.Symbol(r"\dot{\theta}"),
                x: sp.Symbol("x"), th: sp.Symbol(r"\theta")}
        disp2 = {X2: sp.Symbol(r"\dot{x}"), T1: sp.Symbol(r"\theta"), T2: sp.Symbol(r"\dot{\theta}")}
        qs = sp.Symbol("q")
        den3q = s**3 + b * (I + m * l**2) / qs * s**2 - (M + m) * m * g * l / qs * s - b * m * g * l / qs
        items = {
            "(3)": sp.Eq((res3_ctms + F).subs(disp), F),
            "(6)": sp.Eq((I + m * l**2) * thdd + m * g * l * sp.sin(th), -m * l * xdd * sp.cos(th)).subs(disp),
            "x_dd": sp.Eq(sp.Symbol(r"\ddot{x}"), xdd_closed.subs(disp2)),
            "theta_dd": sp.Eq(sp.Symbol(r"\ddot{\theta}"), thdd_closed.subs(disp2)),
            "A": A_ctms, "B": B_ctms,
            "P_pend (18)": (m * l / qs * s) / den3q,
            "P_cart (19)": (((I + m * l**2) * s**2 - g * m * l) / qs) / (s * den3q),
        }
        for k, v in items.items():
            print(f"{k}: {sp.latex(v)}")


def monic_tf(expr, digits=4):
    """Chuỗi 'tử / mẫu' với mẫu monic theo s."""
    n, d = sp.fraction(sp.cancel(sp.together(expr)))
    lc = sp.Poly(d, s).LC()
    n, d = sp.expand(n / lc), sp.expand(d / lc)
    return f"({sp.N(n, digits)}) / ({sp.N(d, digits)})"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--latex", action="store_true")
    main(ap.parse_args().latex)
