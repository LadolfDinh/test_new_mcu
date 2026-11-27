# Mô hình con lắc ngược trên xe (cart–pendulum)

Nguồn: CTMS, Univ. of Michigan – [Inverted Pendulum: System Modeling](https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling).
Số thứ tự phương trình (1)–(21) theo thứ tự trên trang. Các phương trình trang hiển thị bằng ảnh đã được viết lại và **kiểm tra bằng sympy** (`pendulum_symbolic.py`, suy dẫn lại bằng cả Newton–Euler và Lagrange).

| File | Nội dung |
|---|---|
| `README.md` | Toàn bộ phương trình (LaTeX + text thuần) và số liệu |
| `pendulum_model.py` | Code numpy: phi tuyến, tuyến tính A/B/C/D, hàm truyền, ZOH, mô phỏng RK4, tự kiểm tra |
| `pendulum_symbolic.py` | Suy dẫn ký hiệu (sympy), kiểm tra mọi công thức, in LaTeX (`--latex`) |
| `pendulum_model.m` | Bản MATLAB: `tf`, `ss`, `c2d`, mô phỏng phi tuyến bằng `ode45` |
| `pendulum_data.json` | Số liệu sinh từ `python pendulum_model.py --json pendulum_data.json` |

---

## 1. Ký hiệu và thông số

| Ký hiệu | Ý nghĩa | Giá trị CTMS |
|---|---|---|
| $M$ | khối lượng xe | 0.5 kg |
| $m$ | khối lượng con lắc | 0.2 kg |
| $b$ | hệ số ma sát của xe | 0.1 N/(m/s) |
| $l$ | khoảng cách khớp quay → trọng tâm con lắc | 0.3 m |
| $I$ | mô men quán tính con lắc quanh trọng tâm | 0.006 kg·m² |
| $g$ | gia tốc trọng trường | 9.8 m/s² |
| $F$ | lực ngang tác dụng lên xe (chiều $+x$) | đầu vào |
| $x$ | vị trí xe | |
| $\theta$ | góc con lắc, tính từ phương thẳng đứng **hướng xuống** | |
| $\phi$ | lệch khỏi vị trí thẳng đứng hướng lên: $\theta = \pi + \phi$ | |
| $N,\ P$ | lực liên kết ngang / dọc tại khớp xe–con lắc | |

Trọng tâm con lắc: $x_p = x + l\sin\theta,\quad y_p = -l\cos\theta$ (trục $y$ hướng lên).
Với $\theta=\pi+\phi$: $x_p = x - l\sin\phi$ → **$\phi>0$ là con lắc nghiêng về phía $-x$**.

## 2. Phân tích lực – phương trình phi tuyến

Xe, phương ngang:
$$M\ddot{x} + b\dot{x} + N = F \tag{1}$$

Con lắc, phương ngang:
$$N = m\ddot{x} + ml\ddot{\theta}\cos\theta - ml\dot{\theta}^2\sin\theta \tag{2}$$

Thế (2) vào (1) → **phương trình chuyển động thứ nhất**:
$$(M+m)\ddot{x} + b\dot{x} + ml\ddot{\theta}\cos\theta - ml\dot{\theta}^2\sin\theta = F \tag{3}$$

Con lắc, phương vuông góc với thanh:
$$P\sin\theta + N\cos\theta - mg\sin\theta = ml\ddot{\theta} + m\ddot{x}\cos\theta \tag{4}$$

Mô men quanh trọng tâm con lắc:
$$-Pl\sin\theta - Nl\cos\theta = I\ddot{\theta} \tag{5}$$

Khử $P, N$ ($l\cdot$(4) + (5)) → **phương trình chuyển động thứ hai**:
$$(I+ml^2)\ddot{\theta} + mgl\sin\theta = -ml\ddot{x}\cos\theta \tag{6}$$

Lực dọc tại khớp (không có trên trang, suy từ $P - mg = m\ddot{y}_p$):
$$P = mg + ml\ddot{\theta}\sin\theta + ml\dot{\theta}^2\cos\theta$$

### Dạng tường minh để mô phỏng (giải (3), (6))

$$\begin{bmatrix} M+m & ml\cos\theta \\ ml\cos\theta & I+ml^2 \end{bmatrix}
\begin{bmatrix} \ddot{x} \\ \ddot{\theta} \end{bmatrix} =
\begin{bmatrix} F - b\dot{x} + ml\dot{\theta}^2\sin\theta \\ -mgl\sin\theta \end{bmatrix}$$

$$\Delta(\theta) = (M+m)(I+ml^2) - m^2l^2\cos^2\theta$$

$$\ddot{x} = \frac{(I+ml^2)\left(F - b\dot{x} + ml\dot{\theta}^2\sin\theta\right) + m^2l^2g\sin\theta\cos\theta}{\Delta(\theta)}$$

$$\ddot{\theta} = \frac{-(M+m)mgl\sin\theta - ml\cos\theta\left(F - b\dot{x} + ml\dot{\theta}^2\sin\theta\right)}{\Delta(\theta)}$$

Viết theo $\phi$ ($\sin\theta=-\sin\phi$, $\cos\theta=-\cos\phi$, $\ddot\phi=\ddot\theta$):
$$(M+m)\ddot{x} + b\dot{x} - ml\ddot{\phi}\cos\phi + ml\dot{\phi}^2\sin\phi = F$$
$$(I+ml^2)\ddot{\phi} - mgl\sin\phi = ml\ddot{x}\cos\phi$$

## 3. Tuyến tính hóa quanh $\theta = \pi$ ($\phi \approx 0$)

$$\cos\theta = \cos(\pi+\phi) \approx -1 \tag{7}$$
$$\sin\theta = \sin(\pi+\phi) \approx -\phi \tag{8}$$
$$\dot{\theta}^2 = \dot{\phi}^2 \approx 0 \tag{9}$$

Thay vào (6), (3), đặt $u = F$:
$$(I+ml^2)\ddot{\phi} - mgl\phi = ml\ddot{x} \tag{10}$$
$$(M+m)\ddot{x} + b\dot{x} - ml\ddot{\phi} = u \tag{11}$$

## 4. Hàm truyền

Biến đổi Laplace (điều kiện đầu bằng 0):
$$(I+ml^2)\Phi(s)s^2 - mgl\Phi(s) = mlX(s)s^2 \tag{12}$$
$$(M+m)X(s)s^2 + bX(s)s - ml\Phi(s)s^2 = U(s) \tag{13}$$

Từ (12):
$$X(s) = \left[\frac{I+ml^2}{ml} - \frac{g}{s^2}\right]\Phi(s) \tag{14}$$

Thế vào (13):
$$(M+m)\left[\frac{I+ml^2}{ml} - \frac{g}{s^2}\right]\Phi(s)s^2 + b\left[\frac{I+ml^2}{ml} - \frac{g}{s^2}\right]\Phi(s)s - ml\Phi(s)s^2 = U(s) \tag{15}$$

$$P_{pend}(s) = \frac{\Phi(s)}{U(s)} = \frac{\frac{ml}{q}s^2}{s^4 + \frac{b(I+ml^2)}{q}s^3 - \frac{(M+m)mgl}{q}s^2 - \frac{bmgl}{q}s} \tag{16}$$

$$q = (M+m)(I+ml^2) - (ml)^2 \tag{17}$$

Triệt cực và zero tại gốc:
$$P_{pend}(s) = \frac{\frac{ml}{q}s}{s^3 + \frac{b(I+ml^2)}{q}s^2 - \frac{(M+m)mgl}{q}s - \frac{bmgl}{q}} \quad \left[\frac{rad}{N}\right] \tag{18}$$

$$P_{cart}(s) = \frac{X(s)}{U(s)} = \frac{\frac{(I+ml^2)s^2 - gml}{q}}{s^4 + \frac{b(I+ml^2)}{q}s^3 - \frac{(M+m)mgl}{q}s^2 - \frac{bmgl}{q}s} \quad \left[\frac{m}{N}\right] \tag{19}$$

## 5. Không gian trạng thái

Trạng thái $[x,\ \dot{x},\ \phi,\ \dot{\phi}]^T$, đầu vào $u$, đầu ra $[x,\ \phi]^T$:

$$\begin{bmatrix}\dot{x}\\ \ddot{x}\\ \dot{\phi}\\ \ddot{\phi}\end{bmatrix} =
\begin{bmatrix}
0 & 1 & 0 & 0\\
0 & \frac{-(I+ml^2)b}{p} & \frac{m^2gl^2}{p} & 0\\
0 & 0 & 0 & 1\\
0 & \frac{-mlb}{p} & \frac{mgl(M+m)}{p} & 0
\end{bmatrix}
\begin{bmatrix}x\\ \dot{x}\\ \phi\\ \dot{\phi}\end{bmatrix} +
\begin{bmatrix}0\\ \frac{I+ml^2}{p}\\ 0\\ \frac{ml}{p}\end{bmatrix}u \tag{20}$$

$$y = \begin{bmatrix}1&0&0&0\\0&0&1&0\end{bmatrix}\begin{bmatrix}x\\ \dot{x}\\ \phi\\ \dot{\phi}\end{bmatrix} + \begin{bmatrix}0\\0\end{bmatrix}u \tag{21}$$

$$p = I(M+m) + Mml^2 \qquad (\text{chú ý: } p = q)$$

## 6. Số liệu với thông số CTMS

$q = p = 0.0132$

$$A = \begin{bmatrix}0&1&0&0\\0&-0.1818&2.6727&0\\0&0&0&1\\0&-0.4545&31.1818&0\end{bmatrix},\quad
B = \begin{bmatrix}0\\1.8182\\0\\4.5455\end{bmatrix}$$

$$P_{pend}(s) = \frac{4.5455\,s}{s^3 + 0.1818s^2 - 31.1818s - 4.4545}$$

$$P_{cart}(s) = \frac{1.8182s^2 - 44.5455}{s^4 + 0.1818s^3 - 31.1818s^2 - 4.4545s}$$

| | Giá trị |
|---|---|
| Cực hệ hở (trị riêng $A$) | $-5.6041,\ -0.1428,\ 0,\ +5.5651$ (cực $+5.5651$ → không ổn định) |
| Zero $P_{pend}$ | $0$ |
| Zero $P_{cart}$ | $\pm 4.9497$ |
| Rời rạc ZOH $T_s = 2$ ms | $A_d, B_d$ trong `pendulum_data.json` (`discrete_zoh`) |

Khớp với số MATLAB in trên trang CTMS (A: −0.1818, 2.673, −0.4545, 31.18; B: 1.818, 4.545).

## 7. Dạng text thuần (copy vào code)

```text
# thông số
M=0.5; m=0.2; b=0.1; l=0.3; I=0.006; g=9.8
q = (M+m)*(I+m*l^2) - (m*l)^2          # = p = I*(M+m) + M*m*l^2

# phi tuyến (th tính từ phương thẳng đứng hướng xuống)
(1) M*x_dd + b*x_d + N = F
(2) N = m*x_dd + m*l*th_dd*cos(th) - m*l*th_d^2*sin(th)
(3) (M+m)*x_dd + b*x_d + m*l*th_dd*cos(th) - m*l*th_d^2*sin(th) = F
(4) P*sin(th) + N*cos(th) - m*g*sin(th) = m*l*th_dd + m*x_dd*cos(th)
(5) -P*l*sin(th) - N*l*cos(th) = I*th_dd
(6) (I + m*l^2)*th_dd + m*g*l*sin(th) = -m*l*x_dd*cos(th)
    P = m*g + m*l*th_dd*sin(th) + m*l*th_d^2*cos(th)

# dạng tường minh
D     = (M+m)*(I+m*l^2) - (m*l*cos(th))^2
r1    = F - b*x_d + m*l*th_d^2*sin(th)
r2    = -m*g*l*sin(th)
x_dd  = ((I+m*l^2)*r1 - m*l*cos(th)*r2) / D
th_dd = ((M+m)*r2   - m*l*cos(th)*r1) / D

# tuyến tính hóa: th = pi + phi, cos(th) ~ -1, sin(th) ~ -phi, th_d^2 ~ 0
(10) (I + m*l^2)*phi_dd - m*g*l*phi = m*l*x_dd
(11) (M+m)*x_dd + b*x_d - m*l*phi_dd = u

# hàm truyền
(18) Phi/U = (m*l/q)*s / (s^3 + b*(I+m*l^2)/q*s^2 - (M+m)*m*g*l/q*s - b*m*g*l/q)
(19) X/U   = ((I+m*l^2)*s^2 - g*m*l)/q / (s^4 + b*(I+m*l^2)/q*s^3 - (M+m)*m*g*l/q*s^2 - b*m*g*l/q*s)

# không gian trạng thái, x_vec = [x, x_d, phi, phi_d]
A = [[0, 1,                  0,                0],
     [0, -(I+m*l^2)*b/p,     m^2*g*l^2/p,      0],
     [0, 0,                  0,                1],
     [0, -m*l*b/p,           m*g*l*(M+m)/p,    0]]
B = [0, (I+m*l^2)/p, 0, m*l/p]^T
C = [[1,0,0,0],[0,0,1,0]];  D = [0,0]^T
```

## 8. Dùng nhanh

```bash
python pendulum_model.py                          # in số liệu + tự kiểm tra
python pendulum_model.py --json pendulum_data.json --Ts 0.002
python pendulum_model.py --plot                   # phi tuyến vs tuyến tính (cần matplotlib)
python pendulum_symbolic.py --latex               # kiểm tra ký hiệu + in LaTeX
```

```python
from pendulum_model import PendulumParams, linear_ss, tf_coeffs, c2d_zoh, simulate_nonlinear

par = PendulumParams(M=0.5, m=0.2, b=0.1, l=0.3, I=0.006)   # thay số thật của xe
A, B, C, D = linear_ss(par)
Ad, Bd = c2d_zoh(A, B, 0.002)
t, S, U = simulate_nonlinear(par, [0, 0, 0.05, 0], 1.0, 0.001)            # F = 0, phi0 = 0.05 rad
# gắn bộ điều khiển: force(t, state) -> F, giữ không đổi trong 1 bước dt (giống MCU)
# t, S, U = simulate_nonlinear(par, s0, 3.0, 0.002, force=lambda t, s: my_controller(s))
```

## Lưu ý khi dùng cho xe 2 bánh tự cân bằng

Đây là mô hình xe đẩy + con lắc với đầu vào là **lực** $F$. Xe 2 bánh có thêm khối lượng/quán tính bánh xe và đầu vào là **mô men động cơ**, nên hệ số A, B sẽ khác; dùng mô hình này để kiểm tra nhanh, rồi thay thông số và (nếu cần) mô hình bánh xe thật của robot.
