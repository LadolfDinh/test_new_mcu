%% Mô hình con lắc ngược trên xe - CTMS "Inverted Pendulum: System Modeling"
% theta : góc con lắc tính từ phương thẳng đứng HƯỚNG XUỐNG; phi = theta - pi
%         (phi > 0: con lắc nghiêng về phía -x, vì x_p = x - l*sin(phi))
% Trạng thái [x; x_dot; phi; phi_dot], vào u = F (N), ra [x; phi]
% Cần Control System Toolbox (tf, ss, c2d, initial). MATLAB >= R2016b (hàm cục bộ trong script).
clear; clc;

%% Thông số (giá trị CTMS)
M = 0.5;     % kg      khối lượng xe
m = 0.2;     % kg      khối lượng con lắc
b = 0.1;     % N/m/s   ma sát xe
I = 0.006;   % kg.m^2  mô men quán tính con lắc quanh trọng tâm
g = 9.8;     % m/s^2
l = 0.3;     % m       khớp quay -> trọng tâm con lắc
par = struct('M',M,'m',m,'b',b,'I',I,'g',g,'l',l);

%% Hàm truyền (16)-(19)
q = (M+m)*(I+m*l^2) - (m*l)^2;
s = tf('s');
den4 = s^4 + b*(I+m*l^2)/q*s^3 - (M+m)*m*g*l/q*s^2 - b*m*g*l/q*s;
den3 = s^3 + b*(I+m*l^2)/q*s^2 - (M+m)*m*g*l/q*s   - b*m*g*l/q;
P_cart = ((I+m*l^2)/q*s^2 - g*m*l/q) / den4;     % X(s)/U(s)   [m/N]
P_pend = (m*l/q*s) / den3;                       % Phi(s)/U(s) [rad/N] (đã triệt s)
sys_tf = [P_cart; P_pend];
sys_tf.InputName  = {'u'};
sys_tf.OutputName = {'x'; 'phi'};

%% Không gian trạng thái (20),(21)
p = I*(M+m) + M*m*l^2;                           % p == q
A = [0  1               0                0;
     0 -(I+m*l^2)*b/p   (m^2*g*l^2)/p    0;
     0  0               0                1;
     0 -(m*l*b)/p        m*g*l*(M+m)/p   0];
B = [0; (I+m*l^2)/p; 0; m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0; 0];
sys_ss = ss(A, B, C, D, 'StateName', {'x' 'x_dot' 'phi' 'phi_dot'}, ...
            'InputName', {'u'}, 'OutputName', {'x'; 'phi'});

sys_tf
sys_ss
disp('Trị riêng A (cực hệ hở):'); disp(eig(A).');

% kiểm tra: tf(ss) == tf giải tích (so đáp ứng tần số tại vài điểm)
w = [0.5 2 7];
err = abs(squeeze(freqresp(sys_ss, w)) - squeeze(freqresp(sys_tf, w)));
fprintf('Sai lệch ss <-> tf: %.3g\n', max(err(:)));

%% Rời rạc ZOH (chu kỳ điều khiển trên MCU, vd 2 ms)
Ts = 0.002;
sys_d = c2d(sys_ss, Ts, 'zoh')

%% Mô phỏng phi tuyến (ode45) vs tuyến tính, F = 0, phi0 = 0.05 rad
phi0  = 0.05;
Tend  = 1.0;
z0    = [0; 0; phi0; 0];
F_fun = @(t, z) 0;                               % thay bằng luật điều khiển nếu cần
opts  = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
[tn, zn] = ode45(@(t, z) pendulum_rhs_phi(t, z, F_fun(t, z), par), [0 Tend], z0, opts);
[~, tl, zl] = initial(sys_ss, z0, Tend);

figure('Name', 'Con lắc ngược: phi tuyến vs tuyến tính');
subplot(2,1,1); plot(tn, zn(:,3), tl, zl(:,3), '--'); grid on;
ylabel('\phi [rad]'); legend('phi tuyến', 'tuyến tính', 'Location', 'northwest');
subplot(2,1,2); plot(tn, zn(:,1), tl, zl(:,1), '--'); grid on;
ylabel('x [m]'); xlabel('t [s]');

%% ===================== Hàm cục bộ =====================
function dz = pendulum_rhs_phi(~, z, F, par)
% Phi tuyến, z = [x; x_dot; phi; phi_dot], theta = pi + phi
    [xdd, thdd] = pendulum_accel(pi + z(3), z(2), z(4), F, par);
    dz = [z(2); xdd; z(4); thdd];
end

function dz = pendulum_rhs_theta(~, z, F, par) %#ok<DEFNU>
% Phi tuyến, z = [x; x_dot; theta; theta_dot]
    [xdd, thdd] = pendulum_accel(z(3), z(2), z(4), F, par);
    dz = [z(2); xdd; z(4); thdd];
end

function [xdd, thdd] = pendulum_accel(th, xd, thd, F, par)
% Giải (3),(6):
%  [M+m        m*l*cos(th)] [xdd ]   [F - b*xd + m*l*thd^2*sin(th)]
%  [m*l*cos(th)  I + m*l^2] [thdd] = [-m*g*l*sin(th)              ]
    M = par.M; m = par.m; b = par.b; l = par.l; I = par.I; g = par.g;
    c = cos(th); sn = sin(th);
    a11 = M + m;  a12 = m*l*c;  a22 = I + m*l^2;
    r1 = F - b*xd + m*l*thd^2*sn;
    r2 = -m*g*l*sn;
    dt = a11*a22 - a12^2;
    xdd  = (a22*r1 - a12*r2) / dt;
    thdd = (a11*r2 - a12*r1) / dt;
end
