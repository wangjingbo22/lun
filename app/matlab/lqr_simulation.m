%% LQR 轮腿机器人闭环仿真
% 本脚本用于仿真验证 LQR 控制器效果
% 需要先运行 lqr_1_20.m 生成 get_A_matrix.m 和 get_B_matrix.m
clear; clc; close all;

%% 1. 物理参数定义 (与 lqr_1_20.m 保持一致)
% -----------------------------------------------------------
val_R_num  = 0.044;          
val_Mw_num = 0.260;          
val_M_num  = 1.075;          
val_mp_num = 0.065;          
val_g_num  = 9.81;

% 惯量预计算
val_Iw_num = 0.5 * val_Mw_num * val_R_num^2;               
val_IM_num = val_M_num * (0.1^2 + 0.15^2) / 12;    
val_Ip_num = val_mp_num * (0.1)^2 / 12;            

%% 2. 选择腿长并计算 A, B, K 矩阵
% -----------------------------------------------------------
L0 = 0.10;  % 腿长 (米), 可以修改测试不同腿长

% 参数准备
p_L = L0 / 2;
p_LM = L0 / 2;
p_l = 0;

% 检查函数是否存在
if ~exist('get_A_matrix', 'file') || ~exist('get_B_matrix', 'file')
    error('请先运行 lqr_1_20.m 生成 get_A_matrix.m 和 get_B_matrix.m');
end

% 获取状态空间矩阵
A = double(get_A_matrix(p_L, p_LM, val_mp_num, val_Mw_num, val_M_num, ...
                        val_Ip_num, val_Iw_num, val_IM_num, val_R_num, val_g_num, p_l));
B = double(get_B_matrix(p_L, p_LM, val_mp_num, val_Mw_num, val_M_num, ...
                        val_Ip_num, val_Iw_num, val_IM_num, val_R_num, val_g_num, p_l));

% 输出矩阵 C (我们观测所有状态)
C = eye(6);
D = zeros(6, 2);

% 显示系统信息
fprintf('========== 系统信息 (L0 = %.2f m) ==========\n', L0);
fprintf('A 矩阵:\n'); disp(A);
fprintf('B 矩阵:\n'); disp(B);

% 检查能控性
Co = ctrb(A, B);
if rank(Co) == 6
    fprintf('✓ 系统完全能控\n');
else
    fprintf('✗ 警告：系统不完全能控！rank = %d\n', rank(Co));
end

% 开环极点
fprintf('\n开环极点:\n');
eig_open = eig(A);
disp(eig_open);
if any(real(eig_open) > 0)
    fprintf('✗ 开环系统不稳定（存在正实部极点）\n');
else
    fprintf('✓ 开环系统稳定\n');
end

%% 3. LQR 控制器设计
% -----------------------------------------------------------
% Q 和 R 权重 (与 lqr_1_20.m 保持一致)
% 状态顺序: [theta, d_theta, x, d_x, phi, d_phi]
Q = diag([300, 5, 10, 1, 150, 1]); 
R = diag([1.5, 1.5]);

% 计算 LQR 增益
[K, S, e] = lqr(A, B, Q, R);

fprintf('\n========== LQR 控制器 ==========\n');
fprintf('K 矩阵 (2x6):\n');
fprintf('  轮子: [%8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f]\n', K(1,:));
fprintf('  髋关节: [%8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f]\n', K(2,:));

fprintf('\n闭环极点:\n');
eig_closed = eig(A - B*K);
disp(eig_closed);
if all(real(eig_closed) < 0)
    fprintf('✓ 闭环系统稳定\n');
else
    fprintf('✗ 闭环系统不稳定！\n');
end

%% 4. 仿真参数设置
% -----------------------------------------------------------
t_end = 5;          % 仿真时长 (秒)
dt = 0.001;         % 仿真步长 (秒)
t = 0:dt:t_end;

% 初始条件 (可以修改测试不同情况)
% 状态顺序: [theta, d_theta, x, d_x, phi, d_phi]
x0 = [0.1;    % theta: 摆杆初始倾角 0.1 rad ≈ 5.7°
      0;      % d_theta: 摆杆初始角速度
      0;      % x: 初始位移
      0;      % d_x: 初始速度
      0.05;   % phi: 机体初始倾角 0.05 rad ≈ 2.9°
      0];     % d_phi: 机体初始角速度

fprintf('\n========== 仿真参数 ==========\n');
fprintf('仿真时长: %.1f s\n', t_end);
fprintf('初始状态:\n');
fprintf('  theta = %.3f rad (%.1f°)\n', x0(1), rad2deg(x0(1)));
fprintf('  phi   = %.3f rad (%.1f°)\n', x0(5), rad2deg(x0(5)));

%% 5. 方法1: 线性系统仿真 (lsim)
% -----------------------------------------------------------
fprintf('\n正在进行线性仿真...\n');

% 构建闭环系统: x_dot = (A-BK)x
A_cl = A - B*K;
sys_cl = ss(A_cl, zeros(6,1), C, zeros(6,1));

% 使用 initial 函数仿真初始条件响应
[y_linear, t_linear, x_linear] = initial(sys_cl, x0, t);

%% 6. 方法2: 非线性仿真 (ode45) - 更精确
% -----------------------------------------------------------
fprintf('正在进行非线性仿真 (含输入饱和)...\n');

% 控制输入限幅
T_max = 0.5;   % 轮子最大力矩 (N·m)
Tp_max = 0.3;  % 髋关节最大力矩 (N·m)

% 非线性动力学函数
dynamics = @(t, x) nonlinear_dynamics(t, x, A, B, K, T_max, Tp_max);

% 使用 ode45 求解
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t_nonlin, x_nonlin] = ode45(dynamics, [0 t_end], x0, options);

% 计算控制输入历史
u_history = zeros(length(t_nonlin), 2);
for i = 1:length(t_nonlin)
    u = -K * x_nonlin(i,:)';
    u(1) = max(min(u(1), T_max), -T_max);
    u(2) = max(min(u(2), Tp_max), -Tp_max);
    u_history(i,:) = u';
end

%% 7. 绘制仿真结果
% -----------------------------------------------------------
state_names = {'θ (摆杆角)', 'dθ (摆杆角速度)', 'x (位移)', 'dx (速度)', 'φ (机体角)', 'dφ (机体角速度)'};
state_units = {'rad', 'rad/s', 'm', 'm/s', 'rad', 'rad/s'};

% 图1: 状态响应对比
figure('Name', '状态响应对比', 'Position', [50, 50, 1400, 900]);
for i = 1:6
    subplot(3, 2, i);
    plot(t_linear, x_linear(:,i), 'b-', 'LineWidth', 1.5, 'DisplayName', '线性仿真');
    hold on;
    plot(t_nonlin, x_nonlin(:,i), 'r--', 'LineWidth', 1.5, 'DisplayName', '非线性(含饱和)');
    xlabel('时间 (s)');
    ylabel([state_names{i}, ' (', state_units{i}, ')']);
    title(state_names{i});
    legend('Location', 'best');
    grid on;
end
sgtitle(sprintf('LQR 闭环响应 (L0 = %.2f m)', L0));

% 图2: 控制输入
figure('Name', '控制输入', 'Position', [100, 100, 1000, 400]);
subplot(1,2,1);
plot(t_nonlin, u_history(:,1), 'b-', 'LineWidth', 1.5);
hold on;
yline(T_max, 'r--', '饱和上限');
yline(-T_max, 'r--', '饱和下限');
xlabel('时间 (s)');
ylabel('T (N·m)');
title('轮子电机力矩');
grid on;

subplot(1,2,2);
plot(t_nonlin, u_history(:,2), 'b-', 'LineWidth', 1.5);
hold on;
yline(Tp_max, 'r--', '饱和上限');
yline(-Tp_max, 'r--', '饱和下限');
xlabel('时间 (s)');
ylabel('Tp (N·m)');
title('髋关节力矩');
grid on;

sgtitle('控制输入历史');

% 图3: 相平面图
figure('Name', '相平面', 'Position', [150, 150, 1200, 400]);
subplot(1,3,1);
plot(x_nonlin(:,1), x_nonlin(:,2), 'b-', 'LineWidth', 1.5);
hold on;
plot(x0(1), x0(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(0, 0, 'r*', 'MarkerSize', 15);
xlabel('θ (rad)');
ylabel('dθ (rad/s)');
title('摆杆相平面');
legend('轨迹', '起点', '目标', 'Location', 'best');
grid on;

subplot(1,3,2);
plot(x_nonlin(:,3), x_nonlin(:,4), 'b-', 'LineWidth', 1.5);
hold on;
plot(x0(3), x0(4), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(0, 0, 'r*', 'MarkerSize', 15);
xlabel('x (m)');
ylabel('dx (m/s)');
title('位移相平面');
legend('轨迹', '起点', '目标', 'Location', 'best');
grid on;

subplot(1,3,3);
plot(x_nonlin(:,5), x_nonlin(:,6), 'b-', 'LineWidth', 1.5);
hold on;
plot(x0(5), x0(6), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(0, 0, 'r*', 'MarkerSize', 15);
xlabel('φ (rad)');
ylabel('dφ (rad/s)');
title('机体相平面');
legend('轨迹', '起点', '目标', 'Location', 'best');
grid on;

sgtitle('相平面图');

% 图4: 3D 动画轨迹
figure('Name', '3D轨迹', 'Position', [200, 200, 800, 600]);
plot3(x_nonlin(:,1), x_nonlin(:,3), x_nonlin(:,5), 'b-', 'LineWidth', 1.5);
hold on;
plot3(x0(1), x0(3), x0(5), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
plot3(0, 0, 0, 'r*', 'MarkerSize', 20);
xlabel('θ (rad)');
ylabel('x (m)');
zlabel('φ (rad)');
title('状态空间3D轨迹');
legend('轨迹', '起点', '目标');
grid on;
view(45, 30);

%% 8. 性能指标分析
% -----------------------------------------------------------
fprintf('\n========== 性能指标 ==========\n');

% 调节时间 (2%准则)
settling_threshold = 0.02;
for i = [1, 3, 5]  % 只分析角度和位移
    settled_idx = find(abs(x_nonlin(:,i)) > settling_threshold * abs(x0(i)), 1, 'last');
    if isempty(settled_idx)
        ts = 0;
    else
        ts = t_nonlin(settled_idx);
    end
    fprintf('%s 调节时间 (2%%): %.3f s\n', state_names{i}, ts);
end

% 最大控制输入
fprintf('\n最大轮子力矩: %.4f N·m\n', max(abs(u_history(:,1))));
fprintf('最大髋关节力矩: %.4f N·m\n', max(abs(u_history(:,2))));

% 能量消耗 (积分 u'*R*u)
energy = 0;
for i = 2:length(t_nonlin)
    dt_i = t_nonlin(i) - t_nonlin(i-1);
    energy = energy + u_history(i,:) * R * u_history(i,:)' * dt_i;
end
fprintf('控制能量消耗: %.4f\n', energy);

fprintf('\n仿真完成!\n');

%% 非线性动力学函数 (含输入饱和)
function dx = nonlinear_dynamics(~, x, A, B, K, T_max, Tp_max)
    % 计算控制输入
    u = -K * x;
    
    % 输入饱和
    u(1) = max(min(u(1), T_max), -T_max);
    u(2) = max(min(u(2), Tp_max), -Tp_max);
    
    % 状态方程 (线性化模型)
    dx = A * x + B * u;
end
