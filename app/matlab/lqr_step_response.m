%% LQR 轮腿机器人 - 速度阶跃响应仿真
% 给定速度阶跃指令，观察状态变量变化
% 需要先运行 lqr_1_20.m 生成 get_A_matrix.m 和 get_B_matrix.m
clear; clc; close all;

%% 1. 物理参数定义
% -----------------------------------------------------------
val_R_num  = 0.044;          
val_Mw_num = 0.260;          
val_M_num  = 1.075;          
val_mp_num = 0.065;          
val_g_num  = 9.81;

val_Iw_num = 0.5 * val_Mw_num * val_R_num^2;               
val_IM_num = val_M_num * (0.1^2 + 0.15^2) / 12;    
val_Ip_num = val_mp_num * (0.1)^2 / 12;            

%% 2. 系统矩阵
% -----------------------------------------------------------
L0 = 0.10;  % 腿长 (m)
p_L = L0 / 2;
p_LM = L0 / 2;
p_l = 0;

if ~exist('get_A_matrix', 'file') || ~exist('get_B_matrix', 'file')
    error('请先运行 lqr_1_20.m 生成 get_A_matrix.m 和 get_B_matrix.m');
end

A = double(get_A_matrix(p_L, p_LM, val_mp_num, val_Mw_num, val_M_num, ...
                        val_Ip_num, val_Iw_num, val_IM_num, val_R_num, val_g_num, p_l));
B = double(get_B_matrix(p_L, p_LM, val_mp_num, val_Mw_num, val_M_num, ...
                        val_Ip_num, val_Iw_num, val_IM_num, val_R_num, val_g_num, p_l));

% LQR 设计
Q = diag([300, 5, 10, 1, 150, 1]); 
R_lqr = diag([1.5, 1.5]);
[K, ~, ~] = lqr(A, B, Q, R_lqr);

fprintf('系统矩阵和LQR增益计算完成\n');

%% 3. 阶跃响应参数设置
% -----------------------------------------------------------
t_end = 8;              % 仿真时长 (s)
dt = 0.001;             % 时间步长 (s)
t = 0:dt:t_end;

% ========== 阶跃信号设置 ==========
% 可以修改这里测试不同的阶跃响应

% 速度阶跃
v_step_time = 0;        % 速度阶跃开始时间 (s)
v_step_end = 3;         % 速度阶跃结束时间 (s)
v_target = 1.5;         % 目标速度 (m/s)

% 位置阶跃 (如果想测试位置跟踪)
x_step_time = inf;      % 设为 inf 表示不使用位置阶跃
x_target = 0.5;         % 目标位置 (m)

% 角度阶跃 (如果想测试姿态保持)
phi_step_time = inf;    % 设为 inf 表示不使用
phi_target = 0.1;       % 目标机体角 (rad)

% ========== 控制限幅 ==========
T_max = 0.8;            % 轮子最大力矩 (N·m)
Tp_max = 0.5;           % 髋关节最大力矩 (N·m)

%% 4. 生成参考信号
% -----------------------------------------------------------
% 状态顺序: [theta, d_theta, x, d_x, phi, d_phi]
ref = zeros(length(t), 6);

for i = 1:length(t)
    % 速度参考 (阶跃)
    if t(i) >= v_step_time && t(i) < v_step_end
        ref(i, 4) = v_target;  % d_x 参考
    end
    
    % 位置参考 (阶跃)
    if t(i) >= x_step_time
        ref(i, 3) = x_target;  % x 参考
    end
    
    % 机体角参考 (阶跃)
    if t(i) >= phi_step_time
        ref(i, 5) = phi_target;  % phi 参考
    end
end

%% 5. 仿真 (欧拉积分)
% -----------------------------------------------------------
fprintf('正在进行仿真...\n');

% 状态初始化
x_state = zeros(length(t), 6);
x_state(1, :) = [0; 0; 0; 0; 0; 0]';  % 初始状态全为0

% 控制输入历史
u_history = zeros(length(t), 2);

for i = 1:length(t)-1
    % 当前状态
    x_curr = x_state(i, :)';
    
    % 当前参考
    r_curr = ref(i, :)';
    
    % 误差 = 参考 - 实际
    e = r_curr - x_curr;
    
    % LQR 控制律: u = K * e (注意这里是正号，因为 e = r - x)
    % 原始 LQR 是 u = -K * x，跟踪时是 u = -K * (x - r) = K * (r - x) = K * e
    u = K * e;
    
    % 输入饱和
    u(1) = max(min(u(1), T_max), -T_max);
    u(2) = max(min(u(2), Tp_max), -Tp_max);
    
    u_history(i, :) = u';
    
    % 状态更新 (欧拉法)
    dx = A * x_curr + B * u;
    x_state(i+1, :) = (x_curr + dx * dt)';
end

fprintf('仿真完成!\n');

%% 6. 绘制结果 (类似你图片的格式)
% -----------------------------------------------------------
figure('Name', '阶跃响应', 'Position', [100, 100, 800, 700]);

% 图1: phi (机体角)
subplot(3, 1, 1);
plot(t, x_state(:, 5), 'b-', 'LineWidth', 1.5);
hold on;
plot(t, ref(:, 5), 'r--', 'LineWidth', 1);
xlabel('t (s)');
ylabel('phi (rad)');
title('phi');
legend('phi', 'ref phi', 'Location', 'best');
grid on;
xlim([0, t_end]);

% 图2: theta (摆杆角)
subplot(3, 1, 2);
plot(t, x_state(:, 1), 'b-', 'LineWidth', 1.5);
hold on;
plot(t, ref(:, 1), 'r--', 'LineWidth', 1);
xlabel('t (s)');
ylabel('theta (rad)');
title('theta');
legend('theta', 'ref theta', 'Location', 'best');
grid on;
xlim([0, t_end]);

% 图3: xdot (速度)
subplot(3, 1, 3);
plot(t, ref(:, 4), 'b-', 'LineWidth', 1.5);
hold on;
plot(t, x_state(:, 4), 'r-', 'LineWidth', 1.5);
xlabel('t in s');
ylabel('xdot (m/s)');
title('xdot');
legend('ref xdot', 'xdot', 'Location', 'best');
grid on;
xlim([0, t_end]);

sgtitle('LQR 速度阶跃响应');

%% 7. 更多状态图
% -----------------------------------------------------------
figure('Name', '全状态响应', 'Position', [150, 150, 1200, 600]);

state_names = {'θ (摆杆角)', 'dθ (角速度)', 'x (位移)', 'dx (速度)', 'φ (机体角)', 'dφ (角速度)'};
state_units = {'rad', 'rad/s', 'm', 'm/s', 'rad', 'rad/s'};

for i = 1:6
    subplot(2, 3, i);
    plot(t, x_state(:, i), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(t, ref(:, i), 'r--', 'LineWidth', 1);
    xlabel('t (s)');
    ylabel([state_names{i}, ' (', state_units{i}, ')']);
    title(state_names{i});
    legend('实际', '参考', 'Location', 'best');
    grid on;
    xlim([0, t_end]);
end

sgtitle('全状态响应');

%% 8. 控制输入图
% -----------------------------------------------------------
figure('Name', '控制输入', 'Position', [200, 200, 800, 400]);

subplot(2, 1, 1);
plot(t, u_history(:, 1), 'b-', 'LineWidth', 1.5);
hold on;
yline(T_max, 'r--', 'LineWidth', 1);
yline(-T_max, 'r--', 'LineWidth', 1);
xlabel('t (s)');
ylabel('T (N·m)');
title('轮子电机力矩');
legend('T', '饱和限制');
grid on;
xlim([0, t_end]);

subplot(2, 1, 2);
plot(t, u_history(:, 2), 'b-', 'LineWidth', 1.5);
hold on;
yline(Tp_max, 'r--', 'LineWidth', 1);
yline(-Tp_max, 'r--', 'LineWidth', 1);
xlabel('t (s)');
ylabel('Tp (N·m)');
title('髋关节力矩');
legend('Tp', '饱和限制');
grid on;
xlim([0, t_end]);

sgtitle('控制输入历史');

%% 9. 性能分析
% -----------------------------------------------------------
fprintf('\n========== 性能分析 ==========\n');

% 找到速度阶跃期间的数据
step_idx = find(t >= v_step_time & t < v_step_end);
if ~isempty(step_idx)
    v_actual = x_state(step_idx, 4);
    v_ref = ref(step_idx, 4);
    
    % 上升时间 (10% -> 90%)
    v_10 = 0.1 * v_target;
    v_90 = 0.9 * v_target;
    t_10_idx = find(v_actual >= v_10, 1, 'first');
    t_90_idx = find(v_actual >= v_90, 1, 'first');
    if ~isempty(t_10_idx) && ~isempty(t_90_idx)
        rise_time = t(step_idx(t_90_idx)) - t(step_idx(t_10_idx));
        fprintf('速度上升时间 (10%%->90%%): %.3f s\n', rise_time);
    end
    
    % 最大超调
    overshoot = (max(v_actual) - v_target) / v_target * 100;
    fprintf('速度最大超调: %.1f%%\n', overshoot);
    
    % 稳态误差
    steady_idx = find(t >= 2 & t < v_step_end);
    if ~isempty(steady_idx)
        ss_error = mean(abs(v_ref(steady_idx - step_idx(1) + 1) - v_actual(steady_idx - step_idx(1) + 1)));
        fprintf('速度稳态误差: %.4f m/s\n', ss_error);
    end
end

% theta 和 phi 的最大值
fprintf('\ntheta 最大偏转: %.3f rad (%.1f°)\n', max(abs(x_state(:,1))), rad2deg(max(abs(x_state(:,1)))));
fprintf('phi 最大偏转: %.3f rad (%.1f°)\n', max(abs(x_state(:,5))), rad2deg(max(abs(x_state(:,5)))));

fprintf('\n仿真完成!\n');
