%% 根据你的机器人参数计算最优 Q/R 配置
clear; clc; close all;

%% 1. 你的机器人物理参数
% -----------------------------------------------------------
val_R_num  = 0.044;          % 轮子半径 44mm - 小型轮子
val_Mw_num = 0.260;          % 轮子质量 260g
val_M_num  = 1.075;          % 机体质量 1.075kg  
val_mp_num = 0.065;          % 摆杆质量 65g
val_g_num  = 9.81;

% 惯量
val_Iw_num = 0.5 * val_Mw_num * val_R_num^2;               
val_IM_num = val_M_num * (0.1^2 + 0.15^2) / 12;    
val_Ip_num = val_mp_num * (0.1)^2 / 12;            

% 腿长范围
L0_min = 0.08;   % 最短腿长 8cm
L0_max = 0.14;   % 最长腿长 14cm
L0 = 0.10;       % 标称腿长 10cm

fprintf('========================================\n');
fprintf('    你的机器人参数分析\n');
fprintf('========================================\n');
fprintf('轮子半径: %.1f mm (小型)\n', val_R_num*1000);
fprintf('总质量: %.2f kg (轻量级)\n', val_Mw_num + val_M_num + val_mp_num);
fprintf('腿长范围: %.0f ~ %.0f mm\n', L0_min*1000, L0_max*1000);

%% 2. 根据物理特性估计约束
% -----------------------------------------------------------
fprintf('\n========================================\n');
fprintf('    物理约束估计\n');
fprintf('========================================\n');

% === 状态量最大允许偏差 ===

% theta (摆杆角): 
% 小型机器人重心低，允许偏差可以稍大
% 但太大会失稳，取 8°
max_theta = deg2rad(8);  
fprintf('摆杆角 theta 最大允许: 8° (%.3f rad)\n', max_theta);

% d_theta (摆杆角速度):
% 估计: 如果1秒内从0到max_theta，大约 30°/s
max_dtheta = deg2rad(30);
fprintf('摆杆角速度 d_theta 最大允许: 30°/s (%.3f rad/s)\n', max_dtheta);

% x (位移):
% 平衡控制时不太关心绝对位置，允许较大偏移
% 但跟踪时需要，取 0.3m
max_x = 0.3;
fprintf('位移 x 最大允许: %.1f m\n', max_x);

% d_x (速度):
% 小轮子限制了最大速度，估计 v_max = ω_max * R
% 假设电机最大转速 3000rpm ≈ 314 rad/s
% v_max ≈ 314 * 0.044 ≈ 13.8 m/s (理论值，实际受限于电机力矩)
% 实际合理速度取 1.5 m/s
max_dx = 1.5;
fprintf('速度 d_x 最大允许: %.1f m/s\n', max_dx);

% phi (机体角):
% 这是最关键的！机体倾角太大会倒
% 小型机器人建议控制在 5° 以内
max_phi = deg2rad(5);
fprintf('机体角 phi 最大允许: 5° (%.3f rad) [关键!]\n', max_phi);

% d_phi (机体角速度):
% 估计 20°/s
max_dphi = deg2rad(20);
fprintf('机体角速度 d_phi 最大允许: 20°/s (%.3f rad/s)\n', max_dphi);

% === 输入量最大值 ===
fprintf('\n--- 执行器能力估计 ---\n');

% 轮子电机力矩:
% 小型轮毂电机，估计最大持续力矩 0.3-0.5 N·m
% 峰值可能更高，取 0.4 N·m 作为设计值
max_T = 0.4;
fprintf('轮子电机最大力矩: %.2f N·m\n', max_T);

% 髋关节力矩:
% 关节电机，估计 0.3 N·m
max_Tp = 0.3;
fprintf('髋关节最大力矩: %.2f N·m\n', max_Tp);

%% 3. 使用 Bryson 规则计算基础 Q/R
% -----------------------------------------------------------
fprintf('\n========================================\n');
fprintf('    Bryson 规则计算\n');
fprintf('========================================\n');

Q_bryson = diag([
    1/max_theta^2,   % theta
    1/max_dtheta^2,  % d_theta
    1/max_x^2,       % x
    1/max_dx^2,      % d_x  
    1/max_phi^2,     % phi
    1/max_dphi^2     % d_phi
]);

R_bryson = diag([
    1/max_T^2,       % T
    1/max_Tp^2       % Tp
]);

fprintf('Bryson Q 对角线: [%.1f, %.1f, %.1f, %.2f, %.1f, %.1f]\n', diag(Q_bryson));
fprintf('Bryson R 对角线: [%.1f, %.1f]\n', diag(R_bryson));

%% 4. 根据控制目标微调权重
% -----------------------------------------------------------
fprintf('\n========================================\n');
fprintf('    根据控制目标微调\n');
fprintf('========================================\n');

% 你的机器人是轮腿平衡机器人，主要目标:
% 1. 保持机体稳定 (phi 最重要)
% 2. 保持腿部稳定 (theta 重要)
% 3. 速度跟踪 (dx 中等)
% 4. 位置次要 (x 不太重要)

% 相对权重调整
w_theta  = 1.5;   % 摆杆角度 - 重要
w_dtheta = 0.3;   % 摆杆角速度 - 允许快速响应
w_x      = 0.3;   % 位移 - 不太重要
w_dx     = 0.5;   % 速度 - 跟踪需要
w_phi    = 2.0;   % 机体角度 - 最重要!
w_dphi   = 0.3;   % 机体角速度 - 允许快速响应

w_T  = 1.0;       % 轮子力矩 - 标准
w_Tp = 1.2;       % 髋关节力矩 - 稍微省一点

fprintf('权重调整:\n');
fprintf('  theta:%.1f, d_theta:%.1f, x:%.1f, d_x:%.1f, phi:%.1f, d_phi:%.1f\n', ...
    w_theta, w_dtheta, w_x, w_dx, w_phi, w_dphi);
fprintf('  T:%.1f, Tp:%.1f\n', w_T, w_Tp);

%% 5. 最终推荐配置
% -----------------------------------------------------------
Q_final = diag([
    w_theta  * 1/max_theta^2,
    w_dtheta * 1/max_dtheta^2,
    w_x      * 1/max_x^2,
    w_dx     * 1/max_dx^2,
    w_phi    * 1/max_phi^2,
    w_dphi   * 1/max_dphi^2
]);

R_final = diag([
    w_T  * 1/max_T^2,
    w_Tp * 1/max_Tp^2
]);

% 四舍五入到整数方便使用
Q_rounded = round(diag(Q_final));
R_rounded = round(diag(R_final), 1);

fprintf('\n========================================\n');
fprintf('    ★ 最终推荐配置 ★\n');
fprintf('========================================\n');
fprintf('\n精确值:\n');
fprintf('Q = diag([%.1f, %.1f, %.1f, %.2f, %.1f, %.1f])\n', diag(Q_final));
fprintf('R = diag([%.1f, %.1f])\n', diag(R_final));

fprintf('\n简化值 (推荐直接使用):\n');
fprintf('Q = diag([%d, %d, %d, %d, %d, %d])\n', Q_rounded);
fprintf('R = diag([%.1f, %.1f])\n', R_rounded);

%% 6. 验证仿真
% -----------------------------------------------------------
fprintf('\n========================================\n');
fprintf('    仿真验证\n');
fprintf('========================================\n');

p_L = L0 / 2; p_LM = L0 / 2; p_l = 0;

A = double(get_A_matrix(p_L, p_LM, val_mp_num, val_Mw_num, val_M_num, ...
                        val_Ip_num, val_Iw_num, val_IM_num, val_R_num, val_g_num, p_l));
B = double(get_B_matrix(p_L, p_LM, val_mp_num, val_Mw_num, val_M_num, ...
                        val_Ip_num, val_Iw_num, val_IM_num, val_R_num, val_g_num, p_l));

% 使用简化值
Q_use = diag(Q_rounded);
R_use = diag(R_rounded);

[K_new, ~, ~] = lqr(A, B, Q_use, R_use);

fprintf('\n新配置 K 矩阵:\n');
fprintf('K_wheel = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', K_new(1,:));
fprintf('K_hip   = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', K_new(2,:));

% 对比原配置
Q_old = diag([300, 5, 10, 1, 150, 1]);
R_old = diag([1.5, 1.5]);
[K_old, ~, ~] = lqr(A, B, Q_old, R_old);

fprintf('\n原配置 K 矩阵 (对比):\n');
fprintf('K_wheel = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', K_old(1,:));
fprintf('K_hip   = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', K_old(2,:));

% 闭环极点
eig_new = eig(A - B*K_new);
eig_old = eig(A - B*K_old);

fprintf('\n闭环极点实部 (越负越稳定):\n');
fprintf('新配置: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', real(eig_new));
fprintf('原配置: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', real(eig_old));

%% 7. 仿真对比图
% -----------------------------------------------------------
t_end = 3;
dt = 0.001;
t = 0:dt:t_end;
x0 = [deg2rad(5); 0; 0; 0; deg2rad(3); 0];  % 初始扰动: theta=5°, phi=3°

% 新配置仿真
A_cl_new = A - B * K_new;
x_new = zeros(length(t), 6);
u_new = zeros(length(t), 2);
x_new(1, :) = x0';

for i = 1:length(t)-1
    u = -K_new * x_new(i, :)';
    u(1) = max(min(u(1), max_T), -max_T);
    u(2) = max(min(u(2), max_Tp), -max_Tp);
    u_new(i, :) = u';
    dx = A * x_new(i, :)' + B * u;
    x_new(i+1, :) = (x_new(i, :)' + dx * dt)';
end

% 原配置仿真
A_cl_old = A - B * K_old;
x_old = zeros(length(t), 6);
u_old = zeros(length(t), 2);
x_old(1, :) = x0';

for i = 1:length(t)-1
    u = -K_old * x_old(i, :)';
    u(1) = max(min(u(1), max_T), -max_T);
    u(2) = max(min(u(2), max_Tp), -max_Tp);
    u_old(i, :) = u';
    dx = A * x_old(i, :)' + B * u;
    x_old(i+1, :) = (x_old(i, :)' + dx * dt)';
end

% 绘图
figure('Name', '新旧配置对比', 'Position', [100, 100, 1200, 800]);

subplot(2, 3, 1);
plot(t, rad2deg(x_new(:, 1)), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(x_old(:, 1)), 'r--', 'LineWidth', 1.5);
xlabel('t (s)'); ylabel('theta (°)'); title('摆杆角度 θ');
legend('新配置', '原配置'); grid on;
yline(rad2deg(max_theta), 'k:', '允许上限');
yline(-rad2deg(max_theta), 'k:');

subplot(2, 3, 2);
plot(t, rad2deg(x_new(:, 5)), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(x_old(:, 5)), 'r--', 'LineWidth', 1.5);
xlabel('t (s)'); ylabel('phi (°)'); title('机体角度 φ');
legend('新配置', '原配置'); grid on;
yline(rad2deg(max_phi), 'k:', '允许上限');
yline(-rad2deg(max_phi), 'k:');

subplot(2, 3, 3);
plot(t, x_new(:, 3), 'b-', 'LineWidth', 1.5); hold on;
plot(t, x_old(:, 3), 'r--', 'LineWidth', 1.5);
xlabel('t (s)'); ylabel('x (m)'); title('位移 x');
legend('新配置', '原配置'); grid on;

subplot(2, 3, 4);
plot(t, u_new(:, 1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, u_old(:, 1), 'r--', 'LineWidth', 1.5);
yline(max_T, 'k:', '饱和'); yline(-max_T, 'k:');
xlabel('t (s)'); ylabel('T (N·m)'); title('轮子力矩');
legend('新配置', '原配置'); grid on;

subplot(2, 3, 5);
plot(t, u_new(:, 2), 'b-', 'LineWidth', 1.5); hold on;
plot(t, u_old(:, 2), 'r--', 'LineWidth', 1.5);
yline(max_Tp, 'k:', '饱和'); yline(-max_Tp, 'k:');
xlabel('t (s)'); ylabel('Tp (N·m)'); title('髋关节力矩');
legend('新配置', '原配置'); grid on;

subplot(2, 3, 6);
plot(t, x_new(:, 4), 'b-', 'LineWidth', 1.5); hold on;
plot(t, x_old(:, 4), 'r--', 'LineWidth', 1.5);
xlabel('t (s)'); ylabel('dx (m/s)'); title('速度 dx');
legend('新配置', '原配置'); grid on;

sgtitle('新配置 vs 原配置 对比 (初始扰动: θ=5°, φ=3°)');

%% 8. 性能指标
% -----------------------------------------------------------
fprintf('\n========================================\n');
fprintf('    性能指标对比\n');
fprintf('========================================\n');

% theta 调节时间
idx_new = find(abs(x_new(:,1)) > 0.02*abs(x0(1)), 1, 'last');
idx_old = find(abs(x_old(:,1)) > 0.02*abs(x0(1)), 1, 'last');
ts_theta_new = t(idx_new);
ts_theta_old = t(idx_old);

% phi 调节时间
idx_new = find(abs(x_new(:,5)) > 0.02*abs(x0(5)), 1, 'last');
idx_old = find(abs(x_old(:,5)) > 0.02*abs(x0(5)), 1, 'last');
ts_phi_new = t(idx_new);
ts_phi_old = t(idx_old);

fprintf('theta 调节时间: 新%.3fs vs 原%.3fs\n', ts_theta_new, ts_theta_old);
fprintf('phi 调节时间:   新%.3fs vs 原%.3fs\n', ts_phi_new, ts_phi_old);
fprintf('最大轮子力矩:   新%.3f vs 原%.3f N·m\n', max(abs(u_new(:,1))), max(abs(u_old(:,1))));
fprintf('最大髋关节力矩: 新%.3f vs 原%.3f N·m\n', max(abs(u_new(:,2))), max(abs(u_old(:,2))));

%% 9. 最终输出 - 可直接复制到代码
% -----------------------------------------------------------
fprintf('\n========================================\n');
fprintf('    ★ 直接复制到 lqr_1_20.m ★\n');
fprintf('========================================\n\n');

fprintf('%% Q 和 R 设置 (根据机器人参数优化)\n');
fprintf('%% 状态顺序: [theta, d_theta, x, d_x, phi, d_phi]\n');
fprintf('Q = diag([%d, %d, %d, %d, %d, %d]);\n', Q_rounded);
fprintf('R = diag([%.1f, %.1f]);\n\n', R_rounded);

fprintf('%% 说明:\n');
fprintf('%%   theta (摆杆角):    Q=%d (max允许%.0f°)\n', Q_rounded(1), rad2deg(max_theta));
fprintf('%%   d_theta:           Q=%d\n', Q_rounded(2));
fprintf('%%   x (位移):          Q=%d (max允许%.1fm)\n', Q_rounded(3), max_x);
fprintf('%%   d_x (速度):        Q=%d (max允许%.1fm/s)\n', Q_rounded(4), max_dx);
fprintf('%%   phi (机体角):      Q=%d (max允许%.0f°) [最重要]\n', Q_rounded(5), rad2deg(max_phi));
fprintf('%%   d_phi:             Q=%d\n', Q_rounded(6));
fprintf('%%   T (轮子力矩):      R=%.1f (max %.2fN·m)\n', R_rounded(1), max_T);
fprintf('%%   Tp (髋关节力矩):   R=%.1f (max %.2fN·m)\n', R_rounded(2), max_Tp);
