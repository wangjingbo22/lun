%% LQR Q/R 矩阵调参工具
% 帮助确定合适的 Q 和 R 矩阵
% 需要先运行 lqr_1_20.m 生成 get_A_matrix.m 和 get_B_matrix.m
clear; clc; close all;

%% 1. 系统参数
% -----------------------------------------------------------
val_R_num  = 0.044;          
val_Mw_num = 0.260;          
val_M_num  = 1.075;          
val_mp_num = 0.065;          
val_g_num  = 9.81;

val_Iw_num = 0.5 * val_Mw_num * val_R_num^2;               
val_IM_num = val_M_num * (0.1^2 + 0.15^2) / 12;    
val_Ip_num = val_mp_num * (0.1)^2 / 12;            

L0 = 0.10;
p_L = L0 / 2; p_LM = L0 / 2; p_l = 0;

A = double(get_A_matrix(p_L, p_LM, val_mp_num, val_Mw_num, val_M_num, ...
                        val_Ip_num, val_Iw_num, val_IM_num, val_R_num, val_g_num, p_l));
B = double(get_B_matrix(p_L, p_LM, val_mp_num, val_Mw_num, val_M_num, ...
                        val_Ip_num, val_Iw_num, val_IM_num, val_R_num, val_g_num, p_l));

fprintf('========================================\n');
fprintf('    LQR Q/R 矩阵调参指南\n');
fprintf('========================================\n\n');

%% 2. 方法一: Bryson 规则 (最常用)
% -----------------------------------------------------------
% Q_ii = 1 / (状态量最大允许偏差)^2
% R_jj = 1 / (输入量最大允许值)^2
% 
% 这样做的物理意义: 把不同量纲的变量归一化到同一尺度

fprintf('【方法一: Bryson 规则】\n');
fprintf('原理: Q_ii = 1/max_deviation^2, R_jj = 1/max_input^2\n\n');

% 定义各状态量的最大允许偏差
% 状态: [theta, d_theta, x, d_x, phi, d_phi]
max_theta   = deg2rad(10);   % 摆杆最大允许倾角 10°
max_dtheta  = deg2rad(30);   % 摆杆最大角速度 30°/s
max_x       = 0.5;           % 最大允许位移 0.5m
max_dx      = 2.0;           % 最大允许速度 2m/s
max_phi     = deg2rad(8);    % 机体最大允许倾角 8°
max_dphi    = deg2rad(20);   % 机体最大角速度 20°/s

% 定义输入量的最大值
max_T  = 0.5;   % 轮子电机最大力矩 0.5 N·m
max_Tp = 0.3;   % 髋关节最大力矩 0.3 N·m

% 按 Bryson 规则计算
Q_bryson = diag([1/max_theta^2, 1/max_dtheta^2, 1/max_x^2, ...
                 1/max_dx^2, 1/max_phi^2, 1/max_dphi^2]);
R_bryson = diag([1/max_T^2, 1/max_Tp^2]);

fprintf('最大允许偏差设置:\n');
fprintf('  theta:   %.1f°    -> Q(1,1) = %.1f\n', rad2deg(max_theta), Q_bryson(1,1));
fprintf('  d_theta: %.1f°/s  -> Q(2,2) = %.1f\n', rad2deg(max_dtheta), Q_bryson(2,2));
fprintf('  x:       %.2fm    -> Q(3,3) = %.1f\n', max_x, Q_bryson(3,3));
fprintf('  d_x:     %.1fm/s  -> Q(4,4) = %.2f\n', max_dx, Q_bryson(4,4));
fprintf('  phi:     %.1f°    -> Q(5,5) = %.1f\n', rad2deg(max_phi), Q_bryson(5,5));
fprintf('  d_phi:   %.1f°/s  -> Q(6,6) = %.1f\n', rad2deg(max_dphi), Q_bryson(6,6));
fprintf('\n最大输入设置:\n');
fprintf('  T:  %.2f N·m -> R(1,1) = %.1f\n', max_T, R_bryson(1,1));
fprintf('  Tp: %.2f N·m -> R(2,2) = %.1f\n', max_Tp, R_bryson(2,2));

fprintf('\nBryson 规则 Q 矩阵:\n');
disp(diag(Q_bryson)');
fprintf('Bryson 规则 R 矩阵:\n');
disp(diag(R_bryson)');

%% 3. 方法二: 相对权重法
% -----------------------------------------------------------
% 先用 Bryson 规则归一化，然后乘以相对权重系数

fprintf('\n【方法二: 相对权重法】\n');
fprintf('在 Bryson 基础上，乘以相对重要性系数\n\n');

% 相对权重系数 (根据控制目标调整)
% 想让某个状态收敛更快，就增大其权重
w_theta  = 1.0;   % 摆杆角度重要性
w_dtheta = 0.1;   % 摆杆角速度 (通常比位置低)
w_x      = 0.5;   % 位移重要性
w_dx     = 0.1;   % 速度重要性
w_phi    = 2.0;   % 机体角度重要性 (稳定性关键!)
w_dphi   = 0.1;   % 机体角速度

w_T  = 1.0;       % 轮子力矩代价
w_Tp = 1.0;       % 髋关节力矩代价

Q_weighted = diag([w_theta, w_dtheta, w_x, w_dx, w_phi, w_dphi]) * Q_bryson;
R_weighted = diag([w_T, w_Tp]) * R_bryson;

fprintf('相对权重:\n');
fprintf('  [theta, d_theta, x, d_x, phi, d_phi] = [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n', ...
    w_theta, w_dtheta, w_x, w_dx, w_phi, w_dphi);
fprintf('  [T, Tp] = [%.1f, %.1f]\n', w_T, w_Tp);

fprintf('\n加权后 Q 对角元素:\n');
disp(diag(Q_weighted)');
fprintf('加权后 R 对角元素:\n');
disp(diag(R_weighted)');

%% 4. 方法三: 参数扫描对比
% -----------------------------------------------------------
fprintf('\n【方法三: 参数扫描 - 可视化对比】\n');
fprintf('对不同 Q/R 进行仿真，比较性能\n\n');

% 测试不同的 Q 配置
Q_configs = {
    diag([100, 1, 10, 1, 50, 1]),    '保守 (小Q)';
    diag([300, 5, 10, 1, 150, 1]),   '当前配置';
    diag([500, 10, 20, 2, 300, 2]),  '激进 (大Q)';
    Q_weighted,                       'Bryson加权';
};

R_test = diag([1.5, 1.5]);  % 固定 R 进行对比

% 仿真参数
t_end = 3;
dt = 0.001;
t = 0:dt:t_end;
x0 = [0.1; 0; 0; 0; 0.05; 0];  % 初始扰动

figure('Name', 'Q矩阵对比', 'Position', [100, 100, 1200, 800]);

colors = {'b', 'r', 'g', 'm'};
legends = {};

for c = 1:size(Q_configs, 1)
    Q_test = Q_configs{c, 1};
    config_name = Q_configs{c, 2};
    
    % 计算 K
    [K_test, ~, ~] = lqr(A, B, Q_test, R_test);
    
    % 闭环仿真
    A_cl = A - B * K_test;
    x_sim = zeros(length(t), 6);
    x_sim(1, :) = x0';
    
    for i = 1:length(t)-1
        dx = A_cl * x_sim(i, :)';
        x_sim(i+1, :) = (x_sim(i, :)' + dx * dt)';
    end
    
    % 绘图
    subplot(2, 2, 1);
    plot(t, rad2deg(x_sim(:, 1)), colors{c}, 'LineWidth', 1.5);
    hold on;
    
    subplot(2, 2, 2);
    plot(t, rad2deg(x_sim(:, 5)), colors{c}, 'LineWidth', 1.5);
    hold on;
    
    subplot(2, 2, 3);
    plot(t, x_sim(:, 3), colors{c}, 'LineWidth', 1.5);
    hold on;
    
    subplot(2, 2, 4);
    plot(t, x_sim(:, 4), colors{c}, 'LineWidth', 1.5);
    hold on;
    
    legends{end+1} = config_name;
    
    % 计算性能指标
    settling_idx = find(abs(x_sim(:,1)) > 0.02*abs(x0(1)), 1, 'last');
    if isempty(settling_idx), settling_idx = 1; end
    settling_time = t(settling_idx);
    
    fprintf('%s: 调节时间 %.3fs, K(1,:)=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n', ...
        config_name, settling_time, K_test(1,:));
end

subplot(2, 2, 1);
xlabel('t (s)'); ylabel('theta (°)'); title('摆杆角度 θ');
legend(legends, 'Location', 'best'); grid on;

subplot(2, 2, 2);
xlabel('t (s)'); ylabel('phi (°)'); title('机体角度 φ');
legend(legends, 'Location', 'best'); grid on;

subplot(2, 2, 3);
xlabel('t (s)'); ylabel('x (m)'); title('位移 x');
legend(legends, 'Location', 'best'); grid on;

subplot(2, 2, 4);
xlabel('t (s)'); ylabel('dx (m/s)'); title('速度 dx');
legend(legends, 'Location', 'best'); grid on;

sgtitle('不同 Q 矩阵配置对比 (相同 R)');

%% 5. 方法四: R 矩阵影响分析
% -----------------------------------------------------------
fprintf('\n【方法四: R 矩阵影响分析】\n');
fprintf('R 越大 -> 控制越保守，响应越慢，但输入更平滑\n');
fprintf('R 越小 -> 控制越激进，响应越快，但输入可能饱和\n\n');

Q_fixed = diag([300, 5, 10, 1, 150, 1]);

R_configs = {
    diag([0.5, 0.5]),   'R小 (激进)';
    diag([1.5, 1.5]),   'R中 (当前)';
    diag([5, 5]),       'R大 (保守)';
    diag([1, 3]),       'Tp更贵';
};

figure('Name', 'R矩阵对比', 'Position', [150, 150, 1200, 600]);

for c = 1:size(R_configs, 1)
    R_test = R_configs{c, 1};
    config_name = R_configs{c, 2};
    
    [K_test, ~, ~] = lqr(A, B, Q_fixed, R_test);
    
    % 闭环仿真
    A_cl = A - B * K_test;
    x_sim = zeros(length(t), 6);
    u_sim = zeros(length(t), 2);
    x_sim(1, :) = x0';
    
    for i = 1:length(t)-1
        u = -K_test * x_sim(i, :)';
        u_sim(i, :) = u';
        dx = A_cl * x_sim(i, :)';
        x_sim(i+1, :) = (x_sim(i, :)' + dx * dt)';
    end
    
    subplot(2, 2, 1);
    plot(t, rad2deg(x_sim(:, 1)), colors{c}, 'LineWidth', 1.5);
    hold on;
    
    subplot(2, 2, 2);
    plot(t, rad2deg(x_sim(:, 5)), colors{c}, 'LineWidth', 1.5);
    hold on;
    
    subplot(2, 2, 3);
    plot(t, u_sim(:, 1), colors{c}, 'LineWidth', 1.5);
    hold on;
    
    subplot(2, 2, 4);
    plot(t, u_sim(:, 2), colors{c}, 'LineWidth', 1.5);
    hold on;
    
    fprintf('%s: K(1,1)=%.1f, max|T|=%.3f, max|Tp|=%.3f\n', ...
        config_name, K_test(1,1), max(abs(u_sim(:,1))), max(abs(u_sim(:,2))));
end

subplot(2, 2, 1);
xlabel('t (s)'); ylabel('theta (°)'); title('摆杆角度 θ');
legend(R_configs(:,2), 'Location', 'best'); grid on;

subplot(2, 2, 2);
xlabel('t (s)'); ylabel('phi (°)'); title('机体角度 φ');
legend(R_configs(:,2), 'Location', 'best'); grid on;

subplot(2, 2, 3);
xlabel('t (s)'); ylabel('T (N·m)'); title('轮子力矩');
legend(R_configs(:,2), 'Location', 'best'); grid on;
yline(0.5, 'k--'); yline(-0.5, 'k--');

subplot(2, 2, 4);
xlabel('t (s)'); ylabel('Tp (N·m)'); title('髋关节力矩');
legend(R_configs(:,2), 'Location', 'best'); grid on;
yline(0.3, 'k--'); yline(-0.3, 'k--');

sgtitle('不同 R 矩阵配置对比 (相同 Q)');

%% 6. 调参建议总结
% -----------------------------------------------------------
fprintf('\n========================================\n');
fprintf('    调参建议总结\n');
fprintf('========================================\n\n');

fprintf('【步骤1】使用 Bryson 规则计算初始 Q 和 R\n');
fprintf('  - 估计各状态的最大允许偏差\n');
fprintf('  - 估计执行器的最大输出能力\n');
fprintf('  - Q_ii = 1/max_dev^2, R_jj = 1/max_input^2\n\n');

fprintf('【步骤2】根据控制目标调整相对权重\n');
fprintf('  - 想让某状态收敛更快 -> 增大其 Q 权重\n');
fprintf('  - 想减小某输入的使用 -> 增大其 R 权重\n');
fprintf('  - 通常: 角度 > 位置 > 速度\n\n');

fprintf('【步骤3】仿真验证并微调\n');
fprintf('  - 观察响应曲线是否满足要求\n');
fprintf('  - 检查控制输入是否在物理限制内\n');
fprintf('  - 调整直到满意\n\n');

fprintf('【经验公式】\n');
fprintf('  - 位置类状态 Q: 100 ~ 1000\n');
fprintf('  - 速度类状态 Q: 1 ~ 50\n');
fprintf('  - R 通常取 0.1 ~ 10，越大响应越慢\n\n');

fprintf('【你当前的配置】\n');
fprintf('  Q = diag([300, 5, 10, 1, 150, 1])\n');
fprintf('  R = diag([1.5, 1.5])\n');
fprintf('  这个配置: theta和phi权重较高，适合平衡稳定\n');
