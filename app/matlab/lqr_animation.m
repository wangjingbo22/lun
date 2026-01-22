%% LQR 轮腿机器人动画仿真
% 直观地显示机器人运动过程
% 需要先运行 lqr_1_20.m 生成 get_A_matrix.m 和 get_B_matrix.m
clear; clc; close all;

%% 1. 物理参数定义
% -----------------------------------------------------------
val_R_num  = 0.044;          % 轮子半径 (m)
val_Mw_num = 0.260;          
val_M_num  = 1.075;          
val_mp_num = 0.065;          
val_g_num  = 9.81;

val_Iw_num = 0.5 * val_Mw_num * val_R_num^2;               
val_IM_num = val_M_num * (0.1^2 + 0.15^2) / 12;    
val_Ip_num = val_mp_num * (0.1)^2 / 12;            

% 机器人几何参数 (用于绘图)
R_wheel = val_R_num;    % 轮子半径
L0 = 0.10;              % 腿长 (m)
body_width = 0.08;      % 机体宽度 (m)
body_height = 0.06;     % 机体高度 (m)

%% 2. 计算 A, B, K 矩阵
% -----------------------------------------------------------
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

fprintf('LQR K 矩阵计算完成\n');

%% 3. 仿真
% -----------------------------------------------------------
t_end = 5;
T_max = 0.5;
Tp_max = 0.3;

% 初始条件: [theta, d_theta, x, d_x, phi, d_phi]
% theta: 摆杆(腿)倾角, phi: 机体倾角
x0 = [0.15;   % theta: 摆杆初始倾角 ~8.6°
      0;      
      0;      % x: 初始位移
      0;      
      0.08;   % phi: 机体初始倾角 ~4.6°
      0];     

fprintf('初始状态: theta=%.1f°, phi=%.1f°\n', rad2deg(x0(1)), rad2deg(x0(5)));

% ode45 仿真
dynamics = @(t, x) robot_dynamics(t, x, A, B, K, T_max, Tp_max);
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t_sim, x_sim] = ode45(dynamics, [0 t_end], x0, options);

% 计算控制输入
u_sim = zeros(length(t_sim), 2);
for i = 1:length(t_sim)
    u = -K * x_sim(i,:)';
    u(1) = max(min(u(1), T_max), -T_max);
    u(2) = max(min(u(2), Tp_max), -Tp_max);
    u_sim(i,:) = u';
end

fprintf('仿真完成，共 %d 帧\n', length(t_sim));

%% 4. 创建动画
% -----------------------------------------------------------
fprintf('正在生成动画...\n');

% 创建图形窗口
fig = figure('Name', '轮腿机器人LQR控制仿真', 'Position', [100, 100, 1400, 700]);

% 左侧：机器人动画
ax1 = subplot(1, 2, 1);
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('轮腿机器人动画');

% 设置坐标轴范围 (根据位移动态调整)
x_range = max(abs(x_sim(:,3))) + 0.3;
xlim([-x_range, x_range]);
ylim([-0.05, L0 + body_height + 0.1]);

% 绘制地面
plot([-x_range, x_range], [0, 0], 'k-', 'LineWidth', 2);

% 初始化图形对象
wheel = plot(0, 0, 'b-', 'LineWidth', 2);           % 轮子
wheel_spoke1 = plot([0,0], [0,0], 'b-', 'LineWidth', 1); % 轮辐1
wheel_spoke2 = plot([0,0], [0,0], 'b-', 'LineWidth', 1); % 轮辐2
leg = plot([0,0], [0,0], 'r-', 'LineWidth', 3);     % 腿
body = fill([0], [0], [0.2, 0.6, 1], 'EdgeColor', 'k', 'LineWidth', 2); % 机体
com_marker = plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % 质心

% 时间和状态显示
time_text = text(-x_range+0.05, L0+body_height+0.05, '', 'FontSize', 12);
state_text = text(-x_range+0.05, L0+body_height, '', 'FontSize', 10);

% 右侧：状态曲线
ax2 = subplot(2, 2, 2);
hold on; grid on;
xlabel('时间 (s)'); ylabel('角度 (rad)');
title('角度响应');
h_theta = animatedline('Color', 'r', 'LineWidth', 1.5);
h_phi = animatedline('Color', 'b', 'LineWidth', 1.5);
legend('θ (摆杆)', 'φ (机体)', 'Location', 'best');
xlim([0, t_end]);
ylim([-0.2, 0.2]);

ax3 = subplot(2, 2, 4);
hold on; grid on;
xlabel('时间 (s)'); ylabel('力矩 (N·m)');
title('控制输入');
h_T = animatedline('Color', 'r', 'LineWidth', 1.5);
h_Tp = animatedline('Color', 'b', 'LineWidth', 1.5);
legend('T (轮子)', 'Tp (髋关节)', 'Location', 'best');
xlim([0, t_end]);
ylim([-0.6, 0.6]);

%% 5. 动画循环
% -----------------------------------------------------------
% 降采样以加速动画
skip = max(1, floor(length(t_sim) / 500));
frames = 1:skip:length(t_sim);

% 轮子角度累积 (用于绘制轮辐旋转)
wheel_angle = 0;

for idx = frames
    t = t_sim(idx);
    theta = x_sim(idx, 1);   % 摆杆角
    x_pos = x_sim(idx, 3);   % 位移
    phi = x_sim(idx, 5);     % 机体角
    
    % 计算轮子旋转角度 (根据位移)
    wheel_angle = x_pos / R_wheel;
    
    % === 计算各部件位置 ===
    
    % 轮子圆心
    wheel_cx = x_pos;
    wheel_cy = R_wheel;
    
    % 绘制轮子 (圆)
    theta_circle = linspace(0, 2*pi, 50);
    wheel_x = wheel_cx + R_wheel * cos(theta_circle);
    wheel_y = wheel_cy + R_wheel * sin(theta_circle);
    set(wheel, 'XData', wheel_x, 'YData', wheel_y);
    
    % 绘制轮辐 (显示旋转)
    spoke1_x = [wheel_cx, wheel_cx + R_wheel*cos(wheel_angle)];
    spoke1_y = [wheel_cy, wheel_cy + R_wheel*sin(wheel_angle)];
    spoke2_x = [wheel_cx, wheel_cx + R_wheel*cos(wheel_angle + pi/2)];
    spoke2_y = [wheel_cy, wheel_cy + R_wheel*sin(wheel_angle + pi/2)];
    set(wheel_spoke1, 'XData', spoke1_x, 'YData', spoke1_y);
    set(wheel_spoke2, 'XData', spoke2_x, 'YData', spoke2_y);
    
    % 腿顶端位置 (从轮轴向上延伸，考虑摆杆角度theta)
    % theta > 0 表示向前倾
    leg_top_x = wheel_cx + L0 * sin(theta);
    leg_top_y = wheel_cy + L0 * cos(theta);
    set(leg, 'XData', [wheel_cx, leg_top_x], 'YData', [wheel_cy, leg_top_y]);
    
    % 机体位置 (以腿顶端为pivot，机体有自己的倾角phi)
    % phi > 0 表示机体向前倾
    body_corners_local = [
        -body_width/2, 0;
        body_width/2, 0;
        body_width/2, body_height;
        -body_width/2, body_height
    ]';
    
    % 旋转矩阵 (机体绕腿顶端旋转phi角)
    R_phi = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    body_corners_rotated = R_phi * body_corners_local;
    
    % 平移到腿顶端
    body_x = body_corners_rotated(1,:) + leg_top_x;
    body_y = body_corners_rotated(2,:) + leg_top_y;
    set(body, 'XData', body_x, 'YData', body_y);
    
    % 质心位置
    com_x = leg_top_x + (body_height/2) * sin(phi);
    com_y = leg_top_y + (body_height/2) * cos(phi);
    set(com_marker, 'XData', com_x, 'YData', com_y);
    
    % 更新文字
    set(time_text, 'String', sprintf('t = %.2f s', t));
    set(state_text, 'String', sprintf('θ=%.1f°  φ=%.1f°  x=%.3fm', ...
        rad2deg(theta), rad2deg(phi), x_pos));
    
    % 动态调整x轴范围
    if abs(x_pos) > x_range - 0.1
        x_range = abs(x_pos) + 0.3;
        set(ax1, 'XLim', [-x_range, x_range]);
    end
    
    % 更新状态曲线
    addpoints(h_theta, t, theta);
    addpoints(h_phi, t, phi);
    addpoints(h_T, t, u_sim(idx, 1));
    addpoints(h_Tp, t, u_sim(idx, 2));
    
    % 刷新图形
    drawnow limitrate;
    
    % 控制动画速度 (可以调整)
    pause(0.01);
end

fprintf('\n动画播放完成!\n');

%% 6. 最终状态图
% -----------------------------------------------------------
figure('Name', '最终状态对比', 'Position', [150, 150, 1200, 500]);

subplot(1,3,1);
plot(t_sim, rad2deg(x_sim(:,1)), 'r-', 'LineWidth', 1.5); hold on;
plot(t_sim, rad2deg(x_sim(:,5)), 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('角度 (°)');
title('角度响应');
legend('θ (摆杆)', 'φ (机体)');
grid on;

subplot(1,3,2);
plot(t_sim, x_sim(:,3), 'g-', 'LineWidth', 1.5); hold on;
plot(t_sim, x_sim(:,4), 'm-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('位移/速度');
title('位移与速度');
legend('x (m)', 'v (m/s)');
grid on;

subplot(1,3,3);
plot(t_sim, u_sim(:,1), 'r-', 'LineWidth', 1.5); hold on;
plot(t_sim, u_sim(:,2), 'b-', 'LineWidth', 1.5);
yline(T_max, 'r--'); yline(-T_max, 'r--');
yline(Tp_max, 'b--'); yline(-Tp_max, 'b--');
xlabel('时间 (s)'); ylabel('力矩 (N·m)');
title('控制输入');
legend('T (轮子)', 'Tp (髋关节)');
grid on;

sgtitle('LQR 控制仿真结果');

%% 动力学函数
function dx = robot_dynamics(~, x, A, B, K, T_max, Tp_max)
    u = -K * x;
    u(1) = max(min(u(1), T_max), -T_max);
    u(2) = max(min(u(2), Tp_max), -Tp_max);
    dx = A * x + B * u;
end
