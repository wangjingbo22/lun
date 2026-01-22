clear; clc;

%% 1. 物理参数定义 (SI 单位)
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

%% 2. 符号推导 (Symbolic Derivation)
% -----------------------------------------------------------
fprintf('正在进行符号推导 (这需要约 10-20 秒)...\n');

syms theta x phi real          % 状态: 摆杆角, 位移, 机体角
syms d_theta d_x d_phi real    % 速度
syms dd_theta dd_x dd_phi real % 加速度
syms T Tp real                 % 输入
syms N Nf NM P PM real         % 内力

% 参数符号
syms val_L val_LM val_mp val_Mw val_M val_Ip val_Iw val_IM val_R val_g val_l real

% --- 动力学方程 ---

% (1.1) mw * x_dd = Nf - N
eq1 = val_Mw * dd_x == Nf - N;

% (1.2) Iw * x_dd / R = T - Nf * R
eq2 = val_Iw * dd_x / val_R == T - Nf * val_R;

% 辅助项: 摆杆质心加速度
acc_p_x = dd_x + val_L * (cos(theta)*dd_theta - sin(theta)*d_theta^2);
acc_p_y = val_L * (-sin(theta)*dd_theta - cos(theta)*d_theta^2);

% (1.4) N - NM = mp * acc_p_x
eq4 = N - NM == val_mp * acc_p_x;

% (1.5) P - PM - mp*g = mp * acc_p_y
eq5 = P - PM - val_mp * val_g == val_mp * acc_p_y;

% (1.6) 摆杆力矩平衡
eq6 = val_Ip * dd_theta == (P*val_L + PM*val_LM)*sin(theta) - ...
      (N*val_L + NM*val_LM)*cos(theta) - T + Tp;

% 辅助项: 机体质心加速度
acc_m_x = dd_x + (val_L+val_LM)*(cos(theta)*dd_theta - sin(theta)*d_theta^2) - ...
          val_l*(cos(phi)*dd_phi - sin(phi)*d_phi^2);
acc_m_y = (val_L+val_LM)*(-sin(theta)*dd_theta - cos(theta)*d_theta^2) + ...
          val_l*(-sin(phi)*dd_phi - cos(phi)*d_phi^2);

% (1.7) NM = M * acc_m_x
eq7 = NM == val_M * acc_m_x;

% (1.8) PM - Mg = M * acc_m_y
eq8 = PM - val_M * val_g == val_M * acc_m_y;

% (1.9) 机体力矩平衡
eq9 = val_IM * dd_phi == Tp + NM*val_l*cos(phi) + PM*val_l*sin(phi);

% --- 求解加速度 ---
vars_unknown = [dd_x, dd_theta, dd_phi, N, Nf, NM, P, PM];
eqs_all = [eq1, eq2, eq4, eq5, eq6, eq7, eq8, eq9];

% 获取结构体解
sol = solve(eqs_all, vars_unknown);

% 提取加速度表达式 (f)
f_vec = [sol.dd_theta; sol.dd_x; sol.dd_phi];

%% 3. 线性化与矩阵生成
% -----------------------------------------------------------
fprintf('正在计算 Jacobian 并生成函数...\n');

state_vars = [theta, d_theta, x, d_x, phi, d_phi];
input_vars = [T, Tp];

% 平衡点代入 (先在符号层面代入0，简化表达式)
% 注意：求导后再代入，或者代入求导后的结果。
% 这里我们使用 subs(jacobian(...)) 的方式。

% 计算 A 的动力学部分 (3行 x 6列)
J_A_dyn = jacobian(f_vec, state_vars);
% 计算 B 的动力学部分 (3行 x 2列)
J_B_dyn = jacobian(f_vec, input_vars);

% 代入平衡点 (所有角度速度为0，输入为0)
equil_subs_old = [theta, d_theta, x, d_x, phi, d_phi, T, Tp];
equil_subs_new = [0, 0, 0, 0, 0, 0, 0, 0];

J_A_val = subs(J_A_dyn, equil_subs_old, equil_subs_new);
J_B_val = subs(J_B_dyn, equil_subs_old, equil_subs_new);

% 组装完整的 A 矩阵 (6x6)
% x_dot = A x + B u
% [ d_theta ]   [ 0 1 0 0 0 0 ] [ theta ]
% [ dd_theta] = [ J_A_val(1,:)] [ ...   ]
% ...
A_final = sym(zeros(6,6));
A_final(1,2) = 1; 
A_final(3,4) = 1; 
A_final(5,6) = 1;
A_final([2,4,6], :) = J_A_val;

% 组装完整的 B 矩阵 (6x2)
B_final = sym(zeros(6,2));
B_final([2,4,6], :) = J_B_val;

% 关键步骤：生成两个独立的函数文件，避免 horzcat 错误
% 使用 Optimize=false 可能会生成较大的文件，但兼容性最好
matlabFunction(A_final, 'File', 'get_A_matrix', ...
    'Vars', {val_L, val_LM, val_mp, val_Mw, val_M, val_Ip, val_Iw, val_IM, val_R, val_g, val_l});

matlabFunction(B_final, 'File', 'get_B_matrix', ...
    'Vars', {val_L, val_LM, val_mp, val_Mw, val_M, val_Ip, val_Iw, val_IM, val_R, val_g, val_l});

%% 4. LQR 循环计算
% -----------------------------------------------------------
L0_list = 0.08 : 0.005 : 0.14;  % 更密的采样点用于拟合

% Q 和 R 设置
Q = diag([300, 5, 10, 1, 150, 1]); 
R = diag([1.5, 1.5]);

% 存储所有 K 值用于拟合
K_wheel_all = zeros(length(L0_list), 6);
K_leg_all = zeros(length(L0_list), 6);

fprintf('\n计算完成。以下是针对不同腿长 L0 的 LQR 增益矩阵。\n');
fprintf('----------------------------------------------------------------------------------------\n');
fprintf('L0(cm) |  K_theta   K_d_theta      K_x      K_d_x      K_phi    K_d_phi\n');
fprintf('----------------------------------------------------------------------------------------\n');

for i = 1:length(L0_list)
    L_total = L0_list(i);
    
    % 参数准备
    p_L = L_total / 2;
    p_LM = L_total / 2;
    p_l = 0; % 机体质心偏置
    
    % 调用刚才生成的函数
    A_num = get_A_matrix(p_L, p_LM, val_mp_num, val_Mw_num, val_M_num, val_Ip_num, val_Iw_num, val_IM_num, val_R_num, val_g_num, p_l);
    B_num = get_B_matrix(p_L, p_LM, val_mp_num, val_Mw_num, val_M_num, val_Ip_num, val_Iw_num, val_IM_num, val_R_num, val_g_num, p_l);
    
    % 转换 double
    A_num = double(A_num);
    B_num = double(B_num);
    
    % LQR
    [K, ~, ~] = lqr(A_num, B_num, Q, R);
    
    % 存储轮子和腿部反馈增益
    K_wheel_all(i, :) = K(1, :);
    K_leg_all(i, :) = K(2, :);
    
    fprintf(' %4.1f  | %9.3f %9.3f %9.3f %9.3f %9.3f %9.3f\n', ...
        L_total*100, K(1,1), K(1,2), K(1,3), K(1,4), K(1,5), K(1,6));
end
fprintf('----------------------------------------------------------------------------------------\n');

%% 5. 三次多项式拟合
% -----------------------------------------------------------
fprintf('\n正在进行三次多项式拟合...\n');

% 拟合阶数
poly_order = 3;

% 存储拟合系数 (每行对应一个状态分量的系数 [a3, a2, a1, a0])
% K(L0) = a3*L0^3 + a2*L0^2 + a1*L0 + a0
coeff_wheel = zeros(6, poly_order + 1);
coeff_leg = zeros(6, poly_order + 1);

state_names = {'theta', 'd_theta', 'x', 'd_x', 'phi', 'd_phi'};

% 对每个状态分量进行拟合
for j = 1:6
    coeff_wheel(j, :) = polyfit(L0_list, K_wheel_all(:, j)', poly_order);
    coeff_leg(j, :) = polyfit(L0_list, K_leg_all(:, j)', poly_order);
end

%% 6. 验证拟合精度
% -----------------------------------------------------------
fprintf('\n======================== 拟合精度验证 ========================\n');

% 计算拟合误差
max_err_wheel = zeros(1, 6);
max_err_leg = zeros(1, 6);

for j = 1:6
    K_fit_wheel = polyval(coeff_wheel(j,:), L0_list);
    K_fit_leg = polyval(coeff_leg(j,:), L0_list);
    max_err_wheel(j) = max(abs(K_fit_wheel - K_wheel_all(:,j)'));
    max_err_leg(j) = max(abs(K_fit_leg - K_leg_all(:,j)'));
end

fprintf('轮子控制增益最大拟合误差:\n');
for j = 1:6
    fprintf('  %-8s: %.6f\n', state_names{j}, max_err_wheel(j));
end

fprintf('\n腿部控制增益最大拟合误差:\n');
for j = 1:6
    fprintf('  %-8s: %.6f\n', state_names{j}, max_err_leg(j));
end

%% 7. 生成 C 代码 (与 lqr.c 格式一致)
% -----------------------------------------------------------
fprintf('\n======================== C代码 (可直接复制到 lqr.c) ========================\n\n');

% C代码注释名称
c_names_wheel = {'Wheel_Theta', 'Wheel_Gyro', 'Wheel_X', 'Wheel_V', 'Wheel_Phi', 'Wheel_PhiV'};
c_names_hip = {'Hip_Theta', 'Hip_Gyro', 'Hip_X', 'Hip_V', 'Hip_Phi', 'Hip_PhiV'};

fprintf('// 12个状态增益，每个有4个多项式系数 (a*L^3 + b*L^2 + c*L + d)\n');
fprintf('// 顺序: \n');
fprintf('// 0-5: 轮子电机 [theta, d_theta, x, d_x, phi, d_phi]\n');
fprintf('// 6-11: 髋关节 [theta, d_theta, x, d_x, phi, d_phi]\n');
fprintf('const float LQR_DUAL_COEFFS[12][4] = {\n');

% 轮子控制增益 (0-5)
for j = 1:6
    fprintf('    {%10.4f, %10.4f, %10.4f, %10.4f}, // %s\n', ...
        coeff_wheel(j,1), coeff_wheel(j,2), coeff_wheel(j,3), coeff_wheel(j,4), c_names_wheel{j});
end

% 髋关节控制增益 (6-11)
for j = 1:6
    if j < 6
        fprintf('    {%10.4f, %10.4f, %10.4f, %10.4f}, // %s\n', ...
            coeff_leg(j,1), coeff_leg(j,2), coeff_leg(j,3), coeff_leg(j,4), c_names_hip{j});
    else
        fprintf('    {%10.4f, %10.4f, %10.4f, %10.4f}, // %s\n', ...
            coeff_leg(j,1), coeff_leg(j,2), coeff_leg(j,3), coeff_leg(j,4), c_names_hip{j});
    end
end
fprintf('};\n');

%% 8. 输出到 matlabFunction (参照 lunlun.m)
% -----------------------------------------------------------
fprintf('\n正在生成 LQR_K 函数文件...\n');

% 构造符号K矩阵 (2x6)
K_sym = sym('K', [2, 6]);
syms L0;

for x = 1:2
    for y = 1:6
        if x == 1
            p = coeff_wheel(y, :);
        else
            p = coeff_leg(y, :);
        end
        K_sym(x, y) = p(1)*L0^3 + p(2)*L0^2 + p(3)*L0 + p(4);
    end
end

% 输出到m函数
matlabFunction(K_sym, 'File', 'LQR_K');
fprintf('已生成 LQR_K.m 文件\n');

% 打印示例：代入 L0=0.1 验证
fprintf('\n======================== 验证: L0=0.1m 时的K矩阵 ========================\n');
K_test = double(subs(K_sym, L0, 0.1));
fprintf('K_wheel = [%8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f]\n', K_test(1,:));
fprintf('K_hip   = [%8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f]\n', K_test(2,:));

%% 9. 绘制拟合曲线
% -----------------------------------------------------------
figure('Name', 'LQR增益拟合结果', 'Position', [100, 100, 1200, 800]);

L0_fine = linspace(min(L0_list), max(L0_list), 100);

for j = 1:6
    subplot(2, 3, j);
    
    % 原始数据点
    plot(L0_list*100, K_wheel_all(:,j), 'bo', 'MarkerSize', 6, 'DisplayName', '计算值');
    hold on;
    
    % 拟合曲线
    K_fit = polyval(coeff_wheel(j,:), L0_fine);
    plot(L0_fine*100, K_fit, 'r-', 'LineWidth', 1.5, 'DisplayName', '三次拟合');
    
    xlabel('L0 (cm)');
    ylabel(['K\_', state_names{j}]);
    title(['轮子增益 K\_', state_names{j}]);
    legend('Location', 'best');
    grid on;
end

sgtitle('轮子控制增益三次多项式拟合');

% 髋关节增益图
figure('Name', 'LQR髋关节增益拟合结果', 'Position', [150, 150, 1200, 800]);

for j = 1:6
    subplot(2, 3, j);
    
    % 原始数据点
    plot(L0_list*100, K_leg_all(:,j), 'go', 'MarkerSize', 6, 'DisplayName', '计算值');
    hold on;
    
    % 拟合曲线
    K_fit = polyval(coeff_leg(j,:), L0_fine);
    plot(L0_fine*100, K_fit, 'm-', 'LineWidth', 1.5, 'DisplayName', '三次拟合');
    
    xlabel('L0 (cm)');
    ylabel(['K\_', state_names{j}]);
    title(['髋关节增益 K\_', state_names{j}]);
    legend('Location', 'best');
    grid on;
end

sgtitle('髋关节控制增益三次多项式拟合');

fprintf('\nLQR模型函数生成完毕!\n');