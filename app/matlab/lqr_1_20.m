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
L0_list = 0.08 : 0.01 : 0.14; 

% Q 和 R 设置
Q = diag([300, 5, 10, 1, 150, 1]); 
R = diag([1.5, 1.5]);

fprintf('\n计算完成。以下是针对不同腿长 L0 的 LQR 增益矩阵 (Row 1)。\n');
fprintf('将这些值填入你的控制器: T_wheel = -K * State\n');
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
    
    % 提取轮子反馈增益
    K_wheel = K(1, :);
    
    fprintf(' %4.1f  | %9.3f %9.3f %9.3f %9.3f %9.3f %9.3f\n', ...
        L_total*100, K_wheel(1), K_wheel(2), K_wheel(3), K_wheel(4), K_wheel(5), K_wheel(6));
end
fprintf('----------------------------------------------------------------------------------------\n');