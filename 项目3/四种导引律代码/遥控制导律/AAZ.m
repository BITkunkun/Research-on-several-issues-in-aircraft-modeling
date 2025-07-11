% 定义已知参数
V_T = 12;       % 目标速度 (m/s)
H = 3000;       % 初始距离 (m)
P = 2;          % 速度比 V / V_T
dtheta = 0.01;   % 转弯速率 (rad/s)

% 计算导弹速度 V
V = P * V_T;

% 计算无量纲参数 K
K = (dtheta * H) / V_T;

% 定义 epsilon_T 的范围（单位：弧度）
epsilon_T_rad_range = linspace(0.01, pi/2 - 0.01, 1000);  % 避免奇点，1000 个点更平滑

% 初始化结果存储
valid_epsilon_T = [];
valid_R_M = [];

% 定义 Rm 的范围
Rm_min = 0;  % 最小 Rm (m)
Rm_max = 20000;  % 最大 Rm (m)

% 对每个 epsilon_T 求解 R_M
for i = 1:length(epsilon_T_rad_range)
    epsilon_T = epsilon_T_rad_range(i);
    sin_eps = sin(epsilon_T);
    sin2_eps = sin(2*epsilon_T);
    
    % 物理约束：根号内必须非负
    R_M_max_physical = (P * H) / (sin_eps^2);
    
    % 定义方程函数
    fun = @(R_M) sin_eps^2 * (2 + (R_M * sin2_eps) / sqrt((P*H)^2 - (R_M * sin_eps^2)^2)) - K;
    
    % 初始猜测值
    R_M_guess = 0.5 * R_M_max_physical;
    
    % 求解方程
    options = optimoptions('fsolve', 'Display', 'off', 'TolFun', 1e-8, 'TolX', 1e-8);
    try
        [R_M_sol, fval] = fsolve(fun, R_M_guess, options);
        % 检查解是否满足物理约束和 Rm 范围
        if R_M_sol >= Rm_min && R_M_sol <= Rm_max && R_M_sol <= R_M_max_physical && abs(fval) < 1e-6
            valid_epsilon_T = [valid_epsilon_T, epsilon_T];
            valid_R_M = [valid_R_M, R_M_sol];
        end
    catch
        % 忽略无解的情况
    end
end

% 检查是否有有效数据
if isempty(valid_epsilon_T)
    error('在指定的 Rm 范围内未找到有效解，请检查参数或约束条件。');
end

% 计算新坐标系
x = valid_R_M .* cos(valid_epsilon_T);  % 横坐标: R_M * cos(epsilon_T)
y = valid_R_M .* sin(valid_epsilon_T);  % 纵坐标: R_M * sin(epsilon_T)

% 绘制结果
figure('Color', [1,1,1]);
plot(x, y, 'b-', 'LineWidth', 1.5);  % 使用折线图

% 添加 y = H 的目标运动轨迹
x_min = min(x);
x_max = max(x);
x_target = [x_min, x_max];
y_target = [H, H];
hold on;
plot(x_target, y_target, 'r--', 'LineWidth', 1.5);
hold off;

title('允许攻击区范围边界线');
xlabel('X (m)');
ylabel('Y (m)');
grid on;
axis equal;  % 保持 X 和 Y 轴比例相等

% 添加线标
% legend('允许攻击区范围边界线', '目标运动轨迹');    
legend('允许攻击区范围边界线');    