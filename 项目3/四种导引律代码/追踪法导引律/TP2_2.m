% 初始化参数
V_missile = 380;    % 导弹速度 (m/s)
V_target = 200;     % 目标速度 (m/s)
r0 = 3000;          % 初始相对距离 (m)
q0 = pi/4;          % 初始视线角（目标在导弹右上方45°）
dt = 0.01;          % 时间步长 (s)
collision_thresh = 1; % 碰撞阈值
g = 9.81;           % 重力加速度 (m/s^2)
n_max = 4;          % 法向可用过载

% 检查导弹速度是否大于目标速度
if abs(V_missile) <= abs(V_target)
    error('追踪法要求导弹速度必须大于目标速度，请重新设置参数。');
end

% 初始位置（绝对坐标系）
x_missile(1) = 0;   % 导弹在原点
y_missile(1) = 0;
x_target(1) = r0 * cos(q0); % 目标初始位置（右上方）
y_target(1) = r0 * sin(q0);

% 状态初始化
r(1) = r0;
q(1) = q0;
t(1) = 0;
i = 1;
n(1) = 0; % 初始化法向过载

% 仿真循环（直至碰撞或距离增大）
while i < 20000 && r(i) > collision_thresh
    i = i + 1;
    
    % 运动学方程（追踪法）
    drdt = V_target * cos(q(i-1)) - V_missile;
    dqdt = (V_target * sin(q(i-1))) / r(i-1);
    
    % 计算法向加速度
    a_n = V_missile * dqdt;
    
    % 计算法向过载
    n(i) = a_n / g;
    
    % 检查法向过载是否超过可用过载
    if n(i) > n_max
        % 调整导弹速度方向，使法向过载不超过可用过载
        dqdt = n_max * g / V_missile;
        n(i) = n_max; % 确保法向过载不超过可用过载
    end
    
    % 数值积分（欧拉法）
    r(i) = r(i-1) + drdt * dt;
    q(i) = q(i-1) + dqdt * dt;
    t(i) = t(i-1) + dt;
    
    % 更新目标位置（沿X轴匀速）
    x_target(i) = x_target(i-1) + V_target * dt;
    y_target(i) = y_target(i-1);  % Y坐标不变
    
    % 计算导弹绝对位置
    x_missile(i) = x_target(i) - r(i) * cos(q(i));
    y_missile(i) = y_target(i) - r(i) * sin(q(i));
    
    % 防止距离发散（非物理情况终止）
    if r(i) > 1.5*r0
        break;
    end
end

% 绘制纯轨迹（按照新要求修改）
figure('Color','white','Position',[100 100 800 600]);
plot(x_missile(1:i), y_missile(1:i), '-', 'color', [0 0.4470 0.7410], 'LineWidth', 2); % 导弹轨迹（蓝色）
hold on;
plot(x_target(1:i), y_target(1:i), '-', 'color', [0.4660 0.6740 0.1880], 'LineWidth', 2); % 目标轨迹（绿色）

% 标注起点和碰撞点
plot(x_missile(1), y_missile(1), 'o', 'MarkerSize', 8, 'MarkerFaceColor',[0.9290 0.6940 0.1250], 'Color', [0.9290 0.6940 0.1250]); % 导弹轨迹起点（黄色）
plot(x_target(1), y_target(1), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.9290 0.6940 0.1250], 'Color', [0.9290 0.6940 0.1250]); % 目标轨迹起点（黄色）
plot(x_missile(i), y_missile(i), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.8500 0.3250 0.0980], 'Color', [0.8500 0.3250 0.0980]); % 碰撞点（红色）

hold off;

% 设置图形属性
grid on;
xlabel('X 坐标 (m)');
ylabel('Y 坐标 (m)');
title('追踪法制导轨迹');
legend('导弹轨迹', '目标轨迹', '导弹起点', '目标起点', '碰撞点', 'Location','southeast');
axis equal; % 保持坐标比例一致
axis tight;

% 绘制法向过载曲线图
figure('Color','white','Position',[100 100 800 600]);
plot(t(1:i), n(1:i), 'g-', 'LineWidth', 1.8); % 绿色法向过载曲线
xlabel('时间 (s)');
ylabel('法向过载');
title('导弹法向过载曲线');
grid on;
axis tight;
% 绘制法向可用过载的水平线
line([t(1), t(i)], [n_max, n_max], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
legend('法向过载', '法向可用过载', 'Location', 'southeast');