%% 多速度比弹道仿真分析（修正颜色和图例）
clear;clc;
% 参数设置
V_target = 100;          % 目标速度 (m/s)
K = [1.5, 2, 3];           % 速度比研究参数 (导弹速度/目标速度)
n_max = 5;               % 最大允许法向过载 (g)
g = 9.81;                % 重力加速度
r0 = 3000;               % 初始相对距离 (m)
q0 = deg2rad(45);        % 初始视线角
dt = 0.01;               % 时间步长
sim_time = 30;           % 最大仿真时间

% 预分配存储空间
results = struct('trajectory',[], 'n_load',[], 't',[], 'target_end',[]);
initial_target = [r0*cos(q0), r0*sin(q0)]; % 目标初始坐标

%% 多速度比循环仿真
for k_idx = 1:length(K)
    % 当前速度比参数
    V_missile = K(k_idx)*V_target;
    
    % 初始化状态变量
    [x_missile, y_missile, x_target, y_target] = deal(zeros(1,sim_time/dt));
    [r, q, t, n] = deal(zeros(1,sim_time/dt));
    
    % 初始条件
    x_missile(1) = 0;    % 导弹初始位置
    y_missile(1) = 0;
    x_target(1) = initial_target(1);
    y_target(1) = initial_target(2);
    r(1) = r0;
    q(1) = q0;
    t(1) = 0;
    i = 1;
    
    % 仿真循环
    while i < sim_time/dt && r(i) > 1
        i = i + 1;
        
        % 运动学方程
        drdt = V_target*cos(q(i-1)) - V_missile;
        dqdt = V_target*sin(q(i-1))/r(i-1);
        
        % 过载计算与约束
        a_n = V_missile*dqdt;
        current_n = a_n/g;
        
        % 过载限制逻辑
        if abs(current_n) > n_max
            dqdt = sign(dqdt)*n_max*g/V_missile;
            current_n = sign(current_n)*n_max;
        end
     
        % 状态更新
        r(i) = r(i-1) + drdt*dt;
        q(i) = q(i-1) + dqdt*dt;
        t(i) = t(i-1) + dt;
        n(i) = current_n;
        
        % 更新位置
        x_target(i) = x_target(i-1) + V_target*dt;
        y_target(i) = y_target(i-1);  % Y坐标保持恒定
        x_missile(i) = x_target(i) - r(i)*cos(q(i));
        y_missile(i) = y_target(i) - r(i)*sin(q(i));
    end
    
    % 存储结果
    results(k_idx).trajectory = [x_missile(1:i); y_missile(1:i)];
    results(k_idx).n_load = n(1:i);
    results(k_idx).t = t(1:i);
    results(k_idx).target_end = [x_target(i), y_target(i)]; % 记录目标终点
end

%% 结果可视化
% 自定义颜色方案（蓝、红、绿）
missile_colors = [0 0.4470 0.7410;    % 蓝色
                 0.8500 0.3250 0.0980; % 红色
                 0.4660 0.6740 0.1880]; % 绿色
target_color = [0 0 0];               % 黑色
collision_color = [0.6350 0.0780 0.1840]; % 深红色
start_color = [0.9290 0.6940 0.1250];      % 目标起始点黄色

% 轨迹对比图
figure('Color','w','Position',[100 100 1200 500])
subplot(1,2,1)
hold on
legend_entries = {};

% 绘制目标初始位置标记（目标起始点）
plot(initial_target(1), initial_target(2), 'o',...
    'MarkerSize',6, 'MarkerFaceColor',start_color, 'Color',start_color)
legend_entries{end+1} = '目标起始点';

% 绘制各速度比弹道
for k = 1:length(K)
    % 导弹轨迹（蓝、红、绿）
    plot(results(k).trajectory(1,:), results(k).trajectory(2,:),...
        'Color',missile_colors(k,:), 'LineWidth',1.8)
    legend_entries{end+1} = sprintf('K=%.1f (导弹)', K(k));
    
    % 目标轨迹线段（黑色虚线，仅绘制一次）
    if k == 1
        plot([initial_target(1), results(k).target_end(1)],...
            [initial_target(2), results(k).target_end(2)],...
            '--', 'Color',target_color, 'LineWidth',1.3)
        legend_entries{end+1} = '目标轨迹';
    end
    
    % 标注碰撞点（仅绘制一次）
    
end
for k = 1:length(K)
        plot(results(k).target_end(1), results(k).target_end(2),...
            'o', 'MarkerSize',5, 'MarkerFaceColor',collision_color, 'Color',collision_color)
        
end
legend_entries{end+1} = '碰撞点';
% 添加地面参考线
plot([0 max(xlim)], [0 0], 'k-', 'LineWidth',1.2)

title('不同速度比弹道对比')
xlabel('X位置 (m)'), ylabel('Y位置 (m)')
legend(legend_entries, 'Location','best', 'Box','off')
grid on
axis equal
ylim([0 1.1*max(ylim)])

% 过载曲线对比图（保持原有逻辑）
subplot(1,2,2)
hold on

% 动态生成速度比对应的图例条目
legend_entries = cell(1, length(K));
for k = 1:length(K)
    plot(results(k).t, results(k).n_load,...
        'Color', missile_colors(k,:), 'LineWidth', 1.8);
    legend_entries{k} = sprintf('K=%.1f', K(k)); % 按实际K值生成图例文本
end

yline(n_max, '--', 'Color', target_color, 'LineWidth', 1.5, 'DisplayName', '过载限制');
title('法向过载发展曲线');
xlabel('时间 (s)'), ylabel('法向过载 (g)');
legend([legend_entries, {'过载限制'}], 'Location', 'best', 'Box', 'off'); % 合并速度比和过载限制图例
grid on;

%% 关键参数分析表（保持原有逻辑）
fprintf('\n======= 拦截性能对比 =======\n')
fprintf('速度比 | 拦截时间(s) | 最大过载(g) | 目标终止X坐标(m)\n')
for k = 1:length(K)
    fprintf(' %.1f  |     %.2f    |     %.2f    |     %.0f\n',...
        K(k), results(k).t(end), max(abs(results(k).n_load)), results(k).target_end(1))
end