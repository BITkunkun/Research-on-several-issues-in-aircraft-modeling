% 定义参数P的范围
P_values = [2, 3, 4, 5, 8, 10];
% P_values = [3, 4, 5, 8, 10];
simResults = cell(1, length(P_values));  % 使用cell数组存储结构体

% ====================== 仿真运行与数据提取 ======================
for idx = 1:length(P_values)
    % 设置当前参数值
    P = P_values(idx);
    assignin('base', 'P', P);
    
    % 运行仿真（确保模型配置为自动导出数据到工作区）
    % sim('Three_point_guidance_law_6');
    % sim("Semi_Lead_Guidance_Method_6.slx");
    sim("Lead_Guidance_Method_6.slx");
    
    % ------------ 从工作区直接提取数据 ------------
    % 导弹状态数据
    dtheta_signal = dtheta3;
    an_signal = an3;
    Xm_signal = Xm3;
    Ym_signal = Ym3;
    
    % 目标轨迹数据
    Xt_signal = Xt3;
    Yt_signal = Yt3;
    
    % 时间轴数据
    time_signal = timeout3;
    
    % 存储到结构体
    simResults{idx} = struct(...
        'P',         P,...
        'dtheta',    dtheta_signal.Data(10:end),...
        'an',        an_signal.Data(10:end),...
        'Xm',        Xm_signal.Data(10:end),...
        'Ym',        Ym_signal.Data(10:end),...
        'Xt',        Xt_signal.Data(10:end),...
        'Yt',        Yt_signal.Data(10:end),...
        'Time',      time_signal.Time(10:end)...
    );
end

% ====================== 图形1：不同P值下导弹转弯速率变化 ======================
figure('Color', 'white');
hold on;
colors = lines(length(P_values));  %使用不同颜色区分P值
for idx = 1:length(P_values)
    plot(simResults{idx}.Time, simResults{idx}.dtheta,...
        'Color', colors(idx,:), 'LineWidth', 1.5,...
        'DisplayName', ['P=', num2str(P_values(idx))]);
end
xlabel('时间 (s)', 'FontSize', 12);
ylabel('导弹转弯速率 (rad/s)', 'FontSize', 12);
title('不同P值下导弹转弯速率变化', 'FontSize', 14);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 11, 'GridAlpha', 0.3);
hold off;

% ====================== 图形2：不同P值下法向加速度 ======================
figure('Color', 'white');
hold on;
colors = lines(length(P_values));  
marker_style = {'o','s', 'd', '^', 'v', '>'};  

for idx = 1:length(P_values)
    % 绘制法向加速度曲线
    h_plot = plot(simResults{idx}.Time, simResults{idx}.an,...
        'Color', colors(idx,:), 'LineWidth', 1.5,...
        'DisplayName', ['P = ', num2str(P_values(idx)), '法向加速度曲线']);
    
    % 寻找法向加速度最大值及其索引（比较绝对值）
    [max_abs_an, max_an_idx] = max(abs(simResults{idx}.an));  
    max_an = simResults{idx}.an(max_an_idx);  % 获取实际值（可能为负）
    max_time = simResults{idx}.Time(max_an_idx);
    
    % 标记最大值点
    plot(max_time, max_an,...
        'Marker', marker_style{mod(idx - 1, 6) + 1},...
        'MarkerSize', 6,...
        'MarkerFaceColor', colors(idx,:),...
        'MarkerEdgeColor', 'k',...
        'HandleVisibility', 'off'); 
    
    % 添加数值标签（显示实际值）
    text(max_time, max_an,...
        sprintf('Max: %.3f m/s²', max_an),...  
        'VerticalAlignment', 'bottom',...
        'HorizontalAlignment', 'left',...
        'FontSize', 10,...
        'Color', colors(idx,:));
end

xlabel('时间 (s)', 'FontSize', 12);
ylabel('法向加速度 (m/s²)', 'FontSize', 12);
title('不同P值下导弹法向加速度', 'FontSize', 14); 
legend('Location', 'best', 'NumColumns', 2);
grid on;
set(gca, 'FontSize', 11, 'GridAlpha', 0.3);
hold off;

% ====================== 图形3：导弹与目标运动轨迹对比(修改版) ======================
figure('Color', 'white');
hold on;

% 统一绘制所有目标轨迹(黑色虚线)并隐藏图例
cellfun(@(x) plot(x.Xt, x.Yt, 'k--', 'LineWidth', 0.8,...
    'HandleVisibility', 'off'), simResults); 

% 初始化控制变量
collision_legend_added = false;  %控制相撞点图例
missile_handles = gobjects(1, length(P_values));
collision_threshold = 1;

% 收集所有X和Y坐标数据
all_X = [];
all_Y = [];
for idx = 1:length(P_values)
    data = simResults{idx};
    all_X = [all_X; data.Xm; data.Xt];
    all_Y = [all_Y; data.Ym; data.Yt];
end

% 计算坐标轴范围
x_min = min(all_X);
x_max = max(all_X);
y_min = min(all_Y);
y_max = max(all_Y);
x_padding = (x_max - x_min) * 0.1;
y_padding = (y_max - y_min) * 0.1;
x_lim = [x_min - x_padding, x_max + x_padding];
y_lim = [y_min - y_padding, y_max + y_padding];

% 绘制每个P值的导弹轨迹和碰撞点
for idx = 1:length(P_values)
    data = simResults{idx};
    
    % 绘制导弹轨迹（存储句柄用于图例）
    missile_handles(idx) = plot(data.Xm, data.Ym, '-',...
        'Color', colors(idx,:), 'LineWidth', 1.8,...
        'DisplayName', ['P=', num2str(P_values(idx))]); %显示导弹轨迹图例
    
    % 寻找相撞点
    distances = sqrt((data.Xt - data.Xm).^2 + (data.Yt - data.Ym).^2);
    collision_idx = find(distances < collision_threshold, 1);
    
    if ~isempty(collision_idx)
        collision_x = data.Xt(collision_idx);
        collision_y = data.Yt(collision_idx);
        
        % 命令行输出坐标
        fprintf('P=%2d 相撞点坐标: X = %8.2f m, Y = %8.2f m\n',...
                P_values(idx), collision_x, collision_y);
        
        % 标记相撞点（仅第一个添加图例）
        if ~collision_legend_added
            plot(collision_x, collision_y, 'rp',...
                'MarkerSize', 12,...
                'MarkerFaceColor', 'r',...
                'DisplayName', '相撞点');  %添加图例
            collision_legend_added = true;
        else
            plot(collision_x, collision_y, 'rp',...
                'MarkerSize', 12,...
                'MarkerFaceColor', 'r',...
                'HandleVisibility', 'off'); %不显示图例
        end
    end
end

% 图形美化
axis([x_lim y_lim]);
xlabel('X 坐标 (m)', 'FontSize', 12);
ylabel('Y 坐标 (m)', 'FontSize', 12);
title('导弹与目标运动轨迹', 'FontSize', 14);

% 创建组合图例（导弹轨迹+相撞点）
legend([missile_handles, findobj(gca,'Marker','p')],...
       'Location', 'eastoutside',...
       'Box', 'off');

grid on;
set(gca, 'FontSize', 11, 'GridAlpha', 0.3);
hold off;

% ====================== 图形4：不同P值下导弹法向过载 ======================
g = 9.81;
figure('Color', 'white');
hold on;
colors = lines(length(P_values));
for idx = 1:length(P_values)
    overload = simResults{idx}.an / g;
    plot(simResults{idx}.Time, overload,...
        'Color', colors(idx,:), 'LineWidth', 1.5,...
        'DisplayName', ['P = ', num2str(P_values(idx)), '法向过载曲线']);
    
    % 寻找法向过载最大值及其索引（比较绝对值）
    [max_abs_overload, max_overload_idx] = max(abs(overload));  
    max_overload = overload(max_overload_idx);  % 获取实际值（可能为负）
    max_overload_time = simResults{idx}.Time(max_overload_idx);
    
    % 标记最大值点
    plot(max_overload_time, max_overload,...
        'Marker', marker_style{mod(idx - 1, 6) + 1},...
        'MarkerSize', 6,...
        'MarkerFaceColor', colors(idx,:),...
        'MarkerEdgeColor', 'k',...
        'HandleVisibility', 'off'); 
    
    % 添加数值标签（显示实际值）
    text(max_overload_time, max_overload,...
        sprintf('Max: %.3f g', max_overload),...  
        'VerticalAlignment', 'bottom',...
        'HorizontalAlignment', 'left',...
        'FontSize', 10,...
        'Color', colors(idx,:));
end
xlabel('时间 (s)', 'FontSize', 12);
ylabel('导弹法向过载', 'FontSize', 12);
title('不同P值下导弹法向过载', 'FontSize', 14);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 11, 'GridAlpha', 0.3);
hold off;