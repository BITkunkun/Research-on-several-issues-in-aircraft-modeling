sim('Lead_Guidance_Method.slx');
sim("Semi_Lead_Guidance_Method.slx");
sim("Three_point_guidance_law.slx");
% 三点法
x_target_1 = Xt1.Data(5:end);
y_target_1 = Yt1.Data(5:end);
x_missile_1 = Xm1.Data(5:end);
y_missile_1 = Ym1.Data(5:end);
an_1 = an1.Data(5:end);
time_1 = timeout1.Data(5:end);
depsilon_1 = depsilon1.Data(5:end);
time_11 = timeout1.Time(5:end);
dtheta_1 = dtheta1.Data(5:end);

% 半前置量法
x_target_2 = Xt2.Data(10:end);
y_target_2 = Yt2.Data(10:end);
x_missile_2 = Xm2.Data(10:end);
y_missile_2 = Ym2.Data(10:end);
an_2 = an2.Data(10:end);
time_2 = timeout2.Data(10:end);
depsilon_2 = depsilon2.Data(10:end);
time_22 = timeout2.Time(10:end);
dtheta_2 = dtheta2.Data(10:end);

% 前置量法
x_target_3 = Xt3.Data(10:end);
y_target_3 = Yt3.Data(10:end);
x_missile_3 = Xm3.Data(10:end);
y_missile_3 = Ym3.Data(10:end);
an_3 = an3.Data(10:end);
time_3 = timeout3.Data(10:end);
depsilon_3 = depsilon3.Data(10:end);
time_33 = timeout3.Time(10:end);
dtheta_3 = dtheta3.Data(10:end);

% ====================== 样式定义 ======================
colors = {[1 0 0], [0 1 0], [0 0 1]};
labels = {'三点法', '半前置量法', '前置量法'};
line_width = 2;

% ====================== 数据范围计算 ======================
% 计算所有数据的最大值（忽略初始点）
x_max = max([...
    x_target_1; x_target_2; x_target_3; ...
    x_missile_1; x_missile_2; x_missile_3...
]);
y_max = max([...
    y_target_1; y_target_2; y_target_3; ...
    y_missile_1; y_missile_2; y_missile_3...
]);
x_padding = x_max * 0.1;  % 10%边界扩展
y_padding = y_max * 0.1;

collision_threshold = 1;  % 相撞距离阈值(m)
collision_points = cell(3, 2);  % 存储相撞点坐标

figure('Color', 'white');
hold on;

% 绘制目标轨迹（取前置量法的目标轨迹）
target_plot = plot(x_target_3, y_target_3, 'k--', 'LineWidth', line_width - 0.5);

% 遍历三种导引方法
missile_plots = cell(3, 1);
for method_id = 1:3
    % 选择数据
    switch method_id
        case 1
            x_target = x_target_1;
            y_target = y_target_1;
            x_missile = x_missile_1;
            y_missile = y_missile_1;
            time = time_1;
        case 2
            x_target = x_target_2;
            y_target = y_target_2;
            x_missile = x_missile_2;
            y_missile = y_missile_2;
            time = time_2;
        case 3
            x_target = x_target_3;
            y_target = y_target_3;
            x_missile = x_missile_3;
            y_missile = y_missile_3;
            time = time_3;
    end

    % 相撞点检测（在截断后的数据中查找）
    collision_index = [];
    for i = 1:length(x_target)
        distance = sqrt(...
            (x_target(i) - x_missile(i))^2 + ...
            (y_target(i) - y_missile(i))^2 ...
        );
        if distance < collision_threshold
            collision_index = i;
            break;
        end
    end

    % 处理相撞结果
    if ~isempty(collision_index)
        collision_x = x_target(collision_index);
        collision_y = y_target(collision_index);
        collision_points{method_id, 1} = collision_x;
        collision_points{method_id, 2} = collision_y;
        fprintf('[%s] 相撞时间: %.2fs, 坐标: (%.2fm, %.2fm)\n',...
                labels{method_id}, time(collision_index), collision_x, collision_y);
        plot(collision_x, collision_y, 'go', 'MarkerFaceColor', 'g');
    end

    % 绘制导弹轨迹
    missile_plots{method_id} = plot(...
        x_missile, y_missile,...
        'Color', colors{method_id},...
        'LineWidth', line_width,...
        'DisplayName', labels{method_id}...
    );
end

% ====================== 图形美化 ======================
hold off;
grid on;
axis([0 x_max+x_padding 0 y_max+y_padding]);
set(gca,...
    'FontSize', 12,...
    'GridAlpha', 0.3,...
    'Box', 'off'...
);
xlabel('X 坐标 (m)');
ylabel('Y 坐标 (m)');
title('三种导引方法弹道对比');

% 构建图例
legend_labels = [{'目标轨迹'}, labels];
legend([target_plot, missile_plots{:}], legend_labels, 'Location', 'northwest');

% ====================== 分导引方法绘制 ======================
g = 9.81; % 重力加速度
for method_id = 1:3
    figure('Color', 'white');
    hold on;
    
    % 选择数据
    switch method_id
        case 1
            x_target = x_target_1;
            y_target = y_target_1;
            x_missile = x_missile_1;
            y_missile = y_missile_1;
            time = time_1;
            an = an_1;
        case 2
            x_target = x_target_2;
            y_target = y_target_2;
            x_missile = x_missile_2;
            y_missile = y_missile_2;
            time = time_2;
            an = an_2;
        case 3
            x_target = x_target_3;
            y_target = y_target_3;
            x_missile = x_missile_3;
            y_missile = y_missile_3;
            time = time_3;
            an = an_3;
    end

    % 绘制目标轨迹
    plot(x_target, y_target, 'k--', 'LineWidth', line_width-0.5, 'DisplayName', '目标轨迹');

    % 绘制导弹轨迹
    plot(x_missile, y_missile,...
        'Color', colors{method_id},...
        'LineWidth', line_width,...
        'DisplayName', labels{method_id}...
    );

    % 标记相撞点
    if ~isempty(collision_points{method_id, 1})
        collision_x = collision_points{method_id, 1};
        collision_y = collision_points{method_id, 2};
        plot(collision_x, collision_y, 'go',...
            'MarkerFaceColor', 'g',...
            'DisplayName', '相撞点'...
        );
        text(collision_x, collision_y,...
            sprintf('(%.2f, %.2f)', collision_x, collision_y),...
            'VerticalAlignment', 'bottom',...
            'HorizontalAlignment', 'left',...
            'Color', 'g'...
        );
    end

    % 图形美化
    hold off;
    grid on;
    axis([0 x_max+x_padding 0 y_max+y_padding]);
    set(gca, 'FontSize', 12, 'GridAlpha', 0.3, 'Box', 'off');
    xlabel('X 坐标 (m)');
    ylabel('Y 坐标 (m)');
    title(sprintf('%s导引弹道轨迹', labels{method_id}));
    legend('Location', 'northwest');

    % ====================== 法向过载图 ======================
    figure('Color', 'white');
    
    % 计算法向过载
    overload = an / g;
    
    % 绘制法向过载曲线
    plot(time, overload, 'LineWidth', 1.5);
    hold on;
    
    % 标记最大过载点
    [max_overload, max_idx] = max(abs(overload));
    max_overload = overload(max_idx);
    plot(time(max_idx), max_overload,...
        'ro', 'MarkerFaceColor', 'r',...
        'DisplayName', '峰值点'...
    );
    text(time(max_idx), max_overload,...
        sprintf('  最大过载: %.2f g', max_overload),...
        'VerticalAlignment', 'bottom',...
        'HorizontalAlignment', 'left'...
    );
    
    % 坐标轴设置
    y_lim = max(abs(overload)) * 1.2;  % 扩展20%边界
    ylim([-y_lim, y_lim]);
    grid on;
    xlabel('时间 (s)');
    ylabel('法向过载 (g)');
    title(sprintf('%s法向过载变化', labels{method_id}));
    
    % ====================== 法向加速度图 ======================
    figure('Color', 'white');
    
    % 绘制法向加速度曲线
    plot(time, an, 'LineWidth', 1.5);
    hold on;
    
    % 标记最大加速度点
    [max_an, max_idx] = max(abs(an));
    max_an = an(max_idx);
    plot(time(max_idx), max_an,...
        'ro', 'MarkerFaceColor', 'r',...
        'DisplayName', '峰值点'...
    );
    text(time(max_idx), max_an,...
        sprintf('  最大加速度: %.2f m/s²', max_an),...
        'VerticalAlignment', 'bottom',...
        'HorizontalAlignment', 'left'...
    );
    
    % 坐标轴设置
    y_lim = max(abs(an)) * 1.2;  % 扩展20%边界
    ylim([-y_lim, y_lim]);
    grid on;
    xlabel('时间 (s)');
    ylabel('法向加速度 (m/s²)');
    title(sprintf('%s法向加速度变化', labels{method_id}));
end

% ====================== 导弹高低角导数绘图 ======================
for method_id = 1:3
    figure('Color', 'white');
    switch method_id
        case 1
            depsilon = depsilon_1;
            time = time_11;
        case 2
            depsilon = depsilon_2;
            time = time_22;
        case 3
            depsilon = depsilon_3;
            time = time_33;
    end
    
    % 绘制导弹高低角导数曲线
    plot(time, depsilon, 'LineWidth', 1.5);
    hold on;
    
    % % 标记最大高低角导数点
    % [max_depsilon, max_idx] = max(abs(depsilon));
    % plot(time(max_idx), depsilon(max_idx),...
    %     'bo', 'MarkerFaceColor', 'b',...
    %     'DisplayName', '峰值点'...
    % );
    % text(time(max_idx), depsilon(max_idx),...
    %     sprintf('  最大高低角导数: %.2f rad/s', max_depsilon),...
    %     'VerticalAlignment', 'bottom',...
    %     'HorizontalAlignment', 'left'...
    % );
    
    % 坐标轴设置
    y_lim = max(abs(depsilon)) * 1.2;  % 扩展20%边界
    ylim([-y_lim, y_lim]);
    grid on;
    xlabel('时间 (s)');
    ylabel('导弹高低角导数 (rad/s)');
    title(sprintf('%s导弹高低角导数变化', labels{method_id}));
end   

% 导弹转弯速率绘图
for method_id = 1:3
    figure('Color', 'white');
    switch method_id
        case 1
            dtheta = dtheta_1 * (180 / pi); % 将弧度转换为角度
            time = time_11;
        case 2
            dtheta = dtheta_2 * (180 / pi); % 将弧度转换为角度
            time = time_22;
        case 3
            dtheta = dtheta_3 * (180 / pi); % 将弧度转换为角度
            time = time_33;
    end
    
    % 绘制导弹转弯速率曲线
    plot(time, dtheta, 'LineWidth', 1.5);
    hold on;
    
    % 标记最大转弯速率点
    [max_dtheta, max_idx] = max(abs(dtheta));
    plot(time(max_idx), dtheta(max_idx), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', '峰值点');
    text(time(max_idx), dtheta(max_idx), sprintf('  最大转弯速率: %.3f °/s', max_dtheta),... % 修改文本显示为角度单位
        'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
    
    % 坐标轴设置
    y_lim = max(abs(dtheta)) * 1.2;  % 扩展20%边界
    ylim([-y_lim, y_lim]);
    grid on;
    xlabel('时间 (s)');
    ylabel('导弹转弯速率 (°/s)'); % 修改y轴标签为角度每秒
    title(sprintf('%s导弹转弯速率变化', labels{method_id}));
end