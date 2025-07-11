% 提取数据（从第二组数据开始，去掉第一个数据点）
x_target = Xt1.Data(2:end);      % 从第二个元素开始
y_target = Yt1.Data(2:end);      % 从第二个元素开始
x_missile = Xm1.Data(2:end);     % 从第二个元素开始
y_missile = Ym1.Data(2:end);     % 从第二个元素开始
timeout = timeout1.Data(2:end);  % 从第二个元素开始
time = timeout1.Time(2:end);     % 时间轴同步截断
an = an1.Data(2:end);            % 从第二个元素开始

% 绘制法向加速度图像（使用截断后的数据）
max_an = max(an);  % 数据的最大值
[~, max_idx] = max(an);

figure;
plot(time, an, 'LineWidth', 1.5);
hold on;

plot(time(max_idx), max_an, 'ro', 'MarkerFaceColor', 'r');
text(time(max_idx), max_an, sprintf('  最大法向过载: %.2f m/s^2', max_an), ...
     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');

hold off;
xlabel('时间 (s)');
ylabel('法向加速度 (m/s^2)');
title('导弹法向加速度随时间变化');
grid on; 

% 处理相撞点并截断数据（注意索引对齐）
if length(x_target) ~= length(y_target) || length(x_missile) ~= length(y_missile)
    error('数据长度不一致，请检查数据。');
end

collision_threshold = 1; % 相撞距离阈值

% 寻找相撞点索引（数据已从第二组开始，原索引i对应原数据i+1）
collision_index = [];
for i = 1:length(x_target)
    distance = sqrt((x_target(i) - x_missile(i))^2 + (y_target(i) - y_missile(i))^2);
    if distance < collision_threshold
        collision_index = i;
        break;
    end
end

% 处理相撞结果
if ~isempty(collision_index)
    fprintf('导弹与目标相撞时间为 %.2f 秒，相撞点坐标为 (%.2f, %.2f)\n', ...
            time(collision_index), x_target(collision_index), y_target(collision_index));
    % 截断数据至相撞点（注意索引范围）
    x_target = x_target(1:collision_index);
    y_target = y_target(1:collision_index);
    x_missile = x_missile(1:collision_index);
    y_missile = y_missile(1:collision_index);
else
    fprintf('未找到相撞点。\n');
end

% 绘制静态轨迹图（使用截断后的数据）
figure;
plot(x_target, y_target, 'b', 'DisplayName', '目标轨迹');
hold on;
plot(x_missile, y_missile, 'r', 'DisplayName', '导弹轨迹');

% 标记相撞点（如果存在）
if ~isempty(collision_index)
    collision_x = x_target(end);
    collision_y = y_target(end);
    plot(collision_x, collision_y, 'go', 'MarkerFaceColor', 'g', 'DisplayName', '相撞点');
    text(collision_x, collision_y, sprintf('(%.2f, %.2f)', collision_x, collision_y), ...
         'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'Color', 'g');
end

hold off;
xlabel('X 坐标');
ylabel('Y 坐标');
title('三点法导引弹道轨迹');
legend('Location', 'northwest');