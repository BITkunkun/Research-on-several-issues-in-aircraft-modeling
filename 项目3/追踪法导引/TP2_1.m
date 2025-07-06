% 定义 r0 向量
r0 = linspace(10, 1200, 300);

function f = my_area(r0, q0)
    n_max = 5;
    g = 9.81;
    V = 250;
    Vt = 200;
    p = V / Vt;
    left_const = V * Vt / (g * n_max) * ((1 - p/2)^(1/2)) * ((1 + p/2)^(1/2)) * ((2 + p)^((p + 1) / 2)) / (2 * (2 - p) ^ ((p - 1) / 2));
    f = r0 * sin(q0) / (tan(q0 / 2)^p) - left_const;
end

% 求解 q 值
q0 = [];
for i = 1:300
    % 调用修改后的函数名
    x = fsolve(@(q0) my_area(r0(i), q0), 1);
    q0 = [q0, x];
end

% 找到最外侧圆的半径（r0 的最大值）
max_r0 = max(r0);

% 找到最外侧圆半径对应的索引
max_index = find(r0 == max_r0);

% 获取对应的角度
intersection_angles = [q0(max_index), -q0(max_index)];
max_deg_1 = rad2deg(intersection_angles(1));
max_deg_2 = rad2deg(intersection_angles(2));

% 绘制极坐标图，修改图线颜色和线宽
h1 = polarplot(q0, r0, 'r', 'LineWidth', 1.5);
hold on;
h2 = polarplot(-q0, r0, 'g', 'LineWidth', 1.5);

% 标记与最外侧圆的交点
polarplot(intersection_angles(1), max_r0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
polarplot(intersection_angles(2), max_r0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');

hold off;

% 打印与最外侧圆相交的角度
fprintf('与最外侧圆相交的角度为: %.6f 角度 和 %.6f 角度\n', max_deg_1, max_deg_2);

