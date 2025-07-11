% MATLAB 代码：绘制三点法导引的轨迹、转弯速率、法向加速度和过载（含制导站）
clear all; close all; clc;
sim("CC_Three_point_guidance_law.slx");

%% 数据预处理（统一截断第一个数据点）
time = timeout.Time(2:end);     % 时间轴截断
Xc = Xc.Data(2:end);            % 制导站X坐标
Yc = Yc.Data(2:end);            % 制导站Y坐标
Xm = Xm.Data(2:end);            % 导弹X坐标
Ym = Ym.Data(2:end);            % 导弹Y坐标
Xt = Xt.Data(2:end);            % 目标X坐标
Yt = Yt.Data(2:end);            % 目标Y坐标
dtheta = dtheta.Data(2:end);    % 转弯速率
an = an.Data(2:end);            % 法向加速度
overload = an / 9.81;           % 法向过载

%% 相撞点检测（动态截断数据）
collision_threshold = 1;         % 相撞判定阈值
collision_index = [];
for i = 1:length(Xm)
    distance = sqrt((Xm(i)-Xt(i))^2 + (Ym(i)-Yt(i))^2);
    if distance < collision_threshold
        collision_index = i;
        break;
    end
end

% 统一截断到相撞点（若存在）
if ~isempty(collision_index)
    time = time(1:collision_index);
    Xc = Xc(1:collision_index);
    Yc = Yc(1:collision_index);
    Xm = Xm(1:collision_index);
    Ym = Ym(1:collision_index);
    Xt = Xt(1:collision_index);
    Yt = Yt(1:collision_index);
    dtheta = dtheta(1:collision_index);
    an = an(1:collision_index);
    overload = overload(1:collision_index);
end

%% 运动轨迹图（含制导站）
figure('Name','运动轨迹','Color','white', 'Position',[100 100 800 500]); % 加宽图形窗口
hold on;

% 绘制轨迹（保持原样式）
plot(Xc, Yc, 'r-', 'LineWidth',1.5, 'DisplayName','制导站');
plot(Xm, Ym, 'b-',  'LineWidth',1.5, 'DisplayName','导弹');
plot(Xt, Yt, 'k--', 'LineWidth',1.5, 'DisplayName','目标');

% 标注相撞点
if ~isempty(collision_index)
    plot(Xm(end), Ym(end), 'go', 'MarkerFaceColor', 'g', 'DisplayName', '相撞点');
    text(Xm(end), Ym(end),...
        sprintf('  (%.1f, %.1f)', Xm(end), Ym(end)),...
        'Color','k', 'VerticalAlignment','bottom');
end

% 设置非均匀坐标轴
axis([-50 1200 -50 3000]);  % 保持原坐标范围
daspect([1 3 1]);            % 设置数据纵横比（X:Y = 1:3）
grid on;

% 优化刻度密度
set(gca, 'XTick', -50:200:1200, 'YTick', -50:500:3000);  % Y轴500米间隔

xlabel('X 坐标 (m)'); 
ylabel('Y 坐标 (m)');
title('三点法导引运动轨迹');
legend('Location','best');

%% 转弯速率曲线（独立图形）
figure('Name','转弯速率','Color','white', 'Position',[200 200 800 400]);
[val, idx] = max(abs(dtheta));
plot(time, dtheta, 'b', 'LineWidth',1.5);
hold on;
plot(time(idx), val, 'ro', 'MarkerFaceColor','r');
text(time(idx), val,...
    sprintf(' 最大转弯速率: %.4f rad/s', val),...
    'VerticalAlignment','bottom', 'FontSize',10);
grid on;
title('导弹转弯速率变化');
xlabel('时间 (s)'); 
ylabel('dθ/dt (rad/s)');
set(gca, 'FontSize',10);

%% 法向加速度曲线（独立图形）
figure('Name','法向加速度','Color','white', 'Position',[300 300 800 400]);
[val, idx] = max(abs(an));
plot(time, an, 'b', 'LineWidth',1.5);
hold on;
plot(time(idx), val, 'ro', 'MarkerFaceColor','r');
text(time(idx), val,...
    sprintf(' 最大法向加速度: %.3f m/s²', val),...
    'VerticalAlignment','bottom', 'FontSize',10);
grid on;
title('导弹法向加速度变化');
xlabel('时间 (s)'); 
ylabel('a_n (m/s²)');
set(gca, 'FontSize',10);

%% 法向过载曲线（独立图形）
figure('Name','法向过载','Color','white', 'Position',[400 400 800 400]);
[val, idx] = max(abs(overload));
plot(time, overload, 'b', 'LineWidth',1.5);
hold on;
plot(time(idx), val, 'ro', 'MarkerFaceColor','r');
text(time(idx), val,...
    sprintf(' 最大法向过载: %.3f g', val),...
    'VerticalAlignment','bottom', 'FontSize',10);
grid on;
title('导弹法向过载变化');
xlabel('时间 (s)'); 
ylabel('n (g)');
set(gca, 'FontSize',10);
%% 控制台输出
if ~isempty(collision_index)
    fprintf('[碰撞检测] 时间: %.2fs 坐标: (%.2f, %.2f)m\n',...
            time(end), Xm(end), Ym(end));
else
    fprintf('[状态提示] 未检测到碰撞\n');
end
fprintf('[性能参数] 最大转弯速率: %.2f rad/s\n', max(abs(dtheta)));
fprintf('          最大法向过载: %.2f g\n', max(abs(overload)));