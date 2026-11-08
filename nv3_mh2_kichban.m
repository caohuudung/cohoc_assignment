
clc; clear; close all;

%% cấu hình

L1 = 100; L2 = 400; L3 = 70; L4 = 300; L5 = 60; L6 = 75;
H1 = 50;  H2 = 40;
alpha1 = 80 * pi / 180; 


T1 = 3;   % Giai đoạn 1
T2 = 10;  % Giai đoạn 2
T_total = T1 + T2; 
N = 400;

t_array = linspace(0, T_total, N);
q1 = zeros(1, N); q2 = zeros(1, N); q3 = zeros(1, N);
q4 = zeros(1, N); q5 = zeros(1, N); q6 = zeros(1, N);

% Hàm đa thức bậc 3 nội suy
S3 = @(tau) 3*tau.^2 - 2*tau.^3;

%% hàm quỹ đạo
for i = 1:N
    t = t_array(i);
    if t <= T1 
        s = S3(t / T1);
        q1(i) = -2*pi/9  + (2*pi/9)  * s;   % -40 deg -> 0 deg
        q2(i) = -pi/3    + (7*pi/18) * s;   % -60 deg -> +10 deg
        q3(i) =  pi/18   - (pi/4)    * s;   % +10 deg -> -35 deg
        q4(i) = 0;
        q5(i) = -5*pi/18 + (5*pi/36) * s;   % -50 deg -> -25 deg
        q6(i) = 0;
    else       % GĐ2: Hàn tròn (từ 3s đến 13s)
        tau = (t - T1) / T2; 
        q1(i) = 0;
        q2(i) =  pi/18;                      % +10 deg
        q3(i) = -7*pi/36;                    % -35 deg
        q4(i) = 0;
        q5(i) = -5*pi/36;                    % -25 deg
        q6(i) = 2*pi * tau;                  % Quay tròn 360 deg
    end
end

%% động học thuận

X_joints = zeros(7, N);
Y_joints = zeros(7, N);
Z_joints = zeros(7, N);

for i = 1:N
    q1_i = q1(i); q2_i = q2(i); q3_i = q3(i);
    q4_i = q4(i); q5_i = q5(i); q6_i = q6(i);

    % Ma trận biến đổi thuần nhất từng khâu 
    A01 = [cos(q1_i),  0, -sin(q1_i), L1*cos(alpha1)*cos(q1_i);
           sin(q1_i),  0,  cos(q1_i), L1*cos(alpha1)*sin(q1_i);
           0,       -1,  0,       L1*sin(alpha1);
           0,        0,  0,       1];

    A12 = [cos(q2_i), -sin(q2_i), 0, L2*cos(q2_i);
           sin(q2_i),  cos(q2_i), 0, L2*sin(q2_i);
           0,        0,       1, 0;
           0,        0,       0, 1];

    A23 = [cos(q3_i),  0,  sin(q3_i), L3*cos(q3_i);
           sin(q3_i),  0, -cos(q3_i), L3*sin(q3_i);
           0,        1,  0,       0;
           0,        0,  0,       1];

    A34 = [cos(q4_i),  0,  sin(q4_i), 0;
           sin(q4_i),  0, -cos(q4_i), 0;
           0,        1,  0,       L4;
           0,        0,  0,       1];

    A45 = [cos(q5_i),  0,  sin(q5_i), 0;
           sin(q5_i),  0, -cos(q5_i), 0;
           0,        1,  0,       0;
           0,        0,  0,       1];

    A56 = [cos(q6_i), -sin(q6_i), 0, H2*cos(q6_i);
           sin(q6_i),  cos(q6_i), 0, H2*sin(q6_i);
           0,        0,       1, L5 + L6;
           0,        0,  0,       1];

    T01 = A01;
    T02 = T01 * A12;
    T03 = T02 * A23;
    T04 = T03 * A34;
    T05 = T04 * A45;
    T06 = T05 * A56;

    O0 = [0; 0; 0];
    O1 = T01(1:3, 4);
    O2 = T02(1:3, 4);
    O3 = T03(1:3, 4);
    O4 = T04(1:3, 4);
    O5 = T05(1:3, 4);
    O6 = T06(1:3, 4);

    pts = [O0, O1, O2, O3, O4, O5, O6];
    X_joints(:, i) = pts(1, :)';
    Y_joints(:, i) = pts(2, :)';
    Z_joints(:, i) = pts(3, :)';
end
% Tọa độ điểm tác động cuối E
xE = X_joints(7, :); 
yE = Y_joints(7, :);
zE = Z_joints(7, :);

%%
fig = figure('Name', 'Mô phỏng Trực quan Cánh tay Robot 6 Trục (Mô hình 2)', ...
             'Color', [1 1 1], 'Position', [60, 60, 1100, 700]);

% đồ thị động
ax_anim = subplot(2, 2, [1 3]);
hold(ax_anim, 'on'); grid(ax_anim, 'on'); axis(ax_anim, 'equal');
xlabel(ax_anim, 'X (mm)', 'FontWeight', 'bold');
ylabel(ax_anim, 'Y (mm)', 'FontWeight', 'bold');
zlabel(ax_anim, 'Z (mm)', 'FontWeight', 'bold');
title(ax_anim, 'Mô phỏng Trực quan Không gian 3D - Robot 6-DOF (Mô hình 2)', ...
      'FontSize', 11, 'Color', [0 0.3 0.6]);

% góc nhìn/kích thước
view(ax_anim, 200, 25);
xlim(ax_anim, [-400, 600]);
ylim(ax_anim, [-400, 600]);
zlim(ax_anim, [0, 800]);



% vẽ khâu
h_link_base = plot3(ax_anim, [0 0], [0 0], [0 0], 'b-', 'LineWidth', 2);
h_link_arm1 = plot3(ax_anim, [0 0], [0 0], [0 0], 'm-', 'LineWidth', 2);
h_link_arm2 = plot3(ax_anim, [0 0], [0 0], [0 0], 'c-', 'LineWidth', 2);
h_link_wrist= plot3(ax_anim, [0 0], [0 0], [0 0], 'g-', 'LineWidth', 2);

% vẽ khớp
h_joints = plot3(ax_anim, zeros(1,7), zeros(1,7), zeros(1,7), 'ro', ...
                 'MarkerFaceColor', 'y', 'MarkerSize', 7);
h_ee     = plot3(ax_anim, 0, 0, 0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 3);

% quỹ đạo ee
h_trace  = plot3(ax_anim, NaN, NaN, NaN, 'r--', 'LineWidth', 1);


% đồ thị biến khớp
ax_q = subplot(2, 2, 2);
plot(ax_q, t_array, rad2deg(q1), 'r-', 'LineWidth', 1.2); hold(ax_q, 'on');
plot(ax_q, t_array, rad2deg(q2), 'g-', 'LineWidth', 1.2);
plot(ax_q, t_array, rad2deg(q3), 'b-', 'LineWidth', 1.2);
plot(ax_q, t_array, rad2deg(q4), 'c--', 'LineWidth', 1.2);
plot(ax_q, t_array, rad2deg(q5), 'm-', 'LineWidth', 1.2);
plot(ax_q, t_array, rad2deg(q6), 'k-', 'LineWidth', 1.2);
grid(ax_q, 'on');
xlabel(ax_q, 'Thời gian (s)'); ylabel(ax_q, 'Góc quay (Độ)');
title(ax_q, 'Đồ thị biến khớp');
legend(ax_q, 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'Location', 'best');

% toạ độ ee
ax_xyz = subplot(2, 2, 4);
plot(ax_xyz, t_array, xE, 'r-', 'LineWidth', 1.5); hold(ax_xyz, 'on');
plot(ax_xyz, t_array, yE, 'g-', 'LineWidth', 1.5);
plot(ax_xyz, t_array, zE, 'b-', 'LineWidth', 1.5);
grid(ax_xyz, 'on');
xlabel(ax_xyz, 'Thời gian (s)'); ylabel(ax_xyz, 'Tọa độ (mm)');
title(ax_xyz, 'Tọa độ End-Effector');
legend(ax_xyz, 'xE', 'yE', 'zE', 'Location', 'best');

%%
save_video = true;

if save_video
    v_writer = VideoWriter('Robot_Arm_3D_MH2.mp4', 'MPEG-4');
    v_writer.FrameRate = 30;
    open(v_writer);
    gif_filename = 'Robot_Arm_3D_MH2.gif';
end

disp('Đang chạy mô phỏng trực quan 3D cánh tay robot 6 trục');

xE_trace = []; yE_trace = []; zE_trace = [];


for i = 1:N
    
    X_i = X_joints(:, i);
    Y_i = Y_joints(:, i);
    Z_i = Z_joints(:, i);
    
    
    set(h_link_base,  'XData', X_i(1:2), 'YData', Y_i(1:2), 'ZData', Z_i(1:2));
    set(h_link_arm1,  'XData', X_i(2:3), 'YData', Y_i(2:3), 'ZData', Z_i(2:3));
    set(h_link_arm2,  'XData', X_i(3:5), 'YData', Y_i(3:5), 'ZData', Z_i(3:5));
    set(h_link_wrist, 'XData', X_i(5:7), 'YData', Y_i(5:7), 'ZData', Z_i(5:7));
    
    % Cập nhật vị trí khớp và End-Effector
    set(h_joints, 'XData', X_i, 'YData', Y_i, 'ZData', Z_i);
    set(h_ee,     'XData', X_i(7), 'YData', Y_i(7), 'ZData', Z_i(7));
    
    % Cập nhật vết đường đi của điểm E
    xE_trace = [xE_trace, X_i(7)];
    yE_trace = [yE_trace, Y_i(7)];
    zE_trace = [zE_trace, Z_i(7)];
    set(h_trace, 'XData', xE_trace, 'YData', yE_trace, 'ZData', zE_trace);
    
    % Vẽ lập tức
    drawnow;
    
    
    if save_video
        frame = getframe(fig);
        writeVideo(v_writer, frame);
        
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if i == 1
            imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.03);
        else
            imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.03);
        end
    end
    
    pause(0.005);
end

if save_video
    close(v_writer);
    disp('Đã xuất thành công file video Robot_Arm_3D_MH2.mp4 và ảnh động Robot_Arm_3D_MH2.gif');
end

disp('Mô phỏng 3D Mô hình 2 hoàn thành');
