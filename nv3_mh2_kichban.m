clear; clc; close all;

%% 1. Thông số động học (mm, rad) - theo bảng DH
L1 = 100; L2 = 400; L3 = 70; L4 = 300; L5 = 60; L6 = 75;
H1 = 50;  H2 = 40;
alpha1 = 80*pi/180;

%% 2. Thời gian
T1 = 3;          % giai đoạn 1: tiếp cận (s)
T2 = 10;         % giai đoạn 2: hàn 1 vòng tròn, tốc độ hàn đều ~ 25 mm/s (s)
t_array = linspace(0, T1 + T2, 1300);
N = numel(t_array);

q1 = zeros(1,N); q2 = zeros(1,N); q3 = zeros(1,N);
q4 = zeros(1,N); q5 = zeros(1,N); q6 = zeros(1,N);

S3 = @(tau) 3*tau.^2 - 2*tau.^3;   % hàm làm mượt bậc 3

%% 3. Kịch bản
for i = 1:N
    t = t_array(i);
    if t <= T1   % ---- GĐ1: gập dần xuống, tiếp cận điểm bắt đầu hàn ----
        s = S3(t / T1);
        q1(i) = -2*pi/9  + (2*pi/9)  * s;     % -40 -> 0 độ
        q2(i) = -pi/3    + (7*pi/18) * s;     % -60 -> +10 độ
        q3(i) =  pi/18   - (pi/4)    * s;     % +10 -> -35 độ
        q4(i) = 0;
        q5(i) = -5*pi/18 + (5*pi/36) * s;     % -50 -> -25 độ  (= q2 + q3)
        q6(i) = 0;
    else         % ---- GĐ2: giữ 5 khớp đầu, q6 quay đều 1 vòng ----
        tau = (t - T1) / T2;                  % 0 -> 1
        q1(i) = 0;
        q2(i) =  pi/18;                       % +10 độ
        q3(i) = -7*pi/36;                     % -35 độ
        q4(i) = 0;
        q5(i) = -5*pi/36;                     % -25 độ (= q2 + q3)
        q6(i) = 2*pi * tau;                   % 0 -> 360 độ, tốc độ đều
    end
end

%% 4. Động học thuận (theo bảng DH: d6 = L5 + L6, a6 = H2)
c1 = cos(q1); s1 = sin(q1);
c2 = cos(q2);  s2 = sin(q2);
c4 = cos(q4); s4 = sin(q4);
c5 = cos(q5); s5 = sin(q5);
c6 = cos(q6); s6 = sin(q6);
c23 = cos(q2 + q3); s23 = sin(q2 + q3);

r11 = c1.*c23.*c4.*c5.*c6 + c1.*c23.*s4.*s6 + c1.*s23.*s5.*c6 - s1.*s4.*c5.*c6 + s1.*c4.*s6;
r13 = c1.*c23.*c4.*s5 - c1.*s23.*c5 - s1.*s4.*s5;
r21 = s1.*c23.*c4.*c5.*c6 + s1.*c23.*s4.*s6 + s1.*s23.*s5.*c6 + c1.*s4.*c5.*c6 - c1.*c4.*s6;
r23 = s1.*c23.*c4.*s5 - s1.*s23.*c5 + c1.*s4.*s5;
r31 = c23.*s5.*c6 - s23.*c4.*c5.*c6 - s23.*s4.*s6;
r33 = -c23.*c5 - s23.*c4.*s5;

rho = L1*cos(alpha1) + L2*c2 + L3*c23 + L4*s23;
px = c1.*rho + (L5 + L6).*r13 + H2.*r11;
py = s1.*rho + (L5 + L6).*r23 + H2.*r21;
pz = L1*sin(alpha1) - L2*s2 - L3*s23 + L4*c23 + (L5 + L6).*r33 + H2.*r31;

%% 5. Kiểm tra số liệu
iw = t_array >= T1;                                    % đoạn hàn
xc = mean(px(iw)); yc = mean(py(iw));
R_w = sqrt((px(iw)-xc).^2 + (py(iw)-yc).^2);
v_w = sqrt(diff(px(iw)).^2 + diff(py(iw)).^2 + diff(pz(iw)).^2) ./ diff(t_array(iw));
fprintf('Điểm chờ ban đầu : [%.1f, %.1f, %.1f] mm\n', px(1), py(1), pz(1));
fprintf('Điểm bắt đầu hàn : [%.1f, %.1f, %.1f] mm\n', px(find(iw,1)), py(find(iw,1)), pz(find(iw,1)));
fprintf('Tâm vòng hàn     : (%.2f, %.2f), z = %.2f mm\n', xc, yc, mean(pz(iw)));
fprintf('Bán kính vòng hàn: %.3f ... %.3f mm (H2 = %g)\n', min(R_w), max(R_w), H2);
fprintf('Dao động z trong GĐ2: %.4f mm\n', max(pz(iw)) - min(pz(iw)));
fprintf('Tốc độ hàn       : %.2f ... %.2f mm/s\n', min(v_w), max(v_w));

%% 6. Trực quan hóa
figure('Name', 'Bien thien cac khop', 'Color', 'w', 'Position', [100, 100, 700, 500]);
plot(t_array, rad2deg(q1), 'r', 'LineWidth', 2); hold on;
plot(t_array, rad2deg(q2), 'b', 'LineWidth', 2);
plot(t_array, rad2deg(q3), 'g', 'LineWidth', 2);
plot(t_array, rad2deg(q4), 'c', 'LineWidth', 2);
plot(t_array, rad2deg(q5), 'm', 'LineWidth', 2);
plot(t_array, rad2deg(q6), 'k--', 'LineWidth', 2);
xline(T1, 'k:', 'LineWidth', 1.5, 'Label', 'Bắt đầu hàn');
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'Location', 'northwest');
title('Biến thiên các biến khớp theo thời gian');
xlabel('Thời gian (s)'); ylabel('Góc quay (độ)'); grid on;

figure('Name', 'Toa do theo thoi gian', 'Color', 'w', 'Position', [850, 100, 650, 500]);
plot(t_array, px, 'r', 'LineWidth', 2); hold on;
plot(t_array, py, 'b', 'LineWidth', 2);
plot(t_array, pz, 'g', 'LineWidth', 2);
xline(T1, 'k:', 'LineWidth', 1.5);
legend('x(t)', 'y(t)', 'z(t)', 'Location', 'best');
title('Đồ thị tọa độ (x, y, z) theo thời gian');
xlabel('Thời gian (s)'); ylabel('Tọa độ (mm)'); grid on;

figure('Name', 'Quy dao khong gian 3D', 'Color', 'w', 'Position', [400, 200, 760, 640]);
i1 = t_array <= T1;
plot3(px(i1), py(i1), pz(i1), 'b--', 'LineWidth', 2); hold on;
plot3(px(iw), py(iw), pz(iw), 'r-', 'LineWidth', 3);
plot3(px(1), py(1), pz(1), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
plot3(px(find(iw,1)), py(find(iw,1)), pz(find(iw,1)), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(xc, yc, mean(pz(iw)), 'k+', 'MarkerSize', 10, 'LineWidth', 1.5);   % tâm ống
title('Quỹ đạo chuyển động của khâu cuối (EE) trong không gian 3D');
xlabel('Trục X (mm)'); ylabel('Trục Y (mm)'); zlabel('Trục Z (mm)');
legend('Pha tiếp cận', 'Đường hàn (vòng tròn nằm ngang)', 'Điểm chờ ban đầu', 'Điểm bắt đầu hàn', 'Tâm ống trụ', 'Location', 'best');
axis equal; grid on; view(135, 25);
