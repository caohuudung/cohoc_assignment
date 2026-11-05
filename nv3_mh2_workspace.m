%% ========================================================================
% BÀI TẬP LỚN: CƠ HỌC HỆ NHIỀU VẬT - NHÓM 11
% Mô phỏng miền làm việc Mô hình 2 (Robot 6 DOF) có lọc va chạm bằng Ellipsoid
% Phương pháp: Rời rạc hóa khâu (Point Cloud Discretization) & Ánh xạ cục bộ
%% ========================================================================
clear; clc; close all;

%% BƯỚC 1: KHAI BÁO THÔNG SỐ HÌNH HỌC VÀ BIẾN KHỚP (Giữ nguyên từ file gốc)
N = 10^5; % Số lượng mẫu ngẫu nhiên Monte Carlo

% Kích thước các khâu (mm) theo Bảng thông số DH của báo cáo
L1 = 100; L2 = 400; L3 = 70; 
L4 = 300; L5 = 60;  L6 = 75;
H1 = 50;  H2 = 40;

alpha1 = 4*pi/9; % 80 độ
alpha2 = 5*pi/9; % 100 độ
alpha3 = 8*pi/9; % 160 độ

% Khởi tạo ngẫu nhiên biến khớp chạy full 360 độ (0 -> 2*pi)
q1 = 2*pi*rand(1,N); q2 = 2*pi*rand(1,N); q3 = 2*pi*rand(1,N);
q4 = 2*pi*rand(1,N); q5 = 2*pi*rand(1,N); q6 = 2*pi*rand(1,N);

% Khai báo hàm lượng giác
c1 = cos(q1); s1 = sin(q1);
c2 = cos(q2); s2 = sin(q2);
c3 = cos(q3); s3 = sin(q3);
c4 = cos(q4); s4 = sin(q4);
c5 = cos(q5); s5 = sin(q5);
c6 = cos(q6); s6 = sin(q6);

c23 = cos(q2 + q3);
s23 = sin(q2 + q3);

%% BƯỚC 2: ĐỘNG HỌC THUẬN CỦA END-EFFECTOR (T6_0 và TE6_0)
% Các thành phần của ma trận quay R6_0 (rij)
r11 = c1.*c23.*c4.*c5.*c6 + c1.*c23.*s4.*s6 + c1.*s23.*s5.*c6 - s1.*s4.*c5.*c6 + s1.*c4.*s6;
r12 = -c1.*c23.*c4.*c5.*s6 + c1.*c23.*s4.*c6 - c1.*s23.*s5.*s6 + s1.*s4.*c5.*s6 + s1.*c4.*c6;
r13 = c1.*c23.*c4.*s5 - c1.*s23.*c5 - s1.*s4.*s5;

r21 = s1.*c23.*c4.*c5.*c6 + s1.*c23.*s4.*s6 + s1.*s23.*s5.*c6 + c1.*s4.*c5.*c6 - c1.*c4.*s6;
r22 = -s1.*c23.*c4.*c5.*s6 + s1.*c23.*s4.*c6 - s1.*s23.*s5.*s6 - c1.*s4.*c5.*s6 - c1.*c4.*c6;
r23 = s1.*c23.*c4.*s5 - s1.*s23.*c5 + c1.*s4.*s5;

r31 = c23.*s5.*c6 - s23.*c4.*c5.*c6 - s23.*s4.*s6;
r32 = -c23.*s5.*s6 + s23.*c4.*c5.*s6 - s23.*s4.*c6;
r33 = -c23.*c5 - s23.*c4.*s5;


% Tọa độ điểm tác động cuối Pe (px, py, pz) trong hệ quy chiếu gốc O0
px = c1.*(L1*cos(alpha1) + L2*c2 + L3*c23 + L4*s23) + (L5 + L6).*r13 + H2.*r11;
py = s1.*(L1*cos(alpha1) + L2*c2 + L3*c23 + L4*s23) + (L5 + L6).*r23 + H2.*r21;
pz = L1*sin(alpha1) - L2*s2 - L3*s23 + L4*c23 + (L5 + L6).*r33 + H2.*r31;

% Ma trận hướng thực tế của End-Effector (Re = R6_0 * Ry(20))
Re_11 = r11*cosd(20) - r13*sind(20); Re_12 = r12; Re_13 = r11*sind(20) + r13*cosd(20);
Re_21 = r21*cosd(20) - r23*sind(20); Re_22 = r22; Re_23 = r21*sind(20) + r23*cosd(20);
Re_31 = r31*cosd(20) - r33*sind(20); Re_32 = r32; Re_33 = r31*sind(20) + r33*cosd(20);

%% BƯỚC 3: XÁC ĐỊNH TỌA ĐỘ CÁC KHỚP TRUNG GIAN ĐỂ RỜI RẠC HÓA (T_i^0)
% Dựa vào chuỗi nhân ma trận A_i^{i-1} trong Chương 3 của báo cáo:
d1 = L1*sin(alpha1); 
a1 = L1*cos(alpha1);

% 1. Tọa độ gốc O1 (Đỉnh khâu 1) trong hệ O0:
O1_x = a1 * c1;
O1_y = a1 * s1;
O1_z = d1 * ones(1,N);

% 2. Tọa độ gốc O2 (Đỉnh khâu 2 - Khuỷu tay) trong hệ O0:
O2_x = c1 .* (a1 + L2*c2);
O2_y = s1 .* (a1 + L2*c2);
O2_z = d1 - L2*s2;

% 3. Tọa độ gốc O3 (Đỉnh khâu 3) trong hệ O0:
O3_x = c1 .* (a1 + L2*c2 + L3*c23);
O3_y = s1 .* (a1 + L2*c2 + L3*c23);
O3_z = d1 - L2*s2 - L3*s23;

% 4. Tọa độ gốc O4 trùng O5 (Tâm cụm cổ tay - Đỉnh khâu 4) trong hệ O0:
% Do khâu 4 trượt dọc trục z3 một đoạn d4 = L4
O4_x = O3_x + c1 .* (L4*s23);
O4_y = O3_y + s1 .* (L4*s23);
O4_z = O3_z + L4*c23;

%% BƯỚC 4: THIẾT LẬP THÔNG SỐ CÁC ELLIPSOID BAO BỌC KHÂU (HỆ CỤC BỘ)
% Mỗi Ellipsoid thứ i có tâm P_ci = [x_ci; y_ci; z_ci] và bán trục [a_i, b_i, c_i]
% gắn cố định trong chính hệ trục tọa độ cục bộ của khâu đó:

% - Ellipsoid 0: Bao bọc Đế và Thân (Khâu 0 & 1), gắn vào hệ gốc O0
Pc0 = [0; 0; d1/2]; 
a0 = 90; b0 = 90; c0 = d1/2 + 30; % Bán trục (mm)

% - Ellipsoid 2: Bao bọc Cánh tay dưới (Khâu 2), gắn vào hệ cục bộ O2
% Trong hệ O2, thân Khâu 2 nằm dọc theo chiều âm trục x2 từ 0 đến -L2
Pc2 = [-L2/2; 0; 0]; 
a2 = L2/2; b2 = 55; c2 = 55; % Bán trục dọc thân là L2/2, bán kính ngang 55mm

% - Ellipsoid 4: Bao bọc Cánh tay trên (Khâu 4), gắn vào hệ cục bộ O3
% Trong hệ O3, thân Khâu 4 vươn dọc theo trục z3 từ 0 đến L4
Pc4 = [0; 0; L4/2]; 
a4 = 45; b4 = 45; c4 = L4/2; % Bán trục dọc trục z3 là L4/2, bán kính ngang 45mm

%% BƯỚC 5: RỜI RẠC HÓA KHÂU & KIỂM TRA VA CHẠM (THEO PHƯƠNG PHÁP 1)
% Khởi tạo mảng logic: ban đầu giả sử cả 10^5 cấu hình đều hợp lệ (1 = Hợp lệ, 0 = Va chạm)
is_valid = true(1, N);

% Rời rạc hóa từng khâu thành K điểm lấy mẫu (k = 1..K)
K = 6; 
t_vals = linspace(0.1, 1, K); % Tránh điểm 0 vì là khớp nối trực tiếp giữa 2 khâu liền kề

% -------------------------------------------------------------------------
% 5.1. Kiểm tra Va chạm mặt đất (Sàn xưởng tại z = 0)
% -------------------------------------------------------------------------
% Nếu các cụm khớp hoặc điểm tác động cuối đập xuống sàn (z < 0) -> Loại
is_valid = is_valid & (O2_z > 20) & (O3_z > 20) & (O4_z > 20) & (pz > 0);

% -------------------------------------------------------------------------
% 5.2. Kiểm tra: Khâu 4 và Cụm công cụ (O4 -> Pe) va chạm với Đế (Ellipsoid 0)
% -------------------------------------------------------------------------
for k = 1:K
    tk = t_vals(k);
    % Tọa độ điểm lấy mẫu trên Khâu 4 (nối từ O3 đến O4) trong hệ O0
    P_k4_x = O3_x + tk*(O4_x - O3_x);
    P_k4_y = O3_y + tk*(O4_y - O3_y);
    P_k4_z = O3_z + tk*(O4_z - O3_z);
    
    % Thay vào phương trình chính tắc của Ellipsoid 0 (đã ở sẵn hệ O0)
    D_k4_0 = ((P_k4_x - Pc0(1))/a0).^2 + ((P_k4_y - Pc0(2))/b0).^2 + ((P_k4_z - Pc0(3))/c0).^2;
    is_valid = is_valid & (D_k4_0 > 1);
    
    % Tọa độ điểm lấy mẫu trên Cụm công cụ - Mỏ hàn (nối từ O4 đến Pe) trong hệ O0
    P_ee_x = O4_x + tk*(px - O4_x);
    P_ee_y = O4_y + tk*(py - O4_y);
    P_ee_z = O4_z + tk*(pz - O4_z);
    
    % Kiểm tra va chạm với Ellipsoid 0 (Đế)
    D_ee_0 = ((P_ee_x - Pc0(1))/a0).^2 + ((P_ee_y - Pc0(2))/b0).^2 + ((P_ee_z - Pc0(3))/c0).^2;
    is_valid = is_valid & (D_ee_0 > 1);
end

% -------------------------------------------------------------------------
% 5.3. Kiểm tra: Khâu 4 và Cụm công cụ va chạm với Khâu 2 (Ellipsoid 2 - Hệ O2)
% Áp dụng công thức: P_jk^2 = (T_2^0)^(-1) * P_jk^0 = (R_2^0)^T * (P_jk^0 - O2)
% Với ma trận R_2^0 = [c1*c2, -c1*s2, -s1; s1*c2, -s1*s2, c1; -s2, -c2, 0]
% -------------------------------------------------------------------------
for k = 1:K
    tk = t_vals(k);
    
    % --- A. Xét các điểm trên Khâu 4 (O3 -> O4) so với Ellipsoid 2 ---
    P_k4_x = O3_x + tk*(O4_x - O3_x);
    P_k4_y = O3_y + tk*(O4_y - O3_y);
    P_k4_z = O3_z + tk*(O4_z - O3_z);
    
    % Bước 1: Tịnh tiến về gốc O2 (P_jk^0 - O2)
    dx = P_k4_x - O2_x;
    dy = P_k4_y - O2_y;
    dz = P_k4_z - O2_z;
    
    % Bước 2: Xoay về hệ cục bộ O2 bằng cách nhân với chuyển vị (R_2^0)^T
    x_local_2 = (c1.*c2).*dx + (s1.*c2).*dy - s2.*dz;
    y_local_2 = (-c1.*s2).*dx + (-s1.*s2).*dy - c2.*dz;
    z_local_2 = -s1.*dx + c1.*dy;
    
    % Bước 3: Tính tham số D trong hệ cục bộ O2 (Trừ tọa độ tâm Pc2)
    D_k4_2 = ((x_local_2 - Pc2(1))/a2).^2 + ((y_local_2 - Pc2(2))/b2).^2 + ((z_local_2 - Pc2(3))/c2).^2;
    is_valid = is_valid & (D_k4_2 > 1);
    
    % --- B. Xét các điểm trên Cụm công cụ (O4 -> Pe) so với Ellipsoid 2 ---
    P_ee_x = O4_x + tk*(px - O4_x);
    P_ee_y = O4_y + tk*(py - O4_y);
    P_ee_z = O4_z + tk*(pz - O4_z);
    
    dx_e = P_ee_x - O2_x;
    dy_e = P_ee_y - O2_y;
    dz_e = P_ee_z - O2_z;
    
    x_ee_local_2 = (c1.*c2).*dx_e + (s1.*c2).*dy_e - s2.*dz_e;
    y_ee_local_2 = (-c1.*s2).*dx_e + (-s1.*s2).*dy_e - c2.*dz_e;
    z_ee_local_2 = -s1.*dx_e + c1.*dy_e;
    
    D_ee_2 = ((x_ee_local_2 - Pc2(1))/a2).^2 + ((y_ee_local_2 - Pc2(2))/b2).^2 + ((z_ee_local_2 - Pc2(3))/c2).^2;
    is_valid = is_valid & (D_ee_2 > 1);
end

% -------------------------------------------------------------------------
% 5.4. Kiểm tra: Cụm công cụ (O4 -> Pe) tự gập vào Khâu 4 (Ellipsoid 4 - Hệ O3)
% Áp dụng công thức: P_ee^3 = (T_3^0)^(-1) * P_ee^0 = (R_3^0)^T * (P_ee^0 - O3)
% Với ma trận R_3^0 = [c1*c23, -s1, c1*s23; s1*c23, c1, s1*s23; -s23, 0, c23]
% -------------------------------------------------------------------------
for k = 2:K % Bỏ qua điểm sát khớp cổ tay O4 để tránh báo va chạm giả tại khớp nối
    tk = t_vals(k);
    P_ee_x = O4_x + tk*(px - O4_x);
    P_ee_y = O4_y + tk*(py - O4_y);
    P_ee_z = O4_z + tk*(pz - O4_z);
    
    % Bước 1: Tịnh tiến về gốc O3
    dx_e = P_ee_x - O3_x;
    dy_e = P_ee_y - O3_y;
    dz_e = P_ee_z - O3_z;
    
    % Bước 2: Nhân với (R_3^0)^T để chiếu lên hệ cục bộ O3
    x_ee_local_3 = (c1.*c23).*dx_e + (s1.*c23).*dy_e - s23.*dz_e;
    y_ee_local_3 = -s1.*dx_e + c1.*dy_e;
    z_ee_local_3 = (c1.*s23).*dx_e + (s1.*s23).*dy_e + c23.*dz_e;
    
    % Bước 3: Tính tham số D đối với Ellipsoid 4
    D_ee_4 = ((x_ee_local_3 - Pc4(1))/a4).^2 + ((y_ee_local_3 - Pc4(2))/b4).^2 + ((z_ee_local_3 - Pc4(3))/c4).^2;
    is_valid = is_valid & (D_ee_4 > 1);
end

% Lọc lấy các tọa độ End-Effector hợp lệ (D > 1 trên toàn bộ các khâu)
px_valid = px(is_valid);
py_valid = py(is_valid);
pz_valid = pz(is_valid);

fprintf('Tổng số mẫu khởi tạo: %d\n', N);
fprintf('Số điểm hợp lệ sau khi lọc va chạm Ellipsoid: %d (Chiếm %.2f%%)\n', ...
        length(px_valid), (length(px_valid)/N)*100);

%% BƯỚC 6: BIỂU DIỄN ĐỒ THỊ MIỀN LÀM VIỆC (WORKSPACE VISUALIZATION)
% 1. Đồ thị 3D đối chiếu: Miền làm việc Toán học vs Miền làm việc Thực tế
figure('Name', 'Mien lam viec 3D - Mo hinh 2', 'Color', 'w', 'Position', [100, 100, 1200, 550]);

% Tìm giới hạn trục Z chung từ tập dữ liệu ban đầu (chưa lọc) để ép thang màu
z_min = min(pz);
z_max = max(pz);

subplot(1,2,1);
scatter3(px(1:2:end), py(1:2:end), pz(1:2:end), 1, pz(1:2:end), 'filled');
title('Miền làm việc toán học của Robot', 'FontSize', 12);
xlabel('Trục X (mm)'); ylabel('Trục Y (mm)'); zlabel('Trục Z (mm)');
axis equal; grid on; view(45, 25); colormap(jet); colorbar;
clim([z_min z_max]); % Đồng nhất thang màu
zlim([z_min z_max]);  % Đồng nhất tỷ lệ trục Z

subplot(1,2,2);
scatter3(px_valid(1:2:end), py_valid(1:2:end), pz_valid(1:2:end), 1, pz_valid(1:2:end), 'filled');
title('Miền làm việc thực tế của Robot', 'FontSize', 12);
xlabel('Trục X (mm)'); ylabel('Trục Y (mm)'); zlabel('Trục Z (mm)');
axis equal; grid on; view(45, 25); colormap(jet); colorbar;
clim([z_min z_max]); % Ép thang màu chạy theo dải giống hình 1
zlim([z_min z_max]);  % Ép khung nhìn trục Z giống hình 1

% 2. Đồ thị mặt cắt 2D (Giúp quan sát rõ khoảng trống bị khoét rỗng do Ellipsoid)
figure('Name', 'Mat cat Mien lam viec X-Z', 'Color', 'w', 'Position', [150, 150, 1100, 500]);

% Lấy lát cắt mỏng đi qua tâm robot (-40mm <= Y <= 40mm)
slice_math = abs(py) <= 40;
slice_real = abs(py_valid) <= 40;

subplot(1,2,1);
plot(px(slice_math), pz(slice_math), 'b.', 'MarkerSize', 2);
title('Mặt cắt X-Z Miền làm việc Toán học', 'FontSize', 12);
xlabel('Trục X (mm)'); ylabel('Trục Z (mm)');
axis equal; grid on; xlim([-1000 1000]); ylim([-600 1000]);

subplot(1,2,2);
plot(px_valid(slice_real), pz_valid(slice_real), 'b.', 'MarkerSize', 2);
hold on;
% Vẽ phác họa mặt cắt Ellipsoid 0 (Đế robot) để thấy rõ vùng bị loại bỏ
theta_elip = linspace(0, 2*pi, 100);
plot(Pc0(1) + a0*cos(theta_elip), Pc0(3) + c0*sin(theta_elip), 'r--', 'LineWidth', 1.5);
title('Mặt cắt X-Z Miền làm việc Thực tế (Lọc va chạm)', 'FontSize', 12);
xlabel('Trục X (mm)'); ylabel('Trục Z (mm)');
legend('Điểm EE hợp lệ', 'Ellipsoid Đế', 'Location', 'northeast');
axis equal; grid on; xlim([-1000 1000]); ylim([-600 1000]);
