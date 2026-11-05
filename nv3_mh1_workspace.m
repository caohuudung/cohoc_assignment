% 1. Thông số chiều dài và số mẫu điểm quét
l3 = 600;
N = 100000; % Lấy 100,000 điểm ngẫu nhiên để đám mây điểm đủ mịn

% 2. Tạo vector biến khớp ngẫu nhiên trong giới hạn (mm, rad)
q1 = 100 + 600 * rand(1, N);         % Sinh ngẫu nhiên từ 100 đến 700
q2 = 100 + 400 * rand(1, N);         % Sinh ngẫu nhiên từ 100 đến 500
q3 = -3*pi/4 + (1.5*pi) * rand(1, N); % Sinh ngẫu nhiên từ -3pi/4 đến 3pi/4

% 3. Phương trình vị trí điểm cuối E (Toán tử vector)
xE = (sqrt(3)/2)*q2 + l3*cos(q3 + pi/6);
zE = q1 + 0.5*q2 + l3*sin(q3 + pi/6);

% 2. Lọc thân cột: CHỈ kích hoạt kiểm tra khi q3 < 0
% Điều kiện đâm: (q3 âm) VÀ (X nằm trong cột) VÀ (Z thấp hơn ngọn cột)
hit_body = (q3 < 0) & (xE < 50) & (zE < 800); 

% Điểm an toàn với thân cột là điểm KHÔNG bị đâm
safe_body = ~hit_body; 

% 3. Gộp điều kiện (Vừa không chạm đất, vừa không đâm thân)
valid_idx = safe_body;

% 4. Trích xuất và vẽ đồ thị
xE_real = xE(valid_idx);
zE_real = zE(valid_idx);




% Khởi tạo cửa sổ vẽ đồ thị
figure;

% --- Đồ thị 1: Miền làm việc Toán học (Bên trái) ---
ax1 = subplot(1, 2, 1); % Gán định danh hệ trục 1 vào ax1
scatter(ax1, xE, zE, 1, 'b', '.');
title(ax1, 'Miền làm việc toán học của Robot');
xlabel(ax1, 'Trục X (mm)');
ylabel(ax1, 'Trục Z (mm)');
axis(ax1, 'equal'); 
grid(ax1, 'on');

% --- Đồ thị 2: Miền làm việc Thực tế (Bên phải) ---
ax2 = subplot(1, 2, 2); % Gán định danh hệ trục 2 vào ax2
scatter(ax2, xE_real, zE_real, 1, 'b', '.');
title(ax2, 'Miền làm việc thực tế của Robot');
xlabel(ax2, 'Trục X (mm)');
ylabel(ax2, 'Trục Z (mm)');
axis(ax2, 'equal'); 
grid(ax2, 'on');

% --- ĐỒNG BỘ HÓA HAI TRỤC ---
linkaxes([ax1, ax2], 'xy'); % Buộc hai đồ thị luôn dùng chung tỉ lệ và khoảng zoom