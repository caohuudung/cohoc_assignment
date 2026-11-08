% Khởi tạo mảng thời gian 4 giây
t_array = linspace(0, 4, 400);
N = length(t_array);
q1 = zeros(1, N); q2 = zeros(1, N); q3 = zeros(1, N);

for i = 1:N
    t = t_array(i);
    
    if t < 1 % GĐ1: Vươn q3
        q1(i) = 100;
        q2(i) = 100;
        q3(i) = -2*pi/3 + (2*pi/3)*(3*t^2 - 2*t^3);
        
    elseif t < 2 % GĐ2: Nâng q1
        tl = t - 1; % Thời gian local
        q1(i) = 100 + 600*(3*tl^2 - 2*tl^3);
        q2(i) = 100;
        q3(i) = 0;
        
    elseif t < 3 % GĐ3: Vươn q2
        tl = t - 2;
        q1(i) = 700;
        q2(i) = 100 + 400*(3*tl^2 - 2*tl^3);
        q3(i) = 0;
        
    else % GĐ4: Cụp q3
        tl = t - 3;
        q1(i) = 700;
        q2(i) = 500;
        q3(i) = -(2*pi/3)*(3*tl^2 - 2*tl^3);
    end
end

% Sau đoạn này, thay q1, q2, q3 vào phương trình xE, zE để plot

% 4. Phương trình động học thuận (Forward Kinematics)
% l3 = 600 mm (Theo như giả định ở phần trước)
l3 = 600; 

% Tính góc hướng định vị phi theo phương trình ma trận
phi = q3 + pi/6; 

% Tính tọa độ End-Effector
xE = l3*cos(phi) + (sqrt(3)/2)*q2;
zE = l3*sin(phi) + q1 + 0.5*q2;

% 5. Trực quan hóa
figure;

% --- Đồ thị 1: Sự biến thiên của các khớp theo thời gian ---
subplot(2, 1, 1);
plot(t_array, q1, 'r', 'LineWidth', 2); hold on;
plot(t_array, q2, 'b', 'LineWidth', 2);
plot(t_array, rad2deg(q3)*10, 'g', 'LineWidth', 2); % Phóng to góc để dễ nhìn
legend('Khớp tịnh tiến q1', 'Khớp tịnh tiến q2', 'Khớp quay q3 (x10 deg)');
title('Biến thiên các biến khớp theo thời gian');
xlabel('Thời gian (s)'); ylabel('Giá trị');
grid on;

% --- Đồ thị 2: Quỹ đạo của tay máy trên mặt phẳng làm việc ---
subplot(2, 1, 2);
plot(xE, zE, 'k', 'LineWidth', 2);
title('Quỹ đạo chuyển động của End-Effector (X-Z)');
xlabel('Trục X (mm)'); ylabel('Trục Z (mm)');
axis equal; % Giữ tỉ lệ không bị biến dạng
grid on;

figure
plot(t_array, xE, 'r', 'LineWidth', 2); hold on;
plot(t_array, zE, 'b', 'LineWidth', 2);
legend('x', 'y');
title('Đồ thị toạ độ (x,y) theo thời gian');
xlabel('Thời gian (s)'); ylabel('Toạ độ (mm)');
grid on





% Chiều dài hằng số (theo sơ đồ của bạn)
l3 = 600; 

% Khởi tạo cửa sổ vẽ Animation
figure('Name', 'Mô phỏng chuyển động Cánh tay Robot 3 Bậc', 'Position', [100, 100, 800, 600]);
axis([-200 1000 0 1500]); % Giới hạn khung nhìn đồ thị (Cân chỉnh cho vừa tầm với)
grid on; hold on;
xlabel('Trục X (mm)');
ylabel('Trục Z (mm)');
title('Mô phỏng trực quan - Nhiệm vụ Gắp Đặt');

% Tạo các đối tượng đồ họa ban đầu (để cập nhật lại trong vòng lặp)
% Dùng các đường line dày để giả lập các khâu của cánh tay
link1_plot = plot([0 0], [0 0], 'b', 'LineWidth', 8); % Khâu 1 (Đứng)
link2_plot = plot([0 0], [0 0], 'm', 'LineWidth', 6); % Khâu 2 (Nghiêng)
link3_plot = plot([0 0], [0 0], 'g', 'LineWidth', 4); % Khâu 3 (Quay)
ee_plot = scatter(0, 0, 100, 'r', 'filled'); % Chấm đỏ tại End-Effector

% Đường vẽ lại vết quỹ đạo (Trace)
trajectory_plot = plot(0, 0, 'k--', 'LineWidth', 1.5); 
xE_history = []; % Lưu trữ lịch sử xE
zE_history = []; % Lưu trữ lịch sử zE

% Vòng lặp Animation (Chạy lại mảng thời gian)
for i = 1:N
    % 1. Tính toán tọa độ TẤT CẢ các khớp tại thời điểm hiện tại i
    % Gốc O0
    x0 = 0; 
    z0 = 0;
    
    % Khớp B (Sau tịnh tiến q1 dọc trục Z)
    xB = 0;
    zB = q1(i);
    
    % Khớp C (Sau tịnh tiến q2 góc 60 độ so với phương ngang / 30 độ so với Z)
    xC = xB + q2(i) * cos(pi/6); % q2 * cos(30 độ) ~ sqrt(3)/2 * q2
    zC = zB + q2(i) * sin(pi/6); % q2 * sin(30 độ) ~ 0.5 * q2
    
    % Điểm tác động E (Góc hướng tổng phi = q3 + pi/6)
    phi = q3(i) + pi/6;
    xE_val = xC + l3 * cos(phi);
    zE_val = zC + l3 * sin(phi);
    
    % Lưu lịch sử tọa độ E để vẽ vết
    xE_history = [xE_history, xE_val];
    zE_history = [zE_history, zE_val];
    
    % 2. Cập nhật dữ liệu cho các đường vẽ đồ họa
    % Nối O0 với B
    set(link1_plot, 'XData', [x0 xB], 'YData', [z0 zB]);
    % Nối B với C
    set(link2_plot, 'XData', [xB xC], 'YData', [zB zC]);
    % Nối C với E
    set(link3_plot, 'XData', [xC xE_val], 'YData', [zC zE_val]);
    % Cập nhật vị trí điểm cuối
    set(ee_plot, 'XData', xE_val, 'YData', zE_val);
    % Cập nhật nét vẽ quỹ đạo
    set(trajectory_plot, 'XData', xE_history, 'YData', zE_history);
    
    % 3. Ép MATLAB vẽ ngay lập tức thay vì đợi hết vòng lặp
    drawnow;
    
    % Tùy chọn: Chỉnh tốc độ khung hình (pause ngắn để nhìn rõ)
    % pause(0.01); 
end