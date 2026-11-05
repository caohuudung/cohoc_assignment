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