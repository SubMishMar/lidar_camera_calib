
files_lidar = dir('../right/plane/lidar/*.csv')
files_camera = dir('../right/plane/camera/*.csv')

side_len = 1.016;
objectPts_W = [0, 0, 0, 1; 
               0, side_len, 0, 1; 
               side_len, side_len, 0, 1;
               side_len, 0, 0, 1]';
fx = 6.4372590342756985e+02;
fy = 6.4372590342756985e+02;
cx = 3.9534097290039062e+02;
cy = 3.0199901199340820e+02;
K = [fx, 0, cx; 0, fy, cy; 0, 0, 1];

n = length(files_lidar);
A = [];
b = [];
for i = 1:n
  csv_lidar = load((strcat('../right/plane/lidar/', files_lidar(i).name)));
  csv_camera = load((strcat('../right/plane/camera/', files_camera(i).name)));
  X_l = csv_lidar(:, 1); Y_l = csv_lidar(:, 2); Z_l = csv_lidar(:, 3);
  c_RT_w = csv_camera;
  r3 = c_RT_w(:, 3);
  tvec = c_RT_w(:, 4);
  bi = r3'*tvec;
  a1 = r3(1).*[X_l, Y_l, Z_l];
  a2 = r3(2).*[X_l, Y_l, Z_l];
  a3 = r3(3).*[X_l, Y_l, Z_l];
  a4 = r3'.*ones(size(a3));
  a = [a1, a2, a3, a4];
  A = [A;a];
  b = [b; bi*ones(size(a, 1), 1)];
end
cond(A)
%x = A\b;
%R_ = reshape(x(1:9), 3, 3)';
%[U, S, V] = svd(R_);
%R = U*V'
%alpha = norm(R)/norm(R_)
%t = alpha*x(10:12)
%t_ = x(10:12);
%tz_ = t_(3);

line1_files_lidar =  dir('../right/lines/lidar/line1_*.csv');
line2_files_lidar =  dir('../right/lines/lidar/line2_*.csv');
line3_files_lidar =  dir('../right/lines/lidar/line3_*.csv');
line4_files_lidar =  dir('../right/lines/lidar/line4_*.csv');
line1_files_camera =  dir('../right/lines/camera/camera_line1_view*.csv');
line2_files_camera =  dir('../right/lines/camera/camera_line2_view*.csv');
line3_files_camera =  dir('../right/lines/camera/camera_line3_view*.csv');
line4_files_camera =  dir('../right/lines/camera/camera_line4_view*.csv');
n = length(files_lidar);

for i = 1:n
  csv_lidar_line1 = load((strcat('../right/lines/lidar/', line1_files_lidar(i).name)));
  csv_camera_line1 = load((strcat('../right/lines/camera/', line1_files_camera(i).name)));
  csv_lidar_line2 = load((strcat('../right/lines/lidar/', line2_files_lidar(i).name)));
  csv_camera_line2 = load((strcat('../right/lines/camera/', line2_files_camera(i).name)));
  csv_lidar_line3 = load((strcat('../right/lines/lidar/', line3_files_lidar(i).name)));
  csv_camera_line3 = load((strcat('../right/lines/camera/', line3_files_camera(i).name)));
  csv_lidar_line4 = load((strcat('../right/lines/lidar/', line4_files_lidar(i).name)));
  csv_camera_line4 = load((strcat('../right/lines/camera/', line4_files_camera(i).name)));
  X_1 = csv_lidar_line1(:, 1); Y_1 = csv_lidar_line1(:, 2); Z_1 = csv_lidar_line1(:, 3);
  X_2 = csv_lidar_line2(:, 1); Y_2 = csv_lidar_line2(:, 2); Z_2 = csv_lidar_line2(:, 3);
  X_3 = csv_lidar_line3(:, 1); Y_3 = csv_lidar_line3(:, 2); Z_3 = csv_lidar_line3(:, 3);
  X_4 = csv_lidar_line4(:, 1); Y_4 = csv_lidar_line4(:, 2); Z_4 = csv_lidar_line4(:, 3);
  normal1 = csv_camera_line1';
  normal2 = csv_camera_line2';
  normal3 = csv_camera_line3';
  normal4 = csv_camera_line4';
  
  a1_l1 = normal1(1).*[X_1, Y_1, Z_1];
  a2_l1 = normal1(2).*[X_1, Y_1, Z_1];
  a3_l1 = normal1(3).*[X_1, Y_1, Z_1];
  a4_l1 = normal1'.*ones(size(a3_l1));
  a_l1 = [a1_l1, a2_l1, a3_l1, a4_l1];
  
  a1_l2 = normal2(1).*[X_2, Y_2, Z_2];
  a2_l2 = normal2(2).*[X_2, Y_2, Z_2];
  a3_l2 = normal2(3).*[X_2, Y_2, Z_2];
  a4_l2 = normal2'.*ones(size(a3_l2));
  a_l2 = [a1_l2, a2_l2, a3_l2, a4_l2];

  a1_l3 = normal3(1).*[X_3, Y_3, Z_3];
  a2_l3 = normal3(2).*[X_3, Y_3, Z_3];
  a3_l3 = normal3(3).*[X_3, Y_3, Z_3];
  a4_l3 = normal3'.*ones(size(a3_l3));
  a_l3 = [a1_l3, a2_l3, a3_l3, a4_l3];  
  
  a1_l4 = normal4(1).*[X_4, Y_4, Z_4];
  a2_l4 = normal4(2).*[X_4, Y_4, Z_4];
  a3_l4 = normal4(3).*[X_4, Y_4, Z_4];
  a4_l4 = normal4'.*ones(size(a3_l4));
  a_l4 = [a1_l4, a2_l4, a3_l4, a4_l4];  
  
  a = [ a_l1; a_l2; a_l3; a_l4];
  A = [A; a];
  b = [b; zeros(size(a,1), 1)];
end

cond(A)
x = A\b;
R_ = reshape(x(1:9), 3, 3)';
[U, S, V] = svd(R_);
R = U*V'
alpha = norm(R)/norm(R_)
t = alpha*x(10:12)
%t_ = x(10:12);
%tz_ = t_(3);

T_r  = [R, t; 0 0 0 1];