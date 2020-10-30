clear all;
clc;
files_lidar = dir('../left/plane/lidar/*.csv')
files_camera = dir('../left/plane/camera/*.csv')

CL_T_L = textread('C_T_L_left_VLP.txt')';
CL_T_L = reshape(CL_T_L, 4, 3)';
CL_T_L = [CL_T_L; 0 0 0 1]

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
C1_T_C = [0  0 1 0; 
         -1  0 0 0; 
          0 -1 0 0; 
          0  0 0 1]; 
n = length(files_lidar);
%n = 5;
origin = [0, 0, 0];
np = 0;
for i = 1:n
  figure(1);
  csv_lidar = load((strcat('../left/plane/lidar/', files_lidar(i).name)));
%{
  csv_lidar = removeOutliers(csv_lidar, 
                             side_len, 
                             side_len);
%}                             
  xP = csv_lidar(:, 1);
  yP = csv_lidar(:, 2);
  zP = csv_lidar(:, 3);
  np = np + size(csv_lidar, 1);
  figure(1)
  color_no = (i-1)/length(files_lidar);
  plot3(xP, yP, zP, '.', 'MarkerSize', 10);
  hold on;
  csv_camera = load((strcat('../left/plane/camera/', files_camera(i).name)));
  R_t = csv_camera;
  R_t = [R_t; 0 0 0 1];
  C1_T_W = R_t;
  objectPts_C1 = C1_T_W*objectPts_W;
  objectPts_C1 = objectPts_C1';
  objectPts_C1 = [objectPts_C1; objectPts_C1(1, :)];
  objectPts_C1_L = inv(CL_T_L)*objectPts_C1';
  xC = objectPts_C1_L(1, :);
  yC = objectPts_C1_L(2, :);
  zC = objectPts_C1_L(3, :);
  x =[0.2, 0, 0];
  y =[0, 0.2, 0];
  z =[0, 0, 0.2];
  axis_pts_x = [origin; x];
  axis_pts_y = [origin; y];
  axis_pts_z = [origin; z];
  plot3(axis_pts_x(:, 1), axis_pts_x(:, 2), axis_pts_x(:, 3), 'LineWidth', 15, 'r');
  hold on;
  plot3(axis_pts_y(:, 1), axis_pts_y(:, 2), axis_pts_y(:, 3), 'LineWidth', 15, 'g');
  hold on;
  plot3(axis_pts_z(:, 1), axis_pts_z(:, 2), axis_pts_z(:, 3), 'LineWidth', 15, 'b');
  hold on;
  plot3(xC, yC, zC, '-','LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor',[color_no, color_no, color_no]);
  hold on
end
figure(1)
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('LiDAR Views');
axis equal;
grid;

np = np/n



