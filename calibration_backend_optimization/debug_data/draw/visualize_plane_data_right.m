clear all;
clc;
files_lidar = dir('../right/plane/lidar/*.csv')
files_camera = dir('../right/plane/camera/*.csv')

files1 = dir('../right/lines/lidar/line1_*.csv');
files2 = dir('../right/lines/lidar/line2_*.csv');
files3 = dir('../right/lines/lidar/line3_*.csv');
files4 = dir('../right/lines/lidar/line4_*.csv');

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
for i = 1:1

  csv_lidar = load((strcat('../left/plane/lidar/', files_lidar(i).name)));
%{
  csv_lidar = removeOutliers(csv_lidar, 
                             side_len, 
                             side_len);
%}                             
  xP = csv_lidar(:, 1);
  yP = csv_lidar(:, 2);
  zP = csv_lidar(:, 3);
%  %{
  figure(1);
  color_no = (i-1)/length(files_lidar);
  plot3(xP, yP, zP, '.', 'Color', [0.5, 0.5, 0.5],'MarkerSize',10,'MarkerFaceColor',[0.5, 0.5, 0.5]);
  hold on;
  csv_camera = load((strcat('../left/plane/camera/', files_camera(i).name)));
  R_t = csv_camera;
  R_t = [R_t; 0 0 0 1];
  C1_T_W = C1_T_C*R_t;
  objectPts_C1 = C1_T_W*objectPts_W;
  objectPts_C1 = objectPts_C1';
  objectPts_C1 = [objectPts_C1; objectPts_C1(1, :)];
  xC = objectPts_C1(:, 1);
  yC = objectPts_C1(:, 2);
  zC = objectPts_C1(:, 3);
  x =[0.2, 0, 0];
  y =[0, 0.2, 0];
  z =[0, 0, 0.2];
  axis_pts_x = [origin; x];
  axis_pts_y = [origin; y];
  axis_pts_z = [origin; z];
  plot3(axis_pts_x(:, 1), axis_pts_x(:, 2), axis_pts_x(:, 3), 'LineWidth', 5, 'r');
  hold on;
  plot3(axis_pts_y(:, 1), axis_pts_y(:, 2), axis_pts_y(:, 3), 'LineWidth', 5, 'g');
  hold on;
  plot3(axis_pts_z(:, 1), axis_pts_z(:, 2), axis_pts_z(:, 3), 'LineWidth', 5, 'b');
  hold on;
  
  figure(2)
  plot3(xC, yC, zC, '-','LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor',[color_no, color_no, color_no]);
  hold on
  x =[0.2, 0, 0];
  y =[0, -0.2, 0];
  z =[0, 0, -0.2];
  axis_pts_x = [origin; x];
  axis_pts_y = [origin; y];
  axis_pts_z = [origin; z];
  plot3(axis_pts_x(:, 1), 
        axis_pts_x(:, 2), 
        axis_pts_x(:, 3), 'LineWidth', 5, 'b');
  hold on;
  plot3(axis_pts_y(:, 1), 
        axis_pts_y(:, 2), 
        axis_pts_y(:, 3), 'LineWidth', 5, 'r');
  hold on;
  plot3(axis_pts_z(:, 1), 
        axis_pts_z(:, 2), 
        axis_pts_z(:, 3), 'LineWidth', 5, 'g');
  hold on;
%  %}
end 
%n = length(files1);
figure(1)
n = [1];
for i = n

  csv1 = load((strcat('../right/lines/lidar/', files1(i).name)));
  xP1 = csv1(:, 1); yP1 = csv1(:, 2); zP1 = csv1(:, 3);
  
  csv2 = load((strcat('../right/lines/lidar/', files2(i).name)));
  xP2 = csv2(:, 1); yP2 = csv2(:, 2); zP2 = csv2(:, 3);
  
  csv3 = load((strcat('../right/lines/lidar/', files3(i).name)));
  xP3 = csv3(:, 1); yP3 = csv3(:, 2); zP3 = csv3(:, 3);
  
  csv4 = load((strcat('../right/lines/lidar/', files4(i).name)));
  xP4 = csv4(:, 1); yP4 = csv4(:, 2); zP4 = csv4(:, 3);
  
  plot3(xP1, yP1, zP1, '+', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;
  plot3(xP2, yP2, zP2, '+', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;  
  plot3(xP3, yP3, zP3, '+', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5]);  
  hold on;
  plot3(xP4, yP4, zP4, '+', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','c',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;  
end

figure(1) 
hold off;
grid;
axis equal;
title('LiDAR Views');
figure(2)
hold off;
grid;
axis equal;
title('Camera Views');