clear all;
clc;
files1 = dir('../right/lines/lidar/line1_*.csv');
files2 = dir('../right/lines/lidar/line2_*.csv');
files3 = dir('../right/lines/lidar/line3_*.csv');
files4 = dir('../right/lines/lidar/line4_*.csv');

n = length(files1);
%n = 5;
scale = 1;
for i = 1:n
  figure(1)
  csv1 = load((strcat('../right/lines/lidar/', files1(i).name)));
  xP1 = csv1(:, 1); yP1 = csv1(:, 2); zP1 = csv1(:, 3);
  [A1, B1] = fit3DLine(csv1);
  
  csv2 = load((strcat('../right/lines/lidar/', files2(i).name)));
  xP2 = csv2(:, 1); yP2 = csv2(:, 2); zP2 = csv2(:, 3);
  [A2, B2] = fit3DLine(csv2);
  
  csv3 = load((strcat('../right/lines/lidar/', files3(i).name)));
  xP3 = csv3(:, 1); yP3 = csv3(:, 2); zP3 = csv3(:, 3);
  [A3, B3] = fit3DLine(csv3);
  
  csv4 = load((strcat('../right/lines/lidar/', files4(i).name)));
  xP4 = csv4(:, 1); yP4 = csv4(:, 2); zP4 = csv4(:, 3);
  [A4, B4] = fit3DLine(csv4);
  
  plot3(xP1, yP1, zP1, '+', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;
  C1 = linspace(A1-scale*B1, A1+scale*B1, 100);
  plot3(C1(1, :), C1(2, :), C1(3, :), '.', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;
  plot3(xP2, yP2, zP2, '+', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;  
  C2 = linspace(A2-scale*B2, A2+scale*B2, 100);
  plot3(C2(1, :), C2(2, :), C2(3, :), '.', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;
  plot3(xP3, yP3, zP3, '+', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5]);  
  hold on;
  C3 = linspace(A3-scale*B3, A3+scale*B3, 100);
  plot3(C3(1, :), C3(2, :), C3(3, :), '.', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;  
  plot3(xP4, yP4, zP4, '+', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','c',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;  
  C4 = linspace(A4-scale*B4, A4+scale*B4, 100);
  plot3(C4(1, :), C4(2, :), C4(3, :), '.', 'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','c',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
  hold on;
  origin = [0, 0, 0];
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
  grid;
  axis equal;
end

