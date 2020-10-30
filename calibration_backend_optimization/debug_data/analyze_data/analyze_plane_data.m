clear all;
close all;
clc;
files_lidar = dir('../plane/lidar/*.csv');
files_camera = dir('../plane/camera/*.csv');
n = length(files_lidar);
m = length(files_camera);
assert(n == m);
for i = 1:1
  csv_lidar = load((strcat('../plane/lidar/', files_lidar(i).name)));
  csv_camera = load((strcat('../plane/camera/', files_camera(i).name)));
  R_t = csv_camera;
  r3 = R_t(:, 3);
  t = R_t(:, 4);
  [A, b] = formAb(csv_lidar, csv_camera)
end