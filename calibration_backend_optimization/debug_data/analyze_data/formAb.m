function [A, b] = formAb(lidar_pts, R_T)
  A = [];
  b = [];
  r3 = R_T(:, 3);
  tvec = R_T(:, 4);
  for i = 1:size(lidar_pts, 1)
    p = lidar_pts(i, :);
    a = [r3(1)*p, r3(2)*p, r3(3)*p, r3'];
    b = [b; r3'*tvec];
    A = [A;a];
  end  
end
