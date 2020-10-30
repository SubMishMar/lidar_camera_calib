function [line] = getLineEquation (pt1, pt2)
  x_a = pt1(1); y_a = pt1(2);
  x_b = pt2(1); y_b = pt2(2);
  if x_a == y_a && x_b == y_b
    line = zeros(3, 1);
  elseif x_a == x_b
    line = [1; 0; -x_a];
  elseif y_a == y_b
    line = [0; 1; -y_a];
  else
    m = (y_b - y_a)/(x_b - x_a);
    a = m;
    b = -1;
    c = y_a - m*x_a;
    line = [a; b; c];
  end
end
